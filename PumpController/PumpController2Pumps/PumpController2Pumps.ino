// A water pump controller, single water sensonr, battery and shore powered with 2 pumps.
// Also suports humidity/temp via DHT device,

// A0 is the water sensor (two electrodes between ground and A0 with A0 pulled up to VCC via 200KOhm)
// A1 is the battery voltage via a ~1:10 divider

// D 4/5/6/7 are relays (zero-active): D4 is pump 1 (active low); D5 is pump 2; D6 is charging relay (active low)

// D12 is DHT device

#define LED_PIN 13
#define PUMP1_PIN 4
#define PUMP2_PIN 5
#define CHARGE_PIN 6

#define BEEP_PIN 11
#define DHT_PIN 12

#include <EEPROM.h>
#include <dht11pp.h>

const char build_timestamp[] = __DATE__ " " __TIME__;

////////////////////////////////////////////////////////////////////////////////
// Configuration/EEPROM

typedef struct ConfigData
{
  uint16_t checkSum{0};  // simple check sum of the configuration data
  uint16_t batMV{12860}; // measured 12.86V--> 258 DAC COUNTS (this may be temp dependent!)
  uint16_t batDAC{258}; // corresponding DAC for measured mV
  uint16_t batMinMV{8000}; // min mV (critical baterry level - stop anything else and recover)
  uint16_t batMaxMV{14500}; // max mV (stops charging)
  uint16_t batNomMV{13000}; // nominal mV (start charging below)
  uint16_t batFloMV{13600}; // float mV (after reaching Max idling got to float)
  uint16_t pumpTreshold{700}; // water sense treshold (lower reading means water is present)
  uint16_t pump1OnTime{20}; // seconds to turn the pump on
  uint16_t pump2OnTime{20}; // seconds to turn the pump on
  uint16_t pumpRestTime{30}; // time between pump can go on
  uint16_t stateUpdateTime{10}; // period to output state to Serial
};

class ConfigManager
{
public:  
  static uint16_t computeChecksum(struct ConfigData& configData)
  {
    uint16_t sum = 0;
    for(size_t i = sizeof(configData.checkSum); i < sizeof(configData); ++i)
    {
        sum += ((uint8_t*)&configData)[i];
    }
    return sum;
  }  

  void reset()
  {
    mConfig = ConfigData{};
  }

  void store()
  {
    mConfig.checkSum = computeChecksum(mConfig);
    EEPROM.put(0, mConfig);
  }

  bool load()
  {
    bool success = false;
    ConfigData conf;
    EEPROM.get(0, conf);
    uint16_t cs = computeChecksum(conf);
    if( cs == conf.checkSum )
    {
      mConfig = conf;    
      success = true;
    }
    return success;
  }

  void dump()
  {
    Serial.print("batMV       : ");Serial.println(mConfig.batMV);
    Serial.print("batDAC      : ");Serial.println(mConfig.batDAC);
    Serial.print("batMaxMV    : ");Serial.println(mConfig.batMaxMV);
    Serial.print("batMinMV    : ");Serial.println(mConfig.batMinMV);
    Serial.print("batNomMV    : ");Serial.println(mConfig.batNomMV);
    Serial.print("batFloMV    : ");Serial.println(mConfig.batFloMV);
    Serial.print("pumpTreshold: ");Serial.println(mConfig.pumpTreshold);
    Serial.print("pump1OnTime : ");Serial.println(mConfig.pump1OnTime);
    Serial.print("pump2OnTime : ");Serial.println(mConfig.pump2OnTime);
    Serial.print("pumpRestTime: ");Serial.println(mConfig.pumpRestTime);
    Serial.print("stateUpdateTime: ");Serial.println(mConfig.stateUpdateTime);
  }

  ConfigData& data() { return mConfig; }

private:
  ConfigData mConfig;

} config;


////////////////////////////////////////////////////////////////////////////////
// return Vcc in [mV] measured using internal reference 1.1V

long readVccMV()
{
  // read 1.1V Vref against AVcc first
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1); 
  // wait for Vref to settle 
  delay(2); 
  // read several time and average
  const int AVERAGECOUNT = 16;
  uint32_t sum = 0;
  for(int i=0; i < AVERAGECOUNT; ++i)
  {
    ADCSRA |= _BV(ADSC); 
    // convert 
    while(bit_is_set(ADCSRA, ADSC)); 
    uint16_t v = ADCL;
    v |= ADCH<<8; 
    sum += v;
  }
  sum /= AVERAGECOUNT;
  
  auto result = 1126400L / sum; // Back-calculate AVcc in mV 
  return result; 
}

////////////////////////////////////////////////////////////////////////////////
// battery [mV] (via resistor divider)

int readBatteryMV()
{
  const int AVERAGECOUNT = 16;
  uint32_t sum = 0;
  for(int i=0; i < AVERAGECOUNT; ++i)
    sum += analogRead(A1);
  sum /= AVERAGECOUNT;
  auto batMV = (sum*config.data().batMV)/config.data().batDAC;
  return (int)batMV;
}

////////////////////////////////////////////////////////////////////////////////
// water sensor readout [dac counts]

int readSensor()
{
  const int AVERAGECOUNT = 16;
  int32_t sum = 0;
  for(int i=0; i < AVERAGECOUNT; ++i)
    sum += analogRead(A0);
  auto ret = (int16_t)(sum/AVERAGECOUNT);
  return ret;
}

////////////////////////////////////////////////////////////////////////////////
// DHT temp & humidity

using MYDHT = DHT<DHT_PIN>;

MYDHT::Result readTempHum()
{
  MYDHT dht;
  auto result = dht.read();
  return result;
}

////////////////////////////////////////////////////////////////////////////////

void sleep1s()
{
  digitalWrite(LED_PIN,1);
  delay(500UL);
  digitalWrite(LED_PIN,0);
  delay(500UL);
}

void beep(int duration_ms=100, int waitafter_ms = 0)
{
  digitalWrite(BEEP_PIN,1);
  delay(duration_ms);
  digitalWrite(BEEP_PIN,0);      
  delay(waitafter_ms);
}

////////////////////////////////////////////////////////////////////////////////
// state machine

enum GO { NOTHING, PUMP1_ON, PUMP2_ON, PUMPS_OFF, CHECK } forceGo = GO::NOTHING;

bool highWaterLevel()
{
  bool ret = false;
  auto sensorValue = readSensor();
  
  // check for hard override (sensor manually shorted) to force pumping
  if( sensorValue < 10 )
  {
    digitalWrite(LED_PIN,1);
    beep(1000);
    ret = true;
  }  
  if( sensorValue < config.data().pumpTreshold )
  {
    ret = true;
  }
  return ret;    
}

////////////////////////////////////////////////////////////////////////////////////////

class Timer
{
public:
  void reset() { mCount = 0; }
  void tick() { mCount++; }
  unsigned count() { return mCount; }
private:
  unsigned mCount{0};
};

////////////////////////////////////////////////////////////////////////////////

class ChargingStateMachine
{
public:
  
  enum class State { NORMAL, RECOVERY, FLOAT, REST, ERROR };
  
  State currentState() { return mState; }
  
  const char* currentStateStr() { return state2str(mState); }
  
  unsigned timerCount() { return mTimer.count(); }
  
  bool powerGood() { return mState != State::RECOVERY; }

  void update()
  {
    State nextState = computeNextState();
    if(nextState != currentState())
    {
      // log the change
      Serial.print("Charge ");
      Serial.print(state2str(mState));
      Serial.print("==>");
      Serial.print(state2str(nextState));
      Serial.println();
      
      moveTo(nextState);
      
      // any state change resets the timer to zero
      forceGo = GO::NOTHING;
    }
    else
    {
      // no change - keep ticking
      mTimer.tick();
    }

    // adjust outputs (in Mealy state machine output may have logic that depends on input so must call it always)
    outputs();        
  }
  
private:

  State computeNextState()
  {
    auto nextState = mState;
    auto batteryMV = readBatteryMV();
    if(batteryMV<100)
      nextState = State::ERROR; // prevent overcharge in case battery sensor fails
    else
      switch(mState)
      {      
        case State::NORMAL: // charging on all the time until batMax is reached; normal operating mode (DISCHARGE cycling)        
          if(batteryMV >= config.data().batMaxMV) nextState = State::REST;
          break;
        case State::RECOVERY: // full discharge - charge until batNom is reached but do all allow discharge
          if(batteryMV >= config.data().batNomMV) nextState = State::NORMAL;
          break;      
        case State::FLOAT: // float charging - up to float Volts can switch to is from REST (when no strong dischargin was happening)
          if(batteryMV < config.data().batNomMV) nextState = State::NORMAL; // apparently we are discharging - switch to NORMAL
          else if(batteryMV > config.data().batMaxMV) nextState = State::REST; // out of float when discharge is happening
          break;      
        case State::REST: // no charging, resting
          if(batteryMV < config.data().batNomMV) nextState = State::NORMAL; // apparently we are discharging - switch to NORMAL
          else if(batteryMV < config.data().batFloMV) nextState = State::FLOAT; // back to float charging as we dropped some small V
          break;      
      }
    return nextState;
  }

  void moveTo(State state)
  {
    mState = state;
    // start measuring time in new state from zero
    mTimer.reset();
  }

  void outputs()
  {
    auto charge = false;
    switch(mState)
    {
      case State::NORMAL:
      case State::FLOAT:
      case State::RECOVERY:
        charge = true;
        break;
      default:
        charge = false;
    }
    digitalWrite( CHARGE_PIN, charge ? 0 : 1); // this relay is active low
  }
  
  static const char* state2str(State state)
  {
    // voltage: 
    // <= batMinMV --> RECOVERY
    // >= batMaxMV --> FLOAT charge state turn charge only when < batChargeMV
    // 
    switch(state)
    {
    case State::NORMAL: return "NORMAL"; // battery in normal operating state (charge on) 
    case State::RECOVERY: return "RECOVERY"; // recovering from deep discharge (leave me alone until reaching NOMinal voltage)
    case State::REST: return "REST"; // recovering from deep discharge (leave me alone until reaching NOMinal voltage)
    case State::FLOAT: return "FLOAT"; // battery in float charge state (charging only to 13.8V)
    case State::ERROR: return "ERROR"; // charging in error (cannot read battery voltage)
    }
    return "????";
  }
  
  State mState{State::NORMAL};
  Timer mTimer;
    
} chargingStateMachine;


////////////////////////////////////////////////////////////////////////////////////////

class PumpStateMachine
{
public:  
  enum class State { CHECK, PUMP1, PUMP2, REST };
  
  State currentState() { return mState; }
  
  const char* currentStateStr() { return state2str(mState); }
  
  unsigned timerCount() { return mTimer.count(); }

  void update()
  {
    State nextState = computeNextState();
    if(nextState != currentState())
    {
      // short beep when changing state
      beep(20);
      
      // log the change
      Serial.print("Pump ");
      Serial.print(state2str(mState));
      Serial.print("==>");
      Serial.print(state2str(nextState));
      Serial.println();
      
      moveTo(nextState);
            
      // any state change resets the timer to zero
      forceGo = GO::NOTHING;
    }
    else
    {
      // no change - keep ticking
      mTimer.tick();
    }

    // adjust outputs (in Mealy state machine output may have logic that depends on input so must call it always)
    outputs();    
  }

private:

  // compute next state given forceGo, timer and sensors
  State computeNextState()
  {
    State nextState = mState;
    // first apply external command
    switch(forceGo)
    {
      case GO::PUMP1_ON: nextState = State::PUMP1; break;
      case GO::PUMP2_ON: nextState = State::PUMP2; break;
      case GO::PUMPS_OFF: nextState = State::REST; break;
      case GO::CHECK: nextState = State::CHECK; break;
    }
    // if no command, do normal state change
    if(forceGo == GO::NOTHING)
    {
      if( !chargingStateMachine.powerGood() )
      {
        nextState = State::REST;
      } else switch(mState)
      {
      case State::CHECK:
        if( highWaterLevel() ) 
          nextState = config.data().pump1OnTime > 0 ? State::PUMP1 : State::PUMP2;
        break;
      case State::PUMP1:
        if(mTimer.count() > config.data().pump1OnTime)
          nextState = config.data().pump2OnTime > 0 ? State::PUMP2 : State::REST;
        break;
      case State::PUMP2:
        if(mTimer.count() > config.data().pump2OnTime) 
          nextState = State::REST;
        break;
      case State::REST:
        // if idle battery voltage dropped below battChargeMV - remain in REST
        // otherwise after 'resting' go back to checking
        if( chargingStateMachine.powerGood() &&
            mTimer.count() > config.data().pumpRestTime)
          nextState = State::CHECK;
        break;
      }
    }
    return nextState;
  }

  void outputs()
  {
    // adjust pump pins - must be executed always to turn off pumps
    digitalWrite(PUMP1_PIN, mState == State::PUMP1 ? 0 : 1);
    digitalWrite(PUMP2_PIN, mState == State::PUMP2 ? 0 : 1);    
  }

  void moveTo(State state)
  {
      mState = state;
      // start measuring time in new state from zero
      mTimer.reset();
  }

  static const char* state2str(State state)
  {
    switch(state)
    {
    case State::CHECK: return "CHECK";
    case State::PUMP1: return "PUMP1";
    case State::PUMP2: return "PUMP2";
    case State::REST: return "REST";
    }
    return "????";
  }

  State mState{State::CHECK};
  Timer mTimer;

} pumpStateMachine;

////////////////////////////////////////////////////////////////////////////////

MYDHT::Result temphum{};

void outputState()
{
  temphum = readTempHum();
  Serial.print("{");
  Serial.print("Dsen:");Serial.print(readSensor());
  Serial.print(", Vbat:");Serial.print(readBatteryMV());
  Serial.print(", Vcc:");Serial.print(readVccMV());
  Serial.print(", Temp:");Serial.print(temphum.temp);
  Serial.print(", RH:");Serial.print(temphum.rh);
  Serial.print(", PState:");Serial.print(pumpStateMachine.currentStateStr());
  Serial.print("/");Serial.print(pumpStateMachine.timerCount());
  Serial.print(", CState:");Serial.print(chargingStateMachine.currentStateStr());
  Serial.print("/");Serial.print(chargingStateMachine.timerCount());
  Serial.print(", TS:");Serial.print(millis()/1000);
  Serial.print("}");
  Serial.println();  
}

////////////////////////////////////////////////////////////////////////////////
// serial console input

void help()
{
  Serial.print("build: ");Serial.println(build_timestamp);
  Serial.println("z default the configuration and store");
  Serial.println("b<battery mV as measured> calibration");
  Serial.println("m<battery min mV>");  
  Serial.println("n<battery nom mV>");
  Serial.println("k<battery max mV>");
  Serial.println("f<battery flo mV>");
  Serial.println("t<pump treshhold DAC counts>");
  Serial.println("p<pump1 on time seconds>");
  Serial.println("o<pump2 on time seconds>");
  Serial.println("r<pump rest time seconds >");
  Serial.println("u<state update/dump period seconds>");
  Serial.println("x0 go pumps off");
  Serial.println("x1 go pump on");
  Serial.println("x2 go pump off");
  Serial.println("x2 go pump off");
  Serial.println("x9 restart from CHECK");
  Serial.println("d<SPP> digital write PP=pin number, S=state: 1-write 1; 2-write 0, 3-read, 4-switch into input");
  Serial.println("a<PP> analog read");
}

// commands are in form <CmdChar><number(digits)><CR>

bool configCommand(char cmd, int par)
{
  if( par < 0 ) return false;
  
  bool storeConfig = false;
  switch(cmd)
  {
    case 'z':
      config.reset();
      storeConfig = true;
      break;
    case 'b':
      config.data().batMV = 1;
      config.data().batDAC = 1;
      config.data().batDAC = readBatteryMV();
      config.data().batMV = par;
      storeConfig = true;
      break;
    case 'm':
      config.data().batMinMV = par;
      storeConfig = true;
      break;
    case 'n':
      config.data().batNomMV = par;
      storeConfig = true;
      break;        
    case 'k':
      config.data().batMaxMV = par;
      storeConfig = true;
      break;
    case 'f':
      config.data().batFloMV = par;
      storeConfig = true;
      break;        
    case 't':
      config.data().pumpTreshold = par;
      storeConfig = true;
      break;
    case 'p':
      config.data().pump1OnTime = par;
      storeConfig = true;
      break;
    case 'o':
      config.data().pump2OnTime = par;
      storeConfig = true;
      break;
    case 'r':
      config.data().pumpRestTime = par;
      storeConfig = true;
      break;
    case 'u':
      config.data().stateUpdateTime = par;
      storeConfig = true;
      break;
  }
  return storeConfig;  
}

void pinOperation(int pn, int state)
{
  switch(state)
  {
    case 1: digitalWrite( pn, 1 ); break;
    case 2: digitalWrite( pn, 0 ); break;
    case 3: Serial.println(digitalRead(pn));break;
    case 4: pinMode( pn, INPUT ); Serial.println("switched into input"); break;
  }
}

bool goOperation(int par)
{
  Serial.print("x==> "); Serial.println(par);
  switch(par)
  {
    case 0: forceGo = GO::PUMPS_OFF; return true; // x0
    case 1: forceGo = GO::PUMP1_ON; return true; // x1
    case 2: forceGo = GO::PUMP2_ON; return true; // x2
    case 9: forceGo = GO::CHECK; return true; // x9
  }
  return false;
}

bool normalCommand(char cmd, int par)
{
  bool res = true;
  switch(cmd)
  {
  case 'a':
    Serial.println(analogRead(par));
    break;
  case 'd':
    pinOperation(par % 100, par / 100);
    break;
  case 'x': // go
    res = goOperation(par);
    break;
  case 0:
    outputState();
    break;
  case '?':
    config.dump();
    Serial.println();
    help();
    break;
  default:
    res = false;
    break;
  }
  return res;
}

bool executeCmd(char cmd, int par)
{
  bool res = false;
  Serial.print("cmd ");Serial.print(cmd);Serial.print(' ');Serial.println(par);
  if( configCommand(cmd, par) ) 
  {
    config.dump();
    config.store();
    res = true;
  }
  else // if(!storeConfig) // there was not correct config update cmd
  {
    res = normalCommand(cmd, par);
  }
  if(!res)
  {
    Serial.println("Invalid command");
    help();
  }
  return res;
}


bool checkSerial()
{
  // serial input state 
  static struct {
    char cmd{};
    bool inPar{};
    int par{};

    void reset() {
      cmd = 0;
      inPar = false;
      par = -1;
    }
  } state{};

  bool ret = false;

  if( Serial.available() )
  {
    ret = true;
    char ch = Serial.read();
    if( ch == 13 ) // command terminated -> execute
    {
      executeCmd(state.cmd, state.par);
      state.reset();
    }
    else if( ch <= ' ' )
    {
      // ignore
    }
    // have a command and got a digit?
    else if( state.cmd > 0 && isDigit(ch) )
    {
      if( state.par < 0 ) state.par = 0;
      state.inPar = true;
      state.par *= 10;
      state.par += ch - '0';
    }
    // collecting paramteter or got a digit at start?
    else if( state.inPar || state.cmd == 0 && isDigit(ch) ) // got something else? scrap the whole thing
    {
      Serial.print("unexpected input: ");Serial.println(ch);
      state.reset();
    }
    // use ch as command
    else if( state.cmd == 0 )
    {
      state.cmd = ch;
    }
    else // got something else in command?
    {
      Serial.print("unexpected input: ");Serial.println(ch);
      state.reset();
    }
  }
  return ret;
}

////////////////////////////////////////////////////////////////////////////////

void setup()
{  
  // after turnon the ESP may spew its output for a while to the serial input so delay
  delay(1000);  
  // begin the serial communication now
  Serial.begin(9600);
  
  Serial.println();
  Serial.print("Pump controller (2 pump version) ");
  Serial.print("build: ");
  Serial.print(build_timestamp);

  // configure pins
  digitalWrite(BEEP_PIN, 0);
  pinMode(BEEP_PIN, OUTPUT);

  // 5 x beep to signal starting
  for(auto n = 0; n < 5; ++n) {
    beep(50);
    delay(1000);
    Serial.print('.');
  }
  // throw away the serial input if any for some time
  while(Serial.available()) { Serial.read(); delay(1); }
  
  Serial.println();  

  // load or default configuration
  if( !config.load() )
  {
    Serial.println("EEPROM configuation invalid, defaulting.");
    config.reset();    
    config.store();
  }
  else
  {
    Serial.println("EEPROM configuation restored.");
  }
  config.dump();

  digitalWrite(PUMP1_PIN, 1);
  pinMode(PUMP1_PIN, OUTPUT);
  digitalWrite(PUMP2_PIN, 1);
  pinMode(PUMP2_PIN, OUTPUT);
  digitalWrite(CHARGE_PIN, 1);
  pinMode(CHARGE_PIN, OUTPUT);
  
  beep(200);
  Serial.println("Ready!");
}

////////////////////////////////////////////////////////////////////////////////

int stateUpdateTime = 0;

void loop()
{  
  bool hadInput = checkSerial();
  if(!hadInput)
  {
    sleep1s();
  }

  chargingStateMachine.update();
  pumpStateMachine.update();
  
  if( --stateUpdateTime <= 0 )
  {
    outputState();
    stateUpdateTime = config.data().stateUpdateTime;
  }
}
