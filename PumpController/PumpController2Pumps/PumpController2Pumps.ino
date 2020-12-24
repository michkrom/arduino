#include <EEPROM.h>
#include <dht11pp.h>

// water pump controller, shore powered and 2 pumps
// also suports temp/hum via DHT device

// A0 is water sensor (two electrodes between ground and A0 with A0 pulled up to VCC via 200KOhm)
// A1 is battery voltage via a ~1:10 divider

// D 4/5/6/7 are relays (zero-active): D4 is pump 1 (active low); D5 is pump 2
// D8 is solar panel via MOSFET solid state relay (active high) *** unused

// D12 is DHT device

#define LED_PIN 13
#define PUMP1_PIN 4
#define PUMP2_PIN 5
#define CHARGE_PIN 6

#define BEEP_PIN 11
#define DHT_PIN 12


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
  uint16_t pumpTreshold{450}; // water sense treshold (lower reading means water is present)
  uint16_t pump1OnTime{30}; // seconds to turn the pump on
  uint16_t pump2OnTime{30}; // seconds to turn the pump on
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
    Serial.print("batMV: ");Serial.println(mConfig.batMV);
    Serial.print("batDAC: ");Serial.println(mConfig.batDAC);
    Serial.print("batMaxMV: ");Serial.println(mConfig.batMaxMV);
    Serial.print("batMinMV: ");Serial.println(mConfig.batMinMV);
    Serial.print("batNomMV: ");Serial.println(mConfig.batNomMV);
    Serial.print("batFloMV: ");Serial.println(mConfig.batFloMV);
    Serial.print("pumpTreshold: ");Serial.println(mConfig.pumpTreshold);
    Serial.print("pump1OnTime: ");Serial.println(mConfig.pump1OnTime);
    Serial.print("pump2OnTime: ");Serial.println(mConfig.pump2OnTime);
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
  int count() { return mCount; }
private:
  int  mCount{0};
};

////////////////////////////////////////////////////////////////////////////////

class ChargingStateMachine
{
public:
  
  enum class State { NORMAL, RECOVERY, FLOAT, REST };
  
  State currentState() { return mState; }
  
  const char* currentStateStr() { return state2str(mState); }
  
  int timerCount() { return mTimer.count(); }
  
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
    switch(mState)
    {      
      case State::NORMAL: // charging on all the time until batMax is reached; normal operating mode (DISCHARGE cycling)
        if(batteryMV >= config.data().batMaxMV) nextState = State::REST;
        break;
      case State::RECOVERY: // full discharge - charge until batNom is reached but do all allow discharge
        if(batteryMV >= config.data().batNomMV) nextState = State::NORMAL;
        break;      
      case State::FLOAT:
        if(batteryMV < config.data().batNomMV) nextState = State::NORMAL; // apparently we are discharging - switch to NORMAL
        else if(batteryMV > config.data().batFloMV) nextState = State::REST; // out of float when discharge is happening
        break;      
      case State::REST:
        if(batteryMV < config.data().batNomMV) nextState = State::NORMAL; // apparently we are discharging - switch to NORMAL
        if(batteryMV < config.data().batFloMV - 500/*mV*/) nextState = State::FLOAT; // back to float charging as we dropped 200mV
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
    // outputs
    digitalWrite( CHARGE_PIN, mState != State::REST ? 0 : 1); // active low
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
  
  int timerCount() { return mTimer.count(); }

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

int stateUpdateTime = 0;

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
  Serial.println("t default the configuration and store");
  Serial.println("b<battery mV as measured> calibration");
  Serial.println("i<battery min mV>");  
  Serial.println("a<battery max mV>");
  Serial.println("f<battery flo mV>");
  Serial.println("n<battery nom mV>");
  Serial.println("p<pump treshhold DAC counts>");
  Serial.println("o<pump1 on time seconds>");
  Serial.println("l<pump2 on time seconds>");
  Serial.println("r<pump rest time seconds >");
  Serial.println("u<state update/dump period seconds>");
  Serial.println("x0 go pumps off");
  Serial.println("x1 go pump on");
  Serial.println("x2 go pump off");
  Serial.println("x2 go pump off");
  Serial.println("x9 restart from CHECK");
}

// commands are in form <CmdChar><number(digits)><CR>

bool executeCmd(char cmd, int par)
{
  bool res = false;
  Serial.print("cmd ");Serial.print(cmd);Serial.print(' ');Serial.println(par);
  bool storeConfig = false;
  switch(cmd)
  {
    case 't':
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
    case 'i':
      config.data().batMinMV = par;
      storeConfig = true;
      break;
    case 'a':
      config.data().batMaxMV = par;
      storeConfig = true;
      break;
    case 'n':
      config.data().batNomMV = par;
      storeConfig = true;
      break;        
    case 'f':
      config.data().batFloMV = par;
      storeConfig = true;
      break;        
    case 'p':
      config.data().pumpTreshold = par;
      storeConfig = true;
      break;
    case 'o':
      config.data().pump1OnTime = par;
      storeConfig = true;
      break;
    case 'l':
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
  if( storeConfig ) 
  {
    config.dump();
    config.store();
    res = true;
  }
  else // if(!storeConfig) // there was not correct config update cmd
  {
    switch(cmd)
    {
    case 'x': // go
      switch(par)
      {
        case 0: forceGo = GO::PUMPS_OFF; break; // x0
        case 1: forceGo = GO::PUMP1_ON; break; // x1
        case 2: forceGo = GO::PUMP2_ON; break; // x2
        case 9: forceGo = GO::CHECK; break; // x9
        default:
              res = false;
      }
      break;
    case 0:
      outputState();
      break;
    case '?':
      config.dump();
      help();
      break;
    default:
      Serial.println("Invalid command");
      help();
      break;
    }
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
      par = 0;
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
  // configure pins
  digitalWrite(PUMP1_PIN, 1);
  pinMode(PUMP1_PIN, OUTPUT);
  digitalWrite(PUMP2_PIN, 1);
  pinMode(PUMP2_PIN, OUTPUT);
  digitalWrite(CHARGE_PIN, 1);
  pinMode(CHARGE_PIN, OUTPUT);
  digitalWrite(BEEP_PIN, 0);
  pinMode(BEEP_PIN, OUTPUT);

  // after turnon the ESP may spew its output for a while to the serial input so delay
  delay(1000);  
  // begin the serial communication now
  Serial.begin(9600);
  
  Serial.println();
  Serial.print("Pump controller (2 pump version) ");

  // 5 x beep to signal starting
  for(auto n = 0; n < 5; ++n) {
    beep(50);
    delay(1000);
    Serial.print('.');
  }
  // throw away the serial input if any
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
  
  beep(200);
  Serial.println("Ready!");
}

////////////////////////////////////////////////////////////////////////////////

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
