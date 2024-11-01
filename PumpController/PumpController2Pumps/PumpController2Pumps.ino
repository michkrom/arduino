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
#include "libraries/DHT/dht11pp.h"

const char build_timestamp[] = __DATE__ " " __TIME__;

////////////////////////////////////////////////////////////////////////////////
// Configuration/EEPROM

typedef struct ConfigData
{
  uint16_t checkSum{0};  // simple check sum of the configuration data
  uint16_t batMV{12860}; // measured 12.86V--> 258 DAC COUNTS (this may be temp dependent!)
  uint16_t batDAC{258}; // corresponding DAC for measured mV
  uint16_t batMinMV{8000}; // min mV (critical baterry level - stop anything else and recover)
  uint16_t batMaxMV{14500}; // max mV (target in recovery, and max absolute when charging is unconditionally off)
  uint16_t batNomMV{12000}; // nominal mV (start charging below end at float unless recovering)
  uint16_t batFloMV{13600}; // float mV (the normal up-to voltage while charging)
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
    Serial.print("batMinMV    : ");Serial.println(mConfig.batMinMV);
    Serial.print("batNomMV    : ");Serial.println(mConfig.batNomMV);
    Serial.print("batFloMV    : ");Serial.println(mConfig.batFloMV);
    Serial.print("batMaxMV    : ");Serial.println(mConfig.batMaxMV);
    Serial.print("pumpTreshold: ");Serial.println(mConfig.pumpTreshold);
    Serial.print("pump1OnTime : ");Serial.println(mConfig.pump1OnTime);
    Serial.print("pump2OnTime : ");Serial.println(mConfig.pump2OnTime);
    Serial.print("pumpRestTime: ");Serial.println(mConfig.pumpRestTime);
    Serial.print("sUpdateTime : ");Serial.println(mConfig.stateUpdateTime);
  }

  ConfigData& data() { return mConfig; }

private:
  ConfigData mConfig;

} config;


////////////////////////////////////////////////////////////////////////////////
// return Vcc in [mV] measured using internal reference 1.1V

uint16_t readVccMV()
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
  return (uint16_t)result; 
}


uint16_t analogReadAvg(const uint8_t pin, const uint8_t avgs)
{  
  uint32_t sum = 0;
  for(uint8_t i=0; i < avgs; ++i)
    sum += analogRead(pin);
  sum /= avgs;
  return uint16_t(sum);
}

////////////////////////////////////////////////////////////////////////////////
// battery [mV] (via resistor divider) on A1

uint16_t readBatteryMV()
{
  uint32_t read = analogReadAvg(A1, 64);
  read = (read*config.data().batMV)/config.data().batDAC;
  return read;
}

////////////////////////////////////////////////////////////////////////////////
// water sensor readout [dac counts] on A0

uint16_t readSensor()
{
  uint32_t read = analogReadAvg(A0, 64);
  return uint16_t(read);
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

void sleep1sblink()
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
  // the lower the reading the less resistance there is inferring more water is over the sensor
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
  
  enum class State { RECOVERY, CHARGE, FLOAT, ERROR };
  
  State currentState() const { return mState; }
  
  const char* currentStateStr() const { return state2str(mState); }
  
  unsigned timerCount() const { return mTimer.count(); }
  
  bool powerGood() const { return mState == State::FLOAT || mState == State::CHARGE; }

  bool update()
  {
    bool changed = false;
    auto batteryMV = readBatteryMV();
    State nextState = computeNextState(batteryMV);
    if(nextState != currentState())
    {
      changed = true;
      // log the change
      Serial.print("Charge ");
      Serial.print(state2str(mState));
      Serial.print("==");
      Serial.print(batteryMV);
      Serial.print("==>");
      Serial.print(state2str(nextState));
      Serial.println();      
      moveTo(nextState);
      mTimer.reset();
    }
    else
    {
      // no change - keep ticking
      mTimer.tick();
    }
    // adjust outputs (in Mealy state machine output may have logic that depends on input so must call it always)
    outputs();
    return changed;
  }
  
private:

  void moveTo(State state) { mState = state; }


  // RECOVERY -> charging up to max, use disabled
  // CHARGE -> charging up to float, use OK
  // REST   -> not charging, use ok, got CHARGE when below nom
  State computeNextState(uint16_t batteryMV) const
  {
    auto nextState = mState;
    if(batteryMV<100)
      nextState = State::ERROR; // prevent overcharge in case battery sensor fails
    else if(batteryMV >= config.data().batMaxMV)
      nextState = State::FLOAT; // reached max now FLOAT
    else if(batteryMV < config.data().batMinMV)
      nextState = State::RECOVERY; // recovery forcess reaching full charge and no work (recovery from max discharge)
    else 
    {
      switch(mState)
      {
        case State::CHARGE: // float charging - up to float Volts can switch to is from REST (when no strong dischargin was happening)
          if(batteryMV > config.data().batFloMV) nextState = State::FLOAT;
          break;
        case State::FLOAT:
          if(batteryMV < config.data().batNomMV) nextState = State::CHARGE;
          break;          
      }
    }
    return nextState;
  }

  void outputs() const
  {
    auto charge = false;
    switch(mState)
    {
      case State::CHARGE:
      case State::RECOVERY:
        charge = true;
        break;
      default:
        charge = false;
    }
    digitalWrite(CHARGE_PIN, charge ? 0 : 1); // this relay is active low
  }
  
  static const char* state2str(State state)
  {
    // voltage: 
    // <= batMinMV --> RECOVERY
    // >= batMaxMV --> FLOAT charge state turn charge only when < batChargeMV
    // 
    switch(state)
    {
    case State::CHARGE: return "CHA";   // battery in normal operating state (charge on, work ok) 
    case State::RECOVERY: return "REC"; // recovering from deep discharge (leave me alone until reaching NOMinal voltage)
    case State::FLOAT: return "FLO";     // not changing, all good for work
    case State::ERROR: return "ERROR";  // charging in error (cannot read battery voltage)
    }
    return "????";
  }
  
  State mState{State::CHARGE};
  Timer mTimer;
    
} chargingStateMachine;


////////////////////////////////////////////////////////////////////////////////////////

class PumpStateMachine
{
public:  
  enum class State { CHECK, PUMP1, XPUMP1, PUMP2, REST }; // XPUMP1 transitions to REST while PUMP1 (notmal) transitions to PUMP2
  
  State currentState() const { return mState; }
  
  const char* currentStateStr() const { return state2str(mState); }
  
  unsigned timerCount() const { return mTimer.count(); }

  bool update()
  {
    bool changed = false;

    State nextState = computeNextState();
    if(nextState != currentState())
    {
      changed = true;
      // short beep when changing state
      beep(20);
      
      // log the change
      Serial.print("Pump ");
      Serial.print(state2str(mState));
      Serial.print("==>");
      Serial.print(state2str(nextState));
      Serial.println();      
      moveTo(nextState);      
      // start measuring time in new state from zero
      mTimer.reset();

    }
    else
    {
      // no change - keep ticking
      mTimer.tick();
    }

    // adjust outputs (in Mealy state machine output may have logic that depends on input so must call it always)
    outputs();
    forceGo = GO::NOTHING;

    return changed;
  }

private:

  // compute next state given forceGo, timer and sensors
  State computeNextState() const
  {
    if( !chargingStateMachine.powerGood() )
    {
      return State::REST;
    }

    State nextState = mState;
    // first apply external command
    switch(forceGo)
    {
      case GO::PUMP1_ON: nextState = State::XPUMP1; break;
      case GO::PUMP2_ON: nextState = State::PUMP2;  break;
      case GO::PUMPS_OFF: nextState = State::REST; break;
      case GO::CHECK: nextState = State::CHECK; break;
      default: // GO::NOTHING
        switch(mState)
        {
        case State::CHECK:
          if( highWaterLevel() ) 
            nextState = State::PUMP1;
          break;
        case State::XPUMP1:
        case State::PUMP1:
          if(mTimer.count() > config.data().pump1OnTime)
          {
            nextState = mState == State::XPUMP1 ? State::REST : State::PUMP2;
          }
          break;
        case State::PUMP2:
          if(mTimer.count() > config.data().pump2OnTime) 
            nextState = State::REST;
          break;
        case State::REST:
          // rest until timeout & battery is good again
          if( chargingStateMachine.powerGood() &&
              mTimer.count() > config.data().pumpRestTime)
            nextState = State::CHECK;
          break;
        }
    }
    return nextState;
  }

  void outputs() const
  {
    // adjust pump pins - must be executed always to turn off pumps
    digitalWrite(PUMP1_PIN, mState == State::PUMP1 || mState == State::XPUMP1 ? 0 : 1);
    digitalWrite(PUMP2_PIN, mState == State::PUMP2 ? 0 : 1);    
  }

  void moveTo(State state) { mState = state; }

  static const char* state2str(State state)
  {
    switch(state)
    {
    case State::CHECK: return "CHECK";
    case State::XPUMP1: return "XPUMP1";
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

void printAllState()
{
  temphum = readTempHum();
  Serial.print("{");
  Serial.print("Dsen:");Serial.print(readSensor());
  Serial.print(", Vbat:");Serial.print(readBatteryMV());
  Serial.print(", ");Serial.print(pumpStateMachine.currentStateStr());
  Serial.print("/");Serial.print(pumpStateMachine.timerCount());
  Serial.print(", ");Serial.print(chargingStateMachine.currentStateStr());
  Serial.print("/");Serial.print(chargingStateMachine.timerCount());
  Serial.print(", Vcc:");Serial.print(readVccMV());
  Serial.print(", TP:");Serial.print(temphum.temp);
  Serial.print(", RH:");Serial.print(temphum.rh);
  Serial.print(", TS:");Serial.print(millis()/1000);
  Serial.print("}");
  Serial.println();  
}

////////////////////////////////////////////////////////////////////////////////
// serial console input

void help()
{
  Serial.print("build: ");Serial.println(build_timestamp);
  Serial.println("command format: <char><number><CR>");
  Serial.println("z default the configuration");
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
  Serial.println("x0 pumps off, x1 pump1, x2 pump2, x9 restart CHECK");
  Serial.println("d<SPP> digital write PP=pin number, S=state: 1-wr 1; 2-wr 0, 3-rd, 4-make input");
  Serial.println("a<PP> analog read");
}

// commands are in form <CmdChar><number(digits)><CR>

bool configCommand(char cmd, uint16_t par)
{
  if( par < 0 ) return false;
  
  bool storeConfig = true;
  switch(cmd)
  {
    case 'z':
      config.reset();
      break;
    case 'b':
      config.data().batMV = 1;
      config.data().batDAC = 1;
      config.data().batDAC = readBatteryMV();
      config.data().batMV = par;
      break;
    case 'm':
      config.data().batMinMV = par;
      break;
    case 'n':
      config.data().batNomMV = par;
      break;        
    case 'k':
      config.data().batMaxMV = par;
      break;
    case 'f':
      config.data().batFloMV = par;
      break;        
    case 't':
      config.data().pumpTreshold = par;
      break;
    case 'p':
      config.data().pump1OnTime = par;
      break;
    case 'o':
      config.data().pump2OnTime = par;
      break;
    case 'r':
      config.data().pumpRestTime = par;
      break;
    case 'u':
      config.data().stateUpdateTime = par;
      break;
    default:
      storeConfig = false;
  }
  if(storeConfig) 
  {
    config.store();
    config.dump();
  }
  return storeConfig;
}

void pinOperation(int pn, int state)
{
  switch(state)
  {
    case 1: digitalWrite( pn, 1 ); break;
    case 2: digitalWrite( pn, 0 ); break;
    case 3: Serial.println(digitalRead(pn)); break;
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
  case 'x':
    res = goOperation(par);
    break;
  case 's':
    printAllState();
    break;
  case 'c':
    config.dump();
    break;
  case '?':
    help();
    break;

  default:
    res = false;
    break;
  }
  return res;
}

static bool inputLocked = true;


void executeCmd(char cmd, int par)
{
  Serial.print("cmd ");Serial.print(cmd);Serial.print(' ');Serial.println(par);
  if(inputLocked)
  {
    if(cmd == '!' && par == 4242)
    {
      inputLocked = false;
      Serial.println("input unlocked");
    }
    else
    {
      Serial.println("input locked, !4242 to unlock");
    }
    return;
  }
  if( configCommand(cmd, par) ) 
  {
  }
  else if( normalCommand(cmd, par) )
  {
  }
  else
  {
    Serial.println("command ignored, ? for help");
  }
}

// commands are simply: <char><number><CR>
// the unlock command is !4242

bool checkSerial()
{
  // serial input state 
  static struct {
    char cmd{};
    bool inPar{};
    int par{};
    unsigned long startTime;

    bool hasCmd() const { return cmd > 0; }

    bool addChar(char c)
    {
      if(!hasCmd())
      {
        return start(c);
      }
      else // hasCmd()
      {
        return addPar(c); 
      }
      return false;
    }

    bool expired() const
    {
        return hasCmd() && millis() - startTime > 1000; // 1s for collection of all chars
    }

    void reset()
    {
      cmd = 0;
      inPar = false;
      par = 0;
      startTime = 0;
    }

    private:

    bool start(char c) 
    {
      if( c < 'a' && c > 'z' ) return false;
      startTime = millis();
      cmd = c;
      par = -1;
      return true;
    }

    bool addPar(char c)
    {
      if( !isDigit(c) ) return false;
      if( par < 0 ) par = 0;
      par *= 10;
      par += c - '0';
      return true;
    }

  } state {};

  if( !Serial.available() ) return false;

  if( state.expired() )
  {
    Serial.println("Resetting input");
    state.reset();
  }

  const char TERMINATOR = 13;

  // process commands from serial stream, if any
  while( Serial.available() )
  {
    char ch = Serial.read();
    if( ch == TERMINATOR ) 
    {
      if( state.hasCmd() ) // command terminated -> execute
      {
        executeCmd(state.cmd, state.par);
        state.reset();
      }
      return false; // command terminated, let the state machines run
    }
    else if(!state.addChar(ch))
    {
      Serial.println("Ignoring input");
      state.reset();
      while( Serial.available() && Serial.read() != TERMINATOR ) delay(10);
      return false; // input ignored, continue normal activity
    }
    // wait a bit, maybe more is comming?
    delay(10);
  }
  // there were input chars processed if we were here, but not yet complete
  return true;
}

////////////////////////////////////////////////////////////////////////////////

void setup()
{ 
  // configure pins
  digitalWrite(BEEP_PIN, 0);
  pinMode(BEEP_PIN, OUTPUT);
  // we are here! 
  beep(20);

  // begin the serial communication now
  Serial.begin(9600);

  // throw away the serial input if any for some time (serial monitor boot output)
  int count = 100;
  while(count-- > 0 && Serial.available())
  {
    while(Serial.available()) { Serial.read(); delay(10); }
    beep(5);
  }

  Serial.println();
  Serial.print("Pump controller (2 pump version) ");
  Serial.print("build: ");
  Serial.print(build_timestamp);

  // 5 x beep to signal starting
  for(auto n = 0; n < 5; ++n) {
    beep(50);
    delay(1000);
    Serial.print('.');
  }
  
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

void loop()
{  
  static int stateUpdateTime = 0;
  bool hadInput = checkSerial();
  if(!hadInput)
  {
    // no commands are comming over serial, sleep 1s to slow down state updates
    sleep1sblink();
  }

  bool change = false;
  change |= chargingStateMachine.update();
  change |= pumpStateMachine.update();
  
  if( --stateUpdateTime <= 0 || change )
  {
    printAllState();
    stateUpdateTime = config.data().stateUpdateTime;
  }
}
