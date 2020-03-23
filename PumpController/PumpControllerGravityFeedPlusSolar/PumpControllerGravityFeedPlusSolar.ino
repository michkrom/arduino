#include <EEPROM.h>
#include <dht11pp.h>

// water pump controller with solar charging control 
// - the pump just floods syphoning system and then rests to let it drain with gravity
// - solar changing is monitored via voltage sense through resistive divider ~1:10

// A0 is water sensor (two electrodes between ground and A0 with A0 pulled up to VCC via 200KOhm)
// A1 is battery voltage via a ~1:10 divider

// D 4/5/6/7 are relays (zero-active): D4 is pump (active low)
// D8 is solar panel via MOSFET solid state relay (active high)

#define LED_PIN 13
#define PUMP_PIN 4
#define SOLAR_PIN 8 // positive solar charding signal (1==charge)
#define SOLARN_PIN 9 // inverted (negative) solar charging signal (0--charge)
#define BEEP_PIN 11
#define DHT_PIN 12

// return Vcc in [mV] measured using internal reference 1.1V

long readVcc()
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

using MYDHT = DHT<DHT_PIN>;

MYDHT::Result readTempHum()
{
  MYDHT dht;
  auto result = dht.read();
  return result;
}

////////////////////////////////////////////////////////////////////////////////
// Configuration data

typedef struct ConfigStruct
{
  uint16_t checkSum{0};  // simple check sum of the configuration data
  uint16_t batMV{12860}; // measured 12.86V--> 258 DAC COUNTS (this may be temp dependent!)
  uint16_t batDAC{258}; // corresponding DAC for measured mV
  uint16_t batMaxMV = 13500; // max mV (stops charging)
  uint16_t batMinMV = 8000; // min mV (stops pumping until chaged)
  uint16_t batChargeMV = 13000; // charge start treshold
  uint16_t pumpTreshold{650}; // water sense treshold (lower reading means water is present)
  uint16_t pumpOnTime{20}; // seconds to turn the pump on
  uint16_t pumpRestTime{60*60}; // time between pump can go on
  uint16_t samplingTime{10}; // sampling time for checking water level
  uint16_t stateUpdateTime{10}; // period to output state to Serial
};

ConfigStruct CONFIG;

uint16_t computeChecksum(struct ConfigStruct& config)
{
  uint16_t sum = 0;
  for(size_t i = sizeof(config.checkSum); i < sizeof(config); ++i)
  {
      sum += ((uint8_t*)&config)[i];
  }
  return sum;
}

void storeCONFIG()
{
  CONFIG.checkSum = computeChecksum(CONFIG);
  EEPROM.put(0, CONFIG);
  Serial.println("CONFIG stored into EEPROM");
}

bool loadCONFIG()
{
  ConfigStruct conf;
  EEPROM.get(0, conf);
  uint16_t cs = computeChecksum(conf);
  if( cs == conf.checkSum )
  {
    CONFIG = conf;    
    Serial.println("EEPROM configuation restored");
    return true;
  }
  else
  {
    Serial.println("EEPROM configuation invalid");
    return false;
  }
}

void outputCONFIG()
{
  Serial.print("batMV: ");Serial.println(CONFIG.batMV);
  Serial.print("batDAC: ");Serial.println(CONFIG.batDAC);
  Serial.print("batMaxMV: ");Serial.println(CONFIG.batMaxMV);
  Serial.print("batMinMV: ");Serial.println(CONFIG.batMinMV);
  Serial.print("batChargeMV: ");Serial.println(CONFIG.batChargeMV);
  Serial.print("pumpTreshold: ");Serial.println(CONFIG.pumpTreshold);
  Serial.print("pumpOnTime: ");Serial.println(CONFIG.pumpOnTime);
  Serial.print("pumpRestTime: ");Serial.println(CONFIG.pumpRestTime);
  Serial.print("samplingTime: ");Serial.println(CONFIG.samplingTime);
  Serial.print("stateUpdateTime: ");Serial.println(CONFIG.stateUpdateTime);
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
// battery [mV] (via resistor divider)

int readBattery()
{
  const int AVERAGECOUNT = 16;
  uint32_t sum = 0;
  for(int i=0; i < AVERAGECOUNT; ++i)
    sum += analogRead(A1);
  sum /= AVERAGECOUNT;
  auto batMV = (sum*CONFIG.batMV)/CONFIG.batDAC;
  return (int)batMV;
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
// state

int  batteryMV = 0;
bool charging = false;
bool powerok = true;
int  timer = 0;
int  sensorValue = 0;
bool pumping = false;
MYDHT::Result temphum{};
int stateUpdateTime = 0;

enum { GO_NOTHING, GO_PUMP_ON, GO_PUMP_OFF, GO_CHARGE_ON, GO_CHARGE_OFF } forceGo = GO_NOTHING;

void setTimer(int seconds)
{
  timer=seconds;
}

void outputState()
{
  Serial.print("{");
  Serial.print("Dsen:");Serial.print(sensorValue);
  Serial.print(", Vbat:");Serial.print(batteryMV);
  Serial.print(", Vcc:");Serial.print(readVcc());
  Serial.print(", Temp:");Serial.print(temphum.temp);
  Serial.print(", RH:");Serial.print(temphum.rh);
  if(!powerok || charging || pumping)
  {
    Serial.print(", State:\"");
    if(!powerok) Serial.print("LowPow");
    if(charging) Serial.print("Charging");
    if(pumping) Serial.print("Pumping");
    Serial.print("\"");
  }
  Serial.print(", T:");Serial.print(timer);
  Serial.print(", TS:");Serial.print(millis()/1000);
  Serial.print("}");
  Serial.println();  
}

////////////////////////////////////////////////////////////////////////////////
// serial console input

void help()
{
  Serial.println("b<battery mV as measured>");
  Serial.println("p<pump treshhold DAC counts>");
  Serial.println("o<pump on time seconds>");
  Serial.println("r<pump rest time seconds >");
  Serial.println("s<sampling period seconds>");
  Serial.println("c<charge start treshold mV>");
  Serial.println("e<charge end treshold mV>");
  Serial.println("d<discharge stop treshold mV>");
  Serial.println("u<state update period seconds>");
  Serial.println("x1 go pump on");
  Serial.println("x2 go pump off");
  Serial.println("x3 go charge on");
  Serial.println("x4 go charge off");
}

// commands are in form <CmdChar><number(digits)><CR>

bool executeCmd(char cmd, int par)
{
  bool res = false;
  Serial.print("cmd ");Serial.print(cmd);Serial.print(' ');Serial.println(par);
  bool storeConfig = false;
  switch(cmd)
  {
    case 'b':
      CONFIG.batMV = 1;
      CONFIG.batDAC = 1;
      CONFIG.batDAC = readBattery();
      CONFIG.batMV = par;
      storeConfig = true;
      break;
    case 'd':
      CONFIG.batMinMV = par;
      storeConfig = true;
      break;
    case 'e':
      CONFIG.batMaxMV = par;
      storeConfig = true;
      break;
    case 'c':
      CONFIG.batChargeMV = par;
      storeConfig = true;
      break;        
    case 'p':
      CONFIG.pumpTreshold = par;
      storeConfig = true;
      break;
    case 'o':
      CONFIG.pumpOnTime = par;
      storeConfig = true;
      break;
    case 'r':
      CONFIG.pumpRestTime = par;
      storeConfig = true;
      break;
    case 's':
      CONFIG.samplingTime = par;
      storeConfig = true;
      break;
    case 'u':
      CONFIG.stateUpdateTime = par;
      storeConfig = true;
      break;
  }
  if( storeConfig ) 
  {
    outputCONFIG();
    storeCONFIG();
    res = true;
  }
  else // if(!storeConfig) // there was not correct config update cmd
  {
    switch(cmd)
    {
    case 'x': // go
      switch(par)
      {
        case 1: forceGo = GO_PUMP_ON; break; // x1
        case 2: forceGo = GO_PUMP_OFF; break; // x2
        case 3: forceGo = GO_CHARGE_ON; break; // x3
        case 4: forceGo = GO_CHARGE_OFF; break; // x4
        default:
              res = false;
      }
      break;
    case 0:
      outputCONFIG();
      break;
    case '?':
      outputCONFIG();
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
    if( ch == 13 ) // command terminated - execute
    {
      executeCmd(state.cmd, state.par);
      state.reset();
    }
    else if( ch <= ' ' )
    {
      // ignore
    }
    else if( state.cmd > 0 && isDigit(ch) )
    {
      state.inPar = true;
      state.par *= 10;
      state.par += ch - '0';
    }
    else if( state.inPar || state.cmd == 0 && isDigit(ch) ) // got something else? scrap the whole thing
    {
      Serial.print("unexpected input: ");Serial.println(ch);
      state.reset();
    }
    else if( state.cmd == 0 )
    {
      state.cmd = ch;
    }
    else
    {
      Serial.print("unexpected state: ");Serial.println(ch);
      state.reset();
    }
  }
  return ret;
}

////////////////////////////////////////////////////////////////////////////////
// decide what to do about battery state
void checkBattery()
{
  // update battery state
  batteryMV = readBattery();
  
  if( batteryMV < CONFIG.batChargeMV )
  {
    charging = true;
  }
  else if( batteryMV > CONFIG.batMaxMV )
  {
    charging = false;
  }

  if(forceGo == GO_CHARGE_ON)
  {
    charging = true;
    forceGo = GO_NOTHING;
  }  
  if(forceGo == GO_CHARGE_OFF)
  {
    charging = false;
    forceGo = GO_NOTHING;
  }  
  
  if( batteryMV < CONFIG.batMinMV )
  {
    powerok  = false;
    pumping = false;
    if(timer==0)
    {
      beep(200,100);
      beep(200,100);
    }
  }
  else if( !powerok && batteryMV > CONFIG.batChargeMV )
  {
    powerok = true;
  }  
}

void setSolarCharging(bool on)
{
  digitalWrite(SOLAR_PIN, charging? 1 : 0); // active high
  digitalWrite(SOLARN_PIN, charging? 0 : 1); // relay driver is active low  
}

////////////////////////////////////////////////////////////////////////////////
// decide what to do about pumping
void checkPumping()
{  
  // update sensor state
  if(forceGo == GO_PUMP_ON)
  {
     sensorValue = 0;
     forceGo = GO_NOTHING;
  }
  else if(forceGo == GO_PUMP_OFF)
  {
     if(pumping)
     {
       timer = 0;
     }
  }
  else
  {
    sensorValue = readSensor();
  }
  
  // check for hard override (sensor shorted)
  if( sensorValue < 10 )
  {
    // sensor forcefully grounded -> reset the timer and that will turn the pump on below
    setTimer(0);
    digitalWrite(LED_PIN,1);
    beep(1000);
  }
  
  // if timer expired do: pump off, rest or check sensor
  if( timer <= 0 )
  {
    auto t = CONFIG.samplingTime;
    if( pumping )
    {
      Serial.println("PUMP OFF!");
      pumping = false;
      t = CONFIG.pumpRestTime;
      outputState();
    }
    else // not pumping so start checking sensor
    {
      if( sensorValue < CONFIG.pumpTreshold )
      {
        if( powerok )
        {
          Serial.println("PUMP ON!");
          pumping = true;
          t = CONFIG.pumpOnTime;
          outputState();
          beep(50,50);
          beep(50,50);
          beep(50,50);
        }
        else
        {
          Serial.println("POWER LOW, CANNOT PUMP!");
          beep(2000);
        }
      }
    }
    setTimer(t);
  }
  setSolarCharging(charging); // relay driver is active low
  digitalWrite(PUMP_PIN, pumping ? 0 : 1); // relay driver is active low  
}

////////////////////////////////////////////////////////////////////////////////

void setup()
{
  // after turnon the ESP may spew its output for a while
  delay(1000);
  // begin the serial communication
  Serial.begin(9600);
  digitalWrite(BEEP_PIN, 0);
  pinMode(BEEP_PIN, OUTPUT);
  for(auto n = 0; n < 5; ++n) {
    beep(50);
    delay(1000);
    Serial.print('.');
  }
  // dump the serial input if any
  while(Serial.available()) { Serial.read(); delay(1); }
  
  Serial.println();
  Serial.println("Pump controller.");
  if( !loadCONFIG() )
  {
    storeCONFIG();
  }
  outputCONFIG();
  digitalWrite(PUMP_PIN, 1);
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(SOLAR_PIN, 0);
  pinMode(SOLAR_PIN, OUTPUT);
  digitalWrite(SOLARN_PIN, 1);
  pinMode(SOLARN_PIN, OUTPUT);
  beep(200);
  Serial.println("Ready!");
}

////////////////////////////////////////////////////////////////////////////////

void loop()
{
  bool hadInput = checkSerial();
  checkBattery();
  checkPumping();
  temphum = readTempHum();
  if(!hadInput)
  {
    sleep1s();
    --timer;
  }
  if(timer < 0)
  {
    setTimer(CONFIG.samplingTime);
  }
  if( --stateUpdateTime <= 0 )
  {
    outputState();
    stateUpdateTime = CONFIG.stateUpdateTime;
  }
}
