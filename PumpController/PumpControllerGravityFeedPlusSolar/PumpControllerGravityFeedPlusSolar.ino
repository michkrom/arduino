#include <EEPROM.h>

// water pump controller with solar charging control 
// - the pump just floods syphoning system and then rests to let it drain with gravity
// - solar changing is monitored via voltage sense through resistive divider ~1:10

// A0 is water sensor (two electrodes between ground and A0 with A0 pulled up to VCC via 200KOhm)
// A1 is battery voltage via a ~1:10 divider

// D 4/5/6/7 are relays (zero-active): D4 is pump (active low)
// D8 is solar panel via MOSFET solid state relay (active high)

#define LED_PIN 13
#define PUMP_PIN 4
#define SOLAR_PIN 8
#define BEEP_PIN 11

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
int stateUpdateTime = 0;

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
  if(!powerok || charging || pumping)
  {
    Serial.print(", State:\"");
    if(!powerok) Serial.print("LowPow");
    if(charging) Serial.print("Charging");
    if(pumping) Serial.print("Pumping");
    Serial.print("\"");
  }
  Serial.print(", T:");Serial.print(timer);
  Serial.print("}");
  Serial.println();  
}

////////////////////////////////////////////////////////////////////////////////
// serial console input

void help()
{
  Serial.println("b <battery mV as measured>");
  Serial.println("p <pump treshhold DAC counts>");
  Serial.println("o <pump on time seconds>");
  Serial.println("r <pump rest time seconds >");
  Serial.println("s <sampling period seconds>");
  Serial.println("c <charge start treshold mV>");
  Serial.println("e <charge end treshold mV>");
  Serial.println("d <discharge stop treshold mV>");
  Serial.println("u <state update period seconds>");
}

bool checkSerial()
{
  static char cmd = 0;
  static int par = 0;

  if( Serial.available() )
  {
    char ch = Serial.read();
    if( isDigit(ch) )
    {
      par *= 10;
      par += ch - '0';
    }
    else if( ch == 13 )
    {
      Serial.print("cmd=");Serial.print(cmd);Serial.println(par);
      bool ok = true;
      switch(cmd)
      {
        case 'b':
          CONFIG.batMV = 1;
          CONFIG.batDAC = 1;
          CONFIG.batDAC = readBattery();
          CONFIG.batMV = par;
          break;
        case 'd':
          CONFIG.batMinMV = par;
          break;
        case 'e':
          CONFIG.batMaxMV = par;
          break;
        case 'c':
          CONFIG.batChargeMV = par;
          break;        
        case 'p':
          CONFIG.pumpTreshold = par;
          break;
        case 'o':
          CONFIG.pumpOnTime = par;
          break;
        case 'r':
          CONFIG.pumpRestTime = par;
          break;
        case 's':
          CONFIG.samplingTime = par;
          break;
        case 'u':
          CONFIG.stateUpdateTime = par;
          break;
        default:
          Serial.println("Invalid command");
          help();
          ok = false;
          break;                 
      }
      if( ok ) storeCONFIG();
      cmd = 0;
      par = 0;
    }
    else if( ch <= ' ' )
    {
      // ignore
    }
    else 
    {
      cmd = ch;
    }
  }
  return cmd != 0;
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

////////////////////////////////////////////////////////////////////////////////
// decide what to do about pumping
void checkPumping()
{  
  // update sensor state
  sensorValue = readSensor();
  
  // check for hard override (sensor shorted by user)
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
  digitalWrite(SOLAR_PIN, charging? 1 : 0); // relay driver is active low
  digitalWrite(PUMP_PIN, pumping ? 0 : 1); // relay driver is active low  
}

////////////////////////////////////////////////////////////////////////////////

void setup()
{
  // begin the serial communication
  Serial.begin(9600);
  Serial.println("Pump controller.");
  if( !loadCONFIG() )
  {
    storeCONFIG();
  }
  Serial.print("Pump TreshHold = ");Serial.println(CONFIG.pumpTreshold);
  digitalWrite(PUMP_PIN, 1);
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(BEEP_PIN, 0);
  pinMode(BEEP_PIN, OUTPUT);
  digitalWrite(SOLAR_PIN, 0);
  pinMode(SOLAR_PIN, OUTPUT);
  for(auto n = 0; n < 5; ++n) {
    beep(50);
    delay(1000UL);
    Serial.print('.');
  }
  beep(200);
  Serial.println("Ready!");
}

////////////////////////////////////////////////////////////////////////////////

void loop()
{
  bool inputPending = checkSerial();
  if(!inputPending)
  {  
    checkBattery();
    checkPumping();
    sleep1s();
    --timer;
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
}
