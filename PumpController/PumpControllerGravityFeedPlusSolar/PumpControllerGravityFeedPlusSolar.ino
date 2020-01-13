
// water pump controller with solar charging control 
// - the pump just floods syphoning system and then rest to let it drain with gravity
// - solar changing is monitored via voltage sense through resistive divider ~1:10

// A0 is water sensor (two electrodes between ground and A0 with A0 pulled up to VCC via 200KOhm)
// A1 is battery voltage via a ~1:10 divider

// D 4/5/6/7 are relays (zero-active): D4 is pump (active low)
// D8 is solar panel via MOSFET solid state relay (active high)

#define LED_PIN 13
#define PUMP_PIN 4
#define SOLAR_PIN 8
#define BEEP_PIN 11

// sensor readout to start pumping in dac counts

// water sense treshold (lawer reading means water is present)
int treshHold = 650;

int readSensor()
{
  const int AVERAGECOUNT = 16;
  int32_t sum = 0;
  for(int i=0; i < AVERAGECOUNT; ++i)
    sum += analogRead(A0);
  auto ret = (int16_t)(sum/AVERAGECOUNT);
  return ret;
}


long readBattery()
{
  const int AVERAGECOUNT = 16;
  // measured 12.86V--> 258 DAC COUNTS (this may be temp dependent!)
  const int MUL = 12860;
  const int DIV = 258*AVERAGECOUNT;
  int32_t sum = 0;
  for(int i=0; i < AVERAGECOUNT; ++i)
    sum += analogRead(A1);
  sum = sum*MUL/DIV;
  return (int)sum;
}


long readVcc() 
{
  long result; // Read 1.1V reference against AVcc 
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1); 
  delay(2); 
  // Wait for Vref to settle 
  ADCSRA |= _BV(ADSC); 
  // Convert 
  while (bit_is_set(ADCSRA,ADSC)); 
  result = ADCL;
  result |= ADCH<<8; 
  result = 1126400L / result; // Back-calculate AVcc in mV 
  return result; 
}

void sleep1s()
{
  digitalWrite(LED_PIN,1);
  delay(500UL);
  digitalWrite(LED_PIN,0);
  delay(500UL);
}

void beep(int ms=100, int waitafter = 0)
{
  digitalWrite(BEEP_PIN,1);
  delay(ms);
  digitalWrite(BEEP_PIN,0);      
  delay(waitafter);
}


bool pumping = false;
bool charging = false;
bool powerok = true;
int timer = 0;

const unsigned PumpOnTime = 30;
const unsigned PumpRestTime = 30*60;
const unsigned SamplingTime = 10;

void setTimer(int seconds)
{
  timer=seconds;
}

void setup()
{
  // begin the serial communication
  Serial.begin(9600);
  Serial.println("Pump controller.");
  Serial.print("TreshHold = ");Serial.println(treshHold);
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


void loop()
{
  // read the sensor and battery state
  auto sensorValue = readSensor();
  auto battery_mV= readBattery();
  // decide what to do
  if( battery_mV < 13000 )
  {
    charging = true;
  }
  else if( battery_mV > 14500 )
  {
    charging = false;
    powerok = true;
  }
  if( battery_mV < 8000 )
  {
    powerok  = false;
    pumping = false;
    if(timer==0)
    {
      beep(200,100);
      beep(200,100);
    }
  }
  else if( !powerok && battery_mV > 13000 )
  {
    powerok = true;
  }

  // check for override
  if( sensorValue < 10 )
  {
    // sensor forcefully grounded -> reset the timer
    setTimer(0);
    digitalWrite(LED_PIN,1);
    beep(1000);
  }
  
  // if timer expired do: pump off, rest or check sensor
  if( timer <= 0 )
  {
    if( pumping )
    {
      Serial.println("PUMP OFF!");
      pumping = false;
      setTimer(PumpRestTime);
    }
    else // not pumping so start checking sensor
    {
      if( sensorValue < treshHold )
      {
        if( powerok )
        {
          Serial.println("PUMP ON!");
          pumping = true;
          setTimer(PumpOnTime);
          beep(50);
        }
        else
        {
          Serial.println("POWER LOW, CANNOT PUMP!");
          beep(100,100);
          beep(500,100);
          beep(100,100);
        }
      }
      else
      {
        setTimer(SamplingTime);
      }
    }
  }
  Serial.print("Dsen=");Serial.print(sensorValue);
  Serial.print(" Vbat=");Serial.print(battery_mV);
  Serial.print(" Vcc=");Serial.print(readVcc());
  if(!powerok) Serial.print(" LOW!");
  if(charging) Serial.print(" charging");
  if(pumping) Serial.print(" pumping");
  Serial.print(" T=");Serial.print(timer);

  Serial.println();
  digitalWrite(SOLAR_PIN, charging? 1 : 0); // relay driver is active low
  digitalWrite(PUMP_PIN, pumping ? 0 : 1); // relay driver is active low
  sleep1s();
  // timer should always be >= 0
  --timer;
}
