
// water pump controller with solar charging control 
// - the pump just floods syphoning system and then rest to let it drain with gravity
// - solar changing is monitored via voltage sense through resistive divider ~1:10

// A0 is water sensor (two electrodes between ground and A0 with A0 pulled up to VCC via 200KOhm)

// D 4/5/6/7 are relays (zero-active): D7 is solar panel D4 is pump

#define LED_PIN 13
#define PUMP_PIN 4
#define SOLAR_PIN 5

// sensor readout to start pumping in dac counts

// water sense treshold (lawer reading means water is present)
int treshHold = 550;

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
  // measured 13.01V--> 290 DAC COUNTS
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


void setup()
{
  // begin the serial communication
  Serial.begin(9600);
  Serial.println("Pump controller.");
  Serial.print("TreshHold = ");Serial.println(treshHold);
  digitalWrite(PUMP_PIN, 1);
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(SOLAR_PIN, 1);
  pinMode(SOLAR_PIN, OUTPUT);
  for(auto n = 0; n < 5; ++n) {
    delay(1000UL);
    Serial.print('.');
  }
  Serial.println("Ready!");
}


void sleep1s()
{
  digitalWrite(LED_PIN,1);
  delay(100UL);
  digitalWrite(LED_PIN,0);
  delay(900UL);
}


bool pumping = false;
bool charging = false;
int timer = 0;

const unsigned PumpOnTime = 30;
const unsigned PumpRestTime = 15*60;
const unsigned SamplingTime = 10;

void setTimer(int seconds)
{
  timer=seconds;
}


void loop()
{
  // read the sensor always
  auto sensorValue = readSensor();
  auto battery_mV= readBattery();
  if( sensorValue < 10 )
  {
    // sensor forcefully grounded -> reset the timer
    setTimer(0);
    digitalWrite(LED_PIN,1);
    sleep1s();
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
        Serial.println("PUMP ON!");
        pumping = true;
        setTimer(PumpOnTime);
      }
      else
      {
        setTimer(SamplingTime);
      }
    }
  }
  if( battery_mV < 13000 )
  {
    charging = true;
  }
  else if( battery_mV > 14500 )
  {
    charging = false;
  }
  Serial.print("pumping=");Serial.print(pumping);
  Serial.print(" charging=");Serial.print(charging);
  Serial.print(" timer=");Serial.print(timer);
  Serial.print(" sensor=");Serial.print(sensorValue);
  Serial.print(" battery=");Serial.print(battery_mV);
  Serial.print(" Vcc=");Serial.print(readVcc());
  Serial.println();
  digitalWrite(SOLAR_PIN, charging? 0 : 1); // relay driver is active low
  digitalWrite(PUMP_PIN, pumping ? 0 : 1); // relay driver is active low
  sleep1s();
  // timer should always be >= 0
  --timer;
}
