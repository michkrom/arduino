
// water pump controller - the pump just floods syphoning system and then rest to let it drain with gravity

// A0 is water sensor (two electrodes between ground and A0 with A0 pulled up to VCC via 200KOhm)
// D2 is water pump on/off

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

#define LED_PIN 13
#define PUMP_PIN 2

void setup()
{
  // begin the serial communication
  Serial.begin(9600);
  Serial.println("Pump controller.");
  Serial.print("TreshHold = ");Serial.println(treshHold);
  digitalWrite(PUMP_PIN, 0);
  pinMode(PUMP_PIN, OUTPUT);
}


void sleep1s()
{
  digitalWrite(LED_PIN,1);
  delay(100UL);
  digitalWrite(LED_PIN,0);
  delay(900UL);
}


bool pumping = false;
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
  int16_t sensorValue = readSensor();
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
  Serial.print("pumping=");Serial.print(pumping);
  Serial.print(" timer=");Serial.print(timer);
  Serial.print(" sensor=");Serial.print(sensorValue);
  Serial.println();
  digitalWrite(PUMP_PIN, pumping ? 0 : 1); // relay driver is active low
  sleep1s();
  // timer should always be >= 0
  --timer;
}
