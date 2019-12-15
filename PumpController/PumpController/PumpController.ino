
// A0 is water sensor (two electrodes between ground and A0 with A0 pulled up to VCC via 200KOhm)
// D2 is water pump on/off

// sensor readout to start pumping in dac counts

// water sense treshold (lawer reading means water is present)
int treshHold = 500;

int readSensor()
{
  const int AVERAGECOUNT = 16;
  int32_t sum = 0;
  for(int i=0; i < AVERAGECOUNT; ++i)
    sum += analogRead(A0);
  auto ret = (int16_t)(sum/AVERAGECOUNT);
  Serial.print("Sensor read = ");
  Serial.println(ret);
  return ret;
}

#define LED_PIN D13
#define PUMP_PIN D2

void setup()
{
  // begin the serial communication
  Serial.begin(9600);
  Serial.println("Pump controller.");
  Serial.print("TreshHold = ");Serial.println(treshHold);
  digitalWrite(PUMP_PIN, 0);
  pinMode(PUMP_PIN, OUTPUT);
}


void sleep(unsigned seconds)
{
  Serial.print("waiting: ");
  while(seconds-- > 0)
  {
    Serial.print(seconds);
    digitalWrite(LED_PIN,1);
    delay(500UL);
    digitalWrite(LED_PIN,0);
    delay(500UL);
  }
  Serial.println("done");
}


bool pumping = false;
int timer = 0;

#if CONTINUS_PUMPING
const unsigned MinPumpTime = 20;
const unsigned PumpTime = 60;
const unsigned PumpRestTime = 60;
const unsigned SamplingTime = 10;
#else // gravity feed (pump for 30 seconds than rest for 15 min)
const unsigned MinPumpTime = 30;
const unsigned MaxPumpTime = 30;
const unsigned PumpRestTime = 15*60;
const unsigned SamplingTime = 10;
#endif


void loop()
{
  int16_t sensorValue = readSensor();
  auto prevPumping = pumping;
  if(pumping && timer > MaxPumpTime)
  {
    pumping = false;
  } 
  else
  {
    pumping = sensorValue < treshHold;
  }
  
  if(prevPumping != pumping)
  {
    Serial.print("pumping ");
    Serial.println(pumping?"ON":"OFF");
    digitalWrite(PUMP_PIN, pumping);
    timer = MinPumpTime;
    sleep(pumping ? MinPumpTime : PumpRestTime);
  }
  else
  {
    sleep(SamplingTime);
    timer += SamplingTime;
  }
}
