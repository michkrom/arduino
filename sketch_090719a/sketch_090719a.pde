#define PIN_IR   2 // digital
#define PIN_GYRO 0 // analog
#define PIN_LED 13 // digital

void setup
{
  Serial.begin(9600);
}

int prevGyro = 0;
int prevIR = 0;

void loop
{
  digitalWrite(PIN_LED,0);
  int Gyro = analogRead(PIN_GYRO);
  byte IR = digitalRead(PIN_IR);
  if( IR <> prevIR or Gyro <> prevGyro )
  {
    digitalWrite(PIN_LED,1);
  }
}
