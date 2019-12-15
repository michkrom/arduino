/*
 * Water Pump controller
 * by Michal Krombholz
 *
 * Water sensor:
 * A0 pulled up by 1MOhm
 * GND
 * 
 * Pump drive on D13 (LED)
 */

void setup()
{
  // begin the serial communication
  Serial.begin(9600);
  Serial.println("Water Pump Controller.");
  digitalWrite(13,0);
  pinMode(13,OUTPUT);
}


void loop()
{
  int a0 = analogRead(0);
  Serial.println(a0);
  if(a0 < 500)
  {
    digitalWrite(13,1);
  }
  else
  {
    digitalWrite(13,0);
  }
}
