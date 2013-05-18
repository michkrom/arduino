

const int analogPin = 0;
const int pulsePin = 1;


void setup()
{
  Serial.begin(9600);
}

void loop()
{
  // EZ ranger outputs Vcc/512 per inch
  // AVR has 10bit ADC --> Vcc/1024 so just divide by 2 the ADC read
  int inch = analogRead(0)/2;
  
  Serial.println(inch);
  
  delay(100);
}
