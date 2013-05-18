/*
 * Serial Echo
 * by Michal Krombholz
 *
 * Demonstrates the re-sending data from the computer to the Arduino board,
 *
 */

const int ledPin = 13;

void setup()
{
  // begin the serial communication
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
  Serial.println("Echo Ready\r\n");
}

byte led = 0;

void loop()
{
  // check if data has been sent from the computer
  if (Serial.available()) {
    byte val = Serial.read();
    Serial.print(val);
    led = !led;
  }
  digitalWrite(ledPin,led);
}

