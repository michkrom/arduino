/*
 * Serial Echo
 * by Michal Krombholz
 *
 * Demonstrates the re-sending data from one pin on software serial to computer
 *
 */
#include <SoftwareSerial.h>

const int ledPin = 13;
const int rxPin = 2;
const int txPin = 3;
// set up a new serial port
SoftwareSerial mySerial =  SoftwareSerial(rxPin, txPin);

void setup()
{
  // begin the serial communication
  Serial.begin(115200);
  Serial.println("Software Echo Ready\r\n");
  pinMode(ledPin, OUTPUT);
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  mySerial.begin(9600);
}

int led = 0;

void loop()
{
  // listen for new serial coming in:
  char someChar = mySerial.read();
  // print out the character:
  Serial.print(someChar);
  // toggle an LED just so you see the thing's alive.  
  // this LED will go on with every OTHER character received:
  led=!led;
  digitalWrite(ledPin,led);
}

