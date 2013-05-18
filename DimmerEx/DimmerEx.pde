/*
 * Dimmer
 * by David A. Mellis
 *
 * Demonstrates the sending data from the computer to the Arduino board,
 * in this case to control the brightness of an LED.  The data is sent
 * in individual bytes, each of which ranges from 0 to 255.  Arduino
 * reads these bytes and uses them to set the brightness of the LED.
 *
 * http://www.arduino.cc/en/Tutorial/Dimmer
 */

const int ledPin = 13;

void setup()
{
  // begin the serial communication
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
  Serial.println("Dimmer Ready");
}

byte val = 0;
short n=0;
short dir = 1;

void loop()
{
  // check if data has been sent from the computer
  if (Serial.available()) {
    // read the most recent byte (which will be from 0 to 255)
    val = Serial.read();
    if( val < '0' ) val = '0';
    if( val > '9' ) val = '9';
    val -= '0';
    Serial.println(val,DEC);
    // set the brightness of the LED
    // analogWrite(ledPin, val);    
    n = 0;
  }
  digitalWrite(ledPin,1);
  delay(val);
  digitalWrite(ledPin,0);
  delay(10-val);
  n++;
  if( n > 200 ) 
  {
    val += dir;
    if( val >= 9 ) dir = -1;
    if( val == 0 ) dir = 1;
    n = 180;
  }
}

/* Processing code for this example
 // Dimmer - sends bytes over a serial port
 // by David A. Mellis
 
 import processing.serial.*;
 
 Serial port;
 
 void setup()
 {
 size(256, 150);
 
 println("Available serial ports:");
 println(Serial.list());
 
 // Uses the first port in this list (number 0).  Change this to
 // select the port corresponding to your Arduino board.  The last
 // parameter (e.g. 9600) is the speed of the communication.  It
 // has to correspond to the value passed to Serial.begin() in your
 // Arduino sketch.
 port = new Serial(this, Serial.list()[0], 9600);  
 
 // If you know the name of the port used by the Arduino board, you
 // can specify it directly like this.
 //port = new Serial(this, "COM1", 9600);
 }
 
 void draw()
 {
 // draw a gradient from black to white
 for (int i = 0; i < 256; i++) {
 stroke(i);
 line(i, 0, i, 150);
 }
 
 // write the current X-position of the mouse to the serial port as
 // a single byte
 port.write(mouseX);
 }
 */

/* Max/MSP patch for this example
 
 max v2;
 #N vpatcher 10 59 610 459;
 #P user uslider 286 66 18 128 255 1 0 0;
 #P window setfont "Sans Serif" 9.;
 #P window linecount 1;
 #P message 212 192 32 196617 print;
 #P newex 286 217 71 196617 serial a 9600;
 #P window linecount 2;
 #P comment 316 148 100 196617 Slide the fader to dim the LED;
 #P comment 69 192 125 196617 Click here to get a list of serial ports;
 #P fasten 4 0 2 0 291 215 291 215;
 #P fasten 3 0 2 0 217 212 291 212;
 #P pop;
 */
