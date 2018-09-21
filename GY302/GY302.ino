/*
 GY302 BH1750 light meas sensor - simple sketch
*/
#include <Wire.h>

const char ADDR_L = 0x23;//addr pin L or disconnected
const char ADDR_H = 0x5A;//addr pin H
const char POWOFF = 0x00; // Power off command
const char POWON  = 0x01; // Power on command
const char RESET  = 0x07; // Reset Data Register
const char MEASCONTH  = 0x10; // Request continous measurement of High res, 120ms, 1lx res
const char MEASCONTH2 = 0x11; // Request continous measurement of High res, Mode2, 120ms, 0.5lx res
const char MEASCONTL  = 0x13; // Request continous measurement of Low res, 16ms, 4lx res
const char MEASONCEH  = 0x20; // Request continous measurement of High 1lx  res, auto power down
const char MEASONCEH2 = 0x21; // Request continous measurement of High 0.5lx res, auto power down
const char MEASONCEL  = 0x23; // Request continous measurement of Low 4lx res, auto power down
const char CHANGETIMEHB = 0x40; // change meas time high bits 765 in 210 of command
const char CHANGETIMELB = 0x50; // change meas time high bits 43210 in 43210 of command


const int ADDR = ADDR_L; // default

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  Wire.begin();
  Serial.begin(9600);
}


//H-reslution mode : Illuminance per 1 count ( lx / count ) = 1 / 1.2 *( 69 / X )
//H-reslution mode2 : Illuminance per 1 count ( lx / count ) = 1 / 1.2 *( 69 / X ) / 2 
// where 69 is the default measurement time

uint16_t gyRead()
{
  Wire.requestFrom(ADDR, 2);    // request 6 bytes from slave device #8  
  uint16_t d;
  while (Wire.available()) { // slave may send less than requested
    uint16_t b = Wire.read(); // receive a byte
    d >>= 8; // it's LSB first
    d |= b << 8;
  }
  return d;  
}

void gyCmd(char cmd)
{
  Wire.beginTransmission(ADDR);
  Wire.write(cmd);
  Wire.endTransmission();  
}

void gyMeasTime(char time)
{
  gyCmd(CHANGETIMEHB | (0x07 && (time >> 5)));
  gyCmd(CHANGETIMELB | (0x1F && time ));
}


// the loop function runs over and over again forever
void loop() {  
  gyCmd(POWON); // power on
  gyCmd(MEASONCEH); // single HiRes Meas
  delay(120); // wait 120 ms

  digitalWrite(LED_BUILTIN, HIGH);  
  auto d = gyRead();  
  for(int i = 0; i < d / 1024; i++) { Serial.print("*"); }
  Serial.print(d);
  Serial.print("\r\n");
  digitalWrite(LED_BUILTIN, LOW);
}
