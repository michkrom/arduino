// based on http://www.nearfuturelaboratory.com/2006/09/22/arduino-and-the-lis3lv02dq-triple-axis-accelerometer/

/*
VDD -> 5V
GND -> GND
INT -> N/C
SDO -> Arduino 12
SDA -> Arduino 11
SCL -> Arduino 13
CS  -> Arduino  7
*/

#define DATAOUT 11//MOSI
#define DATAIN  12//MISO 
#define SPICLOCK  13//sck
#define SLAVESELECT 10//ss for /LIS3LV02DQ, active low
#define LISCE  7 //ss for /LIS3LV02DQ, active low

#define PIN_ACM_CE LISCE

#define SPITRANSFER(data) { SPDR = data;  while (!(SPSR & (1<<SPIF))) ; }
#define SPIINDATA SPDR

////////////////////////////////////////////////////////////////////////////////
// ACceleroMeter LIS302DL
////////////////////////////////////////////////////////////////////////////////

class ACM
{
public:

  static inline char ID() { return ReadReg(0x0f); }
  static inline byte STATE() { return (byte)ReadReg(0x27); }
  static inline char X() { return ReadReg(0x29); }
  static inline char Y() { return ReadReg(0x2b); }
  static inline char Z() { return ReadReg(0x2d); }

  // write to a register
  static void WriteReg(byte reg, byte data)
  {
     // SS is active low
     digitalWrite(PIN_ACM_CE, LOW);
     // send the address of the register we want to write
     SPITRANSFER(reg);
     // send the data we're writing
     SPITRANSFER(data);
     // unselect the device
     digitalWrite(PIN_ACM_CE, HIGH);
  }

  // reads a register
  static char ReadReg(byte reg)
  {
    WriteReg(reg|128,0);
    return SPIINDATA;
  }

  static void Initialize()
  {
    // CE pin, disable device  
    pinMode(PIN_ACM_CE,OUTPUT);
    digitalWrite(PIN_ACM_CE,HIGH);
    // start up the device
    // this essentially activates the device, powers it on, enables all axes, and turn off the self test
    // CTRL_REG1 set to 01000111
    WriteReg(0x20, 0x47);
    delay(250);    
  }

}; // ACM


void SPIInit()
{
  // set direction of pins
  pinMode(DATAOUT, OUTPUT);
  pinMode(DATAIN, INPUT);
  pinMode(SPICLOCK,OUTPUT);
  pinMode(SLAVESELECT,OUTPUT);
  pinMode(LISCE,OUTPUT);
  digitalWrite(LISCE,HIGH); //disable device
  
  // SPCR = 01010000
  // Set the SPCR register to 01010000
  //interrupt disabled,spi enabled,msb 1st,master,clk low when idle,
  //sample on leading edge of clk,system clock/4 rate
  SPCR = (1<<SPE)|(1<<MSTR)|(1<<CPOL)|(1<<CPHA);
  byte clr;
  clr=SPSR;
  clr=SPDR;
}

///////////////////////////////////////////////////////////////////////////////////


void setup()
{
  Serial.begin(9600);
  SPIInit();
  ACM::Initialize();
}

void loop()
{
#if 1 
  // the LIS3LV02DQ according to specs, these values are:
  // 2g = 127
  // 1g = 63
  char x =  ACM::X();
  char y =  ACM::Y();
  char z =  ACM::Z();
  Serial.print(x, DEC);
  Serial.print(","); 
  Serial.print(y, DEC);
  Serial.print(","); 
  Serial.print(z, DEC);
  Serial.print(" ");Serial.print(((unsigned)ACM::STATE())&0xff,HEX);
  Serial.println();    
#else  
  for(int i = 0; i < 256; i++ )
  {
    Serial.print(ACM::ReadReg(i),HEX);
    if( (i & 0xF) == 0xF ) 
      Serial.println(); 
    else 
      Serial.print(' ');
  }
#endif
  delay(100);
}
