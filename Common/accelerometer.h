////////////////////////////////////////////////////////////////////////////////
// ACceleroMeter LIS302DL
////////////////////////////////////////////////////////////////////////////////

#ifdef PREDEFINED_PINS
// SPI bus pins
#define PIN_SSEL  10
#define PIN_SDIN  11
#define PIN_SDOU  12
#define PIN_SCLK  13

// ACCELEROMETER 
#define PIN_ACM_CE  7

#define SPITRANSFER(data) { SPDR = data;  while (!(SPSR & (1<<SPIF))) ; }
#define SPIINDATA SPDR

void SPIInit()
{
  // set direction of pins
  pinMode(PIN_SDIN, OUTPUT);
  pinMode(PIN_SCLK, OUTPUT);
  pinMode(PIN_SDOU, INPUT);
  pinMode(PIN_SSEL, OUTPUT);
  
  // SPCR = 01010000
  // Set the SPCR register to 01010000
  //interrupt disabled,spi enabled,msb 1st,master,clk low when idle,
  //sample on leading edge of clk,system clock/4 rate
  SPCR = (1<<SPE)|(1<<MSTR)|(1<<CPOL)|(1<<CPHA);
  byte clr;
  clr=SPSR;
  clr=SPDR;
}

#endif

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

  static void Init()
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


