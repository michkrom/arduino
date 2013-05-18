// SPI bus pins
#define PIN_SSEL  10
#define PIN_SDIN  11
#define PIN_SDOU  12
#define PIN_SCLK  13

#define SPITRANSFER(data) { SPDR = data;  while (!(SPSR & (1<<SPIF))) ; }
#define SPIINDATA SPDR

////////////////////////////////////////////////////////////////////////////////

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

// LCD
#define PIN_LCD_SCE   3
#define PIN_LCD_RESET 4
#define PIN_LCD_DC    6
#include "C:\Users\mink\Documents\Arduino\Common\lcd.h"


////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

void setup(void)
{
  Serial.begin(9600);

  // set direction of pins
  pinMode(PIN_SDIN, OUTPUT);
  pinMode(PIN_SCLK, OUTPUT);
  pinMode(PIN_SDOU, INPUT);
  pinMode(PIN_SSEL, OUTPUT);
  
  pinMode(PIN_LCD_SCE,OUTPUT);
  digitalWrite(PIN_LCD_SCE, HIGH);

  SPIInit();
  LCD::Init();
  LCD::Clear();
}

const int nrows = 6;
const int nbits = LCD::XM*nrows;

#define DIM(a) (sizeof(a)/sizeof(a[0]))

byte bits[nbits/8];

void loop()
{
  byte c = digitalRead(8);
  while(c==digitalRead(8));
  
  for(int b=0; b<DIM(bits);b++)
  {
    byte x = 0;
    for(byte bit = 0; bit<8; bit++)
    {
      x<<=1;
      x|=digitalRead(8);
    }
    bits[b]=x;
  }
  
  LCD::Goto(0,0);
  for(int b=0; b<DIM(bits);b++)
  {
    byte x = bits[b];
    for(byte bit = 8; bit>0; bit--)
    {
      LCD::Fill(x&(1<<(bit-1))?0x40:0x02,1);
    }  
  }
}
