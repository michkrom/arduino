// based on http://www.nearfuturelaboratory.com/2006/09/22/arduino-and-the-lis3lv02dq-triple-axis-accelerometer/

/*
VDD -> 5V
GND -> GND
MISO -> Arduino 12
MOSI -> Arduino 11
SCK  -> Arduino 13
SS   -> Arduino 10
*/

#define SCK 13
#define MISO 12
#define MOSI 11
#define SS 10

#define SPITRANSFER(data) { SPDR = data;  while (!(SPSR & (1<<SPIF))) ; }
#define SPIINDATA SPDR
#define SPISETSS(lev) { if(lev) PINB |= 1<<2; else PINB |= ~(1<<2); }

///////////////////////////////////////////////////////////////////////////////////


void setup()
{
  Serial.begin(9600);
  // set direction of pins
  pinMode(MOSI, OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(SCK,OUTPUT);
  pinMode(SS,OUTPUT);
  SPISETSS(HIGH); //disable device
  
  // SPCR = 01010000
  // Set the SPCR register to 01010000
  //interrupt disabled,spi enabled,msb 1st,master,clk low when idle,
  //sample on leading edge of clk,system clock/4 rate
  SPCR = (1<<SPE)|(1<<MSTR)|(1<<CPOL);//|(1<<CPHA);
  SPSR = SPSR|(1<<SPI2X);
  byte clr;
  clr=SPSR;
  clr=SPDR;
}

#define writeDac(d){\
  SPISETSS(0);\
  SPITRANSFER(d>>8);\
  SPITRANSFER(d&0xFF);\
  SPISETSS(1);\
}

void loop()
{
  writeDac(0);
  writeDac(0);
  writeDac(0);
  writeDac(0xFFFF);
  writeDac(0xFFFF);
  writeDac(0xFFFF);
  writeDac(0);
  writeDac(0);
  writeDac(0);
  writeDac(0x7FFF);  
  writeDac(0x7FFF);  
  writeDac(0x7FFF);  
  writeDac(0);
  writeDac(0);
  writeDac(0);
  for(unsigned i=0;i<0xC000;i+=0x100) writeDac(i);
  for(unsigned i=0xC000;i>0;i-=0x100) writeDac(i);
//  for(unsigned i=0;i<0xFFFF;i++) writeDac(i);
//  for(unsigned i=0xFFFF;i>0;i--) writeDac(i);  
  writeDac(0);
  writeDac(0);
  writeDac(0x7FFF);  
  writeDac(0x7FFF);  
  writeDac(0);
  writeDac(0);
  for(int i=0;i<0x400;i+=4) writeDac(i);
  for(int i=0x3ff;i>0;i-=4) writeDac(i);
}
