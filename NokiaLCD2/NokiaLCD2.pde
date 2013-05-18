#include <Servo.h>

// SPI bus pins
#define PIN_SSEL  10
#define PIN_SDIN  11
#define PIN_SDOU  12
#define PIN_SCLK  13

#define SPI 1
#if SPI
#define SPITRANSFER(data) { SPDR = data;  while (!(SPSR & (1<<SPIF))) ; }
#define SPIINDATA SPDR
#else
#define SPITRANSFER(data)   shiftOut(PIN_SDIN, PIN_SCLK, MSBFIRST, data)
#define SPIINDATA 0
#endif

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

// ACCELEROMETER 
#define PIN_ACM_CE  7
#include "C:\Users\mink\Documents\Arduino\Common\accelerometer.h"

// LCD
#define PIN_LCD_SCE   3
#define PIN_LCD_RESET 4
#define PIN_LCD_DC    6
#include "C:\Users\mink\Documents\Arduino\Common\lcd.h"


////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

Servo servo;

void setup(void)
{
  servo.attach(9);
  Serial.begin(9600);

  // set direction of pins
  pinMode(PIN_SDIN, OUTPUT);
  pinMode(PIN_SCLK, OUTPUT);
  pinMode(PIN_SDOU, INPUT);
  pinMode(PIN_SSEL, OUTPUT);
  
  pinMode(PIN_ACM_CE,OUTPUT);
  digitalWrite(PIN_ACM_CE, HIGH);
  pinMode(PIN_LCD_SCE,OUTPUT);
  digitalWrite(PIN_LCD_SCE, HIGH);

  #if SPI
  SPIInit();
  #endif

  LCD::Init();
  LCD::Clear();
  ACM::Init();
  Serial.println(ACM::ID(),HEX);
  Serial.println(ACM::STATE(),HEX);
}

char pos=0;
char dir=1;

void loop(void)
{
#if TESTS  
  delay(3000);
  for(byte r=0;r<6;r++)
  {
    LCD::Goto(r,0);
    LCD::Char('R');
    LCD::Char('0'+r);
    LCD::Char('X');
  }
  delay(1000);
  LCD::Goto(0,0);
  for(byte r=0;r<6;r++)
  for(byte c=0;c<12;c++)
  {
    LCD::Char('0'+r);
  }
  delay(1000);
  LCD::Clear(0);
  LCD::Goto(0,0);
  LCD::Str("Hello World!");
  LCD::Goto(1,0);
  LCD::Goto(2,0);
  LCD::Str("1=");
  LCD::Num<10>(1);
  LCD::Goto(3,0);
  LCD::Str("10=");
  LCD::Num<10>(10);
  LCD::Goto(4,0);
  LCD::Str("100=");
  LCD::Num<10>(100);
  LCD::Goto(5,0);
  LCD::Str("x=");
  LCD::Num<16>(0x1234,6);
#else
  LCD::Goto(0,0);
  LCD::Str("BAT=");
  LCD::Num<10>((unsigned)((2L*3300L*analogRead(0))/1024L),4);
  LCD::Str("mV");
  LCD::Char(analogRead(2)<100?0x7E:0x7F);
  
//  LCD::Goto(1,pos);LCD::Char(' ');
//  LCD::Goto(1,LCD::XM-1-8-pos);LCD::Char(' ');
  pos += dir;
  if(pos > LCD::XM-1-8) { pos = LCD::XM-1-8; dir = -1; }
  if(pos < 0) { pos = 0; dir = 1; }
  LCD::Goto(0,11*7);LCD::Fill(~((1<<(8*(unsigned)pos)/LCD::XM)-1),7);
//  LCD::Goto(1,pos);LCD::Char(0x80);
//  LCD::Goto(1,LCD::XM-1-8-pos);LCD::Char(0x81);
  LCD::Goto(1,0);LCD::Fill(0xFF,pos);LCD::Fill(0,LCD::XM-pos);
  
  // LIS302DL
  char accx = map(ACM::X(),-127,127,0,LCD::XM);
  LCD::Goto(3,0);LCD::Fill(0,accx-2);LCD::Fill(0xFF,4);LCD::Fill(0,LCD::XM-accx-4);
  char accy = map(ACM::Y(),-127,127,0,LCD::XM);
  LCD::Goto(4,0);LCD::Fill(0,accy-2);LCD::Fill(0xFF,4);LCD::Fill(0,LCD::XM-accy-4);
  char accz = map(ACM::Z(),-127,127,0,LCD::XM);
  LCD::Goto(5,0);LCD::Fill(0,accz-2);LCD::Fill(0xFF,4);LCD::Fill(0,LCD::XM-accz-4);


  // 2Y0A21 IR dist sensor is 0.5-3V output proportional to inverse of 7-60cm
  // getting >100 @ long to <900 at near
  static int avg = 0;
  int dac = analogRead(3);
//  LCD::Goto(4,0);LCD::Num<10>(dac,4);
  if(dac<101)dac=101;
  int cm = 10+8000/(dac-100);
  if(cm>LCD::XM-1) cm=LCD::XM-1;
  avg = 3*avg + cm;
  avg /= 4;
  LCD::Goto(2,0);LCD::Fill(0xFF,avg);LCD::Fill(0,LCD::XM-avg);
  servo.write(cm);
  Serial.println(avg,DEC);
  Serial.println(accx,DEC);
  Serial.println(accy,DEC);
  Serial.println(accz,DEC);
  Serial.println();

#endif
}
