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

// ACCELEROMETER 
#define PIN_ACM_CE  4
#include "C:\Users\mink\Documents\Arduino\Common\accelerometer.h"

// LCD
// LCD
#define PIN_LCD_SCE   8
//#define PIN_LCD_RESET 4 // do not need this as reset is common with CPU
#define PIN_LCD_DC    7
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
  
  pinMode(PIN_ACM_CE,OUTPUT);
  digitalWrite(PIN_ACM_CE, HIGH);
  pinMode(PIN_LCD_SCE,OUTPUT);
  digitalWrite(PIN_LCD_SCE, HIGH);

  SPIInit();

  LCD::Init();
  LCD::Clear();
  ACM::Init();
  Serial.println(ACM::ID(),HEX);
  Serial.println(ACM::STATE(),HEX);
  Serial.println("=============");
}


const byte BALLSIZE = 4;

void DrawBall(byte  x, byte y, byte erase)
{
  byte row = y / 8;
  byte shift = y % 8;
  unsigned pattern = (1<<BALLSIZE)-1;
  pattern <<= shift;
  byte f;
  f = pattern & 0xFF;
  if( f )
  {
    LCD::Goto(row,x);
    LCD::Fill(erase ? 0 : f, BALLSIZE);
  }
  f = pattern >> 8;
  if( f )
  {
    LCD::Goto(row+1,x);
    LCD::Fill(erase ? 0 : f, BALLSIZE);
  }
}

static byte curx = 0;
static byte cury = 0;

void MoveBall(byte x, byte y)
{
  if( curx != x || cury != y )
  {
    DrawBall(curx,cury,1);
    curx = x; cury=y;
    DrawBall(curx,cury,0);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////

// all units are SI : meters, seconds etc.

// G force (earth gravity)
const float G = 9.81; // m/s2

// pixel size
const float pixsize = 3e-2/LCD::XM; // in meters =~ width(3cm)/numpixX
// ball size
const float ballsize = BALLSIZE*pixsize;
const float xsize = LCD::XM * pixsize;
const float ysize = LCD::YM * pixsize;
float x=float(LCD::XM/2)*pixsize;
float y=float(LCD::YM/2)*pixsize;
float px=float(LCD::XM/2)*pixsize;
float py=float(LCD::YM/2)*pixsize;
float vx=0;
float vy=0;
float prevTime = 0;

float elapsed()
{
  if( prevTime == 0 ) prevTime = float(1e-3*millis());
  float time = float(1e-3*millis());
  float delta = time - prevTime;
  prevTime = time;
  return delta;
}

inline float sign(float f)
{
  return f > 0 ? 1 : (f < 0 ? -1 : 0);
}


float ax = 0;
float ay = 0;
float az = 0;

float azx = 0;
float azy = 0;
float azz = 0;


void zero()
{
  ax=ay=az=0;
  azx=azy=azz=0;
  for( byte n = 0; n < 20; n++ )  
  {
    measure();
  }  
  azx=ax;
  azy=ay;
  azz=az;
}

void measure()
{
  const float filtercoef = 0.25;
  ax = filtercoef*(G/64.0)*float(ACM::X())+(1-filtercoef)*ax;
  ay = filtercoef*(G/64.0)*float(ACM::Y())+(1-filtercoef)*ay;
  az = filtercoef*(G/64.0)*float(ACM::Z())+(1-filtercoef)*az;
}

void loop(void)
{  
  zero();
  while(1)
  {
  #if 0
  {
  // LIS302DL
  char ax = ACM::X();
  char ay = ACM::Y();
  char az = ACM::Z();
  char n = map(ax,-64,64,2,LCD::XM-2);
  LCD::Goto(3,0);LCD::Fill(0,n-2);LCD::Fill(0xFF,4);LCD::Fill(0,LCD::XM-n-4);
  n = map(ay,-64,64,2,LCD::XM-2);
  LCD::Goto(4,0);LCD::Fill(0,n-2);LCD::Fill(0xFF,4);LCD::Fill(0,LCD::XM-n-4);
  n = map(az,-64,64,2,LCD::XM-2);
  LCD::Goto(5,0);LCD::Fill(0,n-2);LCD::Fill(0xFF,4);LCD::Fill(0,LCD::XM-n-4);
  }
  #endif 

  float dt = elapsed();
  
#if 0
  const float refrat = -1;
  if( vx == 0 ) vx = 10*pixsize/1; // 10 pixels per second 
  if( vy == 0 ) vy = 10*pixsize/1; // 10 pixels per second 
  x += vx * dt;
  y += vy * dt;
  if( x < 0 )               { x = 0; vx = refrat * vx; }
  if( x >= xsize-ballsize ) { x = xsize-ballsize; vx = refrat * vx; }
  if( y < 0 )               { y = 0; vy = refrat * vy; }
  if( y >= ysize-ballsize ) { y = ysize-ballsize; vy = refrat * vy; }
#else

  measure();
  // apply gravity: the display's y axis is a z axis of acceleromiter due to mounting direction
  vx +=  (az-azz)*dt;
  vy +=  (ay-azy)*dt;
  // apply center pull this emulates a hiperbol shape bowl-like dish in which the ball rolls
  const float fudge = 0.2;
  vx += -fudge*G*(x-(xsize/2-ballsize/2))/xsize*dt;
  vy += -fudge*G*(y-(ysize/2-ballsize/2))/ysize*dt;
//  vx += -0.5*(x>xsize/2?1:-1)*dt;
//  vy += -0.5*(y>ysize/2?1:-1)*dt;
  // apply friction
  vx += -0.1*vx;
  vy += -0.1*vy;
  // move with speed
  x += vx * dt;
  y += vy * dt;
  //  bounce off walls
  const float refrat = 0.9; // bounce reflection ratio (== 1 for perfect bounce)
  if( x < 0 )               { x = 0; vx = refrat * vx; }
  if( x >= xsize-ballsize ) { x = xsize-ballsize; vx = refrat * vx; }
  if( y < 0 )               { y = 0; vy = refrat * vy; }
  if( y >= ysize-ballsize ) { y = ysize-ballsize; vy = refrat * vy; }
  // zero speed if we are trully not moving
  if( abs(px - x) < 1e-10 ) vx = 0;
  if( abs(py - y) < 1e-10 ) vy = 0;
  px = x;
  py = y;
#endif

  byte bx = (byte)(x/pixsize);
  byte by = (byte)(y/pixsize);  
  MoveBall(bx,by);
  #if 0
  Serial.println(dt);
  Serial.println(ax-azx);
  Serial.println(ay-azy);
  Serial.println(az-azz);
  Serial.println(vx);
  Serial.println(vy);
  Serial.println(G*(x-(xsize/2-ballsize/2))/xsize*dt);
  Serial.println(G*(y-(ysize/2-ballsize/2))/ysize*dt);
  Serial.println();
  #endif
//  LCD::Goto(0,0);LCD::Num<10>(bx);
//  LCD::Goto(0,LCD::XM/2);LCD::Num<10>(by);
  delay(20);
  }  
}
