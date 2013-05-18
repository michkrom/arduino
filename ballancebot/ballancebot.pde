#if DEBUG
#define DPRINT(a) Serial.print a
#define DPRINTLN(a) Serial.println a
#else
#define DPRINT(a)
#define DPRINTLN(a)
#endif

// define SPI communication macros & pins
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
#define PIN_LCD_SCE   8
//#define PIN_LCD_RESET 4
#define PIN_LCD_DC    7
#include "C:\Users\mink\Documents\Arduino\Common\lcd.h"

// motor control pins
#define PIN_MOTOR_LF 5
#define PIN_MOTOR_LR 3
#define PIN_MOTOR_RF 9
#define PIN_MOTOR_RR 6

template<char PWM_A,char PWM_B>
void driveOne(int speed)
{
  int pwm = speed >= 0 ? speed : -speed;
  if( pwm > 255 ) pwm = 255;
  byte pwmA, pwmB;  
  if( speed < 0 )
  {
    pwmA = PWM_A;
    pwmB = PWM_B;
  }
  else
  {
    pwmA = PWM_B;
    pwmB = PWM_A;
  }
  // fist zero 'the other' Half-HBridge and then set the active, 
  // otherwise a short short can happen and CPU resets
  analogWrite(pwmB,0);
  analogWrite(pwmA,pwm);
}

char motorLeft = 0; 
char motorRight = 0;

void motors(char left, char right)
{
  #if 1
  driveOne<PIN_MOTOR_LF,PIN_MOTOR_LR>(left);
  driveOne<PIN_MOTOR_RF,PIN_MOTOR_RR>(right);
  #else
  if( left != motorLeft )
  {
    digitalWrite(PIN_MOTOR_LF, LOW);
    digitalWrite(PIN_MOTOR_LR, LOW);
    if( left > 0 )
      digitalWrite( PIN_MOTOR_LF , HIGH);
    else if( left < 0 )
      digitalWrite( PIN_MOTOR_LR , HIGH);
    motorLeft = left;
  }
  if( right != motorRight )
  {
    digitalWrite(PIN_MOTOR_RF, LOW);
    digitalWrite(PIN_MOTOR_RR, LOW);
    if( right > 0 )
      digitalWrite( PIN_MOTOR_RF , HIGH);
    else if( right < 0 )
      digitalWrite( PIN_MOTOR_RR , HIGH);
    motorRight = right;
  }  
  #endif
  if( (left ^ motorLeft) & 0x80 != 0 ) // compare the signs
  {
  }
  if( (right ^ motorRight) & 0x80 != 0 ) // compare the signs
  {
  }
}

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

  // motor PINS
  pinMode(PIN_MOTOR_LF, OUTPUT);
  digitalWrite(PIN_MOTOR_LF, LOW);
  pinMode(PIN_MOTOR_RF, OUTPUT);
  digitalWrite(PIN_MOTOR_RF, LOW);
  pinMode(PIN_MOTOR_LR, OUTPUT);
  digitalWrite(PIN_MOTOR_LR, LOW);
  pinMode(PIN_MOTOR_RR, OUTPUT);
  digitalWrite(PIN_MOTOR_RR, LOW);

  SPIInit();

  LCD::Init();
  LCD::Clear();
  ACM::Init();
  DPRINTLN((ACM::ID(),HEX));
  DPRINTLN((ACM::STATE(),HEX));
  DPRINTLN(("============="));
}

//////////////////////////////////////////////////////////////////////////////////////////

// all units are SI : meters, seconds etc.

// G force (earth gravity)
const float G = 9.81; // m/s2

// pixel size
const float pixsize = 3e-2/LCD::XM; // in meters =~ width(3cm)/numpixX

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
  for( byte n = 0; n < 4; n++ )  
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
  char x,y,z;
  while(1)
  {
    char ax = ACM::X();    
    LCD::Goto(0,0);LCD::Num<10>(ax,3);
    LCD::Goto(3,x);LCD::Fill(0,4);
    x = LCD::XM/2+ax;
    if( x < 0 ) x = 0;
    if( x >= LCD::XM-4-1 ) x = LCD::XM-4-1;
    LCD::Goto(3,x);LCD::Fill(0xff,4);

#if 0    
    char ay = ACM::Y();
    LCD::Goto(0,26);LCD::Num<10>(ay,3);
    LCD::Goto(4,y);LCD::Fill(0,4);
    y = LCD::XM/2+ay;
    if( y < 0 ) y = 0;
    if( y >= LCD::XM-4 ) y = LCD::XM-4-1;
    LCD::Goto(4,y);LCD::Fill(0xff,4);
    
    char az = ACM::Z();    
    LCD::Goto(0,52);LCD::Num<10>(az,3);
    LCD::Goto(5,z);LCD::Fill(0,4);
    z = LCD::XM/2+az;
    if( z < 0 ) z = 0;
    if( z >= LCD::XM-4 ) z = LCD::XM-4-1;
    LCD::Goto(5,z);LCD::Fill(0xff,4);
#endif


    if( ax > 2 ) 
    {
      motors(127,127);
    }
    else if( ax < -2 )
    {
      motors(-128,-128);
    }
    else
    {
      motors(0,0);
    }
  }
}
