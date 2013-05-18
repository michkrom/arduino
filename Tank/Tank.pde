/*
 * Tank Robot Control Code
 * Michal Krombholz
 *
 */

#define DEBUG 
#ifdef DEBUG
#define DBG(m) Serial.print(m);
#define DBGI(num) Serial.print((int)num);
#define DBGNL Serial.println();
#else
#define DBG(m)
#define DBGI(num) Serial.print(num);
#define DBGNL
#endif

////////////////////////////////////////////////////////////////////////////////

// LED PINS
const int LED = 13;
const int LEDFL = 8;
const int LEDFR = 9;

// engine PWM control pint Left/Right Forward/Reverse
const byte PWM_LF = 11;
const byte PWM_LR = 10;
const byte PWM_RF = 6;
const byte PWM_RR = 5;

////////////////////////////////////////////////////////////////////////////////

void blink(int cnt)
{
  for(int i = 0; i < cnt; i++)
  {
    digitalWrite(LED,1);
    delay(100);
    digitalWrite(LED,0);
    delay(100);
  }
}

////////////////////////////////////////////////////////////////////////////////

void driveOne(int LR, int speed)
{
  int pwm = speed >= 0 ? speed : -speed;
  if( pwm > 255 ) pwm = 255;
  byte pwmA, pwmB;
  if( LR )
  {
    pwmA = PWM_LF;
    pwmB = PWM_LR;
  }
  else
  {
    pwmA = PWM_RF;
    pwmB = PWM_RR;
  }
  if( speed < 0 )
  {
    byte tmp = pwmA;
    pwmA = pwmB;
    pwmB = tmp;
  }
  // fist zero 'the other' HBridge and then set the active, 
  // otherwise a short short can happen and CPU resets
  analogWrite(pwmB,0);
  analogWrite(pwmA,pwm);
}

#ifdef DRIVE_DIRECT
void drive(int leftSpeed, int rightSpeed)
{
  DBGI(leftSpeed);DBG(',');DBGI(rightSpeed);DBG("\n");
  #ifdef DEBUG
  return;
  #endif
  driveOne(0,leftSpeed);
  driveOne(1,rightSpeed);
  digitalWrite(LEDFL,0);
  digitalWrite(LEDFR,0);
  if( leftSpeed > rightSpeed )
    digitalWrite(LEDFL,1);
  else if( leftSpeed < rightSpeed )
    digitalWrite(LEDFR,1);
  else if( leftSpeed > 0 && rightSpeed > 0 )
  {
    digitalWrite(LEDFL,1);
    digitalWrite(LEDFR,1);    
  }
}
#else

int curLSpeed;
int curRSpeed;
int targetLSpeed;
int targetRSpeed;

int newSpeed(int curSpeed,int targetSpeed)
{
  const int MAXCHANGERATE = 1;
  int d = targetSpeed-curSpeed;
  if(d > MAXCHANGERATE) d = MAXCHANGERATE;
  if(d < -MAXCHANGERATE) d = -MAXCHANGERATE;
  if(d==0 && targetSpeed!=curSpeed) 
    curSpeed = targetSpeed;
  else
    curSpeed += d;
  return curSpeed;
}


void drive(int leftSpeed, int rightSpeed)
{
  DBGI(leftSpeed);DBG(',');DBGI(rightSpeed);DBG("\n");
  #ifdef DEBUG
  return;
  #endif
  targetLSpeed = leftSpeed;
  targetRSpeed = rightSpeed;

  digitalWrite(LEDFL,0);
  digitalWrite(LEDFR,0);
  if( leftSpeed > rightSpeed )
    digitalWrite(LEDFL,1);
  else if( leftSpeed < rightSpeed )
    digitalWrite(LEDFR,1);
  else if( leftSpeed > 0 && rightSpeed > 0 )
  {
    digitalWrite(LEDFL,1);
    digitalWrite(LEDFR,1);    
  }
}

void applySpeedChange()
{
  curLSpeed = newSpeed(curLSpeed,targetLSpeed);
  curRSpeed = newSpeed(curRSpeed,targetRSpeed);
  driveOne(0,curLSpeed);
  driveOne(1,curRSpeed);
}
#endif

////////////////////////////////////////////////////////////////////////////////

enum State
{
  REST = 1,
  SEARCH = 2,
  DRIVE = 3,
  TURN = 4,
  AVOID = 5,
  TURNAROUND = 6
};

char lastSearchDir = 1;
char lastTurnDir = 1;
const byte SPEED = 180;

void applyState(byte state)
{
  switch(state)
  {
    case REST:
    {
      drive(0,0);
      break;
    }
    case SEARCH:
    {
      drive((3*SPEED/4)*lastSearchDir,-(3*SPEED/4)*lastSearchDir);
      lastSearchDir=-lastSearchDir;
      digitalWrite(LED,1);
      break;
    }
    case DRIVE:
    {
      drive(SPEED,SPEED);
      break;
    }
    case TURN:
    {
      if( lastTurnDir>0 )
        drive(3*SPEED/2,SPEED/4);
      else
        drive(SPEED/4,3*SPEED/2);
      lastTurnDir=-lastTurnDir;
      break;
    }
    case AVOID:
    {  
      drive(-SPEED,-SPEED);
      break;
    }
    case TURNAROUND:
    {
      drive(SPEED*lastSearchDir,-SPEED*lastSearchDir);
      lastSearchDir=-lastSearchDir;
      break;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////

const byte BUMP = 8;
const byte NEAR = 16;
const byte FAR = 32;

// decide what to do
int findNextState(byte currState,
                  byte obsticle,
                  unsigned int timeInState,
                  unsigned int unrestTime)
{
  byte state = currState;
  switch(currState)
  {
    case REST:
    {
      if( timeInState > 4000 ) state=SEARCH;
      if( timeInState > 1000 && obsticle < BUMP ) state=AVOID;
      break;
    }
    case SEARCH:
    {
      if( timeInState > 3000 && obsticle < NEAR ) state=AVOID;
      if( timeInState > 1500 && obsticle >= NEAR ) state=DRIVE;
      if( obsticle >= FAR ) state=DRIVE;
      break;
    }
    case DRIVE:
    {
      if( obsticle < FAR-8 ) state=TURN;
      if( unrestTime > 60000 ) state=REST;
      if( obsticle < BUMP ) state=AVOID;
      break;
    }
    case TURN:
    {
      if( obsticle >= FAR ) state=DRIVE;
      if( timeInState > 2000 ) state=SEARCH;
     // if( obsticle < 24 ) state=SEARCH;      
      if( obsticle < BUMP ) state=AVOID;
      break;
    }
    case AVOID:
    {
      if( timeInState > 1500 && obsticle > NEAR ) state=DRIVE;
      if( timeInState > 1000 && obsticle < NEAR ) 
         state =  ((micros() & 0xF) < 12) ? SEARCH : TURNAROUND;      
      if( timeInState > 1500 ) state=TURNAROUND;
      break;
    }
    case TURNAROUND:
    {
      if( timeInState > 1000 ) state = SEARCH;
      break;
    }
    default:
    {
      state=REST;
    }
  }  
  DBGI(currState);DBG(" ==> ");DBGI(state);DBGNL;
  return state;
}

////////////////////////////////////////////////////////////////////////////////

byte state = REST;
byte cntStateChangesSinceDrive = 0;
unsigned long lastStateChangeTime;
unsigned long lastRestTime;

////////////////////////////////////////////////////////////////////////////////

volatile byte curRange = 255;
volatile unsigned long timeRangeChecked = 0;
volatile unsigned long curRangeTime = 0; 
volatile unsigned long curRangeDuration = 0;

const unsigned long TICKSPERINCH = 147*clockCyclesPerMicrosecond()/64;
#define RANGE_ANALOG
#ifndef RANGE_ANALOG
ISR( PCINT1_vect )
{
  // copied from wiring.c - external time counters updated in TCNT0 overflow ISR
  extern volatile unsigned long timer0_overflow_count;
  extern volatile unsigned long timer0_millis;
  
  unsigned long timeStamp = ((timer0_overflow_count) << 8) | TCNT0;

  static unsigned long startTS = 0;
  
  // if( PCFGR & 2 != 0 /* PCI1 flag */ ) dont care, we are the only ISR here 
  if( (PINC & 2) != 0 ) // start of ranging - rise edge
  {
    startTS = timeStamp;
  }  
  else // must be falling edge - end ranging
  {
    unsigned long stopTS = timeStamp;
    curRangeDuration = ( stopTS > startTS ) ? stopTS-startTS : 0xFFFFFFFF - (startTS-stopTS);
    curRangeTime = timer0_millis;
  }
}
#endif

////////////////////////////////////////////////////////////////////////////////

void setup()
{
  #ifdef DEBUG
  Serial.begin(9600);
  #endif
  lastStateChangeTime = millis();
  lastRestTime = millis();
  // ENABLE Pin change interrupt on AI1==>PC1==>PCINT9
  // this is the EZ3 ranger pulse out pin
  PCICR  |=  2; // enable PCIE1
  PCMSK1 |=  2; // enable PCINT9
  DDRC   &= ~2; // enable PC1 as input
  DBG("TICKSperIN=");DBG(TICKSPERINCH);DBGNL;
}

////////////////////////////////////////////////////////////////////////////////

void loop()
{
  #ifdef DEBUG
  delay(100);
  #else
  delay(1);
  #endif
#ifdef RANGE_ANALOG
  const unsigned long  timeNow = millis();
  const unsigned long  timeRange = timeNow;
  const byte obsticle = analogRead(0)/2;
#else
  const unsigned long  timeNow = millis();
  cli();
  const unsigned long  timeRange = curRangeTime;
  //static const unsigned long MFACTOR = 64 / clockCyclesPerMicrosecond();    
  if( curRangeDuration > 255*TICKSPERINCH )
     curRange = 255;
  else
     curRange = curRangeDuration / TICKSPERINCH;
  const byte obsticle = curRange;
  sei();
#endif  

  const unsigned int timeUnrest = timeNow-lastRestTime;
  const unsigned int timeInState =  timeNow-lastStateChangeTime;  
  const byte time8th = (timeNow>>6) & 0xF; // 1/8 of a second = 8 times a second to generate up to 8 pulses in 2 seconds
  
  timeRangeChecked = timeNow;
  digitalWrite(LED, 1 );
  digitalWrite(LEDFR, 1 );
  digitalWrite(LEDFL, 1 );
  #ifdef DEBUG
  DBG(" RNG=");DBGI(curRange);DBG(" @ ");DBG(timeRange);
  DBGNL;
  return;
  for( int n = timeRange; n > 0; n -= 10 ) Serial.print('.');
  Serial.println();
  return;   
  #endif

  static long prevStateDecisionTime = 0;
  if( timeNow-prevStateDecisionTime > 50 )
  {
    prevStateDecisionTime = timeNow;
    byte nextState = findNextState(state,obsticle,timeInState,timeUnrest);
    DBG(" T=");DBGI(timeInState);DBGI(state);DBG(" => ");DBGI(nextState);DBGNL;
    // apply state change
    if( state != nextState )
    {
      applyState(nextState);
      if( state==REST ) 
        lastRestTime=timeNow;
      state=nextState;
      lastStateChangeTime = timeNow;
      if( state == DRIVE ) 
        cntStateChangesSinceDrive = 0; 
      else
        cntStateChangesSinceDrive++;
    }
  }
  applySpeedChange();

  // heartbeat LED
  if( timeNow-timeRange > 1000 ) 
  {
    // lost the ranger
    drive(0,0);
    digitalWrite(LED, ( time8th & 1 ) ? 1 : 0 );
    return;
  }  
  else if( obsticle <= BUMP )
    digitalWrite(LED, ( time8th == 1 || time8th == 3 || time8th == 5 || time8th == 7 || time8th > 8 ) ? 1 : 0 );
  else if( state <= REST ) 
    digitalWrite(LED, ( time8th == 1 || time8th == 3 || time8th > 8 ) ? 1 : 0 );
  else if( state == REST ) 
    digitalWrite(LED, ( time8th == 3 || time8th > 6) ? 1 : 0 );    
  else 
    digitalWrite(LED, ( time8th == 1 ) ? 1 : 0 );
}
