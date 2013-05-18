#define PIN_IR   2 // digital
#define PIN_GYRO 0 // analog
#define PIN_LED 13 // digital

/*
FWRD	0011011011011100000011
BKWD	0011101011100000000011
LEFT	0010011011101100000011
RGHT	0010101011110000000011
FWLT	0011111011100100000011
FWRT	0010001011101000000011
BKRT	0011011011111000000011
BKLT	0011001011110100000011
HDLT    0011011101011100000111
HDRT	0010111101011100001100
STOP	0011011101011100000000
*/

#define CMD_HDRT 0xBD70C
#define CMD_HDLT 0xDD707
#define CMD_NULL 0xDD700
#define CMD_HP   0xBD500
#define CMD_AIRGUITAR 0xA8E00
#define CMD_AIRDRUMS 0x98D00
#define CMD_MODE_PM 0xA0800
#define CMD_MODE_SA 0xB0900
#define CMD_MODE_VC 0xC0A00
#define CMD_MODE_RC 0x80700
#define CMD_GOODMORNING 0xE4400
#define CMD_YOULIKE 0xA6B00
#define CMD_THATSALL 0x84D00
#define CMD_AMAZING 0xB7300
#define CMD_WHATSUP 0x94700
#define CMD_ANIMAL 0xA9500
#define CMD_TAICHI 0x99B00
#define CMD_CLINK 0xF8400


#define TimeStart 2500
#define TimeZero  500
#define TimeOne   1000
#define TimeOff   500

void irSendSignal(unsigned time)
{
    // outputing 0 (we are open-collector with the isobot's IR sensor
    pinMode(PIN_IR,OUTPUT);
    delayMicroseconds( time );
    // making the PIN an input again will remove the drive to 0, hence let the IR sensor to act
    pinMode(PIN_IR,INPUT);
}

void irSendCmd(unsigned long cmd)
{
    Serial.print("send ");
    Serial.println(cmd,HEX);
    irSendSignal( TimeStart );
    unsigned long bit = ((unsigned long)1) << 21;
    while( bit )
    {
      unsigned time = cmd & bit ? TimeOne : TimeZero;
      delayMicroseconds(time);
      irSendSignal( TimeOff );
      bit >>= 1;
    }
}


//-----------------------------------------------------------------------------------

unsigned long elapsedSince(unsigned long since, unsigned long now)
{
  return since < now ? now-since : 0xFFFFFFFFUL - (now - since);
}

unsigned long elapsedSince(unsigned long since)
{
  return elapsedSince( since, micros() );
}

#define IS_X(t,x) ((t > 3*(x)/4) && (t < 5*(x)/4))
#define IS_0(t) IS_X(t,TimeZero)
#define IS_1(t) IS_X(t,TimeOne)
#define IS_S(t) IS_X(t,TimeStart)
#define IS_O(t) IS_X(t,TimeOff)


#define IRISR

#ifdef IRISR

// pin state change interrupt - observing the IR pin changes
enum 
{ 
  ISR_IDLE, // nothing is/was happening (quiet)
  ISR_START,  // start of sequence, was waiting for a header signal
  ISR_BIT_ON, // transmitting a bit (ie no active signal - quiet)
  ISR_BIT_OFF // in an OFF bit slot (transmiting 500us break between bits)
} 
isrState = ISR_IDLE;

unsigned long isrLastTimeStamp;
unsigned long isrRcvCmd;

ISR( PCINT2_vect )
{
  static unsigned long isrNewCmd;
  static byte isrBitCnt;
  
  // PIN_IR == #2 --> PD2; 
  // receiving a modulated IR signal makes the pin go low (active low)
  byte transmitting = (PIND & (1<<2)) == 0;
  
  // compute elapsed time since last change
  unsigned elapsed;
  {
    #if 1
    unsigned long timeStamp = micros();
    elapsed = elapsedSince(isrLastTimeStamp,timeStamp); //--> faster yet do it inline
    #else
    // copied from wiring.c - external time counters updated in TCNT0 overflow ISR
    extern volatile unsigned long timer0_overflow_count;
    unsigned long timeStamp = ((timer0_overflow_count) << 8) | TCNT0;
    elapsed = timeStamp > isrLastTimeStamp ? timeStamp-isrLastTimeStamp : 0xFFFFFFFFUL - (isrLastTimeStamp-timeStamp);
    #endif
    isrLastTimeStamp = timeStamp;
  }
  
  switch( isrState )
  {
    case ISR_IDLE : 
      if( transmitting ) isrState = ISR_START; 
      break;
    case ISR_START: 
      isrBitCnt = 0;
      isrNewCmd = 0;
      if( !transmitting && IS_S(elapsed) ) isrState = ISR_BIT_ON; // bits are now rolling
                                      else isrState = ISR_IDLE; // wrong timing of start or pin state
      break;
    case ISR_BIT_ON:
      if( transmitting ) 
      {
        isrState = ISR_BIT_OFF;
        isrNewCmd <<= 1;
        isrBitCnt ++;
        if( IS_1(elapsed) )
        {
          isrNewCmd |= 1;
        }
        else if( IS_0(elapsed) )
        {
          // isrNewCmd |= 0;
        }
        else 
          isrState = ISR_START; // bad timing, start over
      }
      else isrState = ISR_IDLE; // bad state (should never get here...)
      break;
    case ISR_BIT_OFF: 
      if( !transmitting && IS_O(elapsed) ) 
      {
        if( isrBitCnt == 22 ) // is this the end? (TODO what about longer commands?)
        {
          isrState = ISR_IDLE;
          isrRcvCmd = isrNewCmd;
        }
        else        
          isrState = ISR_BIT_ON; // keep bits rolling
      }
      else 
        isrState = ISR_IDLE;   // wrong timing of start or pin state
      break;
  }
}
  

long irRecv()
{  
    byte oldSREG = SREG;  
    cli();
    long cmd = isrRcvCmd;
    isrRcvCmd = 0;
    SREG = oldSREG;
    return cmd;
}

#else
// waits for HIGH->LOW and then for LOW->HIGH
// returns time of the low period
// returns 0 if timeout
unsigned irRecvSignal(byte waitFor)
{
  while(digitalRead(PIN_IR)==(waitFor==LOW?HIGH:LOW)) {};
  unsigned long start = micros();
  while(digitalRead(PIN_IR)==waitFor) {};
  return elapsedSince(start);
}

long irRecv()
{  
  Serial.println("Waiting...");
  unsigned time = irRecvSignal(LOW);
  if( !IS_S(time) )
  {
    Serial.println("False Start");
    Serial.println(time);
    return 0;
  }
  long bits = 0;
  for(int i = 0; i < 22; i++ )
  {
    bits <<= 1;
    time = irRecvSignal(HIGH);
    if( IS_1(time) )
    {
      bits |= 1;
    }
    else if( IS_0(time) )
    {
      bits |= 0;
    }
    else
    {
      Serial.println("Bad Bit");
      Serial.println(time);
      return 0;
    }
  }
  return bits;
}
#endif

//-----------------------------------------------------------------------------------

void setup()
{
  Serial.begin(9600);
  #ifdef IRISR
  PCICR  |=  4; // enable PCIE2
  PCMSK2 |=  4; // enable PCINT18 --> Pin Chnage Interrupt of PD2
  #endif
  irSendCmd(CMD_MODE_RC);
  irSendCmd(CMD_GOODMORNING);
}


#define DIM(a) (sizeof(a)/sizeof(a[0]))

void loop()
{
 unsigned long lastAction = 0;
 unsigned long lastLed = 0;
 byte led = 0;
 while(true)
 {
   unsigned long now = millis();
   long cmd = irRecv();
//  Serial.print(isrCnt,DEC);Serial.print(' ');
//  Serial.print(isrState,DEC);Serial.print(' ');
//  Serial.print(isrLastTimeStamp);Serial.print(' ');
//  Serial.println();
  if( cmd != 0 ) 
  {
    Serial.print("got ");
    Serial.println(cmd,HEX);
    lastAction = now;
  }
  if( now - lastLed > 500 )
  {
    lastLed = now;
    led = !led;
    digitalWrite(PIN_LED,led);
  }
  if( now - lastAction > 20000 )
  {
    digitalWrite(PIN_LED,1);
    unsigned long int fun[] = { CMD_HDLT, CMD_HDRT, CMD_AIRGUITAR, CMD_YOULIKE, CMD_AIRDRUMS, CMD_THATSALL, 
                                CMD_AMAZING, CMD_WHATSUP, CMD_ANIMAL, CMD_TAICHI, CMD_CLINK };
    int what = random(DIM(fun));
    irSendCmd(fun[what]);
    lastAction = now;    
    digitalWrite(PIN_LED,0);
  }
 }
}
