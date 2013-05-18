#define DIM(a) (sizeof(a)/sizeof(a[0]))

#define PIN_IR   2 // digital
#define PIN_GYRO 0 // analog
#define PIN_BATT 3 // analog
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

#define MSG_HDRT 0xBD70C
#define MSG_HDLT 0xDD707
#define MSG_NULL 0xDD700
#define MSG_HP   0xBD500
#define MSG_AIRGUITAR 0xA8E00
#define MSG_AIRDRUMS 0x98D00
#define MSG_MODE_PM 0xA0800
#define MSG_MODE_SA 0xB0900
#define MSG_MODE_VC 0xC0A00
#define MSG_MODE_RC 0x80700
#define MSG_GOODMORNING 0xE4400
#define MSG_YOULIKE 0xA6B00
#define MSG_THATSALL 0x84D00
#define MSG_AMAZING 0xB7300
#define MSG_WHATSUP 0x94700
#define MSG_CHICKEN 0xA9500
#define MSG_DOG     0xf9a00
#define MSG_BIRD    0x99400
#define MSG_GORILLA 0xB9600
#define MSG_CAT     0x89300
#define MSG_TAICHI  0x99B00
#define MSG_ARTICULATION 0xF8400
#define MSG_CLINK 0x87E00
#define MSG_LOVE1 0xA6b00

#define CMD_AIRDRUMS 0x8D
#define CMD_BIRD     0x94
#define CMD_HP       0xD5
#define CMD_MODE_RC  0x07
#define CMD_GOODMORNING 0x44
#define CMD_CLINK    0x7E
#define CMD_LOVE1    0x6B

/////////////////////////////////////////////////////////////////////////////////

byte computeCheckSum( long cmd )
{
  // cmd &= ~0x07000000; // clear up existing checksum if any
  // first sum up all bytes
  byte s = cmd + (cmd >> 8) + (cmd >> 16) + (cmd >> 24);  
  // then sum up the result, 3 bits at a time
  s = (s & 7) + ((s >> 3) & 7) + (s >> 6) & 7;
  // return 3 lower bits of the sum
  return s & 7;
}

byte computeCheckSum( byte hdr, byte cmd1, byte cmd2, byte cmd3 )
{
  // first sum up all bytes
  byte s = hdr + cmd1 + cmd2 + cmd3;  
  // then sum up the result, 3 bits at a time
  s = (s & 7) + ((s >> 3) & 7) + (s >> 6) & 7;
  // return 3 lower bits of the sum
  return s & 7;
}

byte makeHdr(byte channel, byte msgtype, byte checksum = 0)
{
  return (channel << 5) + (msgtype << 3) + checksum;
}

// 2 byte command, type = 01
unsigned long irMakeMsg(byte ch, byte cmd, byte par = 0)
{
  byte hdr = makeHdr(ch,1);
  hdr += computeCheckSum( hdr, cmd, par, 0 );
  unsigned long msg = hdr;
  msg <<= 8;
  msg |= cmd;
  msg <<= 8;
  msg |= par;
  msg <<= 8;
  msg |= 0;
  return msg;
}

// 3 byte command, type = 00
inline
unsigned long irMakeMsg(byte ch, byte cmd1, byte cmd2, byte cmd3)
{
  byte hdr = makeHdr(ch,0);
  hdr += computeCheckSum( hdr, cmd1, cmd2, cmd3 );
  unsigned long msg = hdr;
  msg <<= 8;
  msg |= cmd1;
  msg <<= 8;
  msg |= cmd2;
  msg <<= 8;
  msg |= cmd3;
  return msg;
}

/////////////////////////////////////////////////////////////////////////////////


#define TimeStart 2500
#define TimeZero  500
#define TimeOne   1000
#define TimeOff   500


////////////////////////////////////////////////////////////////////////////////////////
// generate a burst of IR for given time
//
void irSendSignal(unsigned time)
{
    // outputing 0 (we are open-collector with the isobot's IR sensor
    pinMode(PIN_IR,OUTPUT);
    delayMicroseconds( time );
    // making the PIN an input again will remove the drive to 0, hence let the IR sensor to act
    pinMode(PIN_IR,INPUT);
}


//////////////////////////////////////////////////////////////////////////////////////
// send full message (either 22 or 30 bits depending on msg type field)
// the cmd format is:
// - header on the most siginificant byte (00CTTSSS - C-channel; TT-type; SSS-checksum)
// - then payload on lower bytes
//
void irSendMsg(unsigned long msg)
{
    byte len = ((msg >> 24) & (3<<2)) == 0 ? 30 : 22;
    Serial.print("sendMsg ");
    Serial.print(msg,HEX);
    Serial.print(' ');
    Serial.print(len,DEC);
    Serial.print(' ');
    
    irSendSignal( TimeStart );
    // the message type field is 
    for(byte bit = len; bit > 0; bit--)
    {      
      // probe the current top bit only (hope the ar-gcc will optimize this '>> &' as a single byte operation)
      // this probe start from the top header bit but then we shift up the cmd
      unsigned time = ((msg >> 24) & (1<<5)) == 0 ? TimeZero : TimeOne;
      Serial.print(time==TimeOne?'1':'0');

      delayMicroseconds(time);
      irSendSignal( TimeOff );
      // shift up the message to prepare for another turn
      msg <<= 1;
    }
    Serial.println();

}

void irSendCmd(unsigned long cmd)
{
    Serial.print("sendCmd ");
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

void irSendMsg(byte ch, byte cmd, byte par = 0)
{
  irSendMsg(irMakeMsg(ch,cmd,par));
}

//-----------------------------------------------------------------------------------

inline
unsigned long elapsedSince(unsigned long since, unsigned long now)
{
  return since < now ? now-since : 0xFFFFFFFFUL - (now - since);
}

inline
unsigned long elapsedSince(unsigned long since)
{
  return elapsedSince( since, micros() );
}

////////////////////////////////////////////////////////////////////////////////////////////

#define IS_X(t,x) ((t > 3*(x)/4) && (t < 5*(x)/4))
#define IS_0(t) IS_X(t,TimeZero)
#define IS_1(t) IS_X(t,TimeOne)
#define IS_S(t) IS_X(t,TimeStart)
#define IS_O(t) IS_X(t,TimeOff)

#define IRISR

#ifdef IRISR

// pin state change interrupt based IR receiver
// observing the IR pin changes, measure time between interrupts
// use state machine to decide what to do

enum 
{ 
  ISR_IDLE,   // nothing is/was happening (quiet)
  ISR_START,  // start of sequence, was waiting for a header signal
  ISR_BIT_ON, // transsmitting a bit (IR carrier turned on)
  ISR_BIT_OFF // in an OFF bit slot (IR carrier turned off)
} 
isrState = ISR_IDLE;

unsigned long isrLastTimeStamp;
unsigned long isrRcvCmd;
unsigned long isrNewCmd;
byte isrBitLen = 22;
byte isrBitCnt;

void isrDebug()
{
  Serial.print("ISR ");
  Serial.print(isrState,DEC);
  Serial.print(' ');
  Serial.print(isrBitLen,DEC);
  Serial.print(' ');
  Serial.print(isrBitCnt,DEC);
  Serial.print(" 0x");
  Serial.print(isrNewCmd,HEX);
  Serial.println();
}

ISR( PCINT2_vect )
{
  
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
      isrBitLen = 22;
      if( !transmitting && IS_S(elapsed) ) 
        isrState = ISR_BIT_ON; // bits are now rolling
      else 
        isrState = ISR_IDLE;   // wrong timing of start or pin state
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
        if( isrBitCnt == 7 ) // we have received 6 bit header (now expecting 7th bit)
        {
          isrBitLen = (isrNewCmd & (3<<3)) == 0 ? 30 : 22; // 2 vs 3 byte commands
        }
      }
      else isrState = ISR_IDLE; // bad state (should never get here...)
      break;
    case ISR_BIT_OFF: 
      if( !transmitting && IS_O(elapsed) ) 
      {
        if( isrBitCnt == isrBitLen ) // is this the end?
        {
          isrState = ISR_IDLE;
          isrRcvCmd = isrNewCmd;
        }
        else        
          isrState = ISR_BIT_ON; // keep bits rolling
      }
      else
        if( IS_S(elapsed) )
          isrState = ISR_START;
        else 
          isrState = ISR_IDLE;
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
//////////////////////////////////////////////////////////////////////
// polling version of IR receiver
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
// waits for IR carrier transitions (on->off & off->on)
// returns duration of the specified state (HIGH->IR transmitting; LOW->IR off)
// returns 0 if timeout
unsigned irRecvSignal(byte waitFor)
{
  while(digitalRead(PIN_IR)==(waitFor==LOW?HIGH:LOW)) {};
  unsigned long start = micros();
  while(digitalRead(PIN_IR)==waitFor) {};
  return elapsedSince(start);
}

////////////////////////////////////////////////////////////////////////
// receives all 22 bits of the messagge
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
  byte len = 22;
  for(int i = 0; i < len; i++ )
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
    // check type of message (when recved) to quess the message lenth (either 22 ot 30 bits)
    if( bit == 3 )
    {
      byte msgtype = bits & 0x3;
      if( msgtype == 0 ) len = 30;
    }
  }
  return bits;
}
#endif

//-----------------------------------------------------------------------------------

void setup()
{
  digitalWrite(PIN_LED,1);
  Serial.begin(9600);
  Serial.println("setup()");
  #ifdef IRISR
  PCICR  |=  4; // enable PCIE2
  PCMSK2 |=  4; // enable PCINT18 --> Pin Chnage Interrupt of PD2
  #endif
  irSendCmd(MSG_MODE_RC);
//  irSendCmd(MSG_GOODMORNING);
//  delay(4000);
}

void specialShow()
{
  irSendMsg(0,CMD_BIRD);
  delay(4000);
  irSendMsg(0,CMD_AIRDRUMS);
  delay(15000);
  irSendMsg(0,CMD_HP); 
  irSendMsg(0,random(0,10)<5?CMD_CLINK:CMD_LOVE1);
}

byte seconds = 0;
char key = 0;
byte cmd = 0;
byte par = 0;
byte num=0;
byte cnt=1;

void loop()
{
  {
    bool newCmd = false;
    if( Serial.available() )
      key = Serial.read();
    else
      key = 0;    
    switch(key)
    {
      case 'i': 
      {
        isrDebug(); 
        unsigned long rcv = irRecv();
        Serial.println(rcv,HEX);        
        break;
      }
      case 'n':
      case '+': cmd++; newCmd=true; break;
      case 'p':
      case '-': cmd--; newCmd=true; break;
      case ' ': newCmd=true; break;
      case 'r': irSendMsg(0,CMD_MODE_RC); break;
      case 'h': irSendMsg(0,CMD_HP); break;
      case 'x': cmd = num; num=0; newCmd=true; break;
      case 'm': cnt = num; num=0; break;
      case 'd': par = num; break;
      case 's': specialShow(); break;
      default:
        if( key >= '0' and key <= '9' )
        {
          num *= 10;
          num += key-'0';
          Serial.println(num,DEC);
        }
    }
    if( newCmd )
    for( int n = cnt; n >= 0; n-- )
    {
        newCmd = false;
        Serial.print(cmd,HEX);Serial.print(',');
        Serial.print(par,HEX);Serial.print(' ');
        irSendMsg(0,cmd,par);
        delay(500);    
    }
    unsigned long rcv = irRecv();
    if(rcv)
    {
      Serial.print("recv ");
      Serial.println(rcv,HEX);
    }
    
  }
}

void loop2()
{
 unsigned long lastAction = 0;
 unsigned long lastLed = 0;
 unsigned long lastReadout = 0;
 byte led = 0;
 Serial.println("Loop");
 test();
 while(true)
 {
  if( seconds == 0 )
  {
    Serial.println("reseeding...");
    int seed = analogRead(PIN_GYRO)*analogRead(PIN_BATT);
    randomSeed(seed);
    irSendMsg(0,CMD_BIRD);
    delay(4000);
    irSendMsg(0,CMD_AIRDRUMS);
    delay(15000);
    irSendMsg(0,CMD_HP); 
    irSendMsg(0,(seed & 2)?CMD_CLINK:CMD_LOVE1);
    seconds++;
  }

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
    seconds += led & 1;
  }
  if( now - lastAction > 20000 )
  {
    digitalWrite(PIN_LED,1);
    byte what = random(0,255);
    irSendMsg(0,what);
    lastAction = now;    
    digitalWrite(PIN_LED,0);
  }
  if( now - lastReadout > 1000 )
  {
    int gyro = analogRead(PIN_GYRO); // around ~425 no much change, ~100 when powered off
    int batt = analogRead(PIN_BATT); // around ~640 full; ~520 will power off; goes down by ~20 when in action
    Serial.print(gyro);
    Serial.print(' ');
    Serial.print(batt);
    Serial.println();
    lastReadout = now;
 }
 }
}
