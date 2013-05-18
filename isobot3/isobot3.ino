#define DIM(a) (sizeof(a)/sizeof(a[0]))

#define PIN_IR   2 // digital - do not change this or change the pin assignment in PCINT receiver
#define PIN_PWR  3 // digital - controlls power to the iSobot 0-ON 1-OFF
#define PIN_LED 13 // digital

#define PIN_GYRO  0 // analog
#define PIN_LIGHT 1 // analog
#define PIN_BATT  3 // analog

#define CMD_AIRDRUMS 0x8D
#define CMD_HP       0xD5
#define CMD_MODE_RC  0x07
#define CMD_GOODMORNING 0x44
#define CMD_CLINK    0x7E
#define CMD_LOVE1    0x6B

#define CMD_TURNON 0x01
#define CMD_ACTIVATED 0x02
#define CMD_READY 0x03
#define CMD_RC_CONFIRM 0x04
#define CMD_RC_PROMPT 0x05
#define CMD_MODE_PROMPT 0x06
#define CMD_IDLE_PROMPT 0x0B // 0x0C,0x0D,0x0E all the same
#define CMD_HUMMING_PROMPT 0x0F
#define CMD_COUGH_PROMPT 0x10
#define CMD_TIRED_PROMPT 0x11
#define CMD_SLEEP_PROMPT 0x12
#define CMD_FART 0x40 // 2A
#define CMD_ALLYOURBASES 0x41
#define CMD_SHOOT_RIGHT 0x64
#define CMD_SHOOT_RIGHT2 0x68
#define CMD_SHOOT2 0x69
#define CMD_BEEP 0x6a
#define CMD_BANZAI 0x7F
#define CMD_CHEER1 0x90
#define CMD_CHEER2 0x91
#define CMD_DOG 0x92
#define CMD_CAR 0x93
#define CMD_EAGLE 0x94
#define CMD_ROOSTER 0x95
#define CMD_GORILLA 0x96
#define CMD_LOOKOUT 0xA1
#define CMD_STORY1 0xA2 // knight and princess
#define CMD_STORY2 0xA3 // ready to start day
#define CMD_GREET1 0xA4 // good morning
#define CMD_GREET2 0xA5 // do somthing fun
#define CMD_POOP 0xA6 // poops his pants
#define CMD_GOOUT 0xA7 // ready to go out dancing
#define CMD_HIBUDDY 0xA8 // .. bring a round of drinks
#define CMD_INTRODUCTION 0xA9
#define CMD_ATYOURSERVICE 0xAA
#define CMD_SMELLS 0xAB
#define CMD_THATWASCLOSE 0xAC
#define CMD_WANNAPICEOFME 0xAD
#define CMD_RUNFORYOURLIFE 0xAE
#define CMD_TONEWTODIE 0xAF
// 0xB0 - nothing?
#define CMD_SWANLAKE 0xB1
#define CMD_DISCO 0xB2
#define CMD_MOONWALK 0xB3
#define CMD_REPEAT_PROMPT 0xB4
#define CMD_REPEAT_PROMPT2 0xB5
#define CMD_REPEAT_PROMPT3 0xB6
// 0xB7-0xC4 steps in different directions
#define CMD_HEADSMASH 0xC5
#define CMD_HEADHIT 0xC6
// 0xCC-0xD2 - unknown (use param?)
#define CMD_HIBEEP 0xD3 
// 0xD4 - unknown (use param?)
#define CMD_BEND_BACK 0xD8 // same up to 0xDB
#define CMD_SQUAT 0xDB // also 0xDC
#define CMD_BEND_FORWARD 0xDD
#define CMD_HEAD_LEFT_60 0xDE
#define CMD_HEAD_LEFT_45 0xDF
#define CMD_HEAD_LEFT_30 0xE0
#define CMD_HEAD_RIGHT_30 0xE1
#define CMD_HEAD_RIGHT_45 0xE2
#define CMD_HEAD_RIGHT_60 0xE3
// seems identical to A & B getups
#define CMD_GETUP_BELLY 0xE4
#define CMD_GETUP_BACK 0xE5
// E6 unknown 
#define CMD_HEAD_SCAN_AND_BEND 0xE7
#define CMD_ARM_TEST 0xE8
#define CMD_FALL_AND_LEG_TEST 0xE9
#define CMD_THANKYOUSIR 0xEA
#define CMD_ILOVEYOU_SHORT 0xEB
// CMD_3BEEPS goes through some test sequence and also turns sounds (starts?) 
// into beeps for all commands (power off to quit this mode)
// (looks like a tool to synchronize sound with moves)
#define CMD_3BEEPS 0xEC
#define CMD_FALL_DEAD 0xED
#define CMD_3BEEPS_AND_SLIDE 0xEE
// EF-FF unknown

/////////////////////////////////////////////////////////////////////////////////

inline
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

inline
byte computeCheckSum( byte hdr, byte cmd1, byte cmd2, byte cmd3 )
{
  // first sum up all bytes
  byte s = hdr + cmd1 + cmd2 + cmd3;  
  // then sum up the result, 3 bits at a time
  s = (s & 7) + ((s >> 3) & 7) + (s >> 6) & 7;
  // return 3 lower bits of the sum
  return s & 7;
}

inline
byte makeHdr(byte channel, byte msgtype, byte checksum = 0)
{
  return (channel << 5) + (msgtype << 3) + checksum;
}

// 2 byte command, type = 01
inline
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

inline 
unsigned long irMakeHandMsg(byte leftRotator, byte leftElboW, byte rightRotator, byte rightElbow)
{
}


/////////////////////////////////////////////////////////////////////////////////

#define TimeStart 2500
#define TimeZero  500
#define TimeOne   1000
#define TimeOff   500

////////////////////////////////////////////////////////////////////////////////////////
// generate (simulate) a burst of IR for given time
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
    byte len = ((msg >> 24) & (3<<3)) == 0 ? 30 : 22;
    
    Serial.print("send ");
    Serial.print(msg,HEX);
    Serial.print(' ');
    Serial.print(len,DEC);
    Serial.print(' ');
    Serial.println();
    
    irSendSignal( TimeStart );
    // the message type field is 
    for(byte bit = len; bit > 0; bit--)
    {      
      // probe the current top bit only (hope the ar-gcc will optimize this '>> &' as a single byte operation)
      // this probe start from the top header bit but then we shift up the cmd
      unsigned time = ((msg >> 24) & (1<<5)) == 0 ? TimeZero : TimeOne;
//      Serial.print(time==TimeOne?'1':'0');
      delayMicroseconds(time);
      irSendSignal( TimeOff );
      // shift up the message to prepare for another turn
      msg <<= 1;
    }
}

void irSendMsg(byte ch, byte cmd, byte par = 0)
{
  irSendMsg(irMakeMsg(ch,cmd,par));
}

void irSendMsg(byte ch, byte cmd, byte par, byte par2)
{
  irSendMsg(irMakeMsg(ch,cmd,par,par2));
}


////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////        IR RECEIVER      ///////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

inline
unsigned long elapsedSince(unsigned long since, unsigned long now)
{
  return since <= now ? now-since : 0xFFFFFFFFUL - (since-now);
}

inline
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

// pin state change interrupt based IR receiver
// observing the IR pin changes, measure time between interrupts
// use state machine to decide what to do

enum 
{ 
  ISR_IDLE,   // nothing is/was happening (quiet)
  ISR_START,  // start of sequence, was waiting for a header signal
  ISR_BIT_ON, // transsmitting a bit (IR carrier turned off so measure its duration and record a bit)
  ISR_BIT_OFF // in an OFF bit slot (IR carrier turned on, waiting of IR to stop)
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
    elapsed = elapsedSince(isrLastTimeStamp,timeStamp);
    #else  //--> faster yet do it inline
    // copied from wiring.c - external time counters updated in TCNT0 overflow ISR
    extern volatile unsigned long timer0_overflow_count;
    unsigned long timeStamp = ((timer0_overflow_count) << 8) | TCNT0;
    elapsed = timeStamp > isrLastTimeStamp ? timeStamp-isrLastTimeStamp : 0xFFFFFFFFUL - (isrLastTimeStamp-timeStamp);
    #endif
    isrLastTimeStamp = timeStamp;
  }
  // when in 'isrState' and we have received a change notification (isr) then do
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
        if( isrBitCnt == 3 ) // we have received 3rd bit of header
        {
          isrBitLen = (isrNewCmd & 3) == 0 ? 30 : 22; // 2 vs 3 byte commands
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
// polling/blocking version of IR receiver
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
    // check type of message (when recved) to guess the message length (either 22 ot 30 bits)
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
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------

bool isOn()
{
  return analogRead(PIN_GYRO)>200;
}

void iSobotOn()
{
  Serial.println("iSOBOT ON");
  digitalWrite(PIN_PWR,1);
  pinMode(PIN_PWR,OUTPUT);
  for( byte i = 0; i < 50; i++ )
  {
    delay(55);
    digitalWrite(PIN_LED,1);
    delay(55);
    digitalWrite(PIN_LED,0);
    if( isOn() ) break;
  }
  for( byte i = 0; i < 20; i++ )
  {
    digitalWrite(PIN_LED,1);
    delay(33);
    irSendMsg(0,CMD_MODE_RC);
    digitalWrite(PIN_LED,0);
    delay(33);
    irSendMsg(0,CMD_HP);
  }
  delay(33);
  irSendMsg(0,CMD_HIBEEP);
  delay(100);
//  irSendCmd(MSG_GOODMORNING);
//  delay(4000);
}

void iSobotOff()
{
  Serial.println("iSOBOT OFF");
  // turn him off!  
  digitalWrite(PIN_PWR,0);
  for( byte i = 0; i < 50; i++ )
  {
    delay(55);
    digitalWrite(PIN_LED,1);
    delay(55);
    digitalWrite(PIN_LED,0);
    if( !isOn() ) break;
  }
}

void setup()
{
  digitalWrite(PIN_PWR,0);
  pinMode(PIN_PWR,OUTPUT);
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED,1);
  Serial.begin(9600);
  Serial.println("setup()");
  #ifdef IRISR
  PCICR  |=  4; // enable PCIE2
  PCMSK2 |=  4; // enable PCINT18 --> Pin Chnage Interrupt of PD2
  #endif
  for( byte i = 0; i < 5; i++ )
  {
    digitalWrite(PIN_LED,0);
    delay(111);
    digitalWrite(PIN_LED,1);
    delay(111);
    digitalWrite(PIN_LED,0);
  }
}

void specialShow()
{
  irSendMsg(0,CMD_EAGLE);
  delay(4000);
  irSendMsg(0,CMD_AIRDRUMS);
  delay(15000);
  irSendMsg(0,CMD_HP); 
  irSendMsg(0,random(0,10)<5?CMD_CLINK:CMD_LOVE1);
}

//-----------------------------------------------------------------------------------

char key = 0;
byte cmd = 0;
byte par = 0;
byte par2 = 0;
byte num = 0;
byte cnt = 1;
unsigned lastCmdSecond=0;
unsigned secondsCount = 0;
char lastLight = 0;

void loop()
{
  {
    iSobotOn();
    byte newCmd = 0;
    unsigned nowSecond = millis()/1000;
    bool secondTick = secondsCount<nowSecond;
    if( secondTick )
    {
      secondsCount = nowSecond;
      unsigned batt = ((long)analogRead(PIN_BATT)*825)/(1024/8); // in [mV] from 2[divider]*3300[Vref]/1024[DAC full scale]
      unsigned gyro = analogRead(PIN_GYRO);
      char light = (25*(1023-analogRead(PIN_LIGHT)))/256; // 100/1025 = 25/256
      {
        Serial.print("batt=");Serial.print(batt);Serial.print("mV");
        Serial.print(" gyro=");Serial.print(gyro);
        Serial.print(" light=");Serial.print(light,DEC);;Serial.print('%');
        Serial.println();
      }
    
      if( secondsCount-lastCmdSecond  > 20 )
      {
        #if 0
        const byte s[] = {CMD_SQUAT,CMD_BEND_BACK,CMD_BEND_FORWARD,CMD_HEAD_LEFT_30,CMD_HEAD_RIGHT_30};
        byte cmd = s[random(0,sizeof(s)-1)];
        #else
        byte cmd = CMD_HP;
        #endif
        irSendMsg(0,cmd,0);
        lastCmdSecond = secondsCount;
      }

      {
        /*
        if( lastLight-light > 300 || lastLight-light < -300 )
        {
          Serial.print("light change ");Serial.println(lastLight-light);
          if( lastLight<light )
             iSobotOn();        
          else
             iSobotOff();        
          lastLight = light;
        }
        */
      }
            
    }
    lastCmdSecond = secondsCount;
    
    key = 0;
    if( Serial.available() )
      key = Serial.read();
    switch(key)
    {
      case 'i': 
      {
        isrDebug(); 
        unsigned long rcv = irRecv();
        Serial.println(rcv,HEX);
        Serial.print("batt  = ");Serial.println(analogRead(PIN_BATT));
        Serial.print("gyro  = ");Serial.println(analogRead(PIN_GYRO));
        Serial.print("light = ");Serial.println(analogRead(PIN_LIGHT));
        Serial.println(lastLight);
        break;
      }
      case 'o': {
        if( num ) iSobotOn();
        else      iSobotOff();
        num = 0;
        break;
      }
      case 'n':
      case '+': cmd++; newCmd=true; break;
      case 'p':
      case '-': cmd--; newCmd=true; break;
      case ' ': newCmd=true; break;
      case 'r': irSendMsg(0,CMD_MODE_RC); break;
      case 'h': irSendMsg(0,CMD_HP); break;
      case 'x': newCmd=1; break;
      case 'z': newCmd=2; break;
      case 'm': cnt = num; num=0; break;
      case 'a': par = num; num=0; break;
      case 'b': par2 = num; num=0; break;
      case 'c': cmd = num; num=0;  break;      
      case 's': specialShow(); break;
      case 'q': irSendMsg(0x30C2080); break;
      default:
        if( key >= '0' and key <= '9' )
        {
          num *= 10;
          num += key-'0';
        }
        else 
          if( key )
            Serial.println("n+p- rhxmds\n");
    }
    if( key )
    {
      Serial.println();
      Serial.print("num=");Serial.println(num,DEC);
      Serial.print("cmd=");Serial.println(cmd,HEX);
      Serial.print("par=");Serial.println(par,HEX);
      Serial.print("par2=");Serial.println(par2,HEX);
      Serial.print("cnt=");Serial.println(cnt,DEC);
    }
    if( newCmd )
    {
      for( int n = cnt; n > 0; n-- )
      {
          if( newCmd == 2 )
            irSendMsg(0,cmd,par,par2);
          else
            irSendMsg(0,cmd,par);
          newCmd = 0;
          delay(500);    
      }
    }
    unsigned long rcv = irRecv();
    if(rcv)
    {
      digitalWrite(PIN_LED,1);
      Serial.print("recv ");
      Serial.println(rcv,HEX);
      lastCmdSecond = secondsCount;
      digitalWrite(PIN_LED,0);
    }    
  }
}


