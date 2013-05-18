/*
 *  File....... IRanalyzer.pde 
 *  Purpose.... Records up to 128 signal changes
 *  Author..... Walter Anderson 
 *  E-mail..... wandrson@walteranderson.us 
 *  Started.... 18 May 2007 
 *  Updated.... 18 May 2007 
 * 
 *
 */
#include <avr/interrupt.h>
#include <avr/io.h>

const int IRpin = 2;
const int SAMPLE_SIZE = 128;
const int TCNT_MAX = 25000; // 100ms
unsigned int TimerValue[SAMPLE_SIZE];

void setup() {
  Serial.begin(115200);
  Serial.println("Capture IR Remote");
  TCCR1A = 0x00;          
  // COM1A1=0, COM1A0=0 => Disconnect Pin OC1 from Timer/Counter 1
  // PWM11=0,PWM10=0 => PWM Operation disabled
  // ICNC1=0 => Capture Noise Canceler disabled 
  // ICES1=0 => Input Capture Edge Select (not used)
  // CTC1=0 => Clear Timer/Counter 1 on Compare/Match
  // CS12=0 CS11=1 CS10=1 => Set prescaler to clock/64
  TCCR1B = 0x03;          // 16MHz clock with prescaler means TCNT1 increments every 4uS
  // ICIE1=0 => Timer/Counter 1, Input Capture Interrupt Enable
  // OCIE1A=0 => Output Compare A Match Interrupt Enable
  // OCIE1B=0 => Output Compare B Match Interrupt Enable
  // TOIE1=0 => Timer 1 Overflow Interrupt Enable
  TIMSK1 = 0x00;          
  pinMode(IRpin, INPUT);
}

int  loopcnt = 0;

void loop()
{
  loopcnt++;
  Serial.print(loopcnt,DEC);
  Serial.print("> ");
  byte change_count = 0;
  byte first_state = digitalRead(IRpin);
  byte curr_state = first_state;
  while(digitalRead(IRpin) == curr_state) ;
  TCNT1=0;// TIMER_RESET
  curr_state = (curr_state == LOW ? HIGH : LOW);
  do {
    while(digitalRead(IRpin) == curr_state) 
    {
      if( TCNT1 > TCNT_MAX ) goto end;   
    }
    TimerValue[change_count++] = TCNT1;
    curr_state = (curr_state == LOW ? HIGH : LOW);
  } 
  while( change_count <= SAMPLE_SIZE );

end:
  if( change_count > 0 )
  {
    #ifdef DUMP
    Serial.print("Captured: ");
    Serial.println((long)change_count);
    long prevtime=0;
    for(byte change = 0; change < change_count; change++ )
    {
      long time = (long) TimerValue[change] * 4;
      Serial.print(time);
      Serial.print("\t");
      Serial.print(time-prevtime);
      Serial.print("\t");
      prevtime=time;
      Serial.println(change & 0x1 == 0 == first_state != HIGH ? "1->0" : "0->1");
    }
    Serial.println("End");
    #endif
    // decode the stream specific for iSobot
    byte cnt0 = 0;
    byte cnt1 = 0;
    byte cnt01 = 0;
    unsigned long num = 0;
    for(byte change = 1; change < change_count; change++ )
    {
      if( (change & 0x1) == 1 )
      {
        num <<= 1;
        int delta = TimerValue[change]-TimerValue[change-1];
        if( delta > 750/4 ) 
        {
          cnt1++;
          num++;
          Serial.print('1');
        }
        else
        {
          cnt0++;
          Serial.print('0');
        }
        if( change == 1 || change == 5 || change == 11 || change == 27 ) Serial.print('.');
        cnt01++;
      }
    }
    //Serial.print(num,BIN);
    Serial.print(" ");
    Serial.print(num >> 21 ? 'B' : 'A');
    Serial.print(' ');
    Serial.print((num >> 20) & 0x1,DEC);
    Serial.print((num >> 19) & 0x1,DEC);
    Serial.print(' ');
    Serial.print((num >> 18) & 0x1,DEC);
    Serial.print((num >> 17) & 0x1,DEC);
    Serial.print((num >> 16) & 0x1,DEC);
    unsigned int cmd = num & 0xFFFF;
//    unsigned int sum = (cmd >>12) ^ ((cmd >> 8) & 0xF) ^ ((cmd >> 4) & 0xF) ^ (cmd & 0xF); 
//    unsigned int sum = (cmd >>12) + ((cmd >> 8) & 0xF) + ((cmd >> 4) & 0xF) + (cmd & 0xF);
    unsigned int sum = 0;
    unsigned int n = cmd;
    while( n > 0 )
    {
      sum += n & 7;
      n >>= 3;
    }
    sum += sum >> 3;
    sum += 5;
    Serial.print('=');
    Serial.print((num >> 16) & 0x7,DEC);
    Serial.print(" ? ");
    Serial.print((sum >> 2) & 0x1,DEC);
    Serial.print((sum >> 1) & 0x1,DEC);
    Serial.print((sum >> 0) & 0x1,DEC);
    Serial.print('=');
    Serial.print(sum & 0x7,DEC);    
    Serial.print('=');
    Serial.print(sum,BIN);    
    Serial.print(' ');
    Serial.print(cmd,HEX);
    Serial.print(" #0=");Serial.print((int)cnt0);
    Serial.print(" #1=");Serial.print((int)cnt1);
    Serial.print(" #B=");Serial.print((int)cnt01);
    Serial.println("");
  }
  else
  {
    Serial.print((long)TCNT1);
    Serial.println(" timeout!");
  }
}
