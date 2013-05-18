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

const int pinIn = 2;
const int SAMPLE_SIZE = 128;
const int TCNT_MAX = (104/4)*(8+2+1)*3;// 25000; // 100ms
unsigned int TimerValue[SAMPLE_SIZE];

void setup() {
  Serial.begin(115200);
  Serial.println("Capture Data on Pin#2");
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
  pinMode(pinIn, INPUT);
}

int  loopcnt = 0;

char awaitIdle()
{
  do 
  {
    while(digitalRead(pinIn) != LOW) ;
    TCNT1=0;// TIMER_RESET
    while(digitalRead(pinIn) == LOW) ;
  } while( TCNT1 < 5*(2+8+1)*104/4 );
}

char readByte()
{
  // must start with 0 to wait for 2 bits of start of 1
  while(digitalRead(pinIn) != LOW) ;
  while(digitalRead(pinIn) == LOW) ;
  TCNT1=0;//reset timer to measure duration of 2 start bits
  while(digitalRead(pinIn) == HIGH) ;
  const int bitTime = 104/4;
  // start reading bits
  byte b = 0;
  // waiting half bit time
  TCNT1=0; 
  while( TCNT1 < bitTime/2 );
  // start reading bits
  for( int n = 0; n < 8; n++ )
  {
     TCNT1=0; 
     b >>= 1;
     b |= digitalRead(pinIn) == HIGH ? 0 : 0x80 ;
     while( TCNT1 < bitTime );
  }
  return b;
}

void loop2()
{
  awaitIdle();
  while(readByte()!=13);
  char c1 = readByte();
  char c2 = readByte();
  char c3 = readByte();
  char c4 = readByte();
  char c5 = readByte();
  Serial.print(c1);  
  Serial.print(c2);  
  Serial.print(c3);  
  Serial.print(c4); 
  Serial.print(c5); 
  Serial.print('\n'); 
}

void loop1()
{
  loopcnt++;
  Serial.print(loopcnt,DEC);
  Serial.print("> ");
  byte change_count = 0;
  byte first_state = digitalRead(pinIn);
  byte curr_state = first_state;
  while(digitalRead(pinIn) == curr_state) ;
  TCNT1=0;// TIMER_RESET
  curr_state = (curr_state == LOW ? HIGH : LOW);
  do {
    while(digitalRead(pinIn) == curr_state) 
    {
      if( TCNT1 > TCNT_MAX ) goto end;   
    }
    TimerValue[change_count++] = TCNT1;
    curr_state = (curr_state == LOW ? HIGH : LOW);
  } 
  while( change_count <= SAMPLE_SIZE && TCNT1 <= TCNT_MAX );

end:
  if( change_count > 0 )
  {
    Serial.print("Captured: ");
    Serial.println((long)change_count);
    long prevtime=0;
    curr_state = first_state;
    for(byte change = 0; change < change_count; change++ )
    {
      long time = (long) TimerValue[change] * 4;
      Serial.print(time);
      Serial.print("\t");
      Serial.print(time-prevtime);
      Serial.print("\t");
      prevtime=time;
      curr_state = (curr_state == LOW ? HIGH : LOW);
      Serial.println(curr_state == LOW ? '0' : '1');
    }
    Serial.println("End");
  }
  else
  {
    Serial.print((long)TCNT1);
    Serial.println(" timeout!");
  }
}

int prevDist = -1;
void loop()
{
      while(digitalRead(pinIn) == LOW) ;
      TCNT1 = 0;
      while(digitalRead(pinIn) == HIGH) ;
      long timeus = TCNT1 * 4;
      int distance = timeus / 147;      
      if( prevDist != distance )
      {
        Serial.println(distance);
        prevDist = distance;
      }
}
