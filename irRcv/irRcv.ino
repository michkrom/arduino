/*
 *  File....... IRcv.pde
 *  Purpose.... IR decoder for SYMA S107G, PROTOCOL TraceJet and U810 HELIcopters remote codes.
 *  Author..... Michal Krombholz
 *  E-mail..... krombholz@gmail.com
 *  Started.... 01/12/2013
 *  Updated.... 01/01/2014
 *
 *
 */

#define IRISR
#define ATTACHISR

#ifndef ATTACHISR
#include <avr/interrupt.h>
#include <avr/io.h>
#endif

#include "irRcv.h"

/**
 * IR decoder for SYMA S107G, PROTOCOL TraceJet and U810 HELIcopters remote controls.
 *
 * @author Michal Krombholz
 * @license Creative Commons Attribution-ShareAlike 3.0 (CC BY-SA 3.0)
 */

#define PIN_IR 2
#define PIN_LED 13

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


inline
int irDetected()
{
  return (PIND & (1<<PIN_IR)) == 0;
}

#ifdef ATTACHISR
void irIsr()
#else
ISR( PCINT2_vect )
#endif
{  
 // digitalWrite(13,1);
  // PIN_IR == #2 --> PD2;
  // receiving a modulated IR signal makes the pin go low (active low)
//  byte transmitting = irDetected();
//  if( transmitting )

  //if( irRcvErrorCount ==  0 )
  {    
    // compute elapsed time since last change
    static unsigned long isrLastTimeStamp;
    unsigned long elapsed;
    {
      #if 1
      unsigned long timeStamp = micros();
      elapsed = elapsedSince(isrLastTimeStamp, micros());
      #else  //--> faster yet do it inline
      // copied from wiring.c - external time counters updated in TCNT0 overflow ISR
      extern volatile unsigned long timer0_overflow_count;
      unsigned long timeStamp = ((timer0_overflow_count) << 8) | TCNT0;
      elapsed = timeStamp > isrLastTimeStamp ? timeStamp-isrLastTimeStamp : 0xFFFFFFFFUL - (isrLastTimeStamp-timeStamp);
      #endif
      isrLastTimeStamp = timeStamp;
      irRcvReportIRDetected( elapsed );
    }    
  }
  
  //digitalWrite(13,0);
}


void setup() 
{
  Serial.begin(115200);
  Serial.println("IR Receiver for RC helicopter remotes.");
  pinMode(PIN_IR, INPUT);
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED,1);
  
  irRcvInitProtocol(1);

#ifdef IRISR
#ifdef ATTACHISR

#ifdef IR_EDGE_RISING
  attachInterrupt(0, irIsr, RISING);
#else
  attachInterrupt(0, irIsr, FALLING);
#endif

#else  
  PCICR  |=  PCIE2;    // enable PCIE2
  PCMSK2 |=  PCINT18; // enable PCINT18 --> Pin Chnage Interrupt of PD2  
#endif
// PIN2 PIND	PCIE2	PCMASK2	PCINT18	PCINT2_vect  
#endif
  
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////


// Motor RL FB
#define PIN_MLF 3
#define PIN_MLB 9
#define PIN_MRF 11
#define PIN_MRB 10

void setMotor(int drive, char pinF, char pinB)
{
  if( drive > 0 )
  {
    digitalWrite( pinF, 0 );
    analogWrite( pinB, drive );
  }
  else
  {
    analogWrite( pinF, -   drive );
    digitalWrite( pinB, 0 );
  }  
}

// left/right range -255..255
void setMotors(int left, int right)
{
  // using pololu motor controller
  setMotor(left, PIN_MLF, PIN_MLB);
  setMotor(right, PIN_MRF, PIN_MRB);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////


void printbin(uint32_t val, int startbit, int bitcount)
{
    while( --bitcount >= 0 )
    {
        Serial.print( (val >> (startbit ++) ) & 1  ? '1' : '0');
    }
}


int  loopcnt = 0;

void loop()
{
    static unsigned long lastCmdTime;
#if 0
    static byte ledcnt;
    if( irDetected() )
      digitalWrite(PIN_LED,(++ledcnt)&1);
    return;
#endif      
  
        // must protect access to cmd against the isr, hence diable interrupts
        // otherwise partial commands may be returned
        cli();
        uint32_t cmd = irRcvGetCommand();
        sei();
        
        if( cmd != 0 ) 
        {
            lastCmdTime = micros();
            digitalWrite(PIN_LED,1);
            unsigned th = cmd >> 16;
            unsigned tl = cmd & 0xFFFF;
            //Serial.print(th,HEX);
            //Serial.print(tl,HEX);
#if 0
//          printf(" %b %b ",th,tl);
            printbin(cmd, 24, 8);
            printf(" ");
            printbin(cmd, 16, 8);
            printf(" ");
            printbin(cmd,  8, 8);
            printf(" ");
            printbin(cmd,  0, 8);
#else
            byte pwr = IR_CMD_PWR(cmd);
            int lr = IR_CMD_LR(cmd);
            int fb = IR_CMD_FB(cmd);
            char ch =  IR_CMD_CH(cmd);
            byte trm = IR_CMD_TRM(cmd);
            byte sum = IR_CMD_SUM(cmd);
            byte sum2 = IR_CMD_COMPUTE_SUM(cmd);
            byte btnL = IR_CMD_BTNL(cmd);
            byte btnR = IR_CMD_BTNR(cmd);
            byte btnM = IR_CMD_BTNM(cmd);
#if 0
            Serial.print(" pwr=");Serial.print(pwr,DEC);
            Serial.print(" fb=");Serial.print(fb-8,DEC);Serial.print(" lr=");Serial.print(lr-8,DEC);
            Serial.print(" ch=");Serial.print(ch,DEC);
            Serial.print(" trm=");Serial.print(trm,DEC);
            //Serial.print((" btn=%i%i%i",btnL,btnM,btnR);
            //Serial.print((" sum=%X %X %s",sum,sum2, IR_CMD_VALID(cmd)?"OK":"err");
            Serial.print(IR_CMD_VALID(cmd)?" ok":" ERR");            
#endif            
#if 1
            // steer motors
            // directions of motors vs fb & lr
            // fb/lr   >> >< <> << 00 0> 0< >0 <0
            // LM      >  >  <  <  0  >  <  >  <
            // RM      >  >  <  <  0  <  >  >  <
            
            // directions of motors vs fb & lr (simplified)
            // fb/lr   >* <* 00 0> 0< 
            // LM      >  <  0  >  <
            // RM      >  >  0  <  >  
            

            // adjust for neutral position            
            fb -= 8;
            lr -= 8;
            
            // l&r motors; max value of 7+7=14
            int lm = fb + lr;
            int rm = fb - lr;
            // apply power coeff
            long p = ( pwr < 128 ) ? pwr : 0;
            lm *= p;
            rm *= p;
            // max(l) = max(fb+lr)*max(p)=16*128 = 9x10*0x7f = 0x7f0 hence
            // const int MAXM = 16*128;
            lm >>= 2;
            rm >>= 2;
            Serial.println();
            Serial.print(lm);Serial.print(" ");Serial.print(rm);
            Serial.println();

            setMotors(lm,rm);            
#endif
#endif
            Serial.println(" ");
            digitalWrite(PIN_LED,0);
        }
        else
        {
            if( irRcvErrorCount > 0 )
            {
              extern volatile char irRcvErrorBit;
              extern volatile uint32_t irRcvErrorDelta;

              Serial.print("E ");
              Serial.print(irRcvErrorDelta,DEC);
              Serial.print(" @ ");
              Serial.println(irRcvErrorBit,DEC);              
              
              irRcvErrorCount = 0;
            }
            else
              if(elapsedSince(lastCmdTime,micros())>1000000)
                 setMotors(0,0);
        }
}


