#ifndef __CC2500_INCLUDED
#define __CC2500_INCLUDED

#include <SPI.h>

#include "cc2500_REG.h"

// generic print in hex with leading zeros
template<class T> void phex(T v)
{
  for(int n = 2*sizeof(v)-1; n >= 0; --n)
  {
    byte nibble = (byte)((v>>(4*n)) & 0xF);
    nibble |= 48;
    if( nibble > 57 ) nibble += 39;
    Serial.print((char)nibble);
  }
}

#define C2500_SPI_SS 10 // RFCS (chip select)

#define C2500_SPI_CLKFREQ 4000000 // SPI clock frequency

// strobe commands
#define STROBE_SRES       0x30 
#define STROBE_SFSTXON    0x31 
#define STROBE_SXOFF      0x32 
#define STROBE_SCAL       0x33 
#define STROBE_SRX        0x34 
#define STROBE_STX        0x35 
#define STROBE_SIDLE      0x36 
#define STROBE_WORTIME0   0x37 
#define STROBE_SWOR       0x38 
#define STROBE_SPWD       0x39 
#define STROBE_SFRX       0x3A 
#define STROBE_SFTX       0x3B 
#define STROBE_SWORRST    0x3C 
#define STROBE_SNOP       0x3D 
//0x3E PATABLE
//0x3F FIFO

class CC2500 {
  
public:
  // initialize SPI and cc2500
  void begin()
  {
    digitalWrite(C2500_SPI_SS, HIGH);
    pinMode(C2500_SPI_SS, OUTPUT);
    SPI.begin();
    
    // preconfigure SPI and soft-reset the CC2500
    SPI.beginTransaction(SPISettings(C2500_SPI_CLKFREQ, MSBFIRST, SPI_MODE0));
    sStrobe(STROBE_SRES);
    SPI.endTransaction();    
  }
  
  // write value to a register
  void wReg(byte addr, byte val)
  {
    digitalWrite(SS, LOW);
    while(digitalRead(MISO)==HIGH) ;
    SPI.transfer(addr);
    SPI.transfer(val);
    digitalWrite(SS, HIGH);
  }

  // read value from a register
  byte rReg(byte addr)
  {
    addr = addr + 0x80;
    digitalWrite(SS, LOW);
    while(digitalRead(MISO)==HIGH);
    SPI.transfer(addr);
    byte rd = SPI.transfer(0);
    digitalWrite(SS,HIGH);
    return rd;  
  }

  // send a strobe command (see datasheet)
  void sStrobe(byte cmd)
  {
  // strobes are commands issued by addressing a specific address in write mode
    digitalWrite(SS, LOW);
    while(digitalRead(MISO)==HIGH);
    SPI.transfer(cmd);
    digitalWrite(SS, HIGH);  
  }

  // prints values of all registers to serial port
  void dumpRegs()
  {
    for(byte i = 0; i < 0x30; i++)
    {
      if( (i & 0x7) == 0 ) { phex(i); Serial.print(": "); }
      auto x = rReg(i);
      phex(x);
      Serial.print(' ');
      if( (i & 0x7) == 0x7 ) Serial.println();
    }
  }
  
  // write registers from a [register, value] array
  // "byte regval[]" contains a list of [reg1,val1, reg2, val2....]
  // arraySize is size of array in bytes ie. sizeof(regval)/sizeof(*regval)
  void initFromRegValArray(byte regval[], size_t arraySize)
  {
	  for(int n = 0; n < arraySize; n += 2)
	  {
		  wReg(regval[n], regval[n+1]);
	  }
  }
};

#endif
