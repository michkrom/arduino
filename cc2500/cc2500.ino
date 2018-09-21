#include <SPI.h>

#include "cc2500.h"

#include "cc2500_init_regval.h"

///////////////////////////////////////////////////////

CC2500 cc2500;

// test roundabout spi comm to the chip
// writes to registers and reads back
int testSPIcoms()
{
  int errorCount = 0;
  for(int n = 0; n <= 256; ++n )
  {
    cc2500.wReg(REG_ADDR,n);
    cc2500.wReg(REG_SYNC1,n);
    auto r1 = cc2500.rReg(REG_ADDR);
    auto r2 = cc2500.rReg(REG_SYNC1);
    if( r1 != n || r2 != n )
    {
      ++errorCount;
    }
  }
  return errorCount;
}

void setup() {
  Serial.begin(9600);
  Serial.print("CC2500 tests\r\n");
  cc2500.begin();
  auto errs = testSPIcoms();
  Serial.print(errs==0 ? "SPI OK" : "SPI comms failed with errors");
}

void loop() {
  Serial.println();    
  cc2500.dumpRegs();
  Serial.println();    
}
