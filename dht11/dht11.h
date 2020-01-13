#ifndef DHT11_LnM
#define DHT11_LnM
//
// lean and mean DHT11 
//
// hand coded to run on nano 8MHz and 16MHz CPUs
// none of the code uses any RAM storage
// pins are limited to 0..7 (PORTD) unless RDPIN macro is changed appropriately
// on 3v3 8MHz it is necessary to disable interrupts otherwise there is 50% failure rate 
// (timing is derailed by interrupts)
// the DHT11 interraction is self-timing, ie it will correctly estimate the 0 and 1 timing

// pin in D register correspond to 0..7 of arduino pins
#ifndef DHTPIN
#define DHTPIN 7
#endif

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
#define RDPIN ((PIND & (1<<DHTPIN)) ? HIGH : LOW)
#define TIMEOUT 1000
#else // presumably something much faster
#define RDPIN (digitalRead(DHTPIN))
#define TIMEOUT 10000
#endif


// measure how many 'loops' the pins is in the specified level
// maxes out at 0xFFFF which should be considered a timeout
inline
uint16_t measure(char level)
{
  uint16_t t=0;
  while( RDPIN == level && ++t < 0xFFFF );
  return t;
}

// read single bit sequence
// starts at LOW and measures its duration untill HIGH (~40us)
// then measures HIGH  (30us for 0 and 70us for 1)
// returns 1 if t(high) > t(low) else 0
inline 
int8_t readBit() {  
  uint16_t lc = 0;
  while(RDPIN==LOW && ++lc < TIMEOUT); // count LOW state
  uint8_t hc = 0;
  while(RDPIN==HIGH && ++hc < TIMEOUT); // count HIGH state
  return hc > lc ? 1 : 0; // the LOW shoul be be about 40us followd by 1 at ~30us for 0 and ~70us for 1
}


// reads the whole byte ie 8x readBit
inline
uint8_t readByte() {
  uint8_t b; // all bits replaced so no need to init it
  for(char i = 0; i < 8; i++) {
    b <<= 1;
    b |=readBit();
  }
  return b;
}

// whole sequence of reading data from DHT11
// return humidity << 16 + temperature or -1 on error
int16_t readDHT() {
  // initiate transfer by asserting pin LOW for 20ms
  pinMode(DHTPIN, OUTPUT);
  digitalWrite(DHTPIN, LOW);
  delay(20); // ms
  // pull up and wait for DHT1 to bring it up
  noInterrupts(); // need to disable interrupts as on 8MHz CPU these will disrupt timing
  pinMode(DHTPIN, INPUT);
  // initial pulse is rather short
  measure(LOW); // wait for DHT to reply by pulling pin HIGH
  measure(HIGH); // wait for DHT to drop to LOW again
  // next pulse is long
  auto t1 = measure(LOW); // wait for DHT to assert a long LOW
  auto t2 = measure(HIGH); // wait for DHT to assert long HIGH ...  
  //.. and then the first bit starts...
  auto b1 = readByte(); // RH integral
  auto b2 = readByte(); // RH fractinal
  auto b3 = readByte(); // T integral
  auto b4 = readByte(); // T fractinal
  auto b5 = readByte(); // check sum
  pinMode(DHTPIN, OUTPUT);
  digitalWrite(DHTPIN, HIGH);
  interrupts();
  #if 0 // for debugging timing
  Serial.print(b1);
  Serial.print(' ');
  Serial.print(b2);
  Serial.print(' ');
  Serial.print(b3);
  Serial.print(' ');
  Serial.print(b4);
  Serial.print(' ');
  Serial.print(b5);
  Serial.print(' ');
  Serial.print((b1+b2+b3+b4) & 0xFF);
  Serial.println();
  #endif
  // check if the checksum matches and return the result
  return ((b1+b2+b3+b4) & 0xFF) == b5 ? (((int16_t)b1) << 8) | b3 : -1;
}
#else
extern int16_t readDHT();
#endif
