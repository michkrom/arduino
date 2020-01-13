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
//

// pin in D register correspond to 0..7 of arduino pins or ESP's GPIO#
#ifndef DHTPIN
#define DHTPIN 7
#endif

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
  #define RDPIN ((PIND & (1<<DHTPIN)) ? HIGH : LOW)
  class InterruptLock {
    public: 
    InterruptLock() { noInterrupts(); }
    ~InterruptLock() { interrupts(); }
  };
#else // presumably something much faster
  #define RDPIN (digitalRead(DHTPIN))
  class InterruptLock {};
#endif

// this timeout works even on slow 8MHz arduing and on ESP8266 
// albeit real time is differnt due to CPU speed
#define TIMEOUT 0xFFFF

// measure how many 'loops' the pins is in the specified level
// maxes out at 0xFFFF which should be considered a timeout
inline
uint16_t measure(char level)
{
  uint16_t t=0;
  while( RDPIN == level && ++t < TIMEOUT );
  return t;
}

// read single bit sequence
// starts at LOW and measures its duration until HIGH (~40us)
// then measures HIGH  (30us for 0 and 70us for 1) until change
// returns 1 if t(high) > t(low) else 0
inline 
int8_t readBit() {  
  uint16_t lc = measure(LOW);
  uint16_t hc = measure(HIGH);
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

// the whole sequence of reading data from DHT11
// returns humidity << 16 + temperature or < 0 on error
int16_t readDHT() {
  // initiate transfer by asserting pin LOW for 20ms
  pinMode(DHTPIN, OUTPUT);
  digitalWrite(DHTPIN, LOW);
  delay(20); // ms
  uint8_t b1,b2,b3,b4,b5;
  // pull up and wait for DHT1 to bring it up
  {
    // need to disable interrupts as on 8MHz CPU as these will disrupt timing  
    InterruptLock il;
    // watch for reply from DHT
    pinMode(DHTPIN, INPUT);
    // initial pulse is rather short, just make sure it happens
    measure(LOW); // wait for DHT to reply by pulling pin HIGH
    measure(HIGH); // wait for DHT to drop to LOW again
    // next pulse is long with precise timing
    auto t1 = measure(LOW); // wait for DHT asserting a long LOW
    auto t2 = measure(HIGH); // wait for DHT asserting long HIGH
    if(t1==TIMEOUT||t2==TIMEOUT) return -2; // timeout reading initial response?
    //.. and then the first bit starts...
    b1 = readByte(); // RH integral
    b2 = readByte(); // RH fractinal
    b3 = readByte(); // T integral
    b4 = readByte(); // T fractinal
    b5 = readByte(); // check sum
    pinMode(DHTPIN, OUTPUT);
    digitalWrite(DHTPIN, HIGH);
  }
  
  #if DHT_DEBUG
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
  return ((b1+b2+b3+b4) & 0xFF) == b5 
    ? (((int16_t)b1) << 8) | b3 
    : -1;
}

#endif
