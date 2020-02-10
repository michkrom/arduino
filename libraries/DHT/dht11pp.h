#pragma once
//
// lean and mean DHT11 
//
// hand coded to run on nano 8MHz and 16MHz CPUs and ESP
// the code does not use any RAM storage
// pins are limited to 0..7 (PORTD) unless DHTREADPIN macro is defined appropriately before including
// on AVR 3v3 8MHz it is necessary to disable interrupts otherwise there is 50% failure rate
// (timing is derailed by interrupts)
//
// In general, the DHT11 interraction is self-timing, ie. it will correctly estimate the 0 and 1 timing and read the data.
//

// For AVR assume the DHTPIN is in D register which corresponds to 0..7 of arduino pins
// For ESP the DHTPIN is GPIO number (not Dx label numbers) see: https://randomnerdtutorials.com/esp8266-pinout-reference-gpios/

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
  #if !defined(DHTPINREG)
    #define DHTPINREG PIND
  #endif
  #if !defined(DHTREADPIN)
    #define DHTREADPIN(PIN) ( (DHTPINREG & (1<<PIN)) ? HIGH : LOW )
  #endif
#else // presumably much faster CPU (non Arduino/AVR so assume ESP)
  #if !defined DHTREADPIN
  #define DHTREADPIN(PIN) digitalRead(PIN)
  #endif
#endif


template<int DHTPIN>
class DHT
{ 
  public:
  
    struct Result { int rh; int temp; } result; 
  
  // complete sequence of reading data from DHT11
  // returns humidity + temperature in a struct
  // when error humidity is set < 0 (-2 for timeout; -1 for checksum)
  static Result read() {  
    // initiate transfer by asserting pin LOW for 20ms
    pinMode(DHTPIN, OUTPUT);
    digitalWrite(DHTPIN, LOW);
    delay(20); // ms
    uint8_t b1,b2,b3,b4,b5;
    {
      // need to disable interrupts as on 8MHz CPU as these will disrupt timing  
      InterruptLock il;
      // pull up and wait for DHT1 to bring it up
      pinMode(DHTPIN, INPUT);
      // watch for reply from DHT
      // initial pulse is rather short, just make sure it happens
      measure(LOW); // wait for DHT to reply by pulling pin HIGH
      measure(HIGH); // wait for DHT to drop to LOW again
      // next pulse is long with precise timing
      auto t1 = measure(LOW); // wait for DHT asserting a long LOW
      auto t2 = measure(HIGH); // wait for DHT asserting long HIGH
      if(t1==TIMEOUT||t2==TIMEOUT) return Result{-2,0}; // timeout reading initial response?
      //.. and then the first bit starts...
      b1 = readByte(); // RH integral
      b2 = readByte(); // RH fractinal
      b3 = readByte(); // T integral
      b4 = readByte(); // T fractional
      b5 = readByte(); // check sum
    }
    pinMode(DHTPIN, OUTPUT);
    digitalWrite(DHTPIN, HIGH);
    
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
    // b1 (RH: 0..100) would never be > 100 so it will never have b7 set (hence the sign bit is always 0)
    // b3 (Temp) may be negative (it's int8_t)
    auto checkOk = ((b1+b2+b3+b4) & 0xFF) == b5;
    
    return Result{ checkOk ? (int)b1 : -1, (int)b3};
  }

private:

  class InterruptLock {
    public: 
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
    InterruptLock() { noInterrupts(); }
    ~InterruptLock() { interrupts(); }
#else 
    // presumably much faster CPU (non Arduino/AVR so assume ESP) so no need to do anything
#endif
  };
  

  static int8_t readPin() { 
    return DHTREADPIN(DHTPIN); 
  }

  // this timeout works even on slow 8MHz arduino and on ESP8266 
  // albeit real time is differnt due to CPU speed difference
  static const int TIMEOUT = 0xFFFF;
  
  // measure how many 'loops' the pins is in the specified level
  // maxes out at 0xFFFF which should be considered a timeout
  static uint16_t measure(char level) {
    uint16_t t=0;
    while( readPin() == level && ++t < TIMEOUT );
    return t;
  }
  
  // read single bit sequence
  // starts at LOW and measures its duration until HIGH (~40us)
  // then measures HIGH  (30us for 0 and 70us for 1) until change
  // returns 1 if t(high) > t(low) else 0
  static int8_t readBit() {  
    uint16_t lc = measure(LOW);
    uint16_t hc = measure(HIGH);
    return hc > lc ? 1 : 0; // the LOW shoul be be about 40us followd by 1 at ~30us for 0 and ~70us for 1
  }
  
  
  // reads the whole byte ie 8x readBit
  static uint8_t readByte() {
    uint8_t b; // all bits replaced so no need to init it
    for(char i = 0; i < 8; i++) {
      b <<= 1;
      b |=readBit();
    }
    return b;
  }

};
