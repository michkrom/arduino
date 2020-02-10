// Simple DHT11 test program.

#include "dht11.h"

///////////////////////////////////////////////////////////////////////

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  Serial.println("Lean&Mean DTH11 start");
}

// uncomment for debugging
//#define DOTIMES 1 
#if DOTIMES

// records up to N times of transitions and outputs them
// expects DHT to start correctly
// used for debugging
void doTimes() {
  const int8_t N = 100;
  int8_t times[N];
  pinMode(DHTPIN, OUTPUT);
  digitalWrite(DHTPIN, LOW);
  delay(20); // ms
  noInterrupts();
  pinMode(DHTPIN, INPUT);
  for(int8_t i = 0; i < N; ++i){
    uint16_t t = measure(i & 1 ? LOW : HIGH); // wait for low from DHT
    if( t > 255 ) t = 255;
    times[i]=t;
  }
  interrupts();
  for(int i = 0; i < N; i++){
    Serial.print((int)times[i]);
    Serial.print(' ');
    if( (i % 16) == 15 ) Serial.println();
  }
  Serial.println();
}

void loop() {  
  doTimes();
  delay(2000);
}

#else

void loop() {  
  digitalWrite(LED_BUILTIN,1);
  int16_t r = readDHT();
  if(r < 0) 
    Serial.println(r);
  else {
    Serial.print(r >> 8); Serial.print("RH ");
    Serial.print(r & 0xFF); Serial.println("C");
  }
  digitalWrite(LED_BUILTIN,0);
  delay(2000);
}

#endif
