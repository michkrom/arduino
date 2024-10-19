/*
  Blink Morse SOS
*/

#ifndef LED_BUILTIN
#define LED_BUILTIN 23 // ESP32 WROOM 4 relay board
#endif


// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
}

void nextRelayState()
{
  static const int relayPins[] = { 32, 33, 25, 26 };
  static auto pinNum = 0;
  static auto pinState = 0;
  const auto pin = relayPins[pinNum++];
  pinMode(pin, OUTPUT);
  digitalWrite(pin, pinState & 1);
  if( pinNum > sizeof(relayPins)/sizeof(relayPins[0]) )
  {
    pinNum = 0;
    pinState++;
  }

    
}

// the loop function runs over and over again forever
void loop() {
  constexpr auto speed = 250;
  int code[] = {2,2,2,0,1,1,1,0,2,2,2}; // S.O.S
  for(auto i = 0; i < sizeof(code)/sizeof(code[0]); i++)
  {
    auto symbol = code[i];
    if(symbol > 0)
    {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(speed*(symbol));
      digitalWrite(LED_BUILTIN, LOW); 
    }
    delay(speed);

  }
  delay(speed*4);
  nextRelayState();
}
