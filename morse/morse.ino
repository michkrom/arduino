/*
  Blink Morse SOS
*/

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
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
}
