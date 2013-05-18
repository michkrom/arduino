/*
 * AnalogInput
 * by DojoDave <http://www.0j0.org>
 *
 * Turns on and off a light emitting diode(LED) connected to digital  
 * pin 13. The amount of time the LED will be on and off depends on
 * the value obtained by analogRead(). In the easiest case we connect
 * a potentiometer to analog pin 2.
 *
 * http://www.arduino.cc/en/Tutorial/AnalogInput
 */

void setup() {
  for(byte n = 8; n <= 13; n++ ) 
    pinMode(n,OUTPUT);
}

void leds(byte pat)
{
  for( byte n = 0; n < 6; n++ )
  {
    digitalWrite(8+n, pat & 1 ? HIGH : LOW);
    pat >>= 1;
  }  
}

// 1 2 4 8 16 32
const byte graph[] = { 
//  0, 4+8, 4+8, 2+32, 1+32, 2+32, 4+8, 4+8, 0, 0, 0, 
 0, 0, 63, 4, 2+8, 1+16+32, 0, 0, 255-1, 1+8, 1+8, 255-1, 0, 0, 1+32, 63, 1+32, 0, 0 /*  KAI */
//  0, 4+8, 4+8, 2+32, 1+32, 2+32, 4+8, 4+8, 0, 0, 0, 
};



void loop() 
{
  byte cur = 0;
  while(1)
  {
    leds(graph[cur]);
    if( cur == sizeof(graph)/sizeof(graph[0]) ) 
      cur = 0;
    else
      cur++;
    delay(1);
  }
}
