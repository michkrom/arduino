/*
 * Pin/PWM tester
 * by Michal Krombholz
 *
 * Allows testing of pins and pwm from serial console input:
 * dN V sets pin N to value V
 * aN V sets pin N to PWM value of V
 * dN reads state of pin
 * aN reads state of pin (analog)
 * e.g
 * to set pin13=1 (led): d13 1
 * to set pin10=128/255 (pwm=0.5) a10 128
 * to read state of pin just send first part e.g. p13\r or a1\r
 * (set serial console to  send \r (CR))
 * (note all other characters ignored so dx1x3x x1x\r would be the same as d13 1\r)
 */

void showHelp()
{
  Serial.println("PinTester:");
  Serial.println("d<N> <value> - sets digital pin N to value V");
  Serial.println("a<N> <value> - sets analog pin N to value V");
  Serial.println("command without V reads the pin state, digital or analog");
  Serial.println("ADC reference: I-internal, D-default, E-external");
  Serial.println("ARDUINO specific:");
  Serial.println("v - Vcc against internal reference (assuming it's 1.1V).");
  Serial.println("t - internal temp internal reference (assuming it's 1.1V).");
  Serial.println("i - for measuing internal reference against Vcc.");
}


void setup()
{
  // begin the serial communication
  Serial.begin(9600);
  showHelp();
}


long readTemp() { 
  // Read temperature sensor against 1.1V reference
  #if defined(__AVR_ATmega32U4__)
    ADMUX = _BV(REFS1) | _BV(REFS0) | _BV(MUX2) | _BV(MUX1) | _BV(MUX0);
    ADCSRB = _BV(MUX5); // the MUX5 bit is in the ADCSRB register
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(REFS1) | _BV(MUX5) | _BV(MUX1);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(REFS1) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1) | _BV(MUX0);
  #else
    ADMUX = _BV(REFS1) | _BV(REFS0) | _BV(MUX3); // interlan temp and internal ref
  #endif

  delay(20); // Wait for ADMUX setting to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both
  long result = (high << 8) | low; // combine the two
  return result;
}

const float IntRefV = 1.089; // as measured

// measuring Vcc with Int Ref
// Dac@Vcc=1024 --> Vcc
// Dac@IntRef   --> IntRefV (measured)
// hence:
// 1024/Vcc = Dac@IntRef/IntRefV
// Vcc = 1024*IntRefV/Dac@IntRefV

// from https://code.google.com/archive/p/tinkerit/wikis/SecretVoltmeter.wiki
float readVcc() 
{ 
  // Read 1.1V reference against Vcc 
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1); 
  delay(20);
  // Wait for Vref to settle 
  ADCSRA |= _BV(ADSC); 
  // Convert 
  while (bit_is_set(ADCSRA,ADSC)); 
  long intrefdac = ADCL;
  intrefdac |= ADCH<<8;
  auto vcc = 1024 * IntRefV/intrefdac; 
  return vcc;
}

////////////////////////////////////////////////////////////////////////////////////

enum class STATE {
  START,
  SPACE,
  ENDL
} state = STATE::START; // 0 waiting for cmd; 1 waiting for space; 2 wating for endln

enum class PINTYPE {
  ANALOG,
  DIGITAL
} pinType = PINTYPE::ANALOG;

enum class OPERATION {
  NONE,
  READ,
  WRITE
} oper = OPERATION::NONE;



byte cmd = 0;
byte pin;
byte val;
bool repeat = false;

void execute()
{
  if(oper == OPERATION::READ )  
  {
    Serial.print(pin);
    Serial.print("->");

    pinMode(pin,INPUT);    
    if( pinType == PINTYPE::DIGITAL ) {
      Serial.print(digitalRead(pin));
    }
    else {
      Serial.print(analogRead(pin));
    }
  }
  else if( oper == OPERATION::WRITE ) 
  {
    Serial.print(pin);
    Serial.print("<-");
    Serial.print(val);
    if( pinType == PINTYPE::DIGITAL ) {
      pinMode(pin,OUTPUT);
      digitalWrite(pin,val);
    }
    else
      analogWrite(pin,val);
  }
}


void loop()
{
  // check if data has been sent from the computer
  if (Serial.available()) {
    // read the most recent byte (which will be from 0 to 255)
    byte in = Serial.read();
    switch(state) {
      case STATE::START:
        cmd = in;
        oper = OPERATION::NONE;   
        if( in == 'h' || in == '?' )
        {
          showHelp();          
        }
        else if( in == 'd' || in == 'a' ) 
        {
          pinType = in == 'd' ? PINTYPE::DIGITAL : PINTYPE::ANALOG;
          oper = OPERATION::READ;
          pin = 0;
          val = 0;
          repeat = false;
          state=STATE::SPACE;
        } 
        else if( in == 'r' )
        {
          repeat = true;
        }
        else if( in == 'I' )
        {
          analogReference(INTERNAL);
          Serial.print("Internal reference.");
        }
        else if( in == 'D' )
        {
          analogReference(DEFAULT);
          Serial.print("Default reference (Vcc).");
        }        
        else if( in == 'E' )
        {
          analogReference(EXTERNAL);
          Serial.println("External reference.");
        }     
        else if( in == 'v' )
        {
          auto v = readVcc();
          Serial.print("Vcc=");
          Serial.print(v);
        }                   
        else if( in == 't' )
        {          
          auto v = readTemp();
          Serial.print("Raw Temp=");
          Serial.print(v);
        }                   
        break;
      case STATE::SPACE:
        if( in >= '0' && in <= '9' )
        {
          pin *= 10;
          pin += in - '0';
        }
        else if( in == ' ' )
        {
          state=STATE::ENDL;
          oper = OPERATION::WRITE;
        }
        break;
      case STATE::ENDL:
        if( in >= '0' && in <= '9' )
        {
          val *= 10;
          val += in - '0';
        }
        break;
    }
    // execute command on CR or NL
    if( in == '\r' || in =='\n' )
    {
      Serial.println();
      execute();
      state = STATE::START;
    }
  }
  if(repeat) execute();
}
