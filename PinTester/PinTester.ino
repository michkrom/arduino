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


void setup()
{
  // begin the serial communication
  Serial.begin(9600);
  Serial.println("PinTester ready.");
  Serial.println("dN V sets digital pin N to value V");
  Serial.println("dN V sets analog pin N to value V");
  Serial.println("command without V reads the pin state");
}


byte state = 0; // 0 waiting for cmd; 1 waiting for space; 2 wating for endln
enum class PINTYPE {
  ANALOG,
  DIGITAL
} pinType = PINTYPE::ANALOG;
enum class OPERATION {
  READ,
  WRITE
} oper = OPERATION::READ;
byte pin;
byte val;
bool repeat = false;


void execute()
{
  if(oper == OPERATION::READ )  
  {
    Serial.print(pin);
    Serial.print("->");
    if( pinType == PINTYPE::DIGITAL ) {
      Serial.print(digitalRead(pin));
    }
    else {
      Serial.print(analogRead(pin));
    }
  }
  else
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
  Serial.println(".\r\n");  
}


void loop()
{
  // check if data has been sent from the computer
  if (Serial.available()) {
    // read the most recent byte (which will be from 0 to 255)
    byte in = Serial.read();
    switch(state) {
      case 0:
        if( in == 'd' || in == 'a' ) 
        {
          pinType = in == 'd' ? PINTYPE::DIGITAL : PINTYPE::ANALOG;
          oper = OPERATION::READ;
          pin = 0;
          val = 0;
          repeat = false;
          state++;
        } 
        else if( in == 'r' )
        {
          repeat = true;
        }
        break;
      case 1:
        if( in >= '0' && in <= '9' )
        {
          pin *= 10;
          pin += in - '0';
        }
        else if( in == ' ' )
        {
          state++;
          oper = OPERATION::WRITE;
        }
        break;
      case 2:
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
      Serial.println("\r\n");
      execute();
      state = 0;
    }
  }
  if(repeat) execute();
}
