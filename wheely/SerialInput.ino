///////////////////////////////////////////////////////////////////////////////////////////////////////////
// this code checks serial console for input
// the input is in the form of
// <command-letter><parameter-number><cr>
// the CheckSerialInput returns true if the input is completed (at <cr>)
// the setCmd hold the command letter
// the serVal hold parameter or NaN if none given
// the number can be of of a form [<->]<digits>[<.><digits>]
// however parsing is not precised so the the sign changes every given <-> and only last <.> is applied

char serCmd;
float serVal;

byte CheckSerialInput()
{
  static char fracpos;
  static char sign;
  static char cmd;
  static char numok = 0;
  static int state = 0;

  byte ret = 0;

  // check if data has been sent from the computer
  if (Serial.available()) {
    // read the most recent byte (which will be from 0 to 255)
    byte in = Serial.read();
    switch(state) {
    case 0:
      if( in >= 'a' && in < 'z' || in == '?' )
      {
        serCmd = in;
        serVal = 0;
        fracpos = 0;
        sign = 1;
        numok = 0;
        state++;
      }
      break;
    case 1://accepting number
      if( in >= '0' && in <= '9' )
      {
        numok = 1;
        serVal *= 10;
        serVal += in - '0';
        if( fracpos ) fracpos++;
      }
      else if( in == '-' )
      {
        sign = - sign;
      }
      else if( in == '.' )
      {
        // start fracs
        fracpos = 1;
      }
      break;
    }
    // execute command on CR
    if( in == '\r' || in == '?' && cmd != '?')
    {
      ret = 1;
      Serial.println("\r\n");
      if( numok )
      {
        serVal = sign * serVal;
        while( --fracpos > 0 ) serVal *= 0.1f;
        Serial.print(serCmd);
        Serial.print(serVal);
        Serial.println("\r\n");
      }
      else
        serVal = NAN;

      state = 0;
      numok = 0;
    }
  }
  return ret;
}
