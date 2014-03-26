///////////////////////////////////////////////////////////////////////////////////////////////
// motor driver DRV8833 is connected to 3,9 and 11,10

// Motor RL FB
#define PIN_MLF 3
#define PIN_MLB 9
#define PIN_MRF 11
#define PIN_MRB 10

void setMotor(int drive, char pinF, char pinB)
{
  if( drive > 255 ) drive = 255;
  if( drive < -255 ) drive = -255;
  if( drive > 0 )
  {
    digitalWrite( pinF, 0 );
    analogWrite( pinB, drive );
  }
  else
  {
    analogWrite( pinF, -   drive );
    digitalWrite( pinB, 0 );
  }
}

// left/right range -255..255
void setMotors(int left, int right)
{
  // using pololu motor controller
  setMotor(left, PIN_MLF, PIN_MLB);
  setMotor(right, PIN_MRF, PIN_MRB);
}
