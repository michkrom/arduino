// Selfbalancing bot "wheely" with MPU6050
// ---------------------------------------
//
// Michal Krombholz
//

#include <Wire.h>
#include "mpu6050.h"
#include "DSP.h"

// gyro ofsets
int gxofs, gyofs, gzofs;

void ZeroGyro()
{
  const int N = 256;
  register long sx,sy,sz;
  sx=sy=sz=0;
  Serial.print(F("Zerroing gyro..."));
  unsigned long time = micros();
  for(int i = 0;  i < N; i++ )
  {
    // Read all raw values (14 bytes at once), containing acceleration, temperature and gyro.
    accel_t_gyro_t accel_t_gyro;
    int error = MPU6050_read_all(&accel_t_gyro);
    if( error == 0 )
    {
      sx += accel_t_gyro.x_gyro;
      sy += accel_t_gyro.y_gyro;
      sz += accel_t_gyro.z_gyro;
    }
    else
      i--;
  }
  time = micros() - time;
  Serial.print(time);
  Serial.println(F("us"));
  gxofs = sx/N;
  gyofs = sy/N;
  gzofs = sz/N;
}
///////////////////////////////////////////////////////////////////////////////////////////////

float ComputeAngle(float x, float y, float z)
{
    // grav vec magnitude
    float ssum = x*x+y*y+z*z;
    float vec = sqrt(ssum);

    // normalize gravity vec
    float norm = 1/vec;
    float ax = x * norm;
    float ay = y * norm;
    float az = z * norm;

    if( az > +0.95 ) return M_PI/2;
    if( az < -0.95 ) return -M_PI/2;

    // compute pitch angle (due to sensor orientation)
    //float angle = atan2( accel_t_gyro.y_accel, accel_t_gyro.z_accel ) + M_PI/2;
    float angle = atan2( ay, az ) + M_PI/2;

    return angle;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////

// printmask
enum {
  PM_ANR = 1,
  PM_ANG = 2,
  PM_POS = 4, 
  PM_UDR = 8, 
  PM_COR = 16,
  PM_RAW = 32,
  PM_TMP = 64,
  PM_ERR = 128,
  PM_MOT = 256,
 
  PM_STICKY = 1024,
};
unsigned PM = 0;


float KM = 1000; // motors gain
float KB = 0.005; // self balancing integration ratio

///////////////////////////////////////////////////////////////////////////////////////////////////////////

void CheckInput()
{
  static float val;
  static char fracpos;
  static char sign;
  static char cmd;
  static char numok = 0;
  static int state = 0;
  // check if data has been sent from the computer
  if (Serial.available()) {
    // read the most recent byte (which will be from 0 to 255)
    byte in = Serial.read();
    switch(state) {
      case 0:
        if( in == 'p' || in == 'i' || in == 'd' || in == 'c' || in == 'm' || in == 'o' || in == 'b') 
        {
          cmd = in;
          val = 0;
          fracpos = 0;
          sign = 1;
          numok = 0;
          state++;
        }
        else if( in == '?' )
        {
          Serial.println(F("[pidc][-][0..9]*[.][0..9]*<cr> or [pidc]<cr> - where pidc are PID Compl coeffs"));
          Serial.print(F("Kp="));
          Serial.print(Kp);
          Serial.print(' ');
          Serial.print(F("Ki="));
          Serial.print(Ki);
          Serial.print(' ');
          Serial.print(F("Kd="));
          Serial.print(Kd);
          Serial.print(' ');
          Serial.print(F("KC="));
          Serial.print(KC2);
          Serial.print(' ');
          Serial.print(F("KM="));
          Serial.print(KM);
          Serial.print(' ');
          Serial.print(F("KB="));
          Serial.print(KB);
          Serial.println();
        }
        break;
      case 1://accepting number
        if( in >= '0' && in <= '9' )
        {
          numok = 1;
          val *= 10;
          val += in - '0';
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
      Serial.println("\r\n");
      
      if( numok )
      {
        val = sign * val;
        while( --fracpos > 0 ) val *= 0.1f;
        Serial.print(cmd);
        Serial.print(val);
        switch(cmd) {
          case 'p': Kp = val; break;
          case 'i': Ki = val; break;
          case 'd': Kd = val; break;
          case 'c': KC2 = val; break;
          case 'b': KB = val; break;
          case 'm': KM = val; break;
          case 'o': PM = (unsigned) val; break;
        }
        Serial.println("\r\n");
      }      
      state = 0;
      numok = 0;
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop()
{
  static unsigned long prevus=0;
  // deisred "stable" position angle (radians)
  static float pos = -0.07;
  
  // compute sample time (loop turnaround)
  unsigned long now = micros();
  unsigned long delta = now >= prevus ? now - prevus : 0xFFFFFFFFUL - (prevus-now);

  //  trying for 100Hz update rate
  if( delta < 10000 ) 
  {
    CheckInput();
    return;
  }
  
  // <---- it's time to adjust ---->
  
  prevus = now;

  // get imu readings
  int error;
  accel_t_gyro_t accel_t_gyro;

  // Read the raw values (14 bytes at once), containing acceleration, temperature and gyro.
  error = MPU6050_read_all(&accel_t_gyro);
  if( error != 0 )
  {
    Serial.print(F("Read error = "));
    Serial.println(error,DEC);
  }
  else
  {
    float angle = ComputeAngle( accel_t_gyro.x_accel, accel_t_gyro.y_accel, accel_t_gyro.z_accel );
    // rotation around x axis is the rate of y axis change
    // float rateDeg = 2000.0*accel_t_gyro.x_gyro/0x7FFF;
    float rate = (500.0*M_PI)*(accel_t_gyro.x_gyro-gxofs)/(0x7FFF*180.0);

    // estimation alg
    float dt = delta*1E-6;
    //float a1 = Complementary(angle,rate,dt);
    float a = Complementary2(angle,rate,dt);
    //float a = Kalman(angle,rate,dt);

    // current position error (radians)
    float error = a - pos;

    float correction = CalcPID( -error );

    // integrate (low-pass) correction actuator to sense "true" stable position
    //static float corrInt = 0;
    //corrInt += 0.05 * correction;
    //corrInt = saturate( corrInt, 1 );
    //if( corrInt > 0.1 ) pos -= .001;
    //if( corrInt < -0.1 ) pos += .001;

    // integrate action into "stable" position (try to drive response to hover around zero)
    //if( correction > +0.001 ) pos += .000001;
    //if( correction < -0.001 ) pos -= .000001;
    //if( error > 0 ) pos -= .0001;
    //if( error < 0 ) pos += .0001;
    
    pos += KB*correction;
    pos = saturate( pos, -0.1f, -0.04f );

    //Serial.print(' ');
    //Serial.println(pos);

//    Serial.print(a2);
//    Serial.print(F(", "));
//    Serial.print(error);
//    Serial.print(F(", "));

    // actuation (motors) driver
    
//    float angleGainAdjustTreshhold = M_PI*10/180; 
//    float KM = 1000+(abs(angle)>angleGainAdjustTreshhold ? (angle-angleGainAdjustTreshhold) * 100 : 0);
      float motors = KM * correction;
      static float last_motors = 0;         
      float motors_delta = motors-last_motors;   
      if( motors > 1 && motors < 50 ) motors += 0.5*motors_delta;
      if( motors < -1 && motors > -50 ) motors += 0.5*motors_delta;
      int motorR, motorL;
      float dir = -0.1;
      motorR = (motors * (1-dir));
      motorL = (motors * (1+dir));
      
      // safet" we failed if a>45deg -> stop motors
      if( a > M_PI/4 || a < -M_PI/4 ) motorL=motorR=0;
      
      setMotors( motorL, motorR );
      
      last_motors = motors;

#define PRINTMASK
#ifdef PRINTMASK    

    // execute printmask    
    if( PM & PM_ANR )
    {    
      Serial.print(F(" A&R="));
      Serial.print(int(angle * 180 / M_PI));
      Serial.print(',');
      Serial.print(int(rate * 180 / M_PI));
    }
    if( PM & PM_ANG )
    {    
      Serial.print(F(" A="));
      Serial.print(int(a * 180 / M_PI));
    }
    if( PM & PM_POS )
    {
      Serial.print(F(" P="));
      Serial.print(pos);
    }
    if( PM & PM_ERR )
    {
      Serial.print(F(" E="));
      Serial.print(error);
    }
    if( PM & PM_COR )
    {
      Serial.print(F(" C="));
      Serial.print(correction);
    }
    
    if( PM & PM_MOT )
    {
      Serial.print(F(" M="));
      Serial.print(motorL,DEC);
      Serial.print(',');
      Serial.print(motorR,DEC);
    }
    
    if( PM & PM_UDR )
    {
      Serial.print(F(" R="));
      Serial.print((int)(1/dt+0.5),DEC);
    }
    if( PM & PM_RAW )
    {
      // Print the raw acceleration values
      Serial.print(F(" A:"));
      Serial.print(accel_t_gyro.x_accel, DEC);
      Serial.print(F(", "));
      Serial.print(accel_t_gyro.y_accel, DEC);
      Serial.print(F(", "));
      Serial.print(accel_t_gyro.z_accel, DEC);
  
      // Print the raw gyro values.
  
      Serial.print(F(" G:"));
      Serial.print(accel_t_gyro.x_gyro - gxofs, DEC);
      Serial.print(F(", "));
      Serial.print(accel_t_gyro.y_gyro - gyofs, DEC);
      Serial.print(F(", "));
      Serial.print(accel_t_gyro.z_gyro - gzofs, DEC);
    }
    if( PM & PM_TMP )
    {
      // The temperature sensor is -40 to +85 degrees Celsius, for a 16 bit signed int.
      // 340 per degrees Celsius, -512 at 35 degrees
      // At 0 degrees: -512 - (340 * 35) = -12412
      Serial.print(F(" T: "));
      //float fT = ( (float) accel_t_gyro.temperature + 12412.0) / 340.0;
      int temp_Cx10 = (accel_t_gyro.temperature/2 + 12412/2) / 17; // /2/17 ==/34
      Serial.print(temp_Cx10/10.0f,1);
      Serial.print(F("C"));
    }
    if( PM )
    {
      Serial.println();
      if( !(PM & PM_STICKY) )
        PM = 0;
    }
    
    #endif

  }
}


///////////////////////////////////////////////////////////////////////////////////

void setup()
{
  int error;
  uint8_t c;


  Serial.begin(115200);
  Serial.println(F("InvenSense MPU-6050"));
  Serial.println(F("June 2012 - modified"));

  // Initialize the 'Wire' class for the I2C-bus.
  Wire.begin();


  // default at power-up:
  //    Gyro at 250 degrees second
  //    Acceleration at 2g
  //    Clock source at internal 8MHz
  //    The device is in sleep mode.
  //

  Serial.print(F("Setting up MPU6050..."));

  Serial.print(F("WHO_AM_I : "));
  error = MPU6050_read(MPU6050_WHO_AM_I, &c, 1);
  Serial.println(c,HEX);
  if( error )
  {
    Serial.print(F(" error = "));
    Serial.println(error,DEC);
  }

  #if 0
  // According to the datasheet, the 'sleep' bit
  // should read a '1'.
  // That bit has to be cleared, since the sensor
  // is in sleep mode at power-up.
  error = MPU6050_read (MPU6050_PWR_MGMT_1, &c, 1);
  if( error )
  {
    Serial.print(F(" error = "));
    Serial.println(error,DEC);
  }
  #endif

  // Clear the 'sleep' bit to start the sensor.
  error = MPU6050_write_reg (MPU6050_PWR_MGMT_1, 0);
  if( error )
  {
    Serial.print(F(" error = "));
    Serial.println(error,DEC);
  }

  // set the build-in filtering
  //MPU6050_write_reg (MPU6050_CONFIG, MPU6050_DLPF_94HZ);
  MPU6050_write_reg (MPU6050_CONFIG, MPU6050_DLPF_94HZ);
  MPU6050_write_reg (MPU6050_GYRO_CONFIG, MPU6050_FS_SEL_500);
  MPU6050_write_reg (MPU6050_ACCEL_CONFIG, MPU6050_AFS_SEL_4G);

  // zeroing gyro
  ZeroGyro();
}

