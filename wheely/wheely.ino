// Selfbalancing bot "wheely" with MPU6050
// ---------------------------------------
//
// Michal Krombholz
//

#include <Wire.h>
#include <math.h>
#include "mpu6050.h"
#include "dsp.h"

// gyro ofsets
int gxofs, gyofs, gzofs;

void ZeroGyro()
{
  const int N = 256;
  register long sx,sy,sz;
  sx=sy=sz=0;
  Serial.print(F("Zerroing gyro..."));
  unsigned long time =  micros();
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

// computes pendulum deflection angle from straight up (reverse gravity) ie a leanning angle
// result in radians

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
  return atan2( ay, az ) + M_PI/2;
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

PID PosPID(0.3,0.3,0.8);
Complementary filter(0.98);

float KM = 55;   // motors gain
float KB = 0.999; // self balancing integration ratio
float KH = 0.5;   // heading filter integration ratio

///////////////////////////////////////////////////////////////////////////////////////////////////////////

void CheckInput()
{  
  if( CheckSerialInput() )
  {
    // in SerialInput.ino
    extern char serCmd;
    extern float serVal;
    switch(serCmd)
    {
    case 'p': PosPID.Kp = serVal; break;
    case 'i': PosPID.Ki = serVal; break;
    case 'd': PosPID.Kd = serVal; break;
    case 'c': filter.K= serVal; break;
    case 'b': KB = serVal; break;
    case 'm': KM = serVal; break;
    case 'o': PM = (unsigned) serVal; break;
    case 0 : 
      {
        Serial.print(F("Kp="));
        Serial.print(PosPID.Kp);
        Serial.print(' ');
        Serial.print(F("Ki="));
        Serial.print(PosPID.Ki);
        Serial.print(' ');
        Serial.print(F("Kd="));
        Serial.print(PosPID.Kd);
        Serial.print(' ');
        Serial.print(F("KC="));
        Serial.print(filter.K);
        Serial.print(' ');
        Serial.print(F("KM="));
        Serial.print(KM);
        Serial.print(' ');
        Serial.print(F("KB="));
        Serial.print(KB,4);
        Serial.print(' ');
        Serial.print(F("KH="));
        Serial.print(KH);
        Serial.println();
      }
      break;
   default:
    Serial.println(F("[param][-][0..9]*[.][0..9]*<cr> or [pidc]<cr> - where param=pidchmo are PID Compl etc. coeffs"));
    break;
    }
    // clear cmd so we do not repeat
    serCmd=0;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop()
{
  // loop time counting
  static unsigned long prevus=0;

  // deisred "stable" position angle (radians)
  static float pos = -0.07;

  // compute sample time (loop turnaround)
  unsigned long now = micros();
  unsigned long delta = now >= prevus ? now - prevus : 0xFFFFFFFFUL - (prevus-now);
  //  aiming for 100Hz update rate
  if( delta < 10000 ) 
  {
    CheckInput();
    return;
  }
  // <---- it's time to adjust ---->
  prevus = now;

  // get imu readings
  accel_t_gyro_t accel_t_gyro;
  // Read the raw values (14 bytes at once), containing acceleration, temperature and gyro.
  int error = MPU6050_read_all(&accel_t_gyro);
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
    //float rate = (500.0*M_PI)*(accel_t_gyro.x_gyro-gxofs)/(0x7FFF*180.0);
    const float GG = (500.0f/0x7fff)*M_PI/180.0f;
    float rate = float(accel_t_gyro.x_gyro-gxofs)*GG;
    

    // estimation alg
    float dt = delta*1E-6;
    float a = filter.Update(angle,rate,dt);
    //float a = Complementary2(angle,rate,dt);
    //float a = Kalman(angle,rate,dt);

    // current position error (radians)
    float error = a - pos;

    float correction = PosPID.Update( -error );

    // integrate (low-pass) correction actuator to sense "true" stable position
    pos = KB*pos + (1-KB)*correction;
    pos = saturate( pos, -0.1f, +0.1f );

    // actuation (motors) driver
    float motors = KM*KM * correction;
    // attemp to linearize the drive a bit (no feedback PWM --> Voltage --> RPM but not torque
    //float sign = motors > 0 ? 1 : -1;
    //motors = sqrt(abs(motors)) * sign;
    
    // try to stabilize heading
    // yaw rate low pass filter
    static float dir = 0;
    dir = KH*dir + (1-KH)*accel_t_gyro.y_gyro*dt;            
    // PD controller for heading
    static float lastdir = 0;
    float headcorr = saturate((dir+(lastdir-dir)*2),50.0f);
    lastdir = dir;
    
    int motorR, motorL;    
    motorR = motors + (1-headcorr);
    motorL = motors - (1+headcorr);

    // safet" we failed if a>45deg -> stop motors
    if( angle > M_PI/4 || angle < -M_PI/4 ) motorL=motorR=0;

    // minus for 38:1 gearing as it reverser rotation
    setMotors( -motorL, -motorR );


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


