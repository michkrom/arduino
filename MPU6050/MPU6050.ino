// MPU-6050 Accelerometer + Gyro
// -----------------------------
//
// Based ib arduino.cc user "Krodal".
// mpu6050 moved to the library
// Michal Krombholz
//

#include <Wire.h>
#include <mpu6050.h>

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

  // Clear the 'sleep' bit to start the sensor.
  error = MPU6050_write_reg (MPU6050_PWR_MGMT_1, 0);
  if( error )
  {
    Serial.print(F(" error = "));
    Serial.println(error,DEC);
  }

  // set up build-in filtering
  //MPU6050_write_reg (MPU6050_CONFIG, MPU6050_DLPF_94HZ);
  MPU6050_write_reg (MPU6050_CONFIG, MPU6050_DLPF_5HZ);
  MPU6050_write_reg (MPU6050_GYRO_CONFIG, MPU6050_FS_SEL_2000);
  MPU6050_write_reg (MPU6050_ACCEL_CONFIG, MPU6050_AFS_SEL_4G);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop()
{
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
    // Print the raw acceleration values
    Serial.print(F("A:"));
    Serial.print(accel_t_gyro.x_accel, DEC);
    Serial.print(F(", "));
    Serial.print(accel_t_gyro.y_accel, DEC);
    Serial.print(F(", "));
    Serial.print(accel_t_gyro.z_accel, DEC);
  
    // Print the raw gyro values.  
    Serial.print(F(" G:"));
    Serial.print(accel_t_gyro.x_gyro, DEC);
    Serial.print(F(", "));
    Serial.print(accel_t_gyro.y_gyro, DEC);
    Serial.print(F(", "));
    Serial.print(accel_t_gyro.z_gyro, DEC);
    // The temperature sensor is -40 to +85 degrees Celsius, for a 16 bit signed int.
    // 340 per degrees Celsius, -512 at 35 degrees
    // At 0 degrees: -512 - (340 * 35) = -12412
    Serial.print(F(" T: "));
    //float fT = ( (float) accel_t_gyro.temperature + 12412.0) / 340.0;
    int temp_Cx10 = (accel_t_gyro.temperature/2 + 12412/2) / 17; // /2/17 ==/34
    Serial.print(temp_Cx10/10.0f,1);
    Serial.print(F("C"));
    
    Serial.println();
  }
}

