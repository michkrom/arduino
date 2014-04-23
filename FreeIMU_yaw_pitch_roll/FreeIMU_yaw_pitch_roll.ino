#include <ADXL345.h>
#include <bma180.h>
#include <HMC58X3.h>
#include <ITG3200.h>
#include <MS561101BA.h>
#include <I2Cdev.h>
#include <MPU60X0.h>
#include <EEPROM.h>

//#define DEBUG
#include "DebugUtils.h"
#include "CommunicationUtils.h"
#include "FreeIMU.h"
#include <Wire.h>
#include <SPI.h>

#include "vector_math.h"


//template<class T> inline T RadToDeg(T rad) { return rad*180/M_PI; }
inline float RadToDeg(float rad) { return rad*180/M_PI; }

struct GravVec : vmath::vec3<float>
{
  GravVec(const float * const q) {
    setQ(q);
  }
    
  void setQ(const float * const q) {
    x = 2 * (q[1]*q[3] - q[0]*q[2]);
    y = 2 * (q[0]*q[1] + q[2]*q[3]);
    z = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
  }

  float pitch() {
    return atan(x / sqrt(y*y + z*z));
  }
  
  float roll() {
    return atan(y / sqrt(x*x + z*z));
  }
};


// compute yaw form quaternion (yaw does not involve gravity)
float  getYawFromQ(const float * const q)
{
  return atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1);
}

void getEulerFromQ(const float * const q, float * angles) {
  angles[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1); // psi
  angles[1] = -asin(2 * q[1] * q[3] + 2 * q[0] * q[2]); // theta
  angles[2] = atan2(2 * q[2] * q[3] - 2 * q[0] * q[1], 2 * q[0] * q[0] + 2 * q[3] * q[3] - 1); // phi
}

/////////////////////////////////////////////////////////////////////////////////////////////////

// Set the FreeIMU object
FreeIMU my3IMU = FreeIMU();

void setup() { 
  Serial.begin(115200);
  Wire.begin();
  
  delay(5);
  my3IMU.init(); // the parameter enable or disable fast mode
  delay(5);
}



void loop() {
  long now = micros();
  float q[4];
  my3IMU.getQ(q);
  //vmath::quat qt(q[0],q[1],q[2],q30]);
  GravVec gv(q);
  float yaw = RadToDeg(getYawFromQ(q));
  float pitch = RadToDeg(gv.pitch());
  float roll = RadToDeg(gv.roll());
  long delta = micros()-now;
  Serial.print("GXYZ ");
  Serial.print(gv.x,3);
  Serial.print(" ");
  Serial.print(gv.y,3);
  Serial.print(" ");
  Serial.print(gv.z,3);
  Serial.print("YPR:");
  Serial.print(yaw);
  Serial.print(" ");
  Serial.print(pitch);
  Serial.print(" ");
  Serial.print(roll);
  Serial.print(" ");
  Serial.print(delta);
  Serial.println(" usec.");
  
  delay(10);
}



