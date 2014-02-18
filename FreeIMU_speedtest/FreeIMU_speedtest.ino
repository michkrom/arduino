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

int raw_values[11];
char str[512];
float val[9], q[4];

unsigned long start, stop;

// Set the default object
FreeIMU my3IMU;// = FreeIMU();

void setup() { 
  Serial.begin(115200);
  Serial.println("FreeIMU speed tests (averages of 1024 samples)");
  Wire.begin();
  
  delay(500);
  my3IMU.init(true); // the parameter enable or disable fast mode
  delay(500);
}

void loop() {
  Serial.print("raw reading        : ");
  start = micros();
  for(int i=0; i<1024; i++) {
    my3IMU.getRawValues(raw_values);
  }
  stop = micros();
  Serial.print((stop - start) / 1024);
  Serial.print(" us ~= ");
  Serial.print(((stop - start) / 1024 + 500) / 1000);
  Serial.println(" ms");
  
  
  Serial.print("calibrated reading : ");
  start = micros();
  for(int i=0; i<1024; i++) {
    my3IMU.getValues(val);
  }
  stop = micros();
  Serial.print((stop - start) / 1024);
  Serial.print(" us ~= ");
  Serial.print(((stop - start) / 1024 + 500) / 1000);
  Serial.println(" ms");
  
  
  Serial.print("sensor fusion      : ");
  start = micros();
  for(int i=0; i<1024; i++) {
    my3IMU.getQ(q);
  }
  stop = micros();
  Serial.print((stop - start) / 1024);
  Serial.print(" us ~= ");
  Serial.print(((stop - start) / 1024 + 500) / 1000);
  Serial.println(" ms");
  
  Serial.println("");
}
