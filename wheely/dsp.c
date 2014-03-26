///////////////////////////////////////////////////////////////////////////////////////////////

// 1st complementary filter
// angle = angle measured with the accelerometer
// rate = gyro rate
// dt = delta time [s]

float KC=0.98;

float Complementary(float angle, float rate, float dt)
{
  static float angleC = 0;
  float k=KC;///(KC+dt);
  angleC = k * (angleC  + rate * dt) + (1 - k) * angle;
  return angleC;
}


// 2nd order complementary filter
// angle = angle measured with the accelerometer
// rate = angle measured using the gyro
// dt - delta time [s]

float KC2=1;

float Complementary2(float angle, float rate, float dt)
{
  // current angle estimate
  static float angleC2 = 0;
  // gyro error accumulator
  static float y1;

  // accel-measured delta
  float x1 = angle - angleC2;
  
  // gyro error estimate
  y1 += (KC2*KC2) * x1 * dt; // sth fishy here - unit is [rad-sec] but rate is [rad/sec]

  // adjusted angle rate
  float x2 = y1 + x1 * (2*KC2) + rate;

  // new angle estimate
  angleC2 = x2 * dt + angleC2;

  return angleC2;
}


// Kalman filter

const float Q_angle  =  0.01;
const float Q_gyro   =  0.0003;
const float R_angle  =  0.01;

float x_angle = 0;
float x_bias = 0;
float P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0;

// newAngle = angle measured with atan2 using the accelerometer
// newRate = angle measured using the gyro
// dt - delta time [s]

float Kalman(float newAngle, float newRate, float dt)
{
  x_angle += dt * (newRate - x_bias);
  P_00 +=  - dt * (P_10 + P_01) + Q_angle * dt;
  P_01 +=  - dt * P_11;
  P_10 +=  - dt * P_11;
  P_11 +=  + Q_gyro * dt;

  float y = newAngle - x_angle;
  float S = P_00 + R_angle;
  float K_0 = P_00 / S;
  float K_1 = P_10 / S;

  x_angle +=  K_0 * y;
  x_bias  +=  K_1 * y;
  P_00 -= K_0 * P_00;
  P_01 -= K_0 * P_01;
  P_10 -= K_1 * P_00;
  P_11 -= K_1 * P_01;

  return x_angle;
}


// PID controller

float Kp = 1;
float Ki = 2;
float Kd = 3;

float CalcPID(float error)
{
  static float last_error;
  float resp = Kp *  error +
               Ki * (error + last_error) +
               Kd * (error - last_error);
  last_error = error;
  return resp;
}


