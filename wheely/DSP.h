///////////////////////////////////////////////////////////////////////////////////////////////
// helpers

template<class T>
inline T saturate(T in, const T mx)
{
    if( in > +mx ) in = +mx;
    if( in < -mx ) in = -mx;
    return in;
}

template<class T>
inline T saturate(T in, const T mi, const T mx)
{
    if( in > mx ) in = mx;
    if( in < mi ) in = mi;
    return in;
}

// 2nd order complementary filter
class Complementary
{
public:

    // tau = dt*K/(1-K) K=tau/(tau+dt)
    float K;
    float outAngle;

    Complementary(float K) K(k), outAngle(0) {};

    // angle - angle measured with the accelerometer
    // rate - angle measured using the gyro
    // dt - delta time [s]
    inline float Update(float angle, float rate, float dt)
    {
      outAngle = K * (outAngle  + rate * dt) 
               + (1 - K) * angle;
      return outAngle;
    }


};


// 2nd order complementary filter
class Complementary2
{
public:
    // filter gain
    float K;
    // filter's integrators
    float int1;
    float int2;

    Complementary2(float K) K(k), outAngle(0), y2(0), K2(k*k);
    {}

    // angle - angle measured with the accelerometer
    // rate - angle measured using the gyro
    // dt - delta time [s]
    float Update(float angle, float rate, float dt)
    {
      float x1 = angle - outAngle;
      int1 += (K*K * x1)*dt;
      float x2 = int1 + 2 * K * x1 + rate;
      int2 += x2 * dt;
      return int2;
    }
};

// Kalman filter
class Kalman
{
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
    float Update(float newAngle, float newRate, float dt)
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
};

///////////////////////////////////////////////////////////////////////////////////

// PID controller
class PID
{
  public:
  PID() :
    last_error(0),
    Kp(1), Ki(2), Kd(3)
  { }

  PID(float _Kp, float _Ki, float _Kd) :
    last_error(0),
    Kp(_Kp), Ki(_Ki), Kd(_Kd)
  { }

  float Kp;
  float Ki;
  float Kd;

  float Update(float error)
  {
    float resp = 0;
    if( Kp != 0 ) resp += Kp *  error;
    if( Ki != 0 ) resp += Ki * (error + last_error);
    if( Kd != 0 ) resp += Kd * (error - last_error);
    last_error = error;
    return resp;
  }
private:
  float last_error;
};

