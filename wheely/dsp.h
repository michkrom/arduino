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

    float K;
    float outAngle;

    Complementary(float k) : 
      K(k), 
      outAngle(0) 
    {}

    // angle - angle measured with the accelerometer
    // rate - angle measured using the gyro
    // dt - delta time [s]
    float Update(float angle, float rate, float dt)
    {
      float k=K;
      //float k = K/(K+dt);
      outAngle = k * (outAngle  + rate * dt) + (1 - k) * angle;
      return outAngle;
    }

};


// 2nd order complementary filter
class Complementary2
{
    // filter gain
    float K;
    // K^2
    float K2;

    // filter's integrators
    float int1;
    float int2;

    Complementary2(float k) :
      K(k), 
      K2(k*k),
      int1(0), 
      int2(0)
    {}

    // angle - angle measured with the accelerometer
    // rate - angle measured using the gyro
    // dt - delta time [s]
    float Update(float angle, float rate, float dt)
    {
      float x1 = angle - int2;
      int1 += (K2 * x1)*dt;
      float x2 = int1 + 2 * K * x1 + rate;
      int2 += x2 * dt;
      return int2;
    }
};


// Kalman filter
float Kalman(float angle, float rate, float dt);

///////////////////////////////////////////////////////////////////////////////////

// PID controller
class PID
{
  public:
  PID() :
    last_error(0),
    Kp(1), Ki(2), Kd(3)
  { }

  PID(float _Kp, float _Ki, float _Kd ) :
    last_error(0),
    Kp(_Kp), Ki(_Ki), Kd(_Kd)
  { }

  float Kp;
  float Ki;
  float Kd;

  float Update(float error)
  {
    float resp = Kp *  error +
                 Ki * (error + last_error) +
                 Kd * (error - last_error);
    last_error = error;
    return resp;
  }
private:
  float last_error;
};


class Kalman
{
    // Kalman filter

    const float Q_angle;
    const float Q_gyro ;
    const float R_angle;

    float x_angle;
    float x_bias;
    float P_00, P_01, P_10, P_11;
    
    Kalman() :
      Q_angle(0.01),
      Q_gyro(0.0003),
      R_angle(0.01),
      x_angle(0),
      x_bias(0),
      P_00(0), P_01(0), P_10(0), P_11(0)
    {}

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
