///////////////////////////////////////////////////////////////////////////////////////////////

// PID parameters
extern float Kp;
extern float Ki;
extern float Kd;

// Complementary param
extern float KC;

// Complementary2 param
extern float KC2;

// 1st order complementary filter
// angle - angle measured with the accelerometer
// rate - gyro rate
// dt = delta time [s]
float Complementary(float angle, float rate, float dt);

// 2nd order complementary filter
// angle - angle measured with the accelerometer
// rate - angle measured using the gyro
// dt - delta time [s]
float Complementary2(float angle, float rate, float dt);


// Kalman filter
float Kalman(float angle, float rate, float dt);

///////////////////////////////////////////////////////////////////////////////////

// PID controller
float CalcPID(float error);


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

