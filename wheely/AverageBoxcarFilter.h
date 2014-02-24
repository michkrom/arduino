#ifndef _AVERAGEBOXCARFILTAER_H
#define _AVERAGEBOXCARFILTAER_H

// moving average (boxcar) filter:
// samples of T-type,
// accumulator of TA-type,
// length of N (for speed N=2^K so divide is just a shift)

template<class T, class TA, int N>
class AverageBoxcarFilter
{
    // samples history
    T samples[N];
    // keep current sum of all samples so we do not have to recompute
    TA sum;
    // current position in the filter
    byte pos;

public:

    AverageBoxcarFilter()
    {
      pos=0;
      sum=0;
    }


    // add a new sample
    inline void next(T input)
    {
      // next input index
      pos = (pos + 1) % N;


      // now remove oldest sample and add new one
      sum -= samples[pos];
      sum += input;
      samples[pos] = input;
    }


    // compute current filter value
    inline T curr()
    {
      return sum/N;
    }
};


#if 0
// zero gyro through an averaging boxcar filter
// just an example of the filter use (yes a bit over the top)
// note that does not fit in the stack in arduino
void ZeroGyro()
{
  const int N = 256;
  typedef AverageBoxcarFilter<int, long, N> GyroZeroFilter;
  GyroZeroFilter gxf,gyf,gzf;
  int error;
  accel_t_gyro_union accel_t_gyro;
    
  for( int i = 0; i < N; i++ )
  {
    // Read all raw values (14 bytes at once), containing acceleration, temperature and gyro.
    error = MPU6050_read (MPU6050_ACCEL_XOUT_H, (uint8_t *) &accel_t_gyro, sizeof(accel_t_gyro));
    
    // adjust running average for gyro - ideally compute this at startup
    gxf.next(accel_t_gyro.value.x_gyro);
    gyf.next(accel_t_gyro.value.y_gyro);
    gzf.next(accel_t_gyro.value.z_gyro);    
  }
  gxofs = gxf.curr();
  gyofs = gyf.curr();
  gzofs = gzf.curr();
}

#endif


#endif
