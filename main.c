#include <stdio.h>
#include <math.h>
#include "kalman_filter.h"

int main(void)
{
  ResetKalman();
  const float* quat;

  int n = 10;

  for(int i = 0; i < n; i++){
    float t, ax, ay, az;
    t = 0.1*i;
    ax = 9.8*sin(t)*sin(sqrt(t));
    ay = 9.8*cos(t)*sin(sqrt(t));
    az = 9.8*cos(sqrt(t));
    const float accelerometer[3] = {ax,ay,az};

    KalmanAccelerometerUpdate(accelerometer);
    quat = KalmanQuat();
    printf("q=%f,%f,%f,%f  a=%f,%f,%f\n",quat[0],quat[1],quat[2],quat[3],ax,ay,az);
  }

  return 0;
}
