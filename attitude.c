#include "attitude.h"

#include <math.h>

#include "quaternion.h"
#include "vector.h"


// =============================================================================
// Public functions:

void EulerAnglesFromQuaternion(const float quat[4], float * phi, float * theta,
  float * psi)
{
  *phi = atan2(2.0 * (quat[0] * quat[1] + quat[2] * quat[3]), 1.0 - 2.0
    * (quat[1] * quat[1] + quat[2] * quat[2]));
  *theta = asin(2.0 * (quat[0] * quat[2] - quat[1] * quat[3]));
  *psi = HeadingFromQuaternion(quat);
}

// -----------------------------------------------------------------------------
float HeadingFromQuaternion(const float quat[4])
{
  return atan2(2.0 * quat[0] * quat[3] + quat[1] * quat[2], 1.0 - 2.0
    * (quat[2] * quat[2] + quat[3] * quat[3]));
}
