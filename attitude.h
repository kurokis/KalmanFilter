#ifndef ATTITUDE_H_
#define ATTITUDE_H_


// =============================================================================
// Accessors:

void EulerAnglesFromQuaternion(const float quat[4], float * phi, float * theta,
  float * psi);

// -----------------------------------------------------------------------------
float HeadingFromQuaternion(const float quat[4]);


#endif  // ATTITUDE_H_
