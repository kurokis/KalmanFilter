#ifndef QUATERNION_H_
#define QUATERNION_H_


#include <inttypes.h>


// =============================================================================
// Public functions:

float * QuaternionInverse(const float quat[4], float result[4]);

// -----------------------------------------------------------------------------
float * QuaternionInvertSelf(float quat[4]);

// -----------------------------------------------------------------------------
// This functions performs quaternion multiplication of the inverse of quat1
// with quat2.
float * QuaternionInverseMultiply(const float quat1[4], const float quat2[4],
  float result[4]);

// -----------------------------------------------------------------------------
float * QuaternionMultiply(const float quat1[4], const float quat2[4],
  float result[4]);

// -----------------------------------------------------------------------------
// This functions performs quaternion multiplication of quat1 with the inverse
// of quat2.
float * QuaternionMultiplyInverse(const float quat1[4], const float quat2[4],
  float result[4]);

// -----------------------------------------------------------------------------
float QuaternionNorm(const float quat[4]);

// -----------------------------------------------------------------------------
float * QuaternionNormalize(float quat[4]);

// -----------------------------------------------------------------------------
// This filter pushes the quaternion toward unity and is much more efficient
// than direct normalization (no sqrt and no divide).
float * QuaternionNormalizingFilter(float quat[4]);

// -----------------------------------------------------------------------------
float * QuaternionRotateVector(const float quat[4], const float v[3],
  float result[3]);


#endif  // QUATERNION_H_
