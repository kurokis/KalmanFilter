#include "vector.h"

#include <math.h>


// =============================================================================
// Public functions:

float * Vector3Add(const float v1[3], const float v2[3], float result[3])
{
  result[0] = v1[0] + v2[0];
  result[1] = v1[1] + v2[1];
  result[2] = v1[2] + v2[2];

  return result;
}

// -----------------------------------------------------------------------------
float * Vector3AddToSelf(float v1[3], const float v2[3])
{
  v1[0] += v2[0];
  v1[1] += v2[1];
  v1[2] += v2[2];

  return v1;
}

// -----------------------------------------------------------------------------
float * Vector3Copy(const float source[3], float destination[3])
{
  destination[0] = source[0];
  destination[1] = source[1];
  destination[2] = source[2];

  return destination;
}

// -----------------------------------------------------------------------------
float * Vector3Cross(const float v1[3], const float v2[3], float result[3])
{
  result[0] = v1[1] * v2[2] - v1[2] * v2[1];
  result[1] = v1[2] * v2[0] - v1[0] * v2[2];
  result[2] = v1[0] * v2[1] - v1[1] * v2[0];

  return result;
}

// -----------------------------------------------------------------------------
float Vector3Dot(const float v1[3], const float v2[3])
{
  return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

// -----------------------------------------------------------------------------
float Vector3Norm(const float v[3])
{
  return sqrt(Vector3NormSquared(v));
}

// -----------------------------------------------------------------------------
// This function computes the square of the norm of a 3-element vector.
float Vector3NormSquared(const float v[3])
{
  return v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
}

// -----------------------------------------------------------------------------
float * Vector3ScaleSelf(float v[3], float scalar)
{
  v[0] *= scalar;
  v[1] *= scalar;
  v[2] *= scalar;

  return v;
}

// -----------------------------------------------------------------------------
float * Vector3Scale(const float v[3], float scalar, float result[3])
{
  result[0] = v[0] * scalar;
  result[1] = v[1] * scalar;
  result[2] = v[2] * scalar;

  return result;
}

// -----------------------------------------------------------------------------
float * Vector3ScaleAndAccumulate(const float v[3], float scalar,
  float result[3])
{
  result[0] += v[0] * scalar;
  result[1] += v[1] * scalar;
  result[2] += v[2] * scalar;

  return result;
}

// -----------------------------------------------------------------------------
float * Vector3Subtract(const float v1[3], const float v2[3], float result[3])
{
  result[0] = v1[0] - v2[0];
  result[1] = v1[1] - v2[1];
  result[2] = v1[2] - v2[2];

  return result;
}

// -----------------------------------------------------------------------------
float * Vector3SubtractFromSelf(float v1[3], const float v2[3])
{
  v1[0] -= v2[0];
  v1[1] -= v2[1];
  v1[2] -= v2[2];

  return v1;
}

// -----------------------------------------------------------------------------
float * VectorAdd(const float *v1, const float *v2, uint8_t length,
  float * result)
{
  float * result_ptr = result;
  for (uint8_t i = 0; i < length; i++) *result_ptr++ = *v1++ + *v2++;

  return result;
}

// -----------------------------------------------------------------------------
float * VectorCopy(const float * v, uint8_t length, float * result)
{
  float * result_ptr = result;
  for (uint8_t i = length; i; i--) *result_ptr++ = *v++;

  return result;
}

// -----------------------------------------------------------------------------
float * VectorAddToSelf(float *v1, const float *v2, uint8_t length)
{
  float * ptr = v1;
  for (uint8_t i = 0; i < length; i++) *ptr++ += *v2++;

  return v1;
}

// -----------------------------------------------------------------------------
float * VectorScale(const float * v, float scalar, uint8_t length,
  float * result)
{
  float * result_ptr = result;
  for (uint8_t i = length; i; i--) *result_ptr++ = *v++ * scalar;

  return result;
}

// -----------------------------------------------------------------------------
float * VectorScaleSelf(float * v, float scalar, uint8_t length)
{
  float * ptr = v;
  for (uint8_t i = length; i; i--) *ptr++ *= scalar;

  return v;
}

// -----------------------------------------------------------------------------
float * VectorSubtract(const float *v1, const float *v2, uint8_t length,
  float * result)
{
  float * result_ptr = result;
  for (uint8_t i = 0; i < length; i++) *result_ptr++ = *v1++ - *v2++;

  return result;
}

// -----------------------------------------------------------------------------
float * VectorSubtractFromSelf(float *v1, const float *v2, uint8_t length)
{
  float * ptr = v1;
  for (uint8_t i = 0; i < length; i++) *ptr++ -= *v2++;

  return v1;
}
