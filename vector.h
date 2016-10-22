#ifndef VECTOR_H_
#define VECTOR_H_


#include <inttypes.h>


// =============================================================================
// Public functions:

float * Vector3Add(const float v1[3], const float v2[3], float result[3]);

// -----------------------------------------------------------------------------
float * Vector3AddToSelf(float v1[3], const float v2[3]);

// -----------------------------------------------------------------------------
float * Vector3Copy(const float source[3], float destination[3]);

// -----------------------------------------------------------------------------
float * Vector3Cross(const float v1[3], const float v2[3], float result[3]);

// -----------------------------------------------------------------------------
float Vector3Dot(const float v1[3], const float v2[3]);

// -----------------------------------------------------------------------------
float Vector3Norm(const float v[3]);

// -----------------------------------------------------------------------------
// This function computes the square of the norm of a 3-element vector.
float Vector3NormSquared(const float v[3]);

// -----------------------------------------------------------------------------
float * Vector3Scale(const float v[3], float scalar, float result[3]);

// -----------------------------------------------------------------------------
float * Vector3ScaleAndAccumulate(const float v[3], float scalar,
  float result[3]);

// -----------------------------------------------------------------------------
float * Vector3ScaleSelf(float v[3], float scalar);

// -----------------------------------------------------------------------------
float * Vector3Subtract(const float v1[3], const float v2[3], float result[3]);

// -----------------------------------------------------------------------------
float * Vector3SubtractFromSelf(float v1[3], const float v2[3]);

// -----------------------------------------------------------------------------
float * VectorAdd(const float *v1, const float *v2, uint8_t length,
  float * result);

// -----------------------------------------------------------------------------
float * VectorAddToSelf(float *v1, const float *v2, uint8_t length);

// -----------------------------------------------------------------------------
float * VectorCopy(const float * v, uint8_t length, float * result);

// -----------------------------------------------------------------------------
float * VectorScale(const float * v, float scalar, uint8_t length,
  float * result);

// -----------------------------------------------------------------------------
float * VectorScaleSelf(float * v, float scalar, uint8_t length);

// -----------------------------------------------------------------------------
float * VectorSubtract(const float *v1, const float *v2, uint8_t length,
  float * result);

// -----------------------------------------------------------------------------
float * VectorSubtractFromSelf(float *v1, const float *v2, uint8_t length);

#endif  // VECTOR_H_
