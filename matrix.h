#ifndef MATRIX_H_
#define MATRIX_H_


#include <stddef.h>


// =============================================================================
// Public functions:

float * MatrixAdd(const float * A, const float * B, size_t A_rows,
  size_t A_cols, float * result);

// -----------------------------------------------------------------------------
float * MatrixAddIdentityToSelf(float * A, size_t size);

// -----------------------------------------------------------------------------
float * MatrixAddDiagonalToSelf(float * A, const float * B, size_t size);

// -----------------------------------------------------------------------------
float * MatrixAddToSelf(float * A, const float * B, size_t A_rows,
  size_t A_cols);

// -----------------------------------------------------------------------------
float * MatrixCopy(const float * A, size_t A_rows, size_t A_cols,
  float * result);

// -----------------------------------------------------------------------------
float * MatrixCopyToSubmatrix(const float * A, float * B, size_t row,
  size_t col, size_t A_rows, size_t A_cols, size_t B_cols);

// -----------------------------------------------------------------------------
float * MatrixInverse(const float * A, size_t size, float * result);

// -----------------------------------------------------------------------------
// This function multiplies matrix A by matrix B on the left.
float * MatrixMultiply(const float * A, const float * B, size_t A_rows,
  size_t A_cols, size_t B_cols, float *result);

// -----------------------------------------------------------------------------
// This function multiplies matrix A by a vector representing a diagonal-matrix
// B on the left.
float * MatrixMultiplyByDiagonal(const float * A, const float * B,
  size_t A_rows, size_t A_cols, float *result);

// -----------------------------------------------------------------------------
// This function multiplies matrix A by the transpose of matrix B on the left.
float * MatrixMultiplyByTranspose(const float * A, const float * B,
  size_t A_rows, size_t A_cols, size_t B_rows, float *result);

// -----------------------------------------------------------------------------
// This function multiplies the transpose of matrix A by matrix B on the left.
float * MatrixMultiplyTransposeBy(const float * A, const float * B,
  size_t A_rows, size_t A_cols, size_t B_cols, float *result);

// -----------------------------------------------------------------------------
// This function multiplies a vector representing a diagonal-matrix A by matrix
// B on the left.
float * MatrixMultiplyDiagonalByMatrix(const float * A, const float * B,
  size_t B_rows, size_t B_cols, float *result);

// -----------------------------------------------------------------------------
// This function multiplies a vector representing a diagonal-matrix A by matrix
// B on the left.
float * MatrixMultiplyDiagonalBySelf(const float * A, float * B,
  size_t B_rows, size_t B_cols);

// -----------------------------------------------------------------------------
// This function multiplies matrix A by a vector representing a diagonal-matrix
// B on the left.
float * MatrixMultiplySelfByDiagonal(float * A, const float * B,
  size_t A_rows, size_t A_cols);

// -----------------------------------------------------------------------------
// This function reverses the sign of each element of matrix A.
float * MatrixNegateSelf(float * A, size_t A_rows, size_t A_cols);

// -----------------------------------------------------------------------------
float * MatrixScale(const float * A, float scale, size_t A_rows, size_t A_cols,
  float * result);

// -----------------------------------------------------------------------------
float * MatrixScaleSelf(float * A, float scale, size_t A_rows, size_t A_cols);

// -----------------------------------------------------------------------------
float * MatrixSubtract(const float * A, const float * B, size_t A_rows,
  size_t A_cols, float * result);

// -----------------------------------------------------------------------------
float * MatrixSubtractIdentityFromSelf(float * A, size_t size);

// -----------------------------------------------------------------------------
float * MatrixSubtractSelfFromIdentity(float * A, size_t size);

// -----------------------------------------------------------------------------
float * MatrixTranspose(const float * A, size_t A_rows, size_t A_cols,
  float * result);

// -----------------------------------------------------------------------------
float * SetMatrixToIdentity(float * A, size_t size);

// -----------------------------------------------------------------------------
float * SubmatrixCopyToMatrix(const float * A, float * B, size_t row,
  size_t col, size_t A_cols, size_t B_rows, size_t B_cols);

// -----------------------------------------------------------------------------
float * VectorToDiagonal(const float * v, size_t length, float * result);


#endif  // MATRIX_H_
