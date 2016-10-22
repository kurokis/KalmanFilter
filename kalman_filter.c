#include "kalman_filter.h"

#include <stdio.h>

#include "attitude.h"
#include "constants.h"
#include "matrix.h"
#include "quaternion.h"
#include "vector.h"


// =============================================================================
// Private data:

// Vector and Matrix dimensions
#define X_DIM (10)  // Length of state vector x. [q, v_i, r_i]
#define U_DIM (6)  // Length of input u. [gyro; accel]
#define P_DIM (9)  // Size of P (square) matrix
#define Z_DIM_MAX (3)  // Maximum of z (for memory allocation)

// Measurement noise covariance
#define G GRAVITY_ACCELERATION
#define KALMAN_Q_PRIME_ALPHA (0)
#define KALMAN_Q_PRIME_VELOCITY (0.001)
#define KALMAN_Q_PRIME_H_POSITION (0.0)
#define KALMAN_Q_PRIME_V_POSITION (0.0)
// #define KALMAN_Q_PRIME_V_POSITION (0.001)
#define KALMAN_SIGMA_ACCELEROMETER_X (0.005)
#define KALMAN_SIGMA_ACCELEROMETER_Y (0.005)
#define KALMAN_SIGMA_ACCELEROMETER_Z (0.042)
#define KALMAN_SIGMA_ACCELEROMETER_G (0.25)
#define KALMAN_SIGMA_BARO (0.68)
#define KALMAN_SIGMA_GYRO (0.007)
#define KALMAN_SIGMA_VISION (0.02)
#define KALMAN_SIGMA_POSITION (0.05)

static float heading_ = 0.0;
static float x_[X_DIM] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
static float P_[P_DIM*P_DIM] = { 0.0 };
static float baro_altitude_offset = 0.0;


// =============================================================================
// Accessors:

float KalmanHeading(void)
{
  return heading_;
}

// -----------------------------------------------------------------------------
const float * KalmanPosition(void)
{
  return &x_[7];
}

// -----------------------------------------------------------------------------
const float * KalmanQuat(void)
{
  return &x_[0];
}

// -----------------------------------------------------------------------------
const float * KalmanVelocity(void)
{
  return &x_[4];
}


// =============================================================================
// Private function declarations:

static void TimeUpdate(const float * x_est_prev, const float * P_est_prev,
  const float * gyro, const float * accelerometer, float * x_pred,
  float * P_pred);
static void AccelerometerUpdate(const float * x_pred, const float * P_pred,
  const float * accelerometer, float * x_est, float * P_est);
static void BaroAltitudeUpdate(const float *x_pred, const float *P_pred,
  float baro_altitude, float *x_est, float *P_est);
static void VisionUpdate(const float * x_pred, const float * P_pred,
  const float * vision, float * x_est, float * P_est);
static void PositionUpdate(const float * x_pred, const float * P_pred,
  const float * position, float * x_est, float * P_est);
static void MeasurementUpdateCommon(const float * x_pred,
  const float * measurement, const float * predicted_measurement,
  const float * K, float * x_est, int z_dim);
static float * MatrixMultiplySkewSymmetric3(const float * A, const float B[3*3],
  size_t A_rows, float * result);
static float * QuaternionToDCM(const float *quat, float *result);
// static float * SkewSymmetric3Transpose(const float A[3*3] , float result[3*3]);
static float * UpdateQuaternion(const float quat[4],
  const float angular_rate[3], float result[4]);
static float * Vector3ToSkewSymmetric3(const float vec[3], float *result);


// =============================================================================
// Public functions:

void KalmanTimeUpdate(const float gyro[3], const float accelerometer[3])
{
  float x_pred[X_DIM];
  TimeUpdate(x_, P_, gyro, accelerometer, x_pred, P_);
  VectorCopy(x_pred, X_DIM, x_);
}

// -----------------------------------------------------------------------------
void KalmanAccelerometerUpdate(const float accelerometer[3])
{
  float x_est[X_DIM];
  float P_est[P_DIM*P_DIM];
  AccelerometerUpdate(x_, P_, accelerometer, x_est, P_est);
  VectorCopy(x_est, X_DIM, x_);
  MatrixCopy(P_est, P_DIM, P_DIM, P_);
}

// -----------------------------------------------------------------------------
void KalmanBaroAltitudeUpdate(float baro_altitude)
{
  float x_est[X_DIM];
  float P_est[P_DIM*P_DIM];
  BaroAltitudeUpdate(x_, P_, baro_altitude, x_est, P_est);
  VectorCopy(x_est, X_DIM, x_);
  MatrixCopy(P_est, P_DIM, P_DIM, P_);
}

// -----------------------------------------------------------------------------
void KalmanVisionUpdate(const float vision[3])
{
  float x_est[X_DIM];
  float P_est[P_DIM*P_DIM];
  VisionUpdate(x_, P_, vision, x_est, P_est);
  VectorCopy(x_est, X_DIM, x_);
  MatrixCopy(P_est, P_DIM, P_DIM, P_);
}

// -----------------------------------------------------------------------------
void KalmanPositionUpdate(const float position[3])
{
  float x_est[X_DIM];
  float P_est[P_DIM*P_DIM];
  PositionUpdate(x_, P_, position, x_est, P_est);
  VectorCopy(x_est, X_DIM, x_);
  MatrixCopy(P_est, P_DIM, P_DIM, P_);
}

// -----------------------------------------------------------------------------
void ResetKalman(void)
{
  heading_ = 0.0;
  x_[0] = 1.0;
  for (size_t i = 10; --i; ) x_[i] = 0.0;
  for (size_t i = P_DIM * P_DIM; --i; ) P_[i] = 0.0;
  P_[0*9+0] = 1.0e-05;
  P_[1*9+1] = 1.0e-05;
  P_[2*9+2] = 1.0e-05;
}

// -----------------------------------------------------------------------------
void ResetKalmanBaroAltitudeOffset(float baro_altitude, float position_down)
{
  baro_altitude_offset = baro_altitude - (-position_down);
  x_[9] = position_down;
}


// =============================================================================
// Private functions:

static void TimeUpdate(const float * x_est_prev, const float * P_est_prev,
  const float * gyro, const float * accelerometer, float * x_pred,
  float * P_pred)
{
  const float * quat_prev = &x_est_prev[0];
  const float * velocity_prev = &x_est_prev[4];
  const float * position_prev = &x_est_prev[7];

  float * quat_next = &x_pred[0];
  float * velocity_next = &x_pred[4];
  float * position_next = &x_pred[7];

  // Update the quaternion with gyro measurements.
  UpdateQuaternion(quat_prev, gyro, quat_next);
  heading_ = HeadingFromQuaternion(quat_next);

  // Form body to inertial direction-cosine matrix.
  float Cbi[3 * 3], temp[3*3];
  QuaternionToDCM(QuaternionInverse(quat_prev, temp), Cbi);

  // Transform acceleration measured in the body frame to the inertial frame.
  float acceleration[3];  // Specific force measured in the inertial frame
  MatrixMultiply(Cbi, accelerometer, 3, 3, 1, acceleration);
  acceleration[2] += G;  // Remove accelerometer gravity bias

  // Integrate the acceleration to get velocity.
  Vector3Add(velocity_prev, Vector3Scale(acceleration, DT, temp),
    velocity_next);

  // Update position.
  Vector3Add(position_prev, Vector3Scale(velocity_prev, DT, temp),
    position_next);

  // Update error covariance matrix P
  // 1. Discretization of error process model. Calculate Phi and Gamma, where
  //    deltax_next = Phi*deltax_prev + Gamma*deltau + process noise by modeling
  //    error. deltau is the "error input" into the process, which is process
  //    noise by IMU. (deltau=[gyro error; accel error])
  // 2. Definition of Q and Qprime (which are constant throughout the
  //    experiment)
  //    Q: covariance matrix of process noise due to IMU
  //       This can be modeled beforehand.
  //    Qprime: covariance matrix of process noise due to modeling error.
  //            Such noise may be introduced into the system by angular momentum
  //            acting on the body, for example. This term will be used as a
  //            filter design parameter.
  // 3. Propagation of P
  //    P_pred = Phi*P_est_prev*Phi^T + Gamma*Q*Gamma^T + Qprime


  // 3. Propagate P

  // P_pred = Phi*P*Phi^t + Gamma*Q*Gamma^t + Qprime
  float * PhiPPhit = P_pred;  // conserves memory
  {
    float P11[3*3], P12[3*3], P13[3*3];
    float P21[3*3], P22[3*3], P23[3*3];
    float P31[3*3], P32[3*3], P33[3*3];

    SubmatrixCopyToMatrix(P_est_prev, P11, 0, 0, P_DIM, 3, 3);
    SubmatrixCopyToMatrix(P_est_prev, P12, 0, 3, P_DIM, 3, 3);
    SubmatrixCopyToMatrix(P_est_prev, P13, 0, 6, P_DIM, 3, 3);
    SubmatrixCopyToMatrix(P_est_prev, P21, 3, 0, P_DIM, 3, 3);
    SubmatrixCopyToMatrix(P_est_prev, P22, 3, 3, P_DIM, 3, 3);
    SubmatrixCopyToMatrix(P_est_prev, P23, 3, 6, P_DIM, 3, 3);
    SubmatrixCopyToMatrix(P_est_prev, P31, 6, 0, P_DIM, 3, 3);
    SubmatrixCopyToMatrix(P_est_prev, P32, 6, 3, P_DIM, 3, 3);
    SubmatrixCopyToMatrix(P_est_prev, P33, 6, 6, P_DIM, 3, 3);

    // Phi = I + A * dt
    // const float Phi[P_DIM*P_DIM] = {
    //               1,    gyro[2]*DT,   -gyro[1]*DT,    0,    0,    0, 0, 0, 0,
    //     -gyro[2]*DT,             1,    gyro[0]*DT,    0,    0,    0, 0, 0, 0,
    //      gyro[1]*DT,   -gyro[0]*DT,             1,    0,    0,    0, 0, 0, 0,
    //   -Cbissa[0]*DT, -Cbissa[1]*DT, -Cbissa[2]*DT,    1,    0,    0, 0, 0, 0,
    //   -Cbissa[3]*DT, -Cbissa[4]*DT, -Cbissa[5]*DT,    0,    1,    0, 0, 0, 0,
    //   -Cbissa[6]*DT, -Cbissa[7]*DT, -Cbissa[8]*DT,    0,    0,    1, 0, 0, 0,
    //               0,             0,             0, 1*DT,    0,    0, 1, 0, 0,
    //               0,             0,             0,    0, 1*DT,    0, 0, 1, 0,
    //               0,             0,             0,    0,    0, 1*DT, 0, 0, 1,
    // };

    const float Phi11[3*3] = {
                1,    gyro[2]*DT,   -gyro[1]*DT,
      -gyro[2]*DT,             1,    gyro[0]*DT,
       gyro[1]*DT,   -gyro[0]*DT,             1,
    };

    float Phi21[3*3];
    Vector3ToSkewSymmetric3(accelerometer, temp);
    MatrixMultiplySkewSymmetric3(Cbi, temp, 3, Phi21);
    MatrixScaleSelf(Phi21, -DT, 3, 3);  // Phi21 = Cbi * ssa * DT

    float PhiPPhit11[3*3], PhiPPhit12[3*3], PhiPPhit13[3*3];
    float PhiPPhit21[3*3], PhiPPhit22[3*3], PhiPPhit23[3*3];
    float PhiPPhit31[3*3], PhiPPhit32[3*3], PhiPPhit33[3*3];

    // PhiPPhit11
    MatrixMultiply(Phi11, P11, 3, 3, 3, temp);
    MatrixMultiplyByTranspose(temp, Phi11, 3, 3, 3, PhiPPhit11);

    // PhiPPhit12
    MatrixMultiplyByTranspose(temp, Phi21, 3, 3, 3, PhiPPhit12);
    MatrixMultiply(Phi11, P12, 3, 3, 3, temp);
    MatrixAddToSelf(PhiPPhit12, temp, 3, 3);

    // PhiPPhit13
    MatrixScaleSelf(temp, DT, 3, 3);
    MatrixMultiply(Phi11, P13, 3, 3, 3, PhiPPhit13);
    MatrixAddToSelf(PhiPPhit13, temp, 3, 3);

    // PhiPPhit21
    MatrixMultiply(Phi21, P11, 3, 3, 3, temp);
    MatrixAddToSelf(temp, P21, 3, 3);
    MatrixMultiplyByTranspose(temp, Phi11, 3, 3, 3, PhiPPhit21);

    // PhiPPhit22
    MatrixMultiplyByTranspose(temp, Phi21, 3, 3, 3, PhiPPhit22);
    MatrixMultiply(Phi21, P12, 3, 3, 3, temp);
    MatrixAddToSelf(temp, P22, 3, 3);
    MatrixAddToSelf(PhiPPhit22, temp, 3, 3);

    // PhiPPhit23
    MatrixScaleSelf(temp, DT, 3, 3);
    MatrixMultiply(Phi21, P13, 3, 3, 3, PhiPPhit23);
    MatrixAddToSelf(PhiPPhit23, P23, 3, 3);
    MatrixAddToSelf(PhiPPhit23, temp, 3, 3);

    // PhiPPhit31
    MatrixScale(P21, DT, 3, 3, temp);
    MatrixAddToSelf(temp, P31, 3, 3);
    MatrixMultiplyByTranspose(temp, Phi11, 3, 3, 3, PhiPPhit31);

    // PhiPPhit32
    MatrixMultiplyByTranspose(temp, Phi21, 3, 3, 3, PhiPPhit32);
    MatrixScale(P22, DT, 3, 3, temp);
    MatrixAddToSelf(temp, P32, 3, 3);
    MatrixAddToSelf(PhiPPhit32, temp, 3, 3);

    // PhiPPhit33
    MatrixScaleSelf(temp, DT, 3, 3);
    MatrixScale(P23, DT, 3, 3, PhiPPhit33);
    MatrixAddToSelf(PhiPPhit33, P33, 3, 3);
    MatrixAddToSelf(PhiPPhit33, temp, 3, 3);

    MatrixCopyToSubmatrix(PhiPPhit11, PhiPPhit, 0, 0, 3, 3, P_DIM);
    MatrixCopyToSubmatrix(PhiPPhit12, PhiPPhit, 0, 3, 3, 3, P_DIM);
    MatrixCopyToSubmatrix(PhiPPhit13, PhiPPhit, 0, 6, 3, 3, P_DIM);
    MatrixCopyToSubmatrix(PhiPPhit21, PhiPPhit, 3, 0, 3, 3, P_DIM);
    MatrixCopyToSubmatrix(PhiPPhit22, PhiPPhit, 3, 3, 3, 3, P_DIM);
    MatrixCopyToSubmatrix(PhiPPhit23, PhiPPhit, 3, 6, 3, 3, P_DIM);
    MatrixCopyToSubmatrix(PhiPPhit31, PhiPPhit, 6, 0, 3, 3, P_DIM);
    MatrixCopyToSubmatrix(PhiPPhit32, PhiPPhit, 6, 3, 3, 3, P_DIM);
    MatrixCopyToSubmatrix(PhiPPhit33, PhiPPhit, 6, 6, 3, 3, P_DIM);
  }

  // P_pred = Phi*P*Phi^t + Gamma*Q*Gamma^t + Qprime
  float GammaQGammat[P_DIM*P_DIM] = { 0 };
  {
    // Gamma = B * dt
    // const float Gamma[P_DIM * U_DIM] = {
    //   -1*DT,     0,     0,          0,          0,          0,
    //       0, -1*DT,     0,          0,          0,          0,
    //       0,     0, -1*DT,          0,          0,          0,
    //       0,     0,     0, -Cbi[0]*DT, -Cbi[1]*DT, -Cbi[2]*DT,
    //       0,     0,     0, -Cbi[3]*DT, -Cbi[4]*DT, -Cbi[5]*DT,
    //       0,     0,     0, -Cbi[6]*DT, -Cbi[7]*DT, -Cbi[8]*DT,
    //       0,     0,     0,          0,          0,          0,
    //       0,     0,     0,          0,          0,          0,
    //       0,     0,     0,          0,          0,          0,
    // };

    // const float QDiag[U_DIM] = {
    //   KALMAN_SIGMA_GYRO * KALMAN_SIGMA_GYRO,
    //   KALMAN_SIGMA_GYRO * KALMAN_SIGMA_GYRO,
    //   KALMAN_SIGMA_GYRO * KALMAN_SIGMA_GYRO,
    //   KALMAN_SIGMA_ACCELEROMETER_X * KALMAN_SIGMA_ACCELEROMETER_X,
    //   KALMAN_SIGMA_ACCELEROMETER_Y * KALMAN_SIGMA_ACCELEROMETER_Y,
    //   KALMAN_SIGMA_ACCELEROMETER_Z * KALMAN_SIGMA_ACCELEROMETER_Z,
    // };

    const float GammaQGammat11[3*3] = {
      -DT*KALMAN_SIGMA_GYRO*KALMAN_SIGMA_GYRO*-DT, 0, 0,
      0, -DT*KALMAN_SIGMA_GYRO*KALMAN_SIGMA_GYRO*-DT, 0,
      0, 0, -DT*KALMAN_SIGMA_GYRO*KALMAN_SIGMA_GYRO*-DT,
    };

    const float Gamma22[3*3] = {
      -Cbi[0]*DT, -Cbi[1]*DT, -Cbi[2]*DT,
      -Cbi[3]*DT, -Cbi[4]*DT, -Cbi[5]*DT,
      -Cbi[6]*DT, -Cbi[7]*DT, -Cbi[8]*DT,
    };

    float GammaQGammat22[3*3];
    const float Q22Gamma22t[3*3] = {
      -Cbi[0]*DT*KALMAN_SIGMA_ACCELEROMETER_X*KALMAN_SIGMA_ACCELEROMETER_X,
      -Cbi[3]*DT*KALMAN_SIGMA_ACCELEROMETER_X*KALMAN_SIGMA_ACCELEROMETER_X,
      -Cbi[6]*DT*KALMAN_SIGMA_ACCELEROMETER_X*KALMAN_SIGMA_ACCELEROMETER_X,
      -Cbi[1]*DT*KALMAN_SIGMA_ACCELEROMETER_Y*KALMAN_SIGMA_ACCELEROMETER_Y,
      -Cbi[4]*DT*KALMAN_SIGMA_ACCELEROMETER_Y*KALMAN_SIGMA_ACCELEROMETER_Y,
      -Cbi[7]*DT*KALMAN_SIGMA_ACCELEROMETER_Y*KALMAN_SIGMA_ACCELEROMETER_Y,
      -Cbi[2]*DT*KALMAN_SIGMA_ACCELEROMETER_Z*KALMAN_SIGMA_ACCELEROMETER_Z,
      -Cbi[5]*DT*KALMAN_SIGMA_ACCELEROMETER_Z*KALMAN_SIGMA_ACCELEROMETER_Z,
      -Cbi[8]*DT*KALMAN_SIGMA_ACCELEROMETER_Z*KALMAN_SIGMA_ACCELEROMETER_Z,
    };
    MatrixMultiply(Gamma22, Q22Gamma22t, 3, 3, 3, GammaQGammat22);

    MatrixCopyToSubmatrix(GammaQGammat11, GammaQGammat, 0, 0, 3, 3, 9);
    MatrixCopyToSubmatrix(GammaQGammat22, GammaQGammat, 3, 3, 3, 3, 9);
  }

  const float QprimeDiag[P_DIM] = { KALMAN_Q_PRIME_ALPHA * DT,
    KALMAN_Q_PRIME_ALPHA * DT, KALMAN_Q_PRIME_ALPHA * DT,
    KALMAN_Q_PRIME_VELOCITY * DT, KALMAN_Q_PRIME_VELOCITY * DT,
    KALMAN_Q_PRIME_VELOCITY * DT, KALMAN_Q_PRIME_H_POSITION * DT,
    KALMAN_Q_PRIME_H_POSITION * DT, KALMAN_Q_PRIME_V_POSITION * DT };

  // P_pred = Phi*P*Phi^t + Gamma*Q*Gamma^t + Qprime
  // recall that (float *)P_pred = (float *)PhiPPhit
  MatrixAddDiagonalToSelf(MatrixAddToSelf(PhiPPhit, GammaQGammat, P_DIM, P_DIM),
    QprimeDiag, P_DIM);
}

// -----------------------------------------------------------------------------
static void AccelerometerUpdate(const float * x_pred, const float * P_pred,
  const float * accelerometer, float * x_est, float * P_est)
{
  const float * quat_pred = &x_pred[0]; // predicted attitude quaternion
  const float R_diag[3] = {
    KALMAN_SIGMA_ACCELEROMETER_G * KALMAN_SIGMA_ACCELEROMETER_G,
    KALMAN_SIGMA_ACCELEROMETER_G * KALMAN_SIGMA_ACCELEROMETER_G,
    KALMAN_SIGMA_ACCELEROMETER_G * KALMAN_SIGMA_ACCELEROMETER_G,
  };

  float C[3 * 3], U[3 * 3], D[3 * 3], E[3 * 3], F[3 * 3], T[3 * 3];
  float temp[3 * 3];
  QuaternionToDCM(quat_pred, C);
  MatrixMultiply(C, accelerometer, 3, 3, 1, temp);
  Vector3ToSkewSymmetric3(temp, U); // U = [(C*accelerometer) x]
  
  SubmatrixCopyToMatrix(P_pred, temp, 0, 0, 9, 3, 3);
  MatrixMultiply(U, temp, 3, 3, 3, D);  // D = U*P11
  
  SubmatrixCopyToMatrix(P_pred, temp, 0, 3, 9, 3, 3);
  MatrixMultiply(U, temp, 3, 3, 3, E);  // E = U*P12
  
  SubmatrixCopyToMatrix(P_pred, temp, 0, 6, 9, 3, 3);
  MatrixMultiply(U, temp, 3, 3, 3, F);  // F = U*P13
  
  MatrixMultiplyByTranspose(D, U, 3, 3, 3, temp);
  MatrixAddDiagonalToSelf(temp, R_diag, 3);
  MatrixInverse(temp, 3, T); // T = (D*U^t+R_diag)^(-1)
  
  float K[P_DIM * 3];
  MatrixMultiplyTransposeBy(D, T, 3, 3, 3, &K[0]); // K1 = D^t*T
  MatrixMultiplyTransposeBy(E, T, 3, 3, 3, &K[9]); // K2 = E^t*T
  MatrixMultiplyTransposeBy(F, T, 3, 3, 3, &K[18]); // K3 = F^t*T

  float temp2[3 * 3], KHP[P_DIM * P_DIM];
  MatrixMultiply(&K[0], D, 3, 3, 3, temp); // K1D
  MatrixCopyToSubmatrix(temp, KHP, 0, 0, 3, 3, P_DIM);
  MatrixMultiply(&K[0], E, 3, 3, 3, temp); // K1E
  MatrixTranspose(temp, 3, 3, temp2);
  MatrixCopyToSubmatrix(temp, KHP, 0, 3, 3, 3, P_DIM);
  MatrixCopyToSubmatrix(temp2, KHP, 3, 0, 3, 3, P_DIM);
  MatrixMultiply(&K[0], F, 3, 3, 3, temp); // K1F
  MatrixTranspose(temp, 3, 3, temp2);
  MatrixCopyToSubmatrix(temp, KHP, 0, 6, 3, 3, P_DIM);
  MatrixCopyToSubmatrix(temp2, KHP, 6, 0, 3, 3, P_DIM);
  MatrixMultiply(&K[9], E, 3, 3, 3, temp); // K2E
  MatrixCopyToSubmatrix(temp, KHP, 3, 3, 3, 3, P_DIM);
  MatrixMultiply(&K[9], F, 3, 3, 3, temp); // K2F
  MatrixTranspose(temp, 3, 3, temp2);
  MatrixCopyToSubmatrix(temp, KHP, 3, 6, 3, 3, P_DIM);
  MatrixCopyToSubmatrix(temp2, KHP, 6, 3, 3, 3, P_DIM);
  MatrixMultiply(&K[18], F, 3, 3, 3, temp); // K3F
  MatrixCopyToSubmatrix(temp, KHP, 6, 6, 3, 3, P_DIM);
  ////
  // KHP = [K1D K1E K1F]
  //         [    K2E K2F]
  //         [sym.    K3F]
  ////

  MatrixSubtract(P_pred, KHP, P_DIM, P_DIM, P_est);
  	 
  // predicted measurement = C * [0 0 -G]^t
  const float predicted_measurement[3] = { C[0*3+2] * -G, C[1*3+2] * -G,
    C[2*3+2] * -G };

  MeasurementUpdateCommon(x_pred, accelerometer, predicted_measurement, K,
    x_est, 3);
}

// -----------------------------------------------------------------------------
static void BaroAltitudeUpdate(const float *x_pred, const float *P_pred,
  float baro_altitude, float *x_est, float *P_est)
{
  const float *r_pred = &x_pred[7]; // predicted position in i-frame

  const float R_diag[1] = { KALMAN_SIGMA_BARO * KALMAN_SIGMA_BARO };

  float P21[1*8], P22[1*1];
  SubmatrixCopyToMatrix(P_pred, P21, 8, 0, P_DIM, 1, 8);
  SubmatrixCopyToMatrix(P_pred, P22, 8, 8, P_DIM, 1, 1);

  float t = 1.0/(P22[0] + R_diag[0]);

  float K[P_DIM * 1];
  SubmatrixCopyToMatrix(P_pred, K, 0, 8, P_DIM, 9, 1);
  MatrixScaleSelf(K, t, P_DIM, 1);

  float KHP[P_DIM * P_DIM];
  float temp[8 * 8], temp2[1 * 8];
  MatrixMultiply(&K[0], P21, 8, 1, 8, temp);
  MatrixCopyToSubmatrix(temp, KHP, 0, 0, 8, 8, P_DIM);
  MatrixScale(&K[0], P22[0], 8, 1, temp);
  MatrixCopyToSubmatrix(temp, KHP, 0, 8, 8, 1, P_DIM);
  MatrixTranspose(temp, 8, 1, temp2);
  MatrixCopyToSubmatrix(temp2, KHP, 8, 0, 1, 8, P_DIM);
  KHP[8 * P_DIM + 8] = P22[0] * K[8];

  MatrixSubtract(P_pred, KHP, P_DIM, P_DIM, P_est);

  // predicted measurement = r_pred[2] + baro_altitude_offset;
  float predicted_measurement[1] = { r_pred[2] + baro_altitude_offset };

  MeasurementUpdateCommon(x_pred, &baro_altitude, predicted_measurement, K,
    x_est, 1);
}

// -----------------------------------------------------------------------------
static void VisionUpdate(const float * x_pred, const float * P_pred,
  const float * vision, float * x_est, float * P_est)
{
  const float * quat_pred = &x_pred[0]; // predicted attitude quaternion
  const float * velocity_pred = &x_pred[4];  // predicted velocity in i-frame

  const float R_diag[3] = {
    KALMAN_SIGMA_VISION * KALMAN_SIGMA_VISION,
    KALMAN_SIGMA_VISION * KALMAN_SIGMA_VISION,
    KALMAN_SIGMA_VISION * KALMAN_SIGMA_VISION,
  };

  float C[3 * 3], U[3 * 3], D[3 * 3], E[3 * 3], F[3 * 3], T[3 * 3];
  float temp[3 * 3], temp2[3 * 3], temp3[3 * 3];
  QuaternionToDCM(quat_pred, C);
  MatrixMultiply(C, velocity_pred, 3, 3, 1, temp);
  Vector3ToSkewSymmetric3(temp, U); // U = [(C*v^i) x]

  SubmatrixCopyToMatrix(P_pred, temp, 0, 0, 9, 3, 3);
  MatrixMultiply(U, temp, 3, 3, 3, temp2);
  SubmatrixCopyToMatrix(P_pred, temp, 3, 0, 9, 3, 3);
  MatrixMultiply(C, temp, 3, 3, 3, temp3);
  MatrixAdd(temp2, temp3, 3, 3, D); // D = U*P11 + C*P21
 
  SubmatrixCopyToMatrix(P_pred, temp, 0, 3, 9, 3, 3);
  MatrixMultiply(U, temp, 3, 3, 3, temp2);
  SubmatrixCopyToMatrix(P_pred, temp, 3, 3, 9, 3, 3);
  MatrixMultiply(C, temp, 3, 3, 3, temp3);
  MatrixAdd(temp2, temp3, 3, 3, E); // E = U*P12 + C*P22

  SubmatrixCopyToMatrix(P_pred, temp, 0, 6, 9, 3, 3);
  MatrixMultiply(U, temp, 3, 3, 3, temp2);
  SubmatrixCopyToMatrix(P_pred, temp, 3, 6, 9, 3, 3);
  MatrixMultiply(C, temp, 3, 3, 3, temp3);
  MatrixAdd(temp2, temp3, 3, 3, F); // F = U*P13 + C*P23

  MatrixMultiplyByTranspose(D, U, 3, 3, 3, temp);
  MatrixMultiplyByTranspose(E, C, 3, 3, 3, temp2);
  MatrixAdd(temp, temp2, 3, 3, temp3);
  MatrixAddDiagonalToSelf(temp3, R_diag, 3);
  MatrixInverse(temp3, 3, T); // T = (D*U^t+E*C^t+R_diag)^(-1)

  float K[P_DIM * 3];
  MatrixMultiplyTransposeBy(D, T, 3, 3, 3, &K[0]); // K1 = D^t*T
  MatrixMultiplyTransposeBy(E, T, 3, 3, 3, &K[9]); // K2 = E^t*T
  MatrixMultiplyTransposeBy(F, T, 3, 3, 3, &K[18]); // K3 = F^t*T

  float KHP[P_DIM * P_DIM];
  MatrixMultiply(&K[0], D, 3, 3, 3, temp); // K1D
  MatrixCopyToSubmatrix(temp, KHP, 0, 0, 3, 3, P_DIM);
  MatrixMultiply(&K[0], E, 3, 3, 3, temp); // K1E
  MatrixTranspose(temp, 3, 3, temp2);
  MatrixCopyToSubmatrix(temp, KHP, 0, 3, 3, 3, P_DIM);
  MatrixCopyToSubmatrix(temp2, KHP, 3, 0, 3, 3, P_DIM);
  MatrixMultiply(&K[0], F, 3, 3, 3, temp); // K1F
  MatrixTranspose(temp, 3, 3, temp2);
  MatrixCopyToSubmatrix(temp, KHP, 0, 6, 3, 3, P_DIM);
  MatrixCopyToSubmatrix(temp2, KHP, 6, 0, 3, 3, P_DIM);
  MatrixMultiply(&K[9], E, 3, 3, 3, temp); // K2E
  MatrixCopyToSubmatrix(temp, KHP, 3, 3, 3, 3, P_DIM);
  MatrixMultiply(&K[9], F, 3, 3, 3, temp); // K2F
  MatrixTranspose(temp, 3, 3, temp2);
  MatrixCopyToSubmatrix(temp, KHP, 3, 6, 3, 3, P_DIM);
  MatrixCopyToSubmatrix(temp2, KHP, 6, 3, 3, 3, P_DIM);
  MatrixMultiply(&K[18], F, 3, 3, 3, temp); // K3F
  MatrixCopyToSubmatrix(temp, KHP, 6, 6, 3, 3, P_DIM);
  ////
  // KHP = [K1D K1E K1F]
  //       [    K2E K2F]
  //       [sym.    K3F]
  ////

  MatrixSubtract(P_pred, KHP, P_DIM, P_DIM, P_est);

  // predicted measurement = C * velocity_pred
  float predicted_measurement[3];
  MatrixMultiply(C, velocity_pred, 3, 3, 1, predicted_measurement);

  MeasurementUpdateCommon(x_pred, vision, predicted_measurement, K, x_est, 3);
}

static void PositionUpdate(const float * x_pred, const float * P_pred,
  const float * position, float * x_est, float * P_est)
{
  const float *r_pred = &x_pred[7]; // predicted position in i-frame
  const float R_diag[3] = {
	  KALMAN_SIGMA_POSITION * KALMAN_SIGMA_POSITION,
	  KALMAN_SIGMA_POSITION * KALMAN_SIGMA_POSITION,
	  KALMAN_SIGMA_POSITION * KALMAN_SIGMA_POSITION,
  };
  
  float P13[3 * 3], P23[3 * 3], P33[3 * 3];
  SubmatrixCopyToMatrix(P_pred, P13, 0, 6, P_DIM, 3, 3);
  SubmatrixCopyToMatrix(P_pred, P23, 3, 6, P_DIM, 3, 3);
  SubmatrixCopyToMatrix(P_pred, P33, 6, 6, P_DIM, 3, 3);
  
  float temp[3 * 3], temp2[3 * 3];
  MatrixCopy(P33, 3, 3, temp);
  MatrixAddDiagonalToSelf(temp, R_diag, 3);
  MatrixInverse(temp, 3, temp2);
  
  float K[P_DIM * 3];
  MatrixMultiply(P13, temp2, 3, 3, 3, &K[0]); // P13*T
  MatrixMultiply(P23, temp2, 3, 3, 3, &K[9]); // P23*T
  MatrixMultiply(P33, temp2, 3, 3, 3, &K[18]); //P33*T
  
  float KHP[P_DIM * P_DIM];
  MatrixMultiplyByTranspose(&K[0], P13, 3, 3, 3, temp); // K1*P31
  MatrixCopyToSubmatrix(temp, KHP, 0, 0, 3, 3, P_DIM);
  MatrixMultiplyByTranspose(&K[0], P23, 3, 3, 3, temp); // K1*P32
  MatrixTranspose(temp, 3, 3, temp2);
  MatrixCopyToSubmatrix(temp, KHP, 0, 3, 3, 3, P_DIM);
  MatrixCopyToSubmatrix(temp2, KHP, 3, 0, 3, 3, P_DIM);
  MatrixMultiplyByTranspose(&K[0], P33, 3, 3, 3, temp); // K1*P33
  MatrixTranspose(temp, 3, 3, temp2);
  MatrixCopyToSubmatrix(temp, KHP, 0, 6, 3, 3, P_DIM);
  MatrixCopyToSubmatrix(temp2, KHP, 6, 0, 3, 3, P_DIM);
  MatrixMultiplyByTranspose(&K[9], P23, 3, 3, 3, temp); // K2*P32
  MatrixCopyToSubmatrix(temp, KHP, 3, 3, 3, 3, P_DIM);
  MatrixMultiplyByTranspose(&K[9], P33, 3, 3, 3, temp); // K2*P33
  MatrixTranspose(temp, 3, 3, temp2);
  MatrixCopyToSubmatrix(temp, KHP, 3, 6, 3, 3, P_DIM);
  MatrixCopyToSubmatrix(temp2, KHP, 6, 3, 3, 3, P_DIM);
  MatrixMultiplyByTranspose(&K[18], P33, 3, 3, 3, temp); // K3*P33
  MatrixCopyToSubmatrix(temp, KHP, 6, 6, 3, 3, P_DIM);
  ////
  // KHP = [K1*P31 K1*P32 K1*P33]
  //       [       K2*P32 K2*P33]
  //       [sym.          K3*P33]
  ////

  MatrixSubtract(P_pred, KHP, P_DIM, P_DIM, P_est);

  // predicted measurement = r_pred
  float predicted_measurement[3];
  MatrixCopy(r_pred, 3, 1, predicted_measurement);

  MeasurementUpdateCommon(x_pred, position, predicted_measurement, K, x_est, 3);
}

// -----------------------------------------------------------------------------
static void MeasurementUpdateCommon(const float * x_pred,
  const float * measurement, const float * predicted_measurement,
  const float * K, float * x_est, int z_dim)
{
  float residual[Z_DIM_MAX];
  VectorSubtract(measurement, predicted_measurement, z_dim, residual);

  float delta_x[P_DIM];
//////////////////////////////////////////////////////////////////////////////// 21
  MatrixMultiply(K, residual, P_DIM, z_dim, 1, delta_x);

  float dq[4];
  {
    const float * quat_pred = &x_pred[0];
    float Psi[4*3];
    Psi[0*3+0] = -quat_pred[1];
    Psi[0*3+1] = -quat_pred[2];
    Psi[0*3+2] = -quat_pred[3];
    Psi[1*3+0] = quat_pred[0];
    Psi[1*3+1] = -quat_pred[3];
    Psi[1*3+2] = quat_pred[2];
    Psi[2*3+0] = quat_pred[3];
    Psi[2*3+1] = quat_pred[0];
    Psi[2*3+2] = -quat_pred[1];
    Psi[3*3+0] = -quat_pred[2];
    Psi[3*3+1] = quat_pred[1];
    Psi[3*3+2] = quat_pred[0];

    const float * alpha = &delta_x[0];
//////////////////////////////////////////////////////////////////////////////// 12 + 4
    VectorScaleSelf(MatrixMultiply(Psi, alpha, 4, 3, 1, dq), 0.5, 4);
  }

  x_est[0] = dq[0];
  x_est[1] = dq[1];
  x_est[2] = dq[2];
  x_est[3] = dq[3];
  x_est[4] = delta_x[3];
  x_est[5] = delta_x[4];
  x_est[6] = delta_x[5];
  x_est[7] = delta_x[6];
  x_est[8] = delta_x[7];
  x_est[9] = delta_x[8];
  VectorAddToSelf(x_est, x_pred, X_DIM);

  QuaternionNormalizingFilter(&x_est[0]);  // normalize the quaternion portion
}

// -----------------------------------------------------------------------------
static float * MatrixMultiplySkewSymmetric3(const float * A, const float B[3*3],
  size_t A_rows, float * result)
{
  for (size_t i = 0; i < A_rows; i++)
  {
    for (size_t j = 0; j < 3; j++)
    {
      result[i * 3 + j] = 0;
      for (size_t k = 0; k < 3; k++)
      {
        if (j != k) result[i * 3 + j] += A[3 * i + k] * B[j + 3 * k];
      }
    }
  }

  return result;
}

// -----------------------------------------------------------------------------
static float * QuaternionToDCM(const float *quat, float * result)
{
  float temp;

  result[0*3+0] = quat[0] * quat[0];
  result[1*3+1] = quat[2] * quat[2];
  temp = quat[1] * quat[1];
  result[2*3+2] = 0.5 - temp - result[1*3+1];
  result[1*3+1] += result[0*3+0] - 0.5;
  result[0*3+0] += temp - 0.5;

  result[0*3+1] = quat[1] * quat[2];
  result[1*3+0] = result[0*3+1];
  temp = quat[0] * quat[3];
  result[1*3+0] -= temp;
  result[0*3+1] += temp;

  result[0*3+2] = quat[1] * quat[3];
  result[2*3+0] = result[0*3+2];
  temp = quat[0] * quat[2];
  result[2*3+0] += temp;
  result[0*3+2] -= temp;

  result[1*3+2] = quat[2] * quat[3];
  result[2*3+1] = result[1*3+2];
  temp = quat[0] * quat[1];
  result[2*3+1] -= temp;
  result[1*3+2] += temp;

  // Double the result.
  for (size_t i = 0; i < 3*3; i++) result[i] += result[i];

  return result;
}

// -----------------------------------------------------------------------------
// static float * SkewSymmetric3Transpose(const float A[3*3] , float result[3*3])
// {
//   result[0*3+0] = 0.0;
//   result[0*3+1] = -A[0*3+1];
//   result[0*3+2] = -A[0*3+2];

//   result[1*3+0] = -A[1*3+0];
//   result[1*3+1] = 0.0;
//   result[1*3+2] = -A[1*3+2];

//   result[2*3+0] = -A[2*3+0];
//   result[2*3+1] = -A[2*3+1];
//   result[2*3+2] = 0.0;

//   return result;
// }

// -----------------------------------------------------------------------------
static float * UpdateQuaternion(const float quat[4],
  const float angular_rate[3], float result[4])
{
  float dq[3];
  Vector3Scale(angular_rate, 0.5 * DT, dq);

  result[0] = -dq[0] * quat[1] - dq[1] * quat[2] - dq[2] * quat[3] + quat[0];
  result[1] =  dq[0] * quat[0] - dq[1] * quat[3] + dq[2] * quat[2] + quat[1];
  result[2] =  dq[0] * quat[3] + dq[1] * quat[0] - dq[2] * quat[1] + quat[2];
  result[3] = -dq[0] * quat[2] + dq[1] * quat[1] + dq[2] * quat[0] + quat[3];

  QuaternionNormalizingFilter(result);

  return result;
}

// -----------------------------------------------------------------------------
static float * Vector3ToSkewSymmetric3(const float v[3], float * result)
{
  result[0*3+0] = 0.0;
  result[0*3+1] = -v[2];
  result[0*3+2] = v[1];

  result[1*3+0] = v[2];
  result[1*3+1] = 0.0;
  result[1*3+2] = -v[0];

  result[2*3+0] = -v[1];
  result[2*3+1] = v[0];
  result[2*3+2] = 0.0;

  return result;
}
