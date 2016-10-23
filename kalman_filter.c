#include "kalman_filter.h"
#include <stdio.h>
#include <math.h>
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
#define KALMAN_SIGMA_ACCELEROMETER_X (0.005)
#define KALMAN_SIGMA_ACCELEROMETER_Y (0.005)
#define KALMAN_SIGMA_ACCELEROMETER_Z (0.042)
#define KALMAN_SIGMA_ACCELEROMETER_G (2.5)
#define KALMAN_SIGMA_BARO (0.68)
#define KALMAN_SIGMA_GYRO (0.007)
#define KALMAN_SIGMA_VISION (0.2)
//#define KALMAN_SIGMA_VISION (0.02)
#define KALMAN_SIGMA_POSITION (0.05)

static float heading_ = 0.0;
static float x_[X_DIM] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
static float P_[P_DIM*P_DIM] = { 0.0 };
static float baro_altitude_offset = 0.0;

static struct GpsHome{
  int32_t longitude;
  int32_t latitude;
  int32_t height;
  float cos_lat;
  float lon_scale;
  float lat_scale;
  float height_scale;
  float cos_xaxis;
  float sin_xaxis;
} __attribute__((packed)) gpshome_;

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
// -----------------------------------------------------------------------------
const float * KalmanPAlpha(void)
{
  static float palpha_[3 * 3];
  const float * ptr = (const float *) &palpha_;
  palpha_[0] = P_[0*P_DIM+0];
  palpha_[1] = P_[0*P_DIM+1];
  palpha_[2] = P_[0*P_DIM+2];
  palpha_[3] = P_[1*P_DIM+0];
  palpha_[4] = P_[1*P_DIM+1];
  palpha_[5] = P_[1*P_DIM+2];
  palpha_[6] = P_[2*P_DIM+0];
  palpha_[7] = P_[2*P_DIM+1];
  palpha_[8] = P_[2*P_DIM+2];
  return ptr;
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
static void GPSPositionUpdate(const float * x_pred, const float * P_pred,
  const int32_t longitude, const int32_t latitude,
  const int32_t height_mean_sea_level, const uint32_t horizontal_accuracy,
  const uint32_t vertical_accuracy, float * x_est, float * P_est);
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
void KalmanGPSPositionUpdate(const int32_t longitude, const int32_t latitude,
  const int32_t height_mean_sea_level, const uint32_t horizontal_accuracy,
  const uint32_t vertical_accuracy)
{
  float x_est[X_DIM];
  float P_est[P_DIM*P_DIM];
  GPSPositionUpdate(x_, P_, longitude, latitude, height_mean_sea_level,
    horizontal_accuracy, vertical_accuracy, x_est, P_est);
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
  P_[0*9+0] = 0.1;
  P_[1*9+1] = 0.1;
  P_[2*9+2] = 0.1;
  P_[3*9+3] = 0.1;
  P_[4*9+4] = 0.1;
  P_[5*9+5] = 0.1;
  P_[6*9+6] = 0.1;
  P_[7*9+7] = 0.1;
  P_[8*9+8] = 0.1;
}

// -----------------------------------------------------------------------------
void ResetKalmanBaroAltitudeOffset(float baro_altitude, float position_down)
{
  baro_altitude_offset = baro_altitude - (-position_down);
  x_[9] = position_down;
}

// -----------------------------------------------------------------------------
void SetGpsHome(const int32_t longitude, const int32_t latitude,
  const int32_t height_mean_sea_level, const float heading_xaxis)
{
  // WGS84 spheroid
  gpshome_.longitude = longitude;
  gpshome_.latitude = latitude;
  gpshome_.height = height_mean_sea_level;

  float lat = ((float)latitude)/10000000.0;
  gpshome_.cos_lat = cos(lat);
  gpshome_.lon_scale = 0.0111132954 * gpshome_.cos_lat;
  gpshome_.lat_scale = 0.0111132954 - 0.0000559822 * cos(2*lat) + 0.0000001175 * cos(4*lat);
  gpshome_.height_scale = 0.001;

  gpshome_.cos_xaxis = cos(heading_xaxis);
  gpshome_.sin_xaxis = sin(heading_xaxis);
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

  // P_pred = Phi*P*Phi^T + Gamma*Q*Gamma^T
  //float * PhiPPhit = P_pred;  // conserves memory
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

    // Phi = I + A * DT =
    //  [             I-[gyro x]*DT,    O, O ]
    //  [ -Cbi*[accelerometer x]*DT,    I, O ]
    //  [                         O, I*DT, I ]
    // Gamma = B * dt =
    // [ -I*DT,       O ]
    // [     O, -Cbi*DT ]
    // [     O,       O ]
    // Gamma * Q * Gamma^T =
    // [ Q_11*DT^2,                   O, O ]
    // [           O, Cbi*Q_22*Cbi*DT^2, O ]
    // [           O,                 O, O ]

    const float Phi11[3*3] = {
                1,    gyro[2]*DT,   -gyro[1]*DT,
      -gyro[2]*DT,             1,    gyro[0]*DT,
       gyro[1]*DT,   -gyro[0]*DT,             1,
    };

    float Phi21[3*3];
    Vector3ToSkewSymmetric3(accelerometer, temp);
    MatrixMultiplySkewSymmetric3(Cbi, temp, 3, Phi21);
    MatrixScaleSelf(Phi21, -DT, 3, 3);

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

    // GammaQGammat11
    float a = 0;
    const float GammaQGammat11[3*3] = {
      DT*KALMAN_SIGMA_GYRO*KALMAN_SIGMA_GYRO*DT+a, 0, 0,
      0, DT*KALMAN_SIGMA_GYRO*KALMAN_SIGMA_GYRO*DT+a, 0,
      0, 0, DT*KALMAN_SIGMA_GYRO*KALMAN_SIGMA_GYRO*DT+a,
    };

    // GammaQGammat22
    float b = 0;
    float GammaQGammat22[3*3];
    const float temp2[3*3] = {
     KALMAN_SIGMA_ACCELEROMETER_X*KALMAN_SIGMA_ACCELEROMETER_X*DT*DT+b, 0, 0,
     0, KALMAN_SIGMA_ACCELEROMETER_Y*KALMAN_SIGMA_ACCELEROMETER_Y*DT*DT+b, 0,
     0, 0, KALMAN_SIGMA_ACCELEROMETER_Z*KALMAN_SIGMA_ACCELEROMETER_Z*DT*DT+b,
    };
    MatrixMultiply(Cbi, temp2, 3, 3, 3, temp);
    MatrixMultiply(temp, Cbi, 3, 3, 3, GammaQGammat22);

    MatrixCopyToSubmatrix(MatrixAddToSelf(PhiPPhit11,GammaQGammat11,3,3),
      P_pred, 0, 0, 3, 3, P_DIM);
    MatrixCopyToSubmatrix(PhiPPhit12, P_pred, 0, 3, 3, 3, P_DIM);
    MatrixCopyToSubmatrix(PhiPPhit13, P_pred, 0, 6, 3, 3, P_DIM);
    MatrixCopyToSubmatrix(PhiPPhit21, P_pred, 3, 0, 3, 3, P_DIM);
    MatrixCopyToSubmatrix(MatrixAddToSelf(PhiPPhit22,GammaQGammat22,3,3),
      P_pred, 3, 3, 3, 3, P_DIM);
    MatrixCopyToSubmatrix(PhiPPhit23, P_pred, 3, 6, 3, 3, P_DIM);
    MatrixCopyToSubmatrix(PhiPPhit31, P_pred, 6, 0, 3, 3, P_DIM);
    MatrixCopyToSubmatrix(PhiPPhit32, P_pred, 6, 3, 3, 3, P_DIM);
    MatrixCopyToSubmatrix(PhiPPhit33, P_pred, 6, 6, 3, 3, P_DIM);
  }

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

  // predicted measurement = C * [0 0 -G]^t
  const float predicted_measurement[3] = { C[0*3+2] * -G, C[1*3+2] * -G,
    C[2*3+2] * -G };

  Vector3ToSkewSymmetric3(predicted_measurement, U); // U = [(C*accelerometer^i) x]

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

static void GPSPositionUpdate(const float * x_pred, const float * P_pred,
  const int32_t longitude, const int32_t latitude,
  const int32_t height_mean_sea_level, const uint32_t horizontal_accuracy,
  const uint32_t vertical_accuracy, float * x_est, float * P_est)
{
  float position[3];
  float x_ned = gpshome_.lon_scale * (longitude - gpshome_.longitude);
  float y_ned = gpshome_.lat_scale * (latitude - gpshome_.latitude);
  position[0] = gpshome_.cos_xaxis * x_ned - gpshome_.sin_xaxis * y_ned;
  position[1] = gpshome_.sin_xaxis * x_ned + gpshome_.cos_xaxis * y_ned;
  position[2] = gpshome_.height_scale * (height_mean_sea_level - gpshome_.height);

  const float *r_pred = &x_pred[7]; // predicted position in i-frame
  const float R_diag[3] = {
    horizontal_accuracy * horizontal_accuracy,
    horizontal_accuracy * horizontal_accuracy,
    vertical_accuracy * vertical_accuracy,
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

  // printf("predicted_measurement:%f,%f,%f\n",predicted_measurement[0],predicted_measurement[1],predicted_measurement[2]);
  // printf("measurement:%f,%f,%f\n",measurement[0],measurement[1],measurement[2]);
  // printf("residual:%f,%f,%f\n",residual[0],residual[1],residual[2]);

  float delta_x[P_DIM];
//////////////////////////////////////////////////////////////////////////////// 21
  MatrixMultiply(K, residual, P_DIM, z_dim, 1, delta_x);

  //printf("deltax1:3:%f,%f,%f\n",delta_x[0],delta_x[1],delta_x[2]);
  //printf("deltax4:6:%f,%f,%f\n",delta_x[4],delta_x[5],delta_x[6]);
  //printf("deltax7:9:%f,%f,%f\n",delta_x[7],delta_x[8],delta_x[9]);

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

  //QuaternionNormalizingFilter(&x_est[0]);  // normalize the quaternion portion
  QuaternionNormalize(&x_est[0]);
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
