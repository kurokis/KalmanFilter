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
#define KALMAN_SIGMA_ACCELEROMETER_G (0.25)
#define KALMAN_SIGMA_BARO (0.68)
#define KALMAN_SIGMA_GYRO (0.007)
#define KALMAN_SIGMA_VISION (0.02)
#define KALMAN_SIGMA_POSITION (0.05)

static float heading_ = 0.0;
static float x_[X_DIM] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
static struct P_{
  float aa[3*3];
  float av[3*3];
  float ar[3*3];
  float vv[3*3];
  float vr[3*3];
  float rr[3*3];
} p_;

static float baro_altitude_offset = 0.0;

static float dx[9]={0.0};

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
  palpha_[0] = p_.aa[0*3+0];
  palpha_[1] = p_.aa[0*3+1];
  palpha_[2] = p_.aa[0*3+2];
  palpha_[3] = p_.aa[1*3+0];
  palpha_[4] = p_.aa[1*3+1];
  palpha_[5] = p_.aa[1*3+2];
  palpha_[6] = p_.aa[2*3+0];
  palpha_[7] = p_.aa[2*3+1];
  palpha_[8] = p_.aa[2*3+2];
  return ptr;
}
// -----------------------------------------------------------------------------
const float * KalmanPVelocity(void)
{
  static float pvelocity_[3 * 3];
  const float * ptr = (const float *) &pvelocity_;
  pvelocity_[0] = p_.vv[0*3+0];
  pvelocity_[1] = p_.vv[0*3+1];
  pvelocity_[2] = p_.vv[0*3+2];
  pvelocity_[3] = p_.vv[1*3+0];
  pvelocity_[4] = p_.vv[1*3+1];
  pvelocity_[5] = p_.vv[1*3+2];
  pvelocity_[6] = p_.vv[2*3+0];
  pvelocity_[7] = p_.vv[2*3+1];
  pvelocity_[8] = p_.vv[2*3+2];
  return ptr;
}

// -----------------------------------------------------------------------------
const float * KalmanPPosition(void)
{
  static float pposition_[3 * 3];
  const float * ptr = (const float *) &pposition_;
  pposition_[0] = p_.rr[0*3+0];
  pposition_[1] = p_.rr[0*3+1];
  pposition_[2] = p_.rr[0*3+2];
  pposition_[3] = p_.rr[1*3+0];
  pposition_[4] = p_.rr[1*3+1];
  pposition_[5] = p_.rr[1*3+2];
  pposition_[6] = p_.rr[2*3+0];
  pposition_[7] = p_.rr[2*3+1];
  pposition_[8] = p_.rr[2*3+2];
  return ptr;
}

// =============================================================================
// Private function declarations:

static void TimeUpdate(const float * x_est_prev, const struct P_ * P_est_prev,
  const float * gyro, const float * accelerometer, float * x_pred,
  struct P_ * P_pred);
static void AccelerometerUpdate(const float * x_pred, const struct P_ * P_pred,
  const float * accelerometer, float * x_est, struct P_ * P_est);
//static void BaroAltitudeUpdate(const float *x_pred, const float *P_pred,
//  float baro_altitude, float *x_est, float *P_est);
static void VisionUpdate(const float * x_pred, const struct P_ * P_pred,
  const float * vision, float * x_est, struct P_ * P_est);
static void PositionUpdate(const float * x_pred, const struct P_ * P_pred,
  const float * position, float * x_est, struct P_ * P_est);
static void GPSPositionUpdate(const float * x_pred, const struct P_ * P_pred,
  const int32_t longitude, const int32_t latitude,
  const int32_t height_mean_sea_level, const uint32_t horizontal_accuracy,
  const uint32_t vertical_accuracy, float * x_est, struct P_ * P_est);
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
  TimeUpdate(x_, &p_, gyro, accelerometer, x_pred, &p_);
  VectorCopy(x_pred, X_DIM, x_);
}

// -----------------------------------------------------------------------------
void KalmanAccelerometerUpdate(const float accelerometer[3])
{
  float x_est[X_DIM];
  struct P_ P_est;
  AccelerometerUpdate(x_, &p_, accelerometer, x_est, &P_est);
  VectorCopy(x_est, X_DIM, x_);
  MatrixCopy(P_est.aa, 3, 3, p_.aa);
  MatrixCopy(P_est.av, 3, 3, p_.av);
  MatrixCopy(P_est.ar, 3, 3, p_.ar);
  MatrixCopy(P_est.vv, 3, 3, p_.vv);
  MatrixCopy(P_est.vr, 3, 3, p_.vr);
  MatrixCopy(P_est.rr, 3, 3, p_.rr);
}

//// -----------------------------------------------------------------------------
//void KalmanBaroAltitudeUpdate(float baro_altitude)
//{
//  float x_est[X_DIM];
//  float P_est[P_DIM*P_DIM];
//  BaroAltitudeUpdate(x_, P_, baro_altitude, x_est, P_est);
//  VectorCopy(x_est, X_DIM, x_);
//  MatrixCopy(P_est, P_DIM, P_DIM, P_);
//}

// -----------------------------------------------------------------------------
void KalmanVisionUpdate(const float vision[3])
{
  float x_est[X_DIM];
  struct P_ P_est;
  VisionUpdate(x_, &p_, vision, x_est, &P_est);
  VectorCopy(x_est, X_DIM, x_);
  MatrixCopy(P_est.aa, 3, 3, p_.aa);
  MatrixCopy(P_est.av, 3, 3, p_.av);
  MatrixCopy(P_est.ar, 3, 3, p_.ar);
  MatrixCopy(P_est.vv, 3, 3, p_.vv);
  MatrixCopy(P_est.vr, 3, 3, p_.vr);
  MatrixCopy(P_est.rr, 3, 3, p_.rr);
}

// -----------------------------------------------------------------------------
void KalmanPositionUpdate(const float position[3])
{
  float x_est[X_DIM];
  struct P_ P_est;
  PositionUpdate(x_, &p_, position, x_est, &P_est);
  VectorCopy(x_est, X_DIM, x_);
  MatrixCopy(P_est.aa, 3, 3, p_.aa);
  MatrixCopy(P_est.av, 3, 3, p_.av);
  MatrixCopy(P_est.ar, 3, 3, p_.ar);
  MatrixCopy(P_est.vv, 3, 3, p_.vv);
  MatrixCopy(P_est.vr, 3, 3, p_.vr);
  MatrixCopy(P_est.rr, 3, 3, p_.rr);
}

// -----------------------------------------------------------------------------
void KalmanGPSPositionUpdate(const int32_t longitude, const int32_t latitude,
  const int32_t height_mean_sea_level, const uint32_t horizontal_accuracy,
  const uint32_t vertical_accuracy)
{
  float x_est[X_DIM];
  struct P_ P_est;
  GPSPositionUpdate(x_, &p_, longitude, latitude, height_mean_sea_level,
    horizontal_accuracy, vertical_accuracy, x_est, &P_est);
  VectorCopy(x_est, X_DIM, x_);
  MatrixCopy(P_est.aa, 3, 3, p_.aa);
  MatrixCopy(P_est.av, 3, 3, p_.av);
  MatrixCopy(P_est.ar, 3, 3, p_.ar);
  MatrixCopy(P_est.vv, 3, 3, p_.vv);
  MatrixCopy(P_est.vr, 3, 3, p_.vr);
  MatrixCopy(P_est.rr, 3, 3, p_.rr);
}

// -----------------------------------------------------------------------------
void ResetKalman(void)
{
  heading_ = 0.0;
  x_[0] = 1.0;
  for (size_t i = 10; --i; ) x_[i] = 0.0;
  for (size_t i = 3 * 3; --i; ) p_.aa[i] = 0.0;
  for (size_t i = 3 * 3; --i; ) p_.av[i] = 0.0;
  for (size_t i = 3 * 3; --i; ) p_.ar[i] = 0.0;
  for (size_t i = 3 * 3; --i; ) p_.vv[i] = 0.0;
  for (size_t i = 3 * 3; --i; ) p_.vr[i] = 0.0;
  for (size_t i = 3 * 3; --i; ) p_.rr[i] = 0.0;
  p_.aa[0*3+0] = 0.00001;
  p_.aa[1*3+1] = 0.00001;
  p_.aa[2*3+2] = 0.00001;
  p_.vv[0*3+0] = 0.00001;
  p_.vv[1*3+1] = 0.00001;
  p_.vv[2*3+2] = 0.00001;
  p_.rr[0*3+0] = 0.00001;
  p_.rr[1*3+1] = 0.00001;
  p_.rr[2*3+2] = 0.00001;
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

static void TimeUpdate(const float * x_est_prev, const struct P_ * P_est_prev,
  const float * gyro, const float * accelerometer, float * x_pred,
  struct P_ * P_pred)
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
  float Cbi[3 * 3], temp[3*3], temp2[3*3];
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
    const float * P11 = P_est_prev->aa;
    const float * P12 = P_est_prev->av;
    const float * P13 = P_est_prev->ar;
    const float * P22 = P_est_prev->vv;
    const float * P23 = P_est_prev->vr;
    const float * P33 = P_est_prev->rr;

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
    MatrixMultiply(Phi11, P12, 3, 3, 3, temp); // this will be reused in "13"
    MatrixAddToSelf(PhiPPhit12, temp, 3, 3);

    // PhiPPhit13
    MatrixScaleSelf(temp, DT, 3, 3);
    MatrixMultiply(Phi11, P13, 3, 3, 3, PhiPPhit13);
    MatrixAddToSelf(PhiPPhit13, temp, 3, 3);

    // PhiPPhit22
    //MatrixMultiplyByTranspose(temp, Phi21, 3, 3, 3, PhiPPhit22);
    MatrixMultiply(Phi21, P11, 3, 3, 3, temp);
    MatrixAddToSelf(temp, MatrixTranspose(P12, 3, 3, temp2), 3, 3);
    MatrixMultiply(temp, Phi21, 3, 3, 3, PhiPPhit22); // (Phi21*P11+P12^t)*Phi21
    MatrixMultiply(Phi21, P12, 3, 3, 3, temp);
    MatrixAddToSelf(temp, P22, 3, 3); // Phi21*P12+P22 this will be reused in "23"
    MatrixAddToSelf(PhiPPhit22, temp, 3, 3);

    // PhiPPhit23
    MatrixScaleSelf(temp, DT, 3, 3);
    MatrixMultiply(Phi21, P13, 3, 3, 3, PhiPPhit23);
    MatrixAddToSelf(PhiPPhit23, P23, 3, 3);
    MatrixAddToSelf(PhiPPhit23, temp, 3, 3);

    // PhiPPhit33
    MatrixScale(P22, DT, 3, 3, temp);
    MatrixAddToSelf(temp, MatrixTranspose(P23, 3, 3, temp2), 3, 3); // P22*DT+P23^t
    MatrixScaleSelf(temp, DT, 3, 3);
    MatrixScale(P23, DT, 3, 3, PhiPPhit33);
    MatrixAddToSelf(PhiPPhit33, P33, 3, 3); // P23*DT + P33
    MatrixAddToSelf(PhiPPhit33, temp, 3, 3);

    // GammaQGammat11
    float a = 0; // Qprime terms
    const float GammaQGammat11[3*3] = {
      DT*KALMAN_SIGMA_GYRO*KALMAN_SIGMA_GYRO*DT+a, 0, 0,
      0, DT*KALMAN_SIGMA_GYRO*KALMAN_SIGMA_GYRO*DT+a, 0,
      0, 0, DT*KALMAN_SIGMA_GYRO*KALMAN_SIGMA_GYRO*DT+a,
    };

    // GammaQGammat22
    float b = 0; // Qprime terms
    float GammaQGammat22[3*3];
    const float temp2[3*3] = {
     KALMAN_SIGMA_ACCELEROMETER_X*KALMAN_SIGMA_ACCELEROMETER_X*DT*DT+b, 0, 0,
     0, KALMAN_SIGMA_ACCELEROMETER_Y*KALMAN_SIGMA_ACCELEROMETER_Y*DT*DT+b, 0,
     0, 0, KALMAN_SIGMA_ACCELEROMETER_Z*KALMAN_SIGMA_ACCELEROMETER_Z*DT*DT+b,
    };
    MatrixMultiply(Cbi, temp2, 3, 3, 3, temp);
    MatrixMultiply(temp, Cbi, 3, 3, 3, GammaQGammat22);

    // Qprime33
    float c = 0; // Qprime terms
    const float Qprime33[3*3] = {
      c, 0, 0,
      0, c, 0,
      0, 0, c,
    };

    MatrixAdd(PhiPPhit11,GammaQGammat11,3,3,P_pred->aa);
    MatrixCopy(PhiPPhit12,3,3,P_pred->av);
    MatrixCopy(PhiPPhit13,3,3,P_pred->ar);
    MatrixAdd(PhiPPhit22,GammaQGammat22,3,3,P_pred->vv);
    MatrixCopy(PhiPPhit23,3,3,P_pred->vr);
    //MatrixCopy(PhiPPhit33,3,3,P_pred->rr);
    MatrixAdd(PhiPPhit33,Qprime33,3,3,P_pred->rr);
  }
}

// -----------------------------------------------------------------------------
static void AccelerometerUpdate(const float * x_pred, const struct P_ * P_pred,
  const float * accelerometer, float * x_est, struct P_ * P_est)
{
  const float * quat_pred = &x_pred[0]; // predicted attitude quaternion
  const float R_diag[3] = {
    KALMAN_SIGMA_ACCELEROMETER_G * KALMAN_SIGMA_ACCELEROMETER_G,
    KALMAN_SIGMA_ACCELEROMETER_G * KALMAN_SIGMA_ACCELEROMETER_G,
    KALMAN_SIGMA_ACCELEROMETER_G * KALMAN_SIGMA_ACCELEROMETER_G,
  };
  //const float R_diag[3] = {
  //  KALMAN_SIGMA_ACCELEROMETER_X * KALMAN_SIGMA_ACCELEROMETER_X,
  //  KALMAN_SIGMA_ACCELEROMETER_Y * KALMAN_SIGMA_ACCELEROMETER_Y,
  //  KALMAN_SIGMA_ACCELEROMETER_Z * KALMAN_SIGMA_ACCELEROMETER_Z,
  //};

  const float * Paa = P_pred->aa;
  const float * Pav = P_pred->av;
  const float * Par = P_pred->ar;
  const float * Pvv = P_pred->vv;
  const float * Pvr = P_pred->vr;
  const float * Prr = P_pred->rr;

  float C[3 * 3], U[3 * 3], D[3 * 3], E[3 * 3], F[3 * 3], T[3 * 3];
  float temp[3 * 3];

  QuaternionToDCM(quat_pred, C);
  const float predicted_measurement[3] = { C[0*3+2] * -G, C[1*3+2] * -G,
    C[2*3+2] * -G }; // predicted measurement = C * [0 0 -G]^t
  Vector3ToSkewSymmetric3(predicted_measurement, U); // U = [(C*accelerometer^i) x]
  MatrixMultiply(U, Paa, 3, 3, 3, D);  // D = U*Paa
  MatrixMultiply(U, Pav, 3, 3, 3, E);  // E = U*Pav
  MatrixMultiply(U, Par, 3, 3, 3, F);  // F = U*Par
  MatrixMultiplyByTranspose(D, U, 3, 3, 3, temp);
  MatrixAddDiagonalToSelf(temp, R_diag, 3);
  MatrixInverse(temp, 3, T); // T = (D*U^t+R_diag)^(-1)

  float K[P_DIM * 3];
  float * K1 = &K[0];
  float * K2 = &K[9];
  float * K3 = &K[18];

  MatrixMultiplyTransposeBy(D, T, 3, 3, 3, K1); // K1 = D^t*T
  MatrixMultiplyTransposeBy(E, T, 3, 3, 3, K2); // K2 = E^t*T
  MatrixMultiplyTransposeBy(F, T, 3, 3, 3, K3); // K3 = F^t*T

  MatrixMultiply(K1, D, 3, 3, 3, temp); // K1D
  MatrixSubtract(Paa, temp, 3, 3, P_est->aa);
  MatrixMultiply(K1, E, 3, 3, 3, temp); // K1E
  MatrixSubtract(Pav, temp, 3, 3, P_est->av);
  MatrixMultiply(K1, F, 3, 3, 3, temp); // K1F
  MatrixSubtract(Par, temp, 3, 3, P_est->ar);
  MatrixMultiply(K2, E, 3, 3, 3, temp); // K2E
  MatrixSubtract(Pvv, temp, 3, 3, P_est->vv);
  MatrixMultiply(K2, F, 3, 3, 3, temp); // K2F
  MatrixSubtract(Pvr, temp, 3, 3, P_est->vr);
  MatrixMultiply(K3, F, 3, 3, 3, temp); // K3F
  MatrixSubtract(Prr, temp, 3, 3, P_est->rr);
  ////
  // KHP = [K1D K1E K1F]
  //         [    K2E K2F]
  //         [sym.    K3F]
  ////

  MeasurementUpdateCommon(x_pred, accelerometer, predicted_measurement, K,
    x_est, 3);
}

//// -----------------------------------------------------------------------------
//static void BaroAltitudeUpdate(const float *x_pred, const float *P_pred,
//  float baro_altitude, float *x_est, float *P_est)
//{
//  const float *r_pred = &x_pred[7]; // predicted position in i-frame

//  const float R_diag[1] = { KALMAN_SIGMA_BARO * KALMAN_SIGMA_BARO };

//  float P21[1*8], P22[1*1];
//  SubmatrixCopyToMatrix(P_pred, P21, 8, 0, P_DIM, 1, 8);
//  SubmatrixCopyToMatrix(P_pred, P22, 8, 8, P_DIM, 1, 1);

//  float t = 1.0/(P22[0] + R_diag[0]);

//  float K[P_DIM * 1];
//  SubmatrixCopyToMatrix(P_pred, K, 0, 8, P_DIM, 9, 1);
//  MatrixScaleSelf(K, t, P_DIM, 1);

//  float KHP[P_DIM * P_DIM];
//  float temp[8 * 8], temp2[1 * 8];
//  MatrixMultiply(&K[0], P21, 8, 1, 8, temp);
//  MatrixCopyToSubmatrix(temp, KHP, 0, 0, 8, 8, P_DIM);
//  MatrixScale(&K[0], P22[0], 8, 1, temp);
//  MatrixCopyToSubmatrix(temp, KHP, 0, 8, 8, 1, P_DIM);
//  MatrixTranspose(temp, 8, 1, temp2);
//  MatrixCopyToSubmatrix(temp2, KHP, 8, 0, 1, 8, P_DIM);
//  KHP[8 * P_DIM + 8] = P22[0] * K[8];

//  MatrixSubtract(P_pred, KHP, P_DIM, P_DIM, P_est);

//  // predicted measurement = r_pred[2] + baro_altitude_offset;
//  float predicted_measurement[1] = { r_pred[2] + baro_altitude_offset };

//  MeasurementUpdateCommon(x_pred, &baro_altitude, predicted_measurement, K,
//    x_est, 1);
//}

//// -----------------------------------------------------------------------------
static void VisionUpdate(const float * x_pred, const struct P_ * P_pred,
  const float * vision, float * x_est, struct P_ * P_est)
{
  const float * quat_pred = &x_pred[0]; // predicted attitude quaternion
  const float * velocity_pred = &x_pred[4];  // predicted velocity in i-frame

  const float R_diag[3] = {
    KALMAN_SIGMA_VISION * KALMAN_SIGMA_VISION,
    KALMAN_SIGMA_VISION * KALMAN_SIGMA_VISION,
    KALMAN_SIGMA_VISION * KALMAN_SIGMA_VISION,
  };

  const float * Paa = P_pred->aa;
  const float * Pav = P_pred->av;
  const float * Par = P_pred->ar;
  const float * Pvv = P_pred->vv;
  const float * Pvr = P_pred->vr;
  const float * Prr = P_pred->rr;

  float C[3 * 3], U[3 * 3], D[3 * 3], E[3 * 3], F[3 * 3], T[3 * 3];
  float predicted_measurement[3];
  float temp[3 * 3], temp2[3 * 3];

  QuaternionToDCM(quat_pred, C);
  MatrixMultiply(C, velocity_pred, 3, 3, 1, predicted_measurement);
  Vector3ToSkewSymmetric3(predicted_measurement, U); // U = [(C*v^i) x]

  MatrixMultiply(U, Paa, 3, 3, 3, temp);
  MatrixMultiplyByTranspose(C, Pav, 3, 3, 3, temp2);
  MatrixAdd(temp, temp2, 3, 3, D); // D = U*Paa + C*Pva

  MatrixMultiply(U, Pav, 3, 3, 3, temp);
  MatrixMultiply(C, Pvv, 3, 3, 3, temp2);
  MatrixAdd(temp, temp2, 3, 3, E); // E = U*Pav + C*Pvv

  MatrixMultiply(U, Par, 3, 3, 3, temp);
  MatrixMultiply(C, Pvr, 3, 3, 3, temp2);
  MatrixAdd(temp, temp2, 3, 3, F); // F = U*Par + C*Pvr

  MatrixMultiplyByTranspose(D, U, 3, 3, 3, temp);
  MatrixMultiplyByTranspose(E, C, 3, 3, 3, temp2);
  MatrixAddToSelf(temp, temp2, 3, 3);
  MatrixAddDiagonalToSelf(temp, R_diag, 3);
  MatrixInverse(temp, 3, T); // T = (D*U^t+E*C^t+R_diag)^(-1)

  float K[P_DIM * 3];
  float * K1 = &K[0];
  float * K2 = &K[9];
  float * K3 = &K[18];

  MatrixMultiplyTransposeBy(D, T, 3, 3, 3, K1); // K1 = D^t*T
  MatrixMultiplyTransposeBy(E, T, 3, 3, 3, K2); // K2 = E^t*T
  MatrixMultiplyTransposeBy(F, T, 3, 3, 3, K3); // K3 = F^t*T

  MatrixMultiply(K1, D, 3, 3, 3, temp); // K1D
  MatrixSubtract(Paa, temp, 3, 3, P_est->aa);
  MatrixMultiply(K1, E, 3, 3, 3, temp); // K1E
  MatrixSubtract(Pav, temp, 3, 3, P_est->av);
  MatrixMultiply(K1, F, 3, 3, 3, temp); // K1F
  MatrixSubtract(Par, temp, 3, 3, P_est->ar);
  MatrixMultiply(K2, E, 3, 3, 3, temp); // K2E
  MatrixSubtract(Pvv, temp, 3, 3, P_est->vv);
  MatrixMultiply(K2, F, 3, 3, 3, temp); // K2F
  MatrixSubtract(Pvr, temp, 3, 3, P_est->vr);
  MatrixMultiply(K3, F, 3, 3, 3, temp); // K3F
  MatrixSubtract(Prr, temp, 3, 3, P_est->rr);
  ////
  // KHP = [K1D K1E K1F]
  //       [    K2E K2F]
  //       [sym.    K3F]
  ////

  MeasurementUpdateCommon(x_pred, vision, predicted_measurement, K, x_est, 3);
}

static void PositionUpdate(const float * x_pred, const struct P_ * P_pred,
  const float * position, float * x_est, struct P_ * P_est)
{
  const float *r_pred = &x_pred[7]; // predicted position in i-frame
  const float R_diag[3] = {
    KALMAN_SIGMA_POSITION * KALMAN_SIGMA_POSITION,
    KALMAN_SIGMA_POSITION * KALMAN_SIGMA_POSITION,
    KALMAN_SIGMA_POSITION * KALMAN_SIGMA_POSITION,
  };

  const float * Paa = P_pred->aa;
  const float * Pav = P_pred->av;
  const float * Par = P_pred->ar;
  const float * Pvv = P_pred->vv;
  const float * Pvr = P_pred->vr;
  const float * Prr = P_pred->rr;

  float temp[3 * 3], T[3 * 3];
  MatrixCopy(Prr, 3, 3, temp);
  MatrixAddDiagonalToSelf(temp, R_diag, 3);
  MatrixInverse(temp, 3, T); // T = (Prr+R)^(-1)

  float K[P_DIM * 3];
  float * K1 = &K[0];
  float * K2 = &K[9];
  float * K3 = &K[18];

  MatrixMultiply(Par, T, 3, 3, 3, K1); // K1 = Par*T
  MatrixMultiply(Pvr, T, 3, 3, 3, K2); // K2 = Pvr*T
  MatrixMultiply(Prr, T, 3, 3, 3, K3); // K3 = Prr*T

  MatrixMultiplyByTranspose(K1, Par, 3, 3, 3, temp); // K1*Pra
  MatrixSubtract(Paa, temp, 3, 3, P_est->aa);
  MatrixMultiplyByTranspose(K1, Pvr, 3, 3, 3, temp); // K1*Prv
  MatrixSubtract(Pav, temp, 3, 3, P_est->av);
  MatrixMultiply(K1, Prr, 3, 3, 3, temp); // K1*Prr
  MatrixSubtract(Par, temp, 3, 3, P_est->ar);
  MatrixMultiplyByTranspose(K2, Pvr, 3, 3, 3, temp); // K2*Prv
  MatrixSubtract(Pvv, temp, 3, 3, P_est->vv);
  MatrixMultiply(K2, Prr, 3, 3, 3, temp); // K2*Prr
  MatrixSubtract(Pvr, temp, 3, 3, P_est->vr);
  MatrixMultiply(K3, Prr, 3, 3, 3, temp); // K3*Prr
  MatrixSubtract(Prr, temp, 3, 3, P_est->rr);
  ////
  // KHP = [Ka*Pra Ka*Prv Ka*Prr]
  //       [       Kv*Prv Kv*Prr]
  //       [sym.          Kr*Prr]
  ////

  // predicted measurement = r_pred
  float predicted_measurement[3];
  MatrixCopy(r_pred, 3, 1, predicted_measurement);

  MeasurementUpdateCommon(x_pred, position, predicted_measurement, K, x_est, 3);
}

static void GPSPositionUpdate(const float * x_pred, const struct P_ * P_pred,
  const int32_t longitude, const int32_t latitude,
  const int32_t height_mean_sea_level, const uint32_t horizontal_accuracy,
  const uint32_t vertical_accuracy, float * x_est, struct P_ * P_est)
{
  float position[3];
  float x_ned = gpshome_.lat_scale * (latitude - gpshome_.latitude);
  float y_ned = gpshome_.lon_scale * (longitude - gpshome_.longitude);
  position[0] = gpshome_.cos_xaxis * x_ned - gpshome_.sin_xaxis * y_ned;
  position[1] = gpshome_.sin_xaxis * x_ned + gpshome_.cos_xaxis * y_ned;
  position[2] = gpshome_.height_scale * (height_mean_sea_level - gpshome_.height);

  const float *r_pred = &x_pred[7]; // predicted position in i-frame
  float horzacc_in_meters = 0.001*(float)horizontal_accuracy;
  float vertacc_in_meters = 0.001*(float)horizontal_accuracy;
  const float R_diag[3] = {
    horzacc_in_meters * horzacc_in_meters,
    horzacc_in_meters * horzacc_in_meters,
    vertacc_in_meters * vertacc_in_meters,
  };
  printf("horzacc:%f\n",horzacc_in_meters);

  const float * Paa = P_pred->aa;
  const float * Pav = P_pred->av;
  const float * Par = P_pred->ar;
  const float * Pvv = P_pred->vv;
  const float * Pvr = P_pred->vr;
  const float * Prr = P_pred->rr;

  float temp[3 * 3], T[3 * 3];
  MatrixCopy(Prr, 3, 3, temp);
  MatrixAddDiagonalToSelf(temp, R_diag, 3);
  MatrixInverse(temp, 3, T); // T = (Prr+R)^(-1)

  float K[P_DIM * 3];
  float * K1 = &K[0];
  float * K2 = &K[9];
  float * K3 = &K[18];

  MatrixMultiply(Par, T, 3, 3, 3, K1); // K1 = Par*T
  MatrixMultiply(Pvr, T, 3, 3, 3, K2); // K2 = Pvr*T
  MatrixMultiply(Prr, T, 3, 3, 3, K3); // K3 = Prr*T


  MatrixCopy(Paa,3,3,P_est->aa);
  MatrixCopy(Pav,3,3,P_est->av);
  MatrixCopy(Par,3,3,P_est->ar);
  MatrixCopy(Pvv,3,3,P_est->vv);
  MatrixCopy(Pvr,3,3,P_est->vr);
  MatrixCopy(Prr,3,3,P_est->rr);

  MatrixMultiplyByTranspose(K1, Par, 3, 3, 3, temp); // K1*Pra
  MatrixSubtract(Paa, temp, 3, 3, P_est->aa);
  MatrixMultiplyByTranspose(K1, Pvr, 3, 3, 3, temp); // K1*Prv
  MatrixSubtract(Pav, temp, 3, 3, P_est->av);
  MatrixMultiply(K1, Prr, 3, 3, 3, temp); // K1*Prr
  MatrixSubtract(Par, temp, 3, 3, P_est->ar);
  MatrixMultiplyByTranspose(K2, Pvr, 3, 3, 3, temp); // K2*Prv
  MatrixSubtract(Pvv, temp, 3, 3, P_est->vv);
  MatrixMultiply(K2, Prr, 3, 3, 3, temp); // K2*Prr
  MatrixSubtract(Pvr, temp, 3, 3, P_est->vr);
  MatrixMultiply(K3, Prr, 3, 3, 3, temp); // K3*Prr
  MatrixSubtract(Prr, temp, 3, 3, P_est->rr);
  ////
  // KHP = [K1*P31 K1*P32 K1*P33]
  //       [       K2*P32 K2*P33]
  //       [sym.          K3*P33]
  ////

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

  printf("predicted_measurement:%f,%f,%f\n",predicted_measurement[0],predicted_measurement[1],predicted_measurement[2]);
  printf("measurement:%f,%f,%f\n",measurement[0],measurement[1],measurement[2]);
  printf("residual:%f,%f,%f\n",residual[0],residual[1],residual[2]);

  printf("dx:\n");
  float delta_x[P_DIM];
  MatrixMultiply(K, residual, P_DIM, z_dim, 1, delta_x);
  for(int i=0;i<9;i++){
    dx[i] += delta_x[i];
    printf("%f,",dx[i]);
  }
  printf("\n");

  VectorCopy(x_pred,X_DIM,x_est);

  //float delta_x[P_DIM];
////////////////////////////////////////////////////////////////////////////////// 21
  //MatrixMultiply(K, residual, P_DIM, z_dim, 1, delta_x);

  //float dq[4];
  //{
  //  const float * quat_pred = &x_pred[0];
  //  float Psi[4*3];
  //  Psi[0*3+0] = -quat_pred[1];
  //  Psi[0*3+1] = -quat_pred[2];
  //  Psi[0*3+2] = -quat_pred[3];
  //  Psi[1*3+0] = quat_pred[0];
  //  Psi[1*3+1] = -quat_pred[3];
  //  Psi[1*3+2] = quat_pred[2];
  //  Psi[2*3+0] = quat_pred[3];
  //  Psi[2*3+1] = quat_pred[0];
  //  Psi[2*3+2] = -quat_pred[1];
  //  Psi[3*3+0] = -quat_pred[2];
  //  Psi[3*3+1] = quat_pred[1];
  //  Psi[3*3+2] = quat_pred[0];

  //  const float * alpha = &delta_x[0];
////////////////////////////////////////////////////////////////////////////////// 12 + 4
  //  VectorScaleSelf(MatrixMultiply(Psi, alpha, 4, 3, 1, dq), 0.5, 4);
  //}

  //printf("dr:%f,%f,%f\n",delta_x[6],delta_x[7],delta_x[8]);

  //x_est[0] = dq[0];
  //x_est[1] = dq[1];
  //x_est[2] = dq[2];
  //x_est[3] = dq[3];
  //x_est[4] = delta_x[3];
  //x_est[5] = delta_x[4];
  //x_est[6] = delta_x[5];
  //x_est[7] = delta_x[6];
  //x_est[8] = delta_x[7];
  //x_est[9] = delta_x[8];
  //VectorAddToSelf(x_est, x_pred, X_DIM);

  //QuaternionNormalizingFilter(&x_est[0]);  // normalize the quaternion portion
}
void KalmanPerformMeasurementUpdate(void)
{
  float * x_pred = x_;
  float x_est[X_DIM];

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

    const float * alpha = &dx[0];
  //////////////////////////////////////////////////////////////////////////////// 12 + 4
    VectorScaleSelf(MatrixMultiply(Psi, alpha, 4, 3, 1, dq), 0.5, 4);
  }

  x_est[0] = dq[0];
  x_est[1] = dq[1];
  x_est[2] = dq[2];
  x_est[3] = dq[3];
  x_est[4] = dx[3];
  x_est[5] = dx[4];
  x_est[6] = dx[5];
  x_est[7] = dx[6];
  x_est[8] = dx[7];
  x_est[9] = dx[8];
  VectorAddToSelf(x_est, x_pred, X_DIM);

  QuaternionNormalizingFilter(&x_est[0]);  // normalize the quaternion portion

  // update x
  VectorCopy(x_est, X_DIM, x_);
  // reset dx
  for(int i=0;i<9;i++){
    dx[i]=0.0;
  }
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
