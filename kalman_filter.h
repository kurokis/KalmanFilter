#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include <inttypes.h>

// =============================================================================
// Accessors:


float KalmanHeading(void);

// -----------------------------------------------------------------------------
const float * KalmanPosition(void);

// -----------------------------------------------------------------------------
const float * KalmanQuat(void);

// -----------------------------------------------------------------------------
const float * KalmanVelocity(void);

// -----------------------------------------------------------------------------
const float * KalmanPAlpha(void);

// =============================================================================
// Public functions:

void KalmanTimeUpdate(const float gyro[3], const float accelerometer[3]);

// -----------------------------------------------------------------------------
void KalmanAccelerometerUpdate(const float accelerometer[3]);

// -----------------------------------------------------------------------------
void KalmanBaroAltitudeUpdate(float baro_altitude);

// -----------------------------------------------------------------------------
void KalmanVisionUpdate(const float vision[3]);

// -----------------------------------------------------------------------------
void KalmanPositionUpdate(const float position[3]);

// -----------------------------------------------------------------------------
void KalmanGPSPositionUpdate(const int32_t longitude, const int32_t latitude,
  const int32_t height_mean_sea_level, const uint32_t horizontal_accuracy,
  const uint32_t vertical_accuracy);

// -----------------------------------------------------------------------------
void ResetKalman(void);

// -----------------------------------------------------------------------------
void ResetKalmanBaroAltitudeOffset(float baro_altitude, float position_down);

// -----------------------------------------------------------------------------
void SetGpsHome(const int32_t longitude, const int32_t latitude,
  const int32_t height_mean_sea_level, const float heading_xaxis);

#endif // KALMAN_FILTER_H_
