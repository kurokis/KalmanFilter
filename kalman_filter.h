#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_


// =============================================================================
// Accessors:


float KalmanHeading(void);

// -----------------------------------------------------------------------------
const float * KalmanPosition(void);

// -----------------------------------------------------------------------------
const float * KalmanQuat(void);

// -----------------------------------------------------------------------------
const float * KalmanVelocity(void);


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
void ResetKalman(void);

// -----------------------------------------------------------------------------
void ResetKalmanBaroAltitudeOffset(float baro_altitude, float position_down);


#endif // KALMAN_FILTER_H_