#ifndef CONSTANTS_H_
#define CONSTANTS_H_


#define GRAVITY_ACCELERATION (9.8)  // Acceleration due to gravity (m/s^2)
#define DT (1.0 / 128.0)  // FlightCtrl frame period

enum BodyAxes {
  X_BODY_AXIS = 0,
  Y_BODY_AXIS = 1,
  Z_BODY_AXIS = 2,
};

enum WorldAxes {
  N_WORLD_AXIS = 0,  // North
  E_WORLD_AXIS = 1,  // East
  D_WORLD_AXIS = 2,  // Down
};


#endif  // CONSTANTS_H_
