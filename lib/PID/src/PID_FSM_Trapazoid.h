#ifndef PID_FSM_TRAPEZOID_H
#define PID_FSM_TRAPEZOID_H

#include <Arduino.h>

// Uses its own function/variable names (trap*, planTrapezoid, moveTrapezoid)
// rather than reusing plan_FSM/move_FSM, so this can be compiled alongside
// EITHER the original time-based or the distance-based S-curve file for
// A/B testing, without a linker clash.

// Motion profile helpers
double velocityProfile_trapezoid(double s);
void plan_FSM(double x, double y, double vf_target);
void move_FSM(int x, int y, int vf);

// Max acceleration/deceleration magnitude for this profile (mm/s^2) - tune
// this to your motors' real capability.
extern double trapAccel;

// Shared motion state used by the control loop
extern double trapVf;               // actual peak/cruise velocity for this move
extern double trapS_accelEnd;       // cumulative distance where accel phase ends
extern double trapS_cruiseEnd;      // cumulative distance where cruise phase ends
extern double trapStotal;           // total distance for the whole move
extern bool trapTriangleProfile;

extern double trapUnitX;
extern double trapUnitY;
extern bool trapMoveActive;
extern bool trapMoveStarted;

// Monotonic distance accumulator - this is what drives the velocity lookup
extern double trapDistanceTraveled;
extern double trapPrevX;
extern double trapPrevY;

// NOTE: moveTrapezoid() reuses these PID/motor/encoder globals from your
// existing PID_FSM.cpp / PID_FSM_distance_based.cpp - it does not redefine
// its own copies, so one of those files still needs to be compiled alongside
// this one for these symbols to resolve.
extern double kp_left;
extern double ki_left;
extern double kd_left;
extern double integral_left;
extern double lastError_left;

extern double kp_right;
extern double ki_right;
extern double kd_right;
extern double integral_right;
extern double lastError_right;

extern double kp_sync;
extern double ki_sync;
extern double kd_sync;
extern double integral_sync;
extern double lastError_sync;

extern uint32_t lastControlLoopMicros;
extern const uint32_t CONTROL_LOOP_INTERVAL_US;
extern const double dt;
extern int integral_maxPWM;

extern long moveCurrentLeftCount;
extern long moveCurrentRightCount;
extern long moveTargetLeftCount;
extern long moveTargetRightCount;

extern int8_t leftDir;
extern int8_t rightDir;

#endif // PID_FSM_TRAPEZOID_H