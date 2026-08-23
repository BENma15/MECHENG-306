#ifndef PID_FSM_DISTANCE_H
#define PID_FSM_DISTANCE_H

#include <Arduino.h>

// NOTE: this defines plan_FSM(...) and move_FSM(...) with the SAME names
// as your original time-based PID_FSM.cpp. Only ONE of PID_FSM.cpp or
// PID_FSM_distance_based.cpp can be compiled into a given build - linking
// both defines these symbols twice.

// Motion profile helpers
double stage1_velocity_byDistance(double J, double s);
double stage2_velocity_byDistance(double V1, double a2, double sPrime);
double stage3_velocity_cardano(double J, double a2, double V2, double sPrime);
void computeDistanceBoundaries();
double velocityProfile_byDistance(double s);

void plan_FSM(double x, double y, double vf_target);
void move_FSM(int x, int y, int vf);

// Time constants (fixed jerk-ramp / constant-accel durations)
extern double T1;
extern double T2;
extern double TA;

// Shared motion state used by the control loop
extern double velocity_left;
extern double velocity_right;

extern double moveJ;
extern double moveVf;
extern double moveT4;
extern double moveUnitX;
extern double moveUnitY;
extern double moveStartTime;    // no longer drives control decisions - kept for telemetry only
extern bool moveActive;
extern bool triangleProfile;
extern bool moveStarted;

// Distance-profile boundaries (computed once per move by computeDistanceBoundaries)
extern double moveV1;           // absolute velocity at end of stage 1
extern double moveA2;           // acceleration during stage 2 (= J*T1)
extern double moveS1;           // cumulative distance, end of stage 1
extern double moveS2;           // cumulative distance, end of stage 2
extern double moveS3;           // cumulative distance, end of stage 3 (end of accel ramp)
extern double moveS4;           // cumulative distance, end of cruise
extern double moveStotal;       // total distance for the whole move

// Monotonic distance accumulator - this is what drives the velocity lookup
extern double distanceTraveled;
extern double prevMoveX;
extern double prevMoveY;

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

#endif // PID_FSM_DISTANCE_H