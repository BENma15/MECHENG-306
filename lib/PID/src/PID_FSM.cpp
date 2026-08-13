#include <Arduino.h>
#include <avr/interrupt.h>
#include <LimitSwitch.h>

#include <Encoder.h>
#include "PID_FSM.h"
#include <Motor.h>
#include <HelperFunctions.h>


// Distance to encoder count equation variables

double T1 = 0.05;
double T2 = 0.10;
double TA = T1 + T2 + T1;

double moveJ = 0;
double moveVf = 0;
double moveT4 = 0;
double moveUnitX = 0;
double moveUnitY = 0;
double moveStartTime = 0;
bool moveActive = false;

bool triangleProfile = false;

double kp_left = 0.1, ki_left = 0.1, kd_left = 0.1;
double integral_left = 0, lastError_left = 0;

double kp_right = 0.1, ki_right = 0.1, kd_right = 0.1;
double integral_right = 0, lastError_right = 0;

double kp_sync = 0, ki_sync = 0, kd_sync = 0;
double integral_sync = 0, lastError_sync = 0;

uint32_t lastControlLoopMicros = 0;
const uint32_t CONTROL_LOOP_INTERVAL_US = 20000; // 50Hz
const double dt = CONTROL_LOOP_INTERVAL_US / 1000000.0; // seconds per control loop tick

bool moveStarted = false;

double velocityProfile_FSM(double J, double Vf, double t4, double t) {
    double V1 = 0.5 * J * T1 * T1;
    double V2 = J * T1 * T2;

    if (t < T1) {
        return 0.5 * J * t * t;
    } else if (t < T1 + T2) {
        double tau = t - T1;
        return V1 + (J * T1) * tau;
    } else if (t < TA) {
        double tau = t - (T1 + T2);
        return V1 + V2 + (J * T1) * tau - 0.5 * J * tau * tau;
    } else if (t < TA + t4) {
        return Vf;
    } else if (t < TA + t4 + T1) {
        double tau = t - (TA + t4);
        return Vf - 0.5 * J * tau * tau;
    } else if (t < TA + t4 + T1 + T2) {
        double tau = t - (TA + t4 + T1);
        return Vf - V1 - (J * T1) * tau;
    } else if (t < TA + t4 + TA) {
        double tau = t - (TA + t4 + T1 + T2);
        return Vf - V1 - V2 - (J * T1) * tau + 0.5 * J * tau * tau;
    } else {
        // move finished
        return 0.0;
    }
}

void plan_FSM(double x, double y, double vf_target) {
    double S = sqrt(x * x + y * y);
    if (S <= 0.0) return;

    moveUnitX = x / S;
    moveUnitY = y / S;
    moveVf = vf_target;
    moveJ = moveVf / (T1 * (T1 + T2));

    double rampDistance = (4.0 / 6.0) * moveJ * T1 * T1 * T1 + moveJ * T1 * T2 * T2;
    double t4 = (S - rampDistance) / moveVf;

    if (t4 < 0) {
        triangleProfile = true;
        moveActive = true;
        return;
    }

    moveT4 = t4;
    moveStartTime = millis() / 1000.0;

    moveActive = true;
    return;
}

void move_FSM(int x, int y, int vf) {
if (!moveStarted) {
    moveStarted = true;

    integral_left = 0;
    lastError_left = 0;

    integral_right = 0;
    lastError_right = 0;

    integral_sync = 0;
    lastError_sync = 0;

    plan_FSM(x, y, vf);

    lastControlLoopMicros = micros();
}

    uint32_t nowMicros = micros();
    if (nowMicros - lastControlLoopMicros < CONTROL_LOOP_INTERVAL_US) {
        return;
    }
    lastControlLoopMicros = nowMicros;

    if (!moveActive) {
        return;
    }

    double t = (millis() / 1000.0) - moveStartTime;
    double V = velocityProfile_FSM(moveJ, moveVf, moveT4, t);

   if (t >= TA + moveT4 + TA) {
    moveActive = false;
    moveStarted = false;

    setLeftMotor(0, 0);
    setRightMotor(0, 0);

    return;
}

    double targetLeft  = V * (moveUnitX + moveUnitY);
    double targetRight = V * (moveUnitX - moveUnitY);

    // left velocity PID
    double error_left = targetLeft - velocity_left;
    integral_left += error_left * dt;
    double derivative_left = (error_left - lastError_left) / dt;
    double outputLeft = kp_left * error_left + ki_left * integral_left + kd_left * derivative_left;
    lastError_left = error_left;

    // right velocity PID
    double error_right = targetRight - velocity_right;
    integral_right += error_right * dt;
    double derivative_right = (error_right - lastError_right) / dt;
    double outputRight = kp_right * error_right + ki_right * integral_right + kd_right * derivative_right;
    lastError_right = error_right;

    // sync PID
    double percentError = 0.0;
    if (velocity_left != 0.0) {
        percentError = (velocity_left - velocity_right) / velocity_left * 100.0;
    }
    integral_sync += percentError * dt;
    double derivative_sync = (percentError - lastError_sync) / dt;
    double syncCorrection = kp_sync * percentError + ki_sync * integral_sync + kd_sync * derivative_sync;
    lastError_sync = percentError;

    double finalOutputLeft  = outputLeft - syncCorrection;
double finalOutputRight = outputRight + syncCorrection;

int8_t leftDir;
int8_t rightDir;

if (finalOutputLeft > 0) {
    leftDir = 1;
}
else if (finalOutputLeft < 0) {
    leftDir = -1;
}
else {
    leftDir = 0;
}

if (finalOutputRight > 0) {
    rightDir = 1;
}
else if (finalOutputRight < 0) {
    rightDir = -1;
}
else {
    rightDir = 0;
}

int leftPWM = abs((int)finalOutputLeft);
int rightPWM = abs((int)finalOutputRight);

if (leftPWM > 255) {
    leftPWM = 255;
}

if (rightPWM > 255) {
    rightPWM = 255;
}

String leftSuccess = setLeftMotor(leftDir, leftPWM);
String rightSuccess = setRightMotor(rightDir, rightPWM);
}