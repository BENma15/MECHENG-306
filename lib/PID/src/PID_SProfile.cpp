#include <Arduino.h>
#include <avr/interrupt.h>
#include <LimitSwitch.h>

#include <Encoder.h>

// Limit switch pins
const int L_LIMIT = 13;
const int R_LIMIT = 12;
const int U_LIMIT = 10;
const int D_LIMIT = 11;

// Motors pins
const int E1 = 5;
const int M1 = 4;
const int E2 = 6;
const int M2 = 7;

// Distance to encoder count equation variables
const double COUNTS_PER_REV = 8256.0;
const double WHEEL_RADIUS_MM = 8.0;

double velocity_left = 0; // In mm/s
double velocity_right = 0;
double distance_per_encoder_tick = 0.006089;

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

double kp_left = 0, ki_left = 0, kd_left = 0;
double integral_left = 0, lastError_left = 0;

double kp_right = 0, ki_right = 0, kd_right = 0;
double integral_right = 0, lastError_right = 0;

double kp_sync = 0, ki_sync = 0, kd_sync = 0;
double integral_sync = 0, lastError_sync = 0;

uint32_t lastControlLoopMicros = 0;
const uint32_t CONTROL_LOOP_INTERVAL_US = 20000; // 50Hz
const double dt = CONTROL_LOOP_INTERVAL_US / 1000000.0; // seconds per control loop tick

bool moveStarted = false;

double velocityProfile(double J, double Vf, double t4, double t) {
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

void plan(double x, double y, double vf_target) {
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

void move(int x, int y, int vf) {
    if (!moveStarted) {
        moveStarted = true;
        plan(x, y, vf);
    }

    uint32_t nowMicros = micros();
    if (nowMicros - lastControlLoopMicros < CONTROL_LOOP_INTERVAL_US) {
        return;
    }
    lastControlLoopMicros += CONTROL_LOOP_INTERVAL_US;

    if (!moveActive) {
        return;
    }

    double t = (millis() / 1000.0) - moveStartTime;
    double V = velocityProfile(moveJ, moveVf, moveT4, t);

    if (t >= TA + moveT4 + TA) {
        moveActive = false;
        V = 0;
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

    double finalOutputLeft  = outputLeft  - syncCorrection;
    double finalOutputRight = outputRight + syncCorrection;

    if (finalOutputLeft >= 0) {
        digitalWrite(M1, HIGH);
    } else {
        digitalWrite(M1, LOW);
        finalOutputLeft = -finalOutputLeft;
    }
    if (finalOutputLeft > 255) finalOutputLeft = 255;
    analogWrite(E1, finalOutputLeft);

    if (finalOutputRight >= 0) {
        digitalWrite(M2, HIGH);
    } else {
        digitalWrite(M2, LOW);
        finalOutputRight = -finalOutputRight;
    }
    if (finalOutputRight > 255) finalOutputRight = 255;
    analogWrite(E2, finalOutputRight);
}