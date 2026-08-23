#include <Arduino.h>
#include <avr/interrupt.h>
#include <LimitSwitch.h>
#include <math.h>   // NEW: needed for cbrt(), sqrt() used by the distance-based profile

#include <Encoder.h>
#include "PID_FSM.h"
#include <Motor.h>
#include <HelperFunctions.h>
#include <Graph.h>

#include <FSM.h>

double T1 = 0.2;
double T2 = 0.4;
double TA = T1 + T2 + T1;

double moveJ = 0;
double moveVf = 0;
double moveT4 = 0;
double moveUnitX = 0;
double moveUnitY = 0;
double moveStartTime = 0;

bool moveActive = false;
bool moveStarted = false;
bool triangleProfile = false;

double moveV1 = 0;
double moveA2 = 0;
double moveS1 = 0, moveS2 = 0, moveS3 = 0, moveS4 = 0, moveStotal = 0;

double distanceTraveled = 0;
double prevMoveX = 0, prevMoveY = 0;

// Left Motor PID Variables
double kp_left = 15, ki_left = 3, kd_left = 1, kff_left = 7;
double integral_left = 0, lastError_left = 0;

// Right Motor PID Variables
double kp_right = 15, ki_right = 3, kd_right = 1, kff_right = 7;
double integral_right = 0, lastError_right = 0;

// Sync PID Variables
double kp_sync = 1, ki_sync = 0, kd_sync = 0;
double integral_sync = 0, lastError_sync = 0;

// Time Control Variables
uint32_t lastControlLoopMicros = 0;
const uint32_t CONTROL_LOOP_INTERVAL_US = 20000;
const double dt = CONTROL_LOOP_INTERVAL_US / 1000000.0;

// Motor Direction Variables
int8_t leftDir;
int8_t rightDir;

// encoder tracking
long moveCurrentLeftCount = 0;
long moveCurrentRightCount = 0;
long moveTargetLeftCount = 0;
long moveTargetRightCount = 0;

const double tolerance_mm = 0.5;
unsigned long elapsed_from_move_start = 0;
int integral_maxPWM = 20;


double stage1_velocity_byDistance(double J, double s)
{
    double tau = cbrt((6.0 * s) / J);
    return 0.5 * J * tau * tau;
}

double stage2_velocity_byDistance(double V1, double a2, double sPrime)
{
    return sqrt(V1 * V1 + 2.0 * a2 * sPrime);
}

double stage3_velocity_cardano(double J, double a2, double V2, double sPrime)
{
    double T1_local = a2 / J;

    double A = -J / 6.0;
    double B = 0.5 * a2;
    double C = V2;
    double D = -sPrime;

    double p = B / A;
    double q = C / A;
    double r = D / A;
    double P = q - (p * p) / 3.0;
    double Q = (2.0 * p * p * p) / 27.0 - (p * q) / 3.0 + r;

    double halfQ  = Q / 2.0;
    double thirdP = P / 3.0;
    double Delta  = halfQ * halfQ + thirdP * thirdP * thirdP;

    if (Delta < 0.0) Delta = 0.0;

    double sqrtDelta = sqrt(Delta);
    double u = cbrt(-halfQ + sqrtDelta);
    double v = cbrt(-halfQ - sqrtDelta);
    double y = u + v;

    double tau = y + T1_local;

    return V2 + a2 * tau - 0.5 * J * tau * tau;
}

void computeDistanceBoundaries()
{
    double a2 = moveJ * T1;
    double V1 = 0.5 * moveJ * T1 * T1;

    double V2 = V1 + a2 * T2;

    double S1 = (1.0 / 6.0) * moveJ * T1 * T1 * T1;
    double S2 = S1 + (V1 * T2 + 0.5 * a2 * T2 * T2);

    double S3 = moveVf * (T1 + T2 / 2.0);

    double S4 = S3 + moveVf * moveT4;
    double Stotal = S4 + S3;

    moveA2 = a2;
    moveV1 = V1;
    moveS1 = S1;
    moveS2 = S2;
    moveS3 = S3;
    moveS4 = S4;
    moveStotal = Stotal;
}

double velocityProfile_byDistance(double s)
{
    if (s >= moveStotal)
    {
        return 0.0;
    }

    double r = moveStotal - s;
    bool decelerating = (s > moveS4);
    double d = decelerating ? r : s;

    double V2 = moveV1 + moveA2 * T2;

    if (d <= moveS1)
    {
        return stage1_velocity_byDistance(moveJ, d);
    }
    else if (d <= moveS2)
    {
        double dPrime = d - moveS1;
        return stage2_velocity_byDistance(moveV1, moveA2, dPrime);
    }
    else if (d <= moveS3)
    {
        double dPrime = d - moveS2;
        return stage3_velocity_cardano(moveJ, moveA2, V2, dPrime);
    }
    else
    {
        return moveVf;
    }
}

void plan_FSM(double x, double y, double vf_target)
{
    double S = sqrt(x * x + y * y);

    Encoder_setLeftEncoderCountZero();
    Encoder_setRightEncoderCountZero();

    if (S <= 0.0)
    {
        moveActive = false;
        moveStarted = false;
        return;
    }
    moveUnitX = x / S;
    moveUnitY = y / S;

    double k = T1 + T2 / 2;

    double vf;
    double t4;
    if (S < 2.0 * k * vf_target)
    {
        vf = S / (2.0 * k);
        t4 = 0.0;
        triangleProfile = true;
    }
    else
    {
        vf = vf_target;
        t4 = (S - 2.0 * k * vf) / vf;
        triangleProfile = false;
    }

    moveVf = vf;
    moveJ = moveVf / (T1 * (T1 + T2));
    moveT4 = t4;

    moveStartTime = millis() / 1000.0;
    moveActive = true;

    computeDistanceBoundaries();

    distanceTraveled = 0;
    prevMoveX = 0;
    prevMoveY = 0;
}

void move_FSM(int x, int y, int vf)
{
    if (moveStarted == false)
    {
        elapsed_from_move_start = millis();
        moveStarted = true;

        integral_left = 0;
        lastError_left = 0;
        integral_right = 0;
        lastError_right = 0;
        integral_sync = 0;
        lastError_sync = 0;

        plan_FSM(x, y, vf);
        if (!moveActive)
        {
            return;
        }

        lastControlLoopMicros = micros();

        cli();
        moveCurrentLeftCount = Encoder_getLeftEncoderCount();
        moveCurrentRightCount = Encoder_getRightEncoderCount();
        sei();

        moveTargetLeftCount = moveCurrentLeftCount + distanceToCounts(x) + distanceToCounts(y);
        moveTargetRightCount = moveCurrentRightCount + distanceToCounts(x) - distanceToCounts(y);
    }
    cli();
    moveCurrentLeftCount = countA;
    moveCurrentRightCount = countB;
    sei();

    SystemState state = FSM_getCurrentState();

    if (state == STATE_FAULT)
    {
        setLeftMotor(0, 0);
        setRightMotor(0, 0);
        return;
    }

    uint32_t nowMicros = micros();
    if (nowMicros - lastControlLoopMicros < CONTROL_LOOP_INTERVAL_US)
    {
        return;
    }
    lastControlLoopMicros += CONTROL_LOOP_INTERVAL_US;

    double velocity_left = Encoder_getLeftVelocity(dt);
    double velocity_right = Encoder_getRightVelocity(dt);

    double currentLeftMM = countsToDistance(moveCurrentLeftCount);
    double currentRightMM = countsToDistance(moveCurrentRightCount);

    double currentX = (currentLeftMM + currentRightMM) / 2.0;
    double currentY = (currentLeftMM - currentRightMM) / 2.0;

    double dx = currentX - prevMoveX;
    double dy = currentY - prevMoveY;
    distanceTraveled += sqrt(dx * dx + dy * dy);
    prevMoveX = currentX;
    prevMoveY = currentY;

    double V = velocityProfile_byDistance(distanceTraveled);

    if (distanceTraveled >= moveStotal)
    {
        moveActive = false;
        moveStarted = false;
        setLeftMotor(0, 0);
        setRightMotor(0, 0);
        return;
    }

    double targetLeft;
    double targetRight;
    targetLeft = V * (moveUnitX + moveUnitY);
    targetRight = V * (moveUnitX - moveUnitY);

    // ---- everything below here is UNCHANGED from your original file ----

    double error_left = targetLeft - velocity_left;
    if (abs((integral_left + error_left * dt) * ki_left) < integral_maxPWM) {
        integral_left += error_left * dt;
    }
    double derivative_left = (error_left - lastError_left) / dt;
    double outputLeft = kp_left * error_left + ki_left * integral_left + kd_left * derivative_left;
    lastError_left = error_left;

    double error_right = targetRight - velocity_right;
    if (abs((integral_right + error_right * dt) * ki_right) < integral_maxPWM) {
        integral_right += error_right * dt;
    }
    double derivative_right = (error_right - lastError_right) / dt;
    double outputRight = kp_right * error_right + ki_right * integral_right + kd_right * derivative_right;
    lastError_right = error_right;

    double ratioLeft = (targetLeft != 0.0) ? (velocity_left / targetLeft) : 1.0;
    double ratioRight = (targetRight != 0.0) ? (velocity_right / targetRight) : 1.0;
    double percentError = (ratioLeft - ratioRight) * 100.0;
    integral_sync += percentError * dt;
    double derivative_sync = (percentError - lastError_sync) / dt;
    double syncCorrection = kp_sync * percentError + ki_sync * integral_sync + kd_sync * derivative_sync;
    lastError_sync = percentError;

    int leftTargetDir = 0;
    int rightTargetDir = 0;
    if (targetLeft > 0) leftTargetDir = 1;
    else if (targetLeft < 0) leftTargetDir = -1;
    if (targetRight > 0) rightTargetDir = 1;
    else if (targetRight < 0) rightTargetDir = -1;

    double feedforwardLeft = targetLeft * kff_left;
    double feedforwardRight = targetRight * kff_right;

    double finalOutputLeft = outputLeft - syncCorrection * leftTargetDir + feedforwardLeft;
    double finalOutputRight = outputRight + syncCorrection * rightTargetDir + feedforwardRight;

    if (finalOutputLeft > 0) leftDir = 1;
    else if (finalOutputLeft < 0) leftDir = -1;
    else leftDir = 0;

    if (finalOutputRight > 0) rightDir = 1;
    else if (finalOutputRight < 0) rightDir = -1;
    else rightDir = 0;

    int leftPWM = (int)abs(finalOutputLeft);
    int rightPWM = (int)abs(finalOutputRight);
    if (leftPWM > 255) leftPWM = 255;
    if (rightPWM > 255) rightPWM = 255;

    String leftSuccess = setLeftMotor(leftDir, leftPWM);
    String rightSuccess = setRightMotor(rightDir, rightPWM);

    static unsigned long previous_time = 0;
    unsigned long current_time = millis();
    unsigned long elapsed_ms = current_time - previous_time;
    unsigned long timeSinceStart = current_time - elapsed_from_move_start;

    long leftEncoderError = moveTargetLeftCount - moveCurrentLeftCount;
    long rightEncoderError = moveTargetRightCount - moveCurrentRightCount;
    if (elapsed_ms >= 10)
    {
        previous_time = current_time;
        addDataPoint(abs(leftEncoderError), abs(rightEncoderError), timeSinceStart);
    }
}