#include <Arduino.h>
#include <avr/interrupt.h>
#include <LimitSwitch.h>
#include <math.h>

#include <Encoder.h>
#include "PID_FSM.h"
#include <Motor.h>
#include <HelperFunctions.h>
#include <Graph.h>

#include <FSM.h>


double trapAccel = 200.0;

double trapVf = 0;
double trapS_accelEnd = 0;
double trapS_cruiseEnd = 0;
double trapStotal = 0;
bool trapTriangleProfile = false;

double trapUnitX = 0;
double trapUnitY = 0;
bool trapMoveActive = false;
bool trapMoveStarted = false;

double trapDistanceTraveled = 0;
double trapPrevX = 0, trapPrevY = 0;

void plan_FSM(double x, double y, double vf_target)
{
    double S = sqrt(x * x + y * y);

    Encoder_setLeftEncoderCountZero();
    Encoder_setRightEncoderCountZero();

    if (S <= 0.0)
    {
        trapMoveActive = false;
        trapMoveStarted = false;
        return;
    }

    trapUnitX = x / S;
    trapUnitY = y / S;

    double S_ramp = (vf_target * vf_target) / (2.0 * trapAccel);

    if (S < 2.0 * S_ramp)
    {
        trapVf = sqrt(trapAccel * S);
        trapS_accelEnd = S / 2.0;
        trapS_cruiseEnd = S / 2.0;
        trapTriangleProfile = true;
    }
    else
    {
        trapVf = vf_target;
        trapS_accelEnd = S_ramp;
        trapS_cruiseEnd = S - S_ramp;
        trapTriangleProfile = false;
    }

    trapStotal = S;

    trapMoveActive = true;
    trapDistanceTraveled = 0;
    trapPrevX = 0;
    trapPrevY = 0;
}

double velocityProfile_trapezoid(double s)
{
    if (s >= trapStotal)
    {
        return 0.0;
    }

    if (s <= trapS_accelEnd)
    {
        return sqrt(2.0 * trapAccel * s);
    }
    else if (s <= trapS_cruiseEnd)
    {
        return trapVf;
    }
    else
    {
        double r = trapStotal - s;
        return sqrt(2.0 * trapAccel * r);
    }
}


void move_FSM(int x, int y, int vf)
{
    if (trapMoveStarted == false)
    {
        trapMoveStarted = true;

        integral_left = 0;
        lastError_left = 0;
        integral_right = 0;
        lastError_right = 0;
        integral_sync = 0;
        lastError_sync = 0;

        planTrapezoid(x, y, vf);
        if (!trapMoveActive)
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

    // Monotonic distance accumulator - same incremental-delta approach as
    // the S-curve version, for the same reason (avoids backward wobble).
    double dx = currentX - trapPrevX;
    double dy = currentY - trapPrevY;
    trapDistanceTraveled += sqrt(dx * dx + dy * dy);
    trapPrevX = currentX;
    trapPrevY = currentY;

    double V = velocityProfile_trapezoid(trapDistanceTraveled);

    // Completion is purely distance-based - no time check anywhere.
    if (trapDistanceTraveled >= trapStotal)
    {
        trapMoveActive = false;
        trapMoveStarted = false;
        setLeftMotor(0, 0);
        setRightMotor(0, 0);
        return;
    }

    double targetLeft = V * (trapUnitX + trapUnitY);
    double targetRight = V * (trapUnitX - trapUnitY);

    // ---- identical PID/feedforward/sync logic to your existing move_FSM ----

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

    setLeftMotor(leftDir, leftPWM);
    setRightMotor(rightDir, rightPWM);
}