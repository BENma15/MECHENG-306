#include "HomingMovement.h"
#include <Encoder.h>
#include <Motor.h>
#include <HelperFunctions.h>

static const double T1 = 0.2;
static const double T2 = 0.4;
static const double TA = T1 + T2 + T1;

static double homingUnitX = 0;
static double homingUnitY = 0;
static double homingVf = 0;
static double homingJ = 0;
static double homingStartTime = 0;
static bool homingActive = false;

static uint32_t lastControlLoopMicros = 0;
static const uint32_t CONTROL_LOOP_INTERVAL_US = 20000;
static const double dt = CONTROL_LOOP_INTERVAL_US / 1000000.0;

static double kp_left = 10, ki_left = 2, kd_left = 0, kff_left = 4;
static double integral_left = 0, lastError_left = 0;

static double kp_right = 10, ki_right = 2, kd_right = 0, kff_right = 4;
static double integral_right = 0, lastError_right = 0;

static double kp_sync = 0, ki_sync = 1, kd_sync = 0;
static double integral_sync = 0, lastError_sync = 0;

static int integral_maxPWM = 40;
static const int MIN_DRIVE_PWM = 60;

static double homingVelocityRamp(double t)
{
    double V1 = 0.5 * homingJ * T1 * T1;
    double V2 = homingJ * T1 * T2;

    if (t < T1)
    {
        return 0.5 * homingJ * t * t;
    }
    else if (t < T1 + T2)
    {
        double tau = t - T1;
        return V1 + (homingJ * T1) * tau;
    }
    else if (t < TA)
    {
        double tau = t - (T1 + T2);
        return V1 + V2 + (homingJ * T1) * tau - 0.5 * homingJ * tau * tau;
    }
    else
    {
        return homingVf;
    }
}

void homingMove_start(double unitX, double unitY, double vf_mm_min)
{
    homingUnitX = unitX;
    homingUnitY = unitY;
    homingVf = vf_mm_min / 60.0;
    homingJ = homingVf / (T1 * (T1 + T2));

    integral_left = 0;
    lastError_left = 0;
    integral_right = 0;
    lastError_right = 0;
    integral_sync = 0;
    lastError_sync = 0;

    homingStartTime = millis() / 1000.0;
    lastControlLoopMicros = micros();
    homingActive = true;
}

void homingMove_stop()
{
    homingActive = false;
    stopMotors();
}

bool homingMove_isActive()
{
    return homingActive;
}

void homingMove_tick()
{
    if (!homingActive)
    {
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

    double t = (millis() / 1000.0) - homingStartTime;
    double V = homingVelocityRamp(t);

    double targetLeft = V * (homingUnitX + homingUnitY);
    double targetRight = V * (homingUnitX - homingUnitY);

    double error_left = targetLeft - velocity_left;
    if (abs((integral_left + error_left * dt) * ki_left) < integral_maxPWM)
    {
        integral_left += error_left * dt;
    }
    double derivative_left = (error_left - lastError_left) / dt;
    double outputLeft = kp_left * error_left + ki_left * integral_left + kd_left * derivative_left;
    lastError_left = error_left;

    double error_right = targetRight - velocity_right;
    if (abs((integral_right + error_right * dt) * ki_right) < integral_maxPWM)
    {
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

    int8_t leftDir = outputToDirection(finalOutputLeft);
    int8_t rightDir = outputToDirection(finalOutputRight);

    int leftPWM = (int)abs(finalOutputLeft);
    int rightPWM = (int)abs(finalOutputRight);

    leftPWM = applyMotorPwmLimits(leftPWM, leftDir, MIN_DRIVE_PWM);
    rightPWM = applyMotorPwmLimits(rightPWM, rightDir, MIN_DRIVE_PWM);

    setLeftMotor(leftDir, leftPWM);
    setRightMotor(rightDir, rightPWM);
}