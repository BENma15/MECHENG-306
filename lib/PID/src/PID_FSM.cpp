#include <Arduino.h>
#include <avr/interrupt.h>
#include <LimitSwitch.h>

#include <Encoder.h>
#include "PID_FSM.h"
#include <Motor.h>
#include <HelperFunctions.h>


// Time Constant Variables
double T1 = 0.05;
double T2 = 0.10;
double TA = T1 + T2 + T1;

// Movement Variables
double moveJ = 0;
double moveVf = 0;
double moveT4 = 0;
double moveUnitX = 0;
double moveUnitY = 0;
double moveStartTime = 0;
double moveDistance = 0;

// Distance-tracking variables (for stopping on distance instead of time)
long moveStartCountA = 0;
long moveStartCountB = 0;
const double DISTANCE_TOLERANCE_MM = 0.5;

bool moveActive = false;
bool moveStarted = false;

bool triangleProfile = false;

// Left Motor PID Variables
double kp_left = 0.5, ki_left = 0.5, kd_left = 0;
double integral_left = 0, lastError_left = 0;

// Right Motor PID Variables
double kp_right = 0.5, ki_right = 0.5, kd_right = 0;
double integral_right = 0, lastError_right = 0;

// Sync PID Variables
double kp_sync = 0, ki_sync = 0, kd_sync = 0;
double integral_sync = 0, lastError_sync = 0;
const double SYNC_TARGET_MIN = 5.0; // below this target velocity, sync correction is skipped

// Time Control Variables
uint32_t lastControlLoopMicros = 0;
const uint32_t CONTROL_LOOP_INTERVAL_US = 2500;            // 400Hz
const double dt = CONTROL_LOOP_INTERVAL_US / 1000000.0;     // Seconds per control loop tick

// Motor Direction Variables
int8_t leftDir;
int8_t rightDir;


// Returns the target velocity at different stages in the movement, using an s-curve profile.
double velocityProfile_FSM(double J, double Vf, double t4, double t) {
    double V1 = 0.5 * J * T1 * T1;          // Velocity after stage 1
    double V2 = J * T1 * T2;                // Velocity after stage 2

    if (t < T1) {                                                       // Stage 1
        return 0.5 * J * t * t;
    } else if (t < T1 + T2) {                                           // Stage 2
        double tau = t - T1;
        return V1 + (J * T1) * tau;
    } else if (t < TA) {                                                // Stage 3
        double tau = t - (T1 + T2);
        return V1 + V2 + (J * T1) * tau - 0.5 * J * tau * tau;
    } else if (t < TA + t4) {                                           // Stage 4
        return Vf;
    } else if (t < TA + t4 + T1) {                                      // Stage 5
        double tau = t - (TA + t4);
        return Vf - 0.5 * J * tau * tau;
    } else if (t < TA + t4 + T1 + T2) {                                 // Stage 6
        double tau = t - (TA + t4 + T1);
        return Vf - V1 - (J * T1) * tau;
    } else if (t < TA + t4 + TA) {                                      // Stage 7
        double tau = t - (TA + t4 + T1 + T2);
        return Vf - V1 - V2 - (J * T1) * tau + 0.5 * J * tau * tau;
    } else {
        // Move finished
        return 0.0;
    }
}

// Initial speed plan to find distance, time at constant velocity and if it is a triangular profile.
void plan_FSM(double x, double y, double vf_target) {
    // Find hypotenuse
    double S = sqrt(x * x + y * y);
    if (S <= 0.0) {
    moveActive = false;
    moveStarted = false;
    return;
}

    // Both the movement vectors
    moveUnitX = x / S;
    moveUnitY = y / S;

    // Target cruise velocity
    moveVf = vf_target;

    // Jerk required
    moveJ = moveVf / (T1 * (T1 + T2));

    // Distance covered during the acceleration ramp
    double rampDistance = (4.0 / 6.0) * moveJ * T1 * T1 * T1 + moveJ * T1 * T2 * T2;

    // Time spent at constant cruise speed
    double t4 = (S - rampDistance) / moveVf;

    // If time at cruise speed is negative then implement triangle profile
    /*if (t4 < 0) {
        triangleProfile = true;
        moveActive = true;
        return;
    }*/

    // Start move time timer
    moveT4 = t4;
    moveDistance = S;
    moveStartTime = millis() / 1000.0;

    // TODO: replace getCountA()/getCountB() with whatever your Encoder.h actually exposes
    // for raw encoder counts on the A and B CoreXY axes.
    moveStartCountA = getCountA();
    moveStartCountB = getCountB();

    moveActive = true;
    return;
}

// Main PID loop.
void move_FSM(int x, int y, int vf) {
        // If movement has not started then set all the errors to 0 and call the movement plan
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
        
        // Ensures there is a constant frequency of 50Hz which we are using for the PID loop as to not
        // call the motors too frequently.
        uint32_t nowMicros = micros();
        if (nowMicros - lastControlLoopMicros < CONTROL_LOOP_INTERVAL_US) {
            return;
        }
        lastControlLoopMicros = nowMicros;

        // Gets current time in reference to start time and then finds target velocity
        double t = (millis() / 1000.0) - moveStartTime;
        double V = velocityProfile_FSM(moveJ, moveVf, moveT4, t);

        // Distance actually travelled so far, computed from encoder counts via CoreXY inverse kinematics.
        // TODO: replace getCountA()/getCountB() and mmPerCount with your actual encoder API / conversion.
        long countA = getCountA();
        long countB = getCountB();
        double dA = (countA - moveStartCountA) * mmPerCount;
        double dB = (countB - moveStartCountB) * mmPerCount;
        double traveledX = (dA + dB) / 2.0;
        double traveledY = (dA - dB) / 2.0;
        double distanceTraveled = sqrt(traveledX * traveledX + traveledY * traveledY);

        // Stop once the planned distance has actually been covered, not just once time is up.
        // Time check kept as a failsafe in case encoder counts stall/jam.
        bool distanceReached = distanceTraveled >= (moveDistance - DISTANCE_TOLERANCE_MM);

        if (distanceReached) {
            moveActive = false;
            moveStarted = false;

            setLeftMotor(0, 0);
            setRightMotor(0, 0);

            return;
        }

        // Splits target velocity into left and right motor target velocities using unit vectors
        double targetLeft  = V * (moveUnitX + moveUnitY);
        double targetRight = V * (moveUnitX - moveUnitY);

        // NEED TO ADD ANTI-WINDUP FOR ALL OF THEM

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

        // sync PID - only correct when both targets are large enough that the velocity ratio is meaningful.
        // Near zero target velocity (start/end of move) the ratio blows up and causes spiking/stuttering.
        double percentError = 0.0;
        if (fabs(targetLeft) > SYNC_TARGET_MIN && fabs(targetRight) > SYNC_TARGET_MIN) {
            double ratioLeft = velocity_left / targetLeft;
            double ratioRight = velocity_right / targetRight;
            percentError = (ratioLeft - ratioRight) * 100.0;
        }
        integral_sync += percentError * dt;
        double derivative_sync = (percentError - lastError_sync) / dt;
        double syncCorrection = kp_sync * percentError + ki_sync * integral_sync + kd_sync * derivative_sync;
        lastError_sync = percentError;

        double finalOutputLeft = outputLeft - syncCorrection;
        double finalOutputRight = outputRight + syncCorrection;

        // Finds the direction of the motors
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

        // Sets the desired pwm of the motors
        int leftPWM = abs((int)finalOutputLeft);
        int rightPWM = abs((int)finalOutputRight);

        if (leftPWM > 255) {
            leftPWM = 255;
        }

        if (rightPWM > 255) {
            rightPWM = 255;
        }

        // Motor movement
        String leftSuccess = setLeftMotor(leftDir, leftPWM);
        String rightSuccess = setRightMotor(rightDir, rightPWM);
}