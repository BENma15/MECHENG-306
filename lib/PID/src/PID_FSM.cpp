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

bool moveActive = false;
bool moveStarted = false;

bool triangleProfile = false;

// Left Motor PID Variables
double kp_left = 8, ki_left = 1, kd_left = 0;
double integral_left = 0, lastError_left = 0;

// Right Motor PID Variables
double kp_right = 8, ki_right = 1, kd_right = 0;
double integral_right = 0, lastError_right = 0;

// Sync PID Variables
// Kept at 0 until the inner velocity loops are confirmed clean/stable on their own.
double kp_sync = 0, ki_sync = 0, kd_sync = 0;
double integral_sync = 0, lastError_sync = 0;

// Time Control Variables
uint32_t lastControlLoopMicros = 0;
const uint32_t CONTROL_LOOP_INTERVAL_US = 20000;            // 50Hz
const double dt = CONTROL_LOOP_INTERVAL_US / 1000000.0;     // Seconds per control loop tick

// Motor Direction Variables
int8_t leftDir;
int8_t rightDir;

// Distance Checker Variables (encoder-based, per-motor)
long moveStartCountLeft = 0;
long moveStartCountRight = 0;

double targetDistanceLeft = 0;
double targetDistanceRight = 0;

double actualDistanceLeft = 0;
double actualDistanceRight = 0;

double distanceErrorLeft = 0;
double distanceErrorRight = 0;

// Velocity is now computed here, from encoder COUNT deltas over a fixed dt window,
// instead of from raw edge-to-edge ISR timing (which was extremely noisy).
double velocity_left = 0;   // mm/s
double velocity_right = 0;  // mm/s

// Previous-loop encoder counts, used to compute the count delta each control tick
static long prevCountLeft = 0;
static long prevCountRight = 0;


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
    if (S <= 0.0) return;

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
    moveStartTime = millis() / 1000.0;
    moveActive = true;
    return;
}

// Main PID loop.
void move_FSM(int x, int y, int vf) {
    do {
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

            // Snapshot starting encoder counts for distance tracking
            moveStartCountLeft  = Encoder_getLeftEncoderCount();
            moveStartCountRight = Encoder_getRightEncoderCount();

            // Also reset the velocity window baseline so the first velocity
            // reading of the move isn't computed against stale counts.
            prevCountLeft  = moveStartCountLeft;
            prevCountRight = moveStartCountRight;
            velocity_left = 0;
            velocity_right = 0;

            // CoreXY: A = X + Y, B = X - Y -> per-motor target distances
            targetDistanceLeft  = fabs((double)x + (double)y);
            targetDistanceRight = fabs((double)x - (double)y);

            actualDistanceLeft = 0;
            actualDistanceRight = 0;
            distanceErrorLeft = targetDistanceLeft;
            distanceErrorRight = targetDistanceRight;

            lastControlLoopMicros = micros();
        }
        
        // Ensures there is a constant frequency for the PID loop as to not
        // call the motors too frequently.
        uint32_t nowMicros = micros();
        if (nowMicros - lastControlLoopMicros < CONTROL_LOOP_INTERVAL_US) {
            continue;
        }
        lastControlLoopMicros = nowMicros;

        // Gets current time in reference to start time and then finds target velocity
        double t = (millis() / 1000.0) - moveStartTime;
        double V = velocityProfile_FSM(moveJ, moveVf, moveT4, t);

        // --- Read encoder counts once per loop, used for BOTH velocity and distance ---
        long currentCountLeft  = Encoder_getLeftEncoderCount();
        long currentCountRight = Encoder_getRightEncoderCount();

        // --- Velocity: count delta over this fixed dt window (much less noisy than
        //     the old edge-to-edge ISR timing) ---
        long tickDeltaLeft  = currentCountLeft  - prevCountLeft;
        long tickDeltaRight = currentCountRight - prevCountRight;

        prevCountLeft  = currentCountLeft;
        prevCountRight = currentCountRight;

        velocity_left  = (tickDeltaLeft  * distance_per_encoder_tick) / dt;
        velocity_right = (tickDeltaRight * distance_per_encoder_tick) / dt;

        // --- Distance traveled so far this move, from the same counts ---
        long dCountLeft  = currentCountLeft  - moveStartCountLeft;
        long dCountRight = currentCountRight - moveStartCountRight;

        actualDistanceLeft  = fabs((double)dCountLeft)  * distance_per_encoder_tick;
        actualDistanceRight = fabs((double)dCountRight) * distance_per_encoder_tick;

        distanceErrorLeft  = targetDistanceLeft  - actualDistanceLeft;
        distanceErrorRight = targetDistanceRight - actualDistanceRight;

        Serial.print("Target L: ");
        Serial.print(targetDistanceLeft);
        Serial.print("  Actual L: ");
        Serial.print(actualDistanceLeft);
        Serial.print("  Err L: ");
        Serial.println(distanceErrorLeft);

        Serial.print("Target R: ");
        Serial.print(targetDistanceRight);
        Serial.print("  Actual R: ");
        Serial.print(actualDistanceRight);
        Serial.print("  Err R: ");
        Serial.println(distanceErrorRight);

        // Stop once both motors have actually covered their planned distance.
        // (Time-based check kept as a fallback in case a motor stalls and never reaches target.)
        bool distanceReached = (actualDistanceLeft >= targetDistanceLeft) && (actualDistanceRight >= targetDistanceRight);
        bool timeoutReached = (t >= (TA + moveT4 + TA) * 1.5);

        if (distanceReached || timeoutReached) {
            moveActive = false;
            moveStarted = false;

            setLeftMotor(0, 0);
            setRightMotor(0, 0);

            return;
        }

        // Splits target velocity into left and right motor target velocities using unit vectors
        double targetLeft  = V * (moveUnitX + moveUnitY);
        double targetRight = V * (moveUnitX - moveUnitY);

        // Stop commanding an axis once IT individually has reached its target distance,
        // rather than letting it keep chasing the shared profile velocity and overshoot
        // while waiting for the other (longer-travel) axis to finish.
        if (actualDistanceLeft >= targetDistanceLeft) {
            targetLeft = 0;
        }
        if (actualDistanceRight >= targetDistanceRight) {
            targetRight = 0;
        }

        // NEED TO ADD ANTI-WINDUP FOR SYNC PID

        // left velocity PID
        double error_left = targetLeft - velocity_left;
        integral_left += error_left * dt;
        integral_left = constrain(integral_left, -50.0, 50.0);
        double derivative_left = (error_left - lastError_left) / dt;
        double outputLeft = kp_left * error_left + ki_left * integral_left + kd_left * derivative_left;
        lastError_left = error_left;

        // right velocity PID
        double error_right = targetRight - velocity_right;
        integral_right += error_right * dt;
        integral_right = constrain(integral_right, -50.0, 50.0);
        double derivative_right = (error_right - lastError_right) / dt;
        double outputRight = kp_right * error_right + ki_right * integral_right + kd_right * derivative_right;
        lastError_right = error_right;

        // sync PID (currently disabled via kp_sync = 0 until inner loops are verified stable)
        double ratioLeft = (targetLeft != 0.0) ? (velocity_left / targetLeft)  : 1.0;
        double ratioRight = (targetRight != 0.0) ? (velocity_right / targetRight) : 1.0;
        double percentError = (ratioLeft - ratioRight) * 100.0;
        integral_sync += percentError * dt;
        double derivative_sync = (percentError - lastError_sync) / dt;
        double syncCorrection = kp_sync * percentError + ki_sync * integral_sync + kd_sync * derivative_sync;
        lastError_sync = percentError;

        double finalOutputLeft = outputLeft - syncCorrection;
        double finalOutputRight = outputRight + syncCorrection;

        Serial.print("Left Output: ");
        Serial.println(finalOutputLeft);
        Serial.print("Right Output: ");
        Serial.println(finalOutputRight);

        Serial.print("Left Error: ");
        Serial.println(error_left);
        Serial.print("Right Error: ");
        Serial.println(error_right);

        Serial.print("Left velocity: ");
        Serial.println(velocity_left);
        Serial.print("Right velocity: ");
        Serial.println(velocity_right);

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
        Serial.print("Left: ");
        Serial.println(leftPWM);
        Serial.print("Right: ");
        Serial.println(rightPWM);

        String leftSuccess = setLeftMotor(leftDir, leftPWM);
        String rightSuccess = setRightMotor(rightDir, rightPWM);
        Serial.println(rightSuccess);
    } while (moveActive);
}