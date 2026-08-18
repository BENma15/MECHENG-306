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
double kp_left = 12, ki_left = 0.01, kd_left = 0;
double integral_left = 0, lastError_left = 0;

// Right Motor PID Variables
double kp_right = 12, ki_right = 0.01, kd_right = 0;
double integral_right = 0, lastError_right = 0;

// Sync PID Variables
double kp_sync = 0, ki_sync = 0, kd_sync = 0;
double integral_sync = 0, lastError_sync = 0;

// Time Control Variables
uint32_t lastControlLoopMicros = 0;
const uint32_t CONTROL_LOOP_INTERVAL_US = 2500;            // 50Hz
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
    double S = sqrt(x * x + y * y);
    if (S <= 0.0) return;

    moveUnitX = x / S;
    moveUnitY = y / S;

    // Ramp distance per unit of commanded velocity (constant, since T1/T2 are fixed)
    double k = ((4.0 / 6.0) * T1 * T1 + T2 * T2) / (T1 + T2);

    double vf;
    double t4;
    if (S < 2.0 * k * vf_target) {
        // Move too short to reach vf_target - solve for the reduced peak velocity
        // that makes the ramp (accel+decel, no cruise) exactly fit S.
        vf = S / (2.0 * k);
        t4 = 0.0;
        triangleProfile = true;
    } else {
        vf = vf_target;
        t4 = (S - 2.0 * k * vf) / vf;
        triangleProfile = false;
    }

    moveVf = vf;
    moveJ = moveVf / (T1 * (T1 + T2));
    moveT4 = t4;

    moveStartTime = millis() / 1000.0;
    moveActive = true;
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

        double velocity_left  = Encoder_getLeftVelocity(dt);
        double velocity_right = Encoder_getRightVelocity(dt);

        // Gets current time in reference to start time and then finds target velocity
        double t = (millis() / 1000.0) - moveStartTime;
        double V = velocityProfile_FSM(moveJ, moveVf, moveT4, t);

        // This currently stops the motors once the time is finsihed, I think we should change this
        // to when the distance is met and not time.
        if (t >= TA + moveT4 + TA) {
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
        // Also need to implement triangular profile

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
        double ratioLeft = (targetLeft != 0.0) ? (velocity_left / targetLeft)  : 1.0;
        double ratioRight = (targetRight != 0.0) ? (velocity_right / targetRight) : 1.0;
        double percentError = (ratioLeft - ratioRight) * 100.0;
        integral_sync += percentError * dt;
        double derivative_sync = (percentError - lastError_sync) / dt;
        double syncCorrection = kp_sync * percentError + ki_sync * integral_sync + kd_sync * derivative_sync;
        lastError_sync = percentError;

        double finalOutputLeft = outputLeft - syncCorrection;
        double finalOutputRight = outputRight + syncCorrection;

        //Serial.print("Right Integral: ")
        //Serial.println(integral_right)
        //Serial.print("Left Integral: ")
        //Serial.println(integral_left)

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
        //Serial.println(leftPWM);
        //Serial.println(rightPWM);

    
}