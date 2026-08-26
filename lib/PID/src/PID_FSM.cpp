#include <Arduino.h>
#include <avr/interrupt.h>
#include <LimitSwitch.h>

#include <Encoder.h>
#include "PID_FSM.h"
#include <Motor.h>
#include <HelperFunctions.h>
#include <Graph.h>

#include <FSM.h>
// Time Constant Variables

// test increase of t1 and t2 from 0.05 and 0.10 to 0.1 and 0.2
double T1 = 0.2;
double T2 = 0.4;
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
double kp_left = 30, ki_left = 10,/**/ kd_left = 0, kff_left = 7;/**/ // all some variation of mm/s
double integral_left = 0, lastError_left = 0;                // kff is k_feedforward

// Right Motor PID Variables
double kp_right = 30, ki_right = 10,/**/ kd_right = 0, kff_right = 7;/**/ // all some variation of mm/s
double integral_right = 0, lastError_right = 0;                  // kff is k_feedforward

// Sync PID Variables
double kp_sync = 0, ki_sync = 2, kd_sync = 0;
double integral_sync = 0, lastError_sync = 0;

// Time Control Variables
uint32_t lastControlLoopMicros = 0;
const uint32_t CONTROL_LOOP_INTERVAL_US = 20000;        // 50Hz
const double dt = CONTROL_LOOP_INTERVAL_US / 1000000.0; // Seconds per control loop tick

// Motor Direction Variables
int8_t leftDir;
int8_t rightDir;

// encoder tracking
long moveCurrentLeftCount = 0;
long moveCurrentRightCount = 0;

long moveTargetLeftCount = 0;
long moveTargetRightCount = 0;

const double tolerance_mm = 0.2;
unsigned long elapsed_from_move_start = 0; // Variable to track elapsed time from the start of the movement

int integral_maxPWM = 20;

const int MIN_DRIVE_PWM = 90; // measure this: lowest PWM that reliably turns the motor under load

bool xTargetReached = false;
bool yTargetReached = false;

bool moveFinished = false;

// Returns the target velocity at different stages in the movement, using an s-curve profile.
double velocityProfile_FSM(double J, double Vf, double t4, double t)
{
    double V1 = 0.5 * J * T1 * T1; // Velocity after stage 1
    double V2 = J * T1 * T2;       // Velocity after stage 2

    if (t < T1)
    { // Stage 1
        return 0.5 * J * t * t;
    }
    else if (t < T1 + T2)
    { // Stage 2
        double tau = t - T1;
        return V1 + (J * T1) * tau;
    }
    else if (t < TA)
    { // Stage 3
        double tau = t - (T1 + T2);
        return V1 + V2 + (J * T1) * tau - 0.5 * J * tau * tau;
    }
    else if (t < TA + t4)
    { // Stage 4
        return Vf;
    }
    else if (t < TA + t4 + T1)
    { // Stage 5
        double tau = t - (TA + t4);
        return Vf - 0.5 * J * tau * tau;
    }
    else if (t < TA + t4 + T1 + T2)
    { // Stage 6
        double tau = t - (TA + t4 + T1);
        return Vf - V1 - (J * T1) * tau;
    }
    else if (t < TA + t4 + TA)
    { // Stage 7
        double tau = t - (TA + t4 + T1 + T2);
        return Vf - V1 - V2 - (J * T1) * tau + 0.5 * J * tau * tau;
    }
    else
    {
        // Move finished
        integral_left = 0;
        integral_right = 0;
        return 0;
    }
}

// Initial speed plan to find distance, time at constant velocity and if it is a triangular profile.
void plan_FSM(double x, double y, double vf_target)
{
    vf_target = vf_target / 60.0;
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

    // Ramp distance per unit of commanded velocity (constant, since T1/T2 are fixed)
    double k = T1 + T2/2; //da chat told me to change ts, lmk why this is the case 

    double vf;
    double t4;
    if (S < 2.0 * k * vf_target)
    {
        // Move too short to reach vf_target - solve for the reduced peak velocity
        // that makes the ramp (accel+decel, no cruise) exactly fit S.
        vf = S / (2.0 * k);
        t4 = 0.0;
        triangleProfile = true;
    }
    else {
        vf = vf_target;
        t4 = (S - 2.0 * k * vf) / vf;
        triangleProfile = false;
        //Serial.println("Vf: " + String(vf));
    }

    moveVf = vf;
    moveJ = moveVf / (T1 * (T1 + T2));
    moveT4 = t4;

    moveStartTime = millis() / 1000.0;
    moveActive = true;
}

// Main PID loop.
void move_FSM(int x, int y, int vf)
{
    // If movement has not started then set all the errors to 0 and call the movement plan
    if (moveStarted == false)
    {
        elapsed_from_move_start = millis(); // Reset the elapsed time from the start of the movement
        moveStarted = true;
        moveFinished = false;

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

        moveTargetLeftCount =
            moveCurrentLeftCount + distanceToCounts(x) + distanceToCounts(y);

        moveTargetRightCount =
            moveCurrentRightCount + distanceToCounts(x) - distanceToCounts(y);
    }
    cli();
    moveCurrentLeftCount = countA;
    moveCurrentRightCount = countB;
    sei();

    SystemState state = FSM_getCurrentState();

    if (state == STATE_FAULT)
    {
        moveActive = false;
        moveStarted = false;
        setLeftMotor(0, 0);
        setRightMotor(0, 0);
        return;
    }

    // Ensures there is a constant frequency of 50Hz which we are using for the PID loop as to not
    // call the motors too frequently.
    uint32_t nowMicros = micros();
    if (nowMicros - lastControlLoopMicros < CONTROL_LOOP_INTERVAL_US)
    {
        return;
    }
    lastControlLoopMicros += CONTROL_LOOP_INTERVAL_US;

    double velocity_left = Encoder_getLeftVelocity(dt);
    double velocity_right = Encoder_getRightVelocity(dt);

    // Gets current time in reference to start time and then finds target velocity
    double t = (millis() / 1000.0) - moveStartTime;
    double V = velocityProfile_FSM(moveJ, moveVf, moveT4, t);

    // This currently stops the motors once the time is finsihed, I think we should change this
    // to when the distance is met and not time.

    double currentLeftMM = countsToDistance(moveCurrentLeftCount);
    double currentRightMM = countsToDistance(moveCurrentRightCount);

    double currentX = (currentLeftMM + currentRightMM) / 2.0;
    double currentY = (currentLeftMM - currentRightMM) / 2.0;

    double xPositionError_mm = x - currentX;
    double yPositionError_mm = y - currentY;

    if (!xTargetReached) {
        xTargetReached = abs(xPositionError_mm) <= tolerance_mm;
    }
    if (!yTargetReached) {
        yTargetReached = abs(yPositionError_mm) <= tolerance_mm;
    }

    if (xTargetReached && yTargetReached)
    {
        moveActive = false;
        moveStarted = false;
        moveFinished = true;

        setLeftMotor(0, 0);
        setRightMotor(0, 0);

        Serial.println("Total horizontal distance travelled: " + String(currentX) + " mm");
        Serial.println("Total vertical distance travelled: " + String(currentY) + " mm");

        yTargetReached = false;
        xTargetReached = false;


        return;
    }

    // timeout function (always happening right now due to velocity profile going to 0 too soon)
    if (t >= TA + moveT4 + TA)
    {
        moveActive = false;
        moveStarted = false;
        moveFinished = true;

        setLeftMotor(0, 0);
        setRightMotor(0, 0);
        Serial.println("Move timed out, stopping motors.");
        Serial.println("Total horizontal distance travelled: " + String(currentX) + " mm");
        Serial.println("Total vertical distance travelled: " + String(currentY) + " mm");

        yTargetReached = false;
        xTargetReached = false;
        return;
    }

    // Splits target velocity into left and right motor target velocities using unit vectors
    double targetLeft;
    double targetRight;

    targetLeft = V * (moveUnitX + moveUnitY);
    targetRight = V * (moveUnitX - moveUnitY);

    // NEED TO ADD ANTI-WINDUP FOR ALL OF THEM

    // left velocity PID
    double error_left = targetLeft - velocity_left;
    if(abs((integral_left + error_left * dt) * ki_left) < integral_maxPWM){
    integral_left += error_left * dt;
    }
    double derivative_left = (error_left - lastError_left) / dt;
    double outputLeft = kp_left * error_left + ki_left * integral_left + kd_left * derivative_left;
    lastError_left = error_left;

    // right velocity PID
    double error_right = targetRight - velocity_right;
    if(abs((integral_right + error_right * dt) * ki_right) < integral_maxPWM ){
    integral_right += error_right * dt;
    }
    double derivative_right = (error_right - lastError_right) / dt;
    double outputRight = kp_right * error_right + ki_right * integral_right + kd_right * derivative_right;
    lastError_right = error_right;

    // sync PID
    double ratioLeft = (targetLeft != 0.0) ? (velocity_left / targetLeft) : 1.0;
    double ratioRight = (targetRight != 0.0) ? (velocity_right / targetRight) : 1.0;
    double percentError = (ratioLeft - ratioRight) * 100.0;
    integral_sync += percentError * dt;
    double derivative_sync = (percentError - lastError_sync) / dt;
    double syncCorrection = kp_sync * percentError + ki_sync * integral_sync + kd_sync * derivative_sync;
    lastError_sync = percentError;

        static unsigned long previous_time = 0;

    unsigned long current_time = millis();
    unsigned long elapsed_ms = current_time - previous_time;

    unsigned long timeSinceStart = current_time - elapsed_from_move_start;

    long leftEncoderError =
        moveTargetLeftCount - moveCurrentLeftCount;

    long rightEncoderError =
        moveTargetRightCount - moveCurrentRightCount;

    int leftTargetDir = 0;
    int rightTargetDir = 0;

    if (targetLeft > 0)
        leftTargetDir = 1;
    else if (targetLeft < 0)
        leftTargetDir = -1;

    if (targetRight > 0)
        rightTargetDir = 1;
    else if (targetRight < 0)
        rightTargetDir = -1;

    double feedforwardLeft = targetLeft * kff_left;
    double feedforwardRight = targetRight * kff_right;

    double finalOutputLeft =
        outputLeft - syncCorrection * leftTargetDir + feedforwardLeft; //<--- feedforward

    double finalOutputRight =
        outputRight + syncCorrection * rightTargetDir + feedforwardRight; //<--- feedforward
    // to get feedforward the target is being added to the output, and the pid will work to correct the error of the target
    // 1mm/s = kff pwm so if kff was 6 and the motor is told to go at "1mm/s" motors will recieve +6 pwm

    // Finds the direction of the motors
    if (finalOutputLeft > 0)
    {
        leftDir = 1;
    }
    else if (finalOutputLeft < 0)
    {
        leftDir = -1;
    }
    else
    {
        leftDir = 0;
    }

    if (finalOutputRight > 0)
    {
        rightDir = 1;
    }
    else if (finalOutputRight < 0)
    {
        rightDir = -1;
    }
    else
    {
        rightDir = 0;
    }

    // Sets the desired pwm of the motors
    int leftPWM = (int)abs(finalOutputLeft);
    int rightPWM = (int)abs(finalOutputRight);

    if (leftDir != 0)
    {
        leftPWM += MIN_DRIVE_PWM;
    }

    if (rightDir != 0)
    {
        rightPWM += MIN_DRIVE_PWM;
    }

    if (leftPWM > 255)
    {
        leftPWM = 255;
    }

    if (rightPWM > 255)
    {
        rightPWM = 255;
    }

    // Motor movement
    String leftSuccess = setLeftMotor(leftDir, leftPWM);
    String rightSuccess = setRightMotor(rightDir, rightPWM);

    /*if (elapsed_ms >= 10)
    {
        previous_time = current_time;
        addDataPoint(abs(leftEncoderError), abs(rightEncoderError), timeSinceStart);
    }*/

    // Serial.println("Left Position Error: " + String(leftPositionError));
    // Serial.println("Right Position Error: " + String(rightPositionError));

    //Serial.println("Left Velocity Error: " + String(error_left));
    //Serial.println("Right Velocity Error: " + String(error_right));
    Serial.println("Left Velocity: " + String(velocity_left));
    Serial.println("Right Velocity: " + String(velocity_right));
    Serial.println("Left PWM: " + String(leftPWM));
    Serial.println("Right PWM: " + String(rightPWM));
    // Serial.println(moveCurrentLeftCount);
}