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

// =============================================================================
// Time Constant Variables (UNCHANGED from your original)
// =============================================================================
double T1 = 0.2;
double T2 = 0.4;
double TA = T1 + T2 + T1;

// =============================================================================
// Movement Variables (UNCHANGED - moveJ, moveVf, moveT4 are still computed by
// plan_FSM exactly as before; they define the SHAPE of the profile. What
// changes is how we look up velocity FROM that shape - by distance, not time)
// =============================================================================
double moveJ = 0;
double moveVf = 0;
double moveT4 = 0;
double moveUnitX = 0;
double moveUnitY = 0;
double moveStartTime = 0;   // kept only for the timeout safety-net check now

bool moveActive = false;
bool moveStarted = false;
bool triangleProfile = false;

// =============================================================================
// NEW: Distance-profile variables
// =============================================================================
// V1   = absolute velocity at the END of stage 1 (= start of stage 2)
// a2   = acceleration during stage 2 (constant) = J*T1, reused for stage 3's
//        starting acceleration too
// S1   = cumulative distance at end of stage 1
// S2   = cumulative distance at end of stage 2
// S3   = cumulative distance at end of stage 3 (= end of the whole accel ramp)
// S4   = cumulative distance at end of cruise (stage 4)
// Stotal = total distance for the whole move (end of stage 7)
//
// NOTE ON CONVENTION: V1 and (implicitly) V2 here are ABSOLUTE velocities,
// not increments. V2 is never stored as its own variable - it's computed
// on the fly inside stage2_velocity_byDistance() each time it's needed,
// because it depends on how far into stage 2 we currently are.
double moveV1 = 0;
double moveA2 = 0;
double moveS1 = 0, moveS2 = 0, moveS3 = 0, moveS4 = 0, moveStotal = 0;

// NEW: monotonic accumulated distance traveled this move (mm), built up
// every control loop tick from actual encoder-derived position - this is
// what drives the velocity lookup now, instead of elapsed time.
double distanceTraveled = 0;
double prevMoveX = 0, prevMoveY = 0;   // previous tick's position, for the incremental-distance calc

// Left Motor PID Variables (UNCHANGED)
double kp_left = 10, ki_left = 0, kd_left = 0, kff_left = 1;
double integral_left = 0, lastError_left = 0;

// Right Motor PID Variables (UNCHANGED)
double kp_right = 10, ki_right = 0, kd_right = 0, kff_right = 1;
double integral_right = 0, lastError_right = 0;

// Sync PID Variables (UNCHANGED)
double kp_sync = 0, ki_sync = 0, kd_sync = 0;
double integral_sync = 0, lastError_sync = 0;

// Time Control Variables (UNCHANGED)
uint32_t lastControlLoopMicros = 0;
const uint32_t CONTROL_LOOP_INTERVAL_US = 20000;
const double dt = CONTROL_LOOP_INTERVAL_US / 1000000.0;

// Motor Direction Variables (UNCHANGED)
int8_t leftDir;
int8_t rightDir;

// encoder tracking (UNCHANGED)
long moveCurrentLeftCount = 0;
long moveCurrentRightCount = 0;
long moveTargetLeftCount = 0;
long moveTargetRightCount = 0;

const double tolerance_mm = 0.5;
unsigned long elapsed_from_move_start = 0;
int integral_maxPWM = 20;


// =============================================================================
// NEW: STAGE 1 (s -> v)  -  starts from rest, pure jerk, exact cube-root inversion
// =============================================================================
// Forward:  v(tau) = 0.5*J*tau^2
//           s(tau) = (1/6)*J*tau^3
// -----------------------------------------------------------------------
// NEW: minimum commanded velocity used while the robot hasn't yet covered
// startupThreshold_mm of real distance this move. Using a small threshold
// instead of an exact "== 0" check makes this immune to encoder noise -
// a tiny nonzero distanceTraveled from vibration or count jitter no longer
// falsely signals "we've started moving" before there's actually enough
// real velocity for the distance-based profile to take over meaningfully.
double breakawayVelocity = 5.0;      // mm/s
double startupThreshold_mm = 1.0;    // mm - tune to your plotter's static friction behaviour

double stage1_velocity_byDistance(double J, double s)
{
    double tau = cbrt((6.0 * s) / J);   // invert s(tau) for tau directly
    return 0.5 * J * tau * tau;         // plug into v(tau)
}

// =============================================================================
// NEW: STAGE 2 (s -> v)  -  constant acceleration, starts from V1 (not rest)
// =============================================================================
// Forward:  v(tau) = V1 + a2*tau
//           s(tau) = V1*tau + 0.5*a2*tau^2
//
// Rather than solve the quadratic for tau and substitute back in (which is
// the by-hand version we worked through on paper), we use v^2 = u^2 + 2as
// directly - algebraically identical result, one line instead of several.
//   sPrime = LOCAL distance into stage 2 only, i.e. (measured distance - S1)
// -----------------------------------------------------------------------
double stage2_velocity_byDistance(double V1, double a2, double sPrime)
{
    return sqrt(V1 * V1 + 2.0 * a2 * sPrime);
}

// =============================================================================
// NEW: STAGE 3 (s -> v)  -  jerk ramps accel down to 0, starts from V2 (absolute)
// =============================================================================
// Forward:  a(tau)  = a2 - J*tau
//           v(tau)  = V2 + a2*tau - 0.5*J*tau^2
//           s(tau)  = V2*tau + 0.5*a2*tau^2 - (J/6)*tau^3
//
// s(tau) is a genuine cubic (the tau^3 term survives), so this needs
// Cardano's formula to invert. V2 here is the ABSOLUTE velocity at the
// end of stage 2 (matches the convention fix from our derivation - NOT
// V1+V2 summed, since V2 already includes V1).
//   sPrime = LOCAL distance into stage 3 only, i.e. (measured distance - S2)
// -----------------------------------------------------------------------
double stage3_velocity_cardano(double J, double a2, double V2, double sPrime)
{
    double T1_local = a2 / J;   // recovers T1 from a2 = J*T1, avoids needing T1 passed separately

    // --- Standard cubic coefficients: A*tau^3 + B*tau^2 + C*tau + D = 0 ---
    double A = -J / 6.0;
    double B = 0.5 * a2;
    double C = V2;
    double D = -sPrime;

    // --- Normalize: tau^3 + p*tau^2 + q*tau + r = 0 ---
    double p = B / A;   // = -3*T1_local, always, for this system
    double q = C / A;
    double r = D / A;

    // --- Depress the cubic: tau = y + T1_local (since p = -3*T1_local) ---
    double P = q - (p * p) / 3.0;
    double Q = (2.0 * p * p * p) / 27.0 - (p * q) / 3.0 + r;

    double halfQ  = Q / 2.0;
    double thirdP = P / 3.0;
    double Delta  = halfQ * halfQ + thirdP * thirdP * thirdP;

    // Stage 3's velocity stays positive and monotonic across its entire
    // physical range (ramping from V2 up toward Vf, never reversing), which
    // guarantees exactly one real root - i.e. Delta >= 0 - for every valid
    // input this function will ever actually be called with. No fallback
    // branch, no loop: clamp away only floating-point noise right at a
    // boundary (e.g. Delta landing at -1e-15 instead of exactly 0).
    if (Delta < 0.0) Delta = 0.0;

    double sqrtDelta = sqrt(Delta);
    double u = cbrt(-halfQ + sqrtDelta);
    double v = cbrt(-halfQ - sqrtDelta);
    double y = u + v;

    double tau = y + T1_local;

    // Plug tau back into the forward velocity equation:
    return V2 + a2 * tau - 0.5 * J * tau * tau;
}

// =============================================================================
// NEW: computes all the distance boundaries ONCE per move, right after
// plan_FSM has computed moveJ, moveVf, moveT4 the way it already does.
// =============================================================================
void computeDistanceBoundaries()
{
    double a2 = moveJ * T1;                       // constant accel during stage 2 (and start of stage 3)
    double V1 = 0.5 * moveJ * T1 * T1;             // absolute velocity at end of stage 1

    // Absolute velocity at end of stage 2 - T2 is a FIXED constant (0.4s),
    // so this is just the forward SUVAT equation, no inversion needed here.
    double V2 = V1 + a2 * T2;

    double S1 = (1.0 / 6.0) * moveJ * T1 * T1 * T1;
    double S2 = S1 + (V1 * T2 + 0.5 * a2 * T2 * T2);

    // S3: total distance covered by the whole accel ramp (stages 1-3).
    // Same shortcut as your original plan_FSM's "k = T1 + T2/2":
    double S3 = moveVf * (T1 + T2 / 2.0);

    double S4 = S3 + moveVf * moveT4;              // end of cruise
    double Stotal = S4 + S3;                        // decel ramp mirrors accel ramp exactly

    moveA2 = a2;
    moveV1 = V1;
    moveS1 = S1;
    moveS2 = S2;
    moveS3 = S3;
    moveS4 = S4;
    moveStotal = Stotal;
}

// =============================================================================
// NEW: master distance-based lookup - replaces velocityProfile_FSM(...)
// =============================================================================
// s = distanceTraveled, the monotonic accumulated distance for this move.
// -----------------------------------------------------------------------
double velocityProfile_byDistance(double s)
{
    if (s >= moveStotal)
    {
        return 0.0;   // move complete
    }

    double r = moveStotal - s;                 // distance remaining
    bool decelerating = (s > moveS4);           // past cruise -> in the decel ramp

    // Stages 5-7 mirror stages 1-3 exactly, so fold both cases onto the
    // same three formulas by choosing which distance ("d") to feed them.
    double d = decelerating ? r : s;

    // Need V2 (absolute) for stage 3 either way - recompute it here since
    // we didn't store it as its own global (see comment above moveV1).
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
        // Only reachable when NOT decelerating - i.e. cruise phase.
        return moveVf;
    }
}


// =============================================================================
// plan_FSM - CHANGED: now also calls computeDistanceBoundaries() and resets
// the distance accumulator, right where you already reset the encoder zero.
// =============================================================================
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

    moveStartTime = millis() / 1000.0;   // kept for the timeout safety net only now
    moveActive = true;

    // NEW: build the distance boundaries for this move's shape
    computeDistanceBoundaries();

    // NEW: reset the distance accumulator for the new move
    distanceTraveled = 0;
    prevMoveX = 0;
    prevMoveY = 0;
}

// =============================================================================
// move_FSM - CHANGED: velocity now comes from distance, not elapsed time.
// Everything below the velocity lookup (PID, feedforward, sync, motor
// output) is UNCHANGED from your original - only how "V" gets computed differs.
// =============================================================================
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

    // NEW: update the monotonic distance accumulator using the incremental
    // change in position since last tick - NOT sqrt(currentX^2+currentY^2)
    // directly, since that can wobble backward slightly if the motors are
    // momentarily out of sync (sync PID hasn't caught up yet).
    double dx = currentX - prevMoveX;
    double dy = currentY - prevMoveY;
    distanceTraveled += sqrt(dx * dx + dy * dy);
    prevMoveX = currentX;
    prevMoveY = currentY;

    // CHANGED: V now comes from distance, not time.
    double V = velocityProfile_byDistance(distanceTraveled);

    // NEW: breakaway kick - stays active until real, meaningful motion
    // (startupThreshold_mm worth) has accumulated. Using a threshold rather
    // than an exact "== 0" check means a little encoder noise right at the
    // start can't prematurely hand control over to the distance profile
    // before there's enough real velocity for its math to mean anything.
    if (distanceTraveled < startupThreshold_mm && moveActive)
    {
        V = fmax(V, breakawayVelocity);
    }

    // CHANGED: completion is now decided PURELY by distance - no time
    // check anywhere in this function anymore. velocityProfile_byDistance
    // returns exactly 0 once distanceTraveled >= moveStotal (i.e. "no
    // distance error remaining"), so checking that directly is equivalent
    // to checking V == 0, and is the single source of truth for "are we done."
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
    Serial.println("Left Velocity Error: " + String(error_left));
    Serial.println("Right Velocity Error: " + String(error_right));

    Serial.println("Left PWM: " + String(leftPWM));
    Serial.println("Right PWM: " + String(rightPWM));
}