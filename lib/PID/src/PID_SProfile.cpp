#include <Arduino.h>
#include <avr/interrupt.h>
#include <LimitSwitch.h>

#include <Encoder.h>

// Motor A encoder pins
const int L_ENCA = 18; // INT3
const int L_ENCB = 19; // INT2
// Motor B encoder pins
const int R_ENCA = 20; // INT1
const int R_ENCB = 21; // INT0

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

// Encoder variables to keep track of encoder count and encoder reading
volatile long countA = 0;
volatile long countB = 0;
volatile uint8_t stateA = 0;
volatile uint8_t stateB = 0;

// Distance to encoder count equation variables
const double COUNTS_PER_REV = 8256.0;
const double WHEEL_RADIUS_MM = 8.0;

// Predetermined tables to see which way the motor is spinning
const int8_t encTable[16] = {
    0, -1, +1, 0,
    +1, 0, 0, -1,
    -1, 0, 0, +1,
    0, +1, -1, 0
};

uint8_t timer1_current_left = 0;
uint8_t timer1_old_left = 0;
uint8_t timer1_current_left_new = 0;
uint8_t timer1_current_right = 0;
uint8_t timer1_old_right = 0;
uint8_t timer1_current_right_new = 0;

double time_per_tick = 1/16000000;

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

uint8_t overflow_counter = 0;

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

// Left encoder reading function
void updateLeft()
{
    // Read both values from the two left encoder channels
    uint8_t a = digitalRead(L_ENCA);
    uint8_t b = digitalRead(L_ENCB);

    // Left shift a encoder value and insert b into LSB to store both
    uint8_t newState = (a << 1) | b;
    // Left shift the current state by 2 bits and insert new state in the 2 LSB's
    uint8_t index = (stateA << 2) | newState;
    // Looking at table to figure out which way the motor is spinning
    countA += encTable[index];
    // Set current state to previous state
    stateA = newState;

    timer1_old_left = timer1_current_left;
    timer1_current_left = TCNT1;

    if (overflow_counter > 0) {
        timer1_current_left_new += overflow_counter * 65535;
    }

    timer1_current_left_new = timer1_current_left;

    velocity_left = distance_per_encoder_tick / ((timer1_current_left_new - timer1_old_left) * time_per_tick);
    overflow_counter = 0;
}

// Right encoder reading function
void updateRight()
{
    uint8_t a = digitalRead(R_ENCA);
    uint8_t b = digitalRead(R_ENCB);

    uint8_t newState = (a << 1) | b;
    uint8_t index = (stateB << 2) | newState;
    countB += encTable[index];
    stateB = newState;

    timer1_old_right = timer1_current_right;
    timer1_current_right = TCNT1;

    if (overflow_counter > 0) {
        timer1_current_right_new += overflow_counter * 65535;
    }

    timer1_current_right_new = timer1_current_right;

    velocity_right = distance_per_encoder_tick / ((timer1_current_right_new - timer1_old_right) * time_per_tick);
    overflow_counter = 0;
}

long distanceToCounts(double distance_mm) {
    return (long)round(COUNTS_PER_REV * distance_mm / (2.0 * PI * WHEEL_RADIUS_MM));
}

void SCruveInitialise() {

    pinMode(L_LIMIT, INPUT_PULLUP);
    pinMode(R_LIMIT, INPUT_PULLUP);
    pinMode(U_LIMIT, INPUT_PULLUP);
    pinMode(D_LIMIT, INPUT_PULLUP);

    pinMode(M1, OUTPUT);
    pinMode(M2, OUTPUT);
    pinMode(E1, OUTPUT);
    pinMode(E2, OUTPUT);

    setupLimitSwitches(E1, E2);

    cli();
    TCCR1B |= (1 << WGM12) | (1 << CS10);
    TIMSK1 |= (1 << TOIE1);
    OCR1A = 65535;
    TCNT1 = 0;
    sei();
}

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

void setup() {
    SCruveInitialise();
}

void loop() {
    if (!moveStarted) {
        moveStarted = true;
        plan(); // NEED PARAMETERS FROM SOMEWHERE
    }

    uint32_t nowMicros = micros();
    if (nowMicros - lastControlLoopMicros < CONTROL_LOOP_INTERVAL_US) {
        continue;
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

ISR(TIMER1_OVF_vect) {
    overflow_counter += 1;
}

ISR(INT0_vect) {
    updateRight();
}

ISR(INT1_vect) {
    updateRight();
}

ISR(INT2_vect) {
    updateLeft();
}

ISR(INT3_vect) {
    updateLeft();
}