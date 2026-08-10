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

uint8_t time_per_tick = 1/16000000;

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

    if (timer1_current_left < timer_old_left) {
        timer1_current_left_new += 65535;
    }

    timer1_current_left_new = timer1_current_left;

    velocity_left = distance_per_encoder_count/((timer1_current_left_new - timer1_old_left)*timer_per_tick);
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

    if (timer1_current_right < timer_old_right) {
        timer1_current_right_new += 65535;
    }

    timer1_current_right_new = timer1_current_right;

    velocity_right = distance_per_encoder_count/((timer1_current_right_new - timer1_old_right)*timer_per_tick);
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
    TCC1RB |= (1 << WGM12);
    OCR1A = 65535;
    TCNT1 = 0;
    sei();
}

int velocityProfile(double J, double Vf, double t4, double t) {
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
        return 0;
    }
}

bool plan(double x, double y, double vf_target) {
    double S = sqrt(x * x + y * y);
    if (S <= 0.0) return false;
 
    moveUnitX = x / S;
    moveUnitY = y / S;
    moveVf = vf_target;
    moveJ = moveVf / (T1 * (T1 + T2));
 
    double rampDistance = (4.0 / 6.0) * moveJ * T1 * T1 * T1 + moveJ * T1 * T2 * T2;
    double t4 = (S - rampDistance) / moveVf;
 
    if (t4 < 0) {
        moveActive = false;
        return false;
    }
 
    moveT4 = t4;
    moveStartTime = millis() / 1000.0;
    moveActive = true;
    return true;
}

void loop() {

}

ISR(INT0_vect) {
    updateB();
}

ISR(INT1_vect) {
    updateB();
}

ISR(INT2_vect) {
    updateA();
}

ISR(INT3_vect) {
    updateA();
}

