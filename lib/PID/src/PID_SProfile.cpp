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

void setup() {

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

int velocityProfile () {

}

int plan () {
    
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

