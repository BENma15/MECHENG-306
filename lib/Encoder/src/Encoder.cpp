#include "Encoder.h"

// Left Motor Encoder Pins
const int L_ENCA = 18; // INT3
const int L_ENCB = 19; // INT2

// Right Motor eEncoder Pins
const int R_ENCA = 20; // INT1
const int R_ENCB = 21; // INT0

// Current Encoder Counts
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

// Current Encoder States (dont need to be in header)
volatile long countA = 0;
volatile long countB = 0;
volatile uint8_t stateA = 0;
volatile uint8_t stateB = 0;

// Predetermined tables to see which way the motor is spinning 
// (doesnt need to be in header)
const int8_t encTable[16] = {
    0, -1, +1, 0,
    +1, 0, 0, -1,
    -1, 0, 0, +1,
    0, +1, -1, 0
};

double velocity_left = 0; // In mm/s
double velocity_right = 0;
double distance_per_encoder_tick = 0.006089;

// Timer variables
uint8_t timer1_current_left = 0;
uint8_t timer1_old_left = 0;
uint8_t timer1_current_left_new = 0;
uint8_t timer1_current_right = 0;
uint8_t timer1_old_right = 0;
uint8_t timer1_current_right_new = 0;

double time_per_tick = 1/16000000;

uint8_t overflow_counter = 0;

void Encoder_Init() {
    pinMode(L_ENCA, INPUT_PULLUP);
    pinMode(L_ENCB, INPUT_PULLUP);
    pinMode(R_ENCA, INPUT_PULLUP);
    pinMode(R_ENCB, INPUT_PULLUP);

    cli();
    EIMSK |= (1 << INT0) | (1 << INT1) | (1 << INT2) | (1 << INT3);
    EICRA |= (1 << ISC00) | (1 << ISC10) | (1 << ISC20) | (1 << ISC30);

    TCCR1B |= (1 << WGM12) | (1 << CS10);
    TIMSK1 |= (1 << TOIE1);
    OCR1A = 65535;
    TCNT1 = 0;
    sei();
}

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

long Encoder_getLeftEncoderCount() {
    long count; 
    cli();  // disable interupts so data is not changed suring copying of long.
    count = leftEncoderCount;
    sei();
    return count;
}

long Encoder_getRightEncoderCount() {
    long count; 
    cli();
    count = rightEncoderCount;
    sei();
    return count;
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