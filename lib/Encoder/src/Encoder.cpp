#include "Encoder.h"

// Left Motor Encoder Pins
const int L_ENCA = 18; // INT3
const int L_ENCB = 19; // INT2

// Right Motor Encoder Pins
const int R_ENCA = 20; // INT1
const int R_ENCB = 21; // INT0

// Current Encoder States 
volatile uint8_t stateA = 0;
volatile uint8_t stateB = 0;

// encoder counters (single definition)
volatile long countA = 0;
volatile long countB = 0;

// Predetermined table to see which way the motor is spinning
const int8_t encTable[16] = {
    0, -1, +1, 0,
    +1, 0, 0, -1,
    -1, 0, 0, +1,
    0, +1, -1, 0
};

// Distance represented by one encoder tick (mm)
double distance_per_encoder_tick = 0.0051358;

static long prevLeftCount = 0;
static long prevRightCount = 0;

void Encoder_Init() {
    pinMode(L_ENCA, INPUT_PULLUP);
    pinMode(L_ENCB, INPUT_PULLUP);
    pinMode(R_ENCA, INPUT_PULLUP);
    pinMode(R_ENCB, INPUT_PULLUP);

    cli();
    EIMSK |= (1 << INT0) | (1 << INT1) | (1 << INT2) | (1 << INT3);

    EICRA |= (1 << ISC00) | (1 << ISC10) | (1 << ISC20) | (1 << ISC30);
    sei();
}

// Left encoder reading function - just updates the count, no timing/velocity here
void updateLeft() {
    uint8_t a = digitalRead(L_ENCA);
    uint8_t b = digitalRead(L_ENCB);
    uint8_t newState = (a << 1) | b;
    uint8_t index = (stateA << 2) | newState;
    countA += encTable[index];
    stateA = newState;
}

// Right encoder reading function - just updates the count, no timing/velocity here
void updateRight() {
    uint8_t a = digitalRead(R_ENCA);
    uint8_t b = digitalRead(R_ENCB);
    uint8_t newState = (a << 1) | b;
    uint8_t index = (stateB << 2) | newState;
    countB += encTable[index];
    stateB = newState;
}

long Encoder_getLeftEncoderCount() {
    long count;
    cli();
    count = countA;
    sei();
    return count;
}

long Encoder_getRightEncoderCount() {
    long count;
    cli();
    count = countB;
    sei();
    return count;
}

double Encoder_getLeftVelocity(double dt) {
    long current = Encoder_getLeftEncoderCount();
    double velocity = (current - prevLeftCount) * distance_per_encoder_tick / dt;
    prevLeftCount = current;
    return velocity; // mm/s
}

double Encoder_getRightVelocity(double dt) {
    long current = Encoder_getRightEncoderCount();
    double velocity = (current - prevRightCount) * distance_per_encoder_tick / dt;
    prevRightCount = current;
    return velocity; // mm/s
}

void Encoder_setLeftEncoderCountZero() {
    
    cli();
    countA = 0;
    prevLeftCount = 0;
    sei();
}

void Encoder_setRightEncoderCountZero() {
    cli();
    countB = 0;
    prevRightCount = 0;
    sei();
}

ISR(INT0_vect) { updateRight(); } // pin 21 = R_ENCB
ISR(INT1_vect) { updateRight(); } // pin 20 = R_ENCA
ISR(INT2_vect) { updateLeft();  } // pin 19 = L_ENCB
ISR(INT3_vect) { updateLeft();  } // pin 18 = L_ENCA