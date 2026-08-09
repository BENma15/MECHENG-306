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
volatile uint8_t leftState = 0;
volatile uint8_t rightState = 0;

// Predetermined tables to see which way the motor is spinning 
// (doesnt need to be in header)
const int8_t encTable[16] = {
    0, -1, +1, 0,
    +1, 0, 0, -1,
    -1, 0, 0, +1,
    0, +1, -1, 0
};

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

// Left Encoder Reading Function
void Encoder_updateLeft() {
    // Read both values from the two left encoder channels
    uint8_t a = digitalRead(L_ENCA);
    uint8_t b = digitalRead(L_ENCB);

    // Left shift a encoder value and insert b into LSB to store both
    uint8_t newState = (a << 1) | b;
    // Left shift the current state by 2 bits and insert new state in the 2 LSB's
    uint8_t index = (leftState << 2) | newState;
    // Looking at table to figure out which way the motor is spinning
    leftEncoderCount += encTable[index];
    // Set current state to previous state
    leftState = newState;
}

// Right Encoder Reading Function
void Encoder_updateRight() {
    uint8_t a = digitalRead(R_ENCA);
    uint8_t b = digitalRead(R_ENCB);

    uint8_t newState = (a << 1) | b;
    uint8_t index = (rightState << 2) | newState;
    rightEncoderCount += encTable[index];
    rightState = newState;
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

ISR(INT0_vect) { Encoder_updateRight(); }
ISR(INT1_vect) { Encoder_updateRight(); }
ISR(INT2_vect) { Encoder_updateLeft(); }
ISR(INT3_vect) { Encoder_updateLeft(); }