#include <Arduino.h>
#include <avr/interrupt.h>

// Motor A encoder pins
const int A_ENCA = 18;   // INT3
const int A_ENCB = 19;   // INT2
// Motor B encoder pins
const int B_ENCA = 20;   // INT1
const int B_ENCB = 21;   // INT0

volatile long countA = 0;
volatile long countB = 0;
volatile uint8_t stateA = 0;
volatile uint8_t stateB = 0;


// 0 - Not Moving
// 1 - Forward or Reverse
// -1 - Forward or Reverse
const int8_t encTable[16] = {
   0, -1, +1,  0,
  +1,  0,  0, -1,
  -1,  0,  0, +1,
   0, +1, -1,  0
};

void updateA() {
    // Read both A and B encoders
    uint8_t a = digitalRead(A_ENCA);
    uint8_t b = digitalRead(A_ENCB);

    uint8_t newState = (a << 1) | b;            // Set newState to a 2 digit binary
    uint8_t index = (stateA << 2) | newState;   // Set index to a 4 digits binary
    countA += encTable[index];                  // Sets countA to a number corresponding a direction
    stateA = newState;                          // Current state of encoders is updated
}

void updateB() {
    uint8_t a = digitalRead(B_ENCA);
    uint8_t b = digitalRead(B_ENCB);
    uint8_t newState = (a << 1) | b;
    uint8_t index = (stateB << 2) | newState;
    countB += encTable[index];
    stateB = newState;
}

void setup() {
    Serial.begin(115200);
    pinMode(A_ENCA, INPUT_PULLUP);
    pinMode(A_ENCB, INPUT_PULLUP);
    pinMode(B_ENCA, INPUT_PULLUP);
    pinMode(B_ENCB, INPUT_PULLUP);

    cli();
    EIMSK |= (1 << INT0) | (1 << INT1) | (1 << INT2) | (1 << INT3);
    EICRA |= (1 << ISC00) | (1 << ISC10) | (1 << ISC20) | (1 << ISC30); // any-edge on all 4
    sei();
}

void loop() {
  
}

ISR(INT0_vect) { updateB(); } // B_ENCB
ISR(INT1_vect) { updateB(); } // B_ENCA
ISR(INT2_vect) { updateA(); } // A_ENCB
ISR(INT3_vect) { updateA(); } // A_ENCA