#include <Arduino.h>
#include <avr/interrupt.h>

// Motor A encoder pins
const int A_ENCA = 18;   // INT3
const int A_ENCB = 19;   // INT2
// Motor B encoder pins
const int B_ENCA = 20;   // INT1
const int B_ENCB = 21;   // INT0

const int E1 = 5;
const int M1 = 4;
const int E2 = 6;
const int M2 = 7;

const int MOVE_SPEED = 150;   // constant drive PWM while moving (0-255)

volatile long countA = 0;
volatile long countB = 0;
volatile uint8_t stateA = 0;
volatile uint8_t stateB = 0;

volatile long targetCountA = 0;
volatile long targetCountB = 0;
volatile bool movingA = false;
volatile bool movingB = false;
volatile int8_t dirA = 1;
volatile int8_t dirB = 1;

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

    if (movingA) {
        if ((dirA > 0 && countA >= targetCountA) || (dirA < 0 && countA <= targetCountA)) {
            analogWrite(E1, 0);
            movingA = false;
        }
    }

    Serial.println(countA);
    
}

void updateB() {
    uint8_t a = digitalRead(B_ENCA);
    uint8_t b = digitalRead(B_ENCB);

    uint8_t newState = (a << 1) | b;
    uint8_t index = (stateB << 2) | newState;
    countB += encTable[index];
    stateB = newState;

    if (movingB) {
        if ((dirB > 0 && countB >= targetCountB) || (dirB < 0 && countB <= targetCountB)) {
            analogWrite(E2, 0);
            movingB = false;
        }
    }

    Serial.println(countB);
}

long leftEncoderCounts(float dx_mm, float dy_mm) {
  return (long)((8256*(dx_mm - dy_mm))/(16*PI));
}

long rightEncoderCounts(float dx_mm, float dy_mm) {
  return (long)((8256*(dx_mm + dy_mm))/(16*PI));
}

void move(float dx_mm, float dy_mm) {
  long deltaA = leftEncoderCounts(dx_mm, dy_mm);
  long deltaB = rightEncoderCounts(dx_mm, dy_mm);

  noInterrupts();
  targetCountA = countA + deltaA;
  targetCountB = countB + deltaB;
  interrupts();

  dirA = (deltaA >= 0) ? 1 : -1;
  dirB = (deltaB >= 0) ? 1 : -1;

  digitalWrite(M1, dirA > 0 ? HIGH : LOW);
  digitalWrite(M2, dirB > 0 ? HIGH : LOW);

  movingA = (deltaA != 0);
  movingB = (deltaB != 0);

  analogWrite(E1, movingA ? MOVE_SPEED : 0);
  analogWrite(E2, movingB ? MOVE_SPEED : 0);
}

bool moving() {
  return movingA || movingB;
}

void setup() {
    Serial.begin(115200);
    pinMode(A_ENCA, INPUT_PULLUP);
    pinMode(A_ENCB, INPUT_PULLUP);
    pinMode(B_ENCA, INPUT_PULLUP);
    pinMode(B_ENCB, INPUT_PULLUP);

    pinMode(M1, OUTPUT);
    pinMode(M2, OUTPUT);

    cli();
    EIMSK |= (1 << INT0) | (1 << INT1) | (1 << INT2) | (1 << INT3);
    EICRA |= (1 << ISC00) | (1 << ISC10) | (1 << ISC20) | (1 << ISC30);
    sei();
}

void loop() {
    static bool testSent = false;
    if (!testSent && !moving()) {
        move(10, -5);
        testSent = true;
    }
}

ISR(INT0_vect) { updateB(); } // B_ENCB
ISR(INT1_vect) { updateB(); } // B_ENCA
ISR(INT2_vect) { updateA(); } // A_ENCB
ISR(INT3_vect) { updateA(); } // A_ENCA