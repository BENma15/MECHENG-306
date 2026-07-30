#include <Arduino.h>
#include <avr/interrupt.h>

// Motor A encoder pins
const int L_ENCA = 18;   // INT3
const int L_ENCB = 19;   // INT2
// Motor B encoder pins
const int R_ENCA = 20;   // INT1
const int R_ENCB = 21;   // INT0

const int L_LIMIT = 13;
const int R_LIMIT = 12;
const int U_LIMIT = 10;
const int D_LIMIT = 11;

const int E1 = 5;
const int M1 = 4;
const int E2 = 6;
const int M2 = 7;

volatile long countA = 0;
volatile long countB = 0;
volatile uint8_t stateA = 0;
volatile uint8_t stateB = 0;

const double COUNTS_PER_REV = 8256.0;
const double WHEEL_RADIUS_MM = 4.0;

int moveSpeed = 150;

const int8_t encTable[16] = {
    0, -1, +1,  0,
    +1,  0,  0, -1,
    -1,  0,  0, +1,
    0, +1, -1,  0
};

void updateA() {
    uint8_t a = digitalRead(L_ENCA);
    uint8_t b = digitalRead(L_ENCB);

    uint8_t newState = (a << 1) | b;
    uint8_t index = (stateA << 2) | newState;
    countA += encTable[index];
    stateA = newState;
}

void updateB() {
    uint8_t a = digitalRead(R_ENCA);
    uint8_t b = digitalRead(R_ENCB);

    uint8_t newState = (a << 1) | b;
    uint8_t index = (stateB << 2) | newState;
    countB += encTable[index];
    stateB = newState;
}

void setLeftMotor(int8_t dir, int pwm) {
    if (dir == 0) {
        analogWrite(E1, 0);
        return;
    }
    digitalWrite(M1, dir > 0 ? HIGH : LOW);
    analogWrite(E1, pwm);
}

void setRightMotor(int8_t dir, int pwm) {
    if (dir == 0) {
        analogWrite(E2, 0);
        return;
    }
    digitalWrite(M2, dir > 0 ? HIGH : LOW);
    analogWrite(E2, pwm);
}

long distanceToCounts(double distance_mm) {
    return (long) round(COUNTS_PER_REV * distance_mm / (2.0 * PI * WHEEL_RADIUS_MM));
}

void runAxis(double deltaA_mm, double deltaB_mm) {
    long targetA = distanceToCounts(deltaA_mm);
    long targetB = distanceToCounts(deltaB_mm);

    long startA = countA;
    long startB = countB;

    long absTargetA = abs(targetA);
    long absTargetB = abs(targetB);

    int8_t dirA = (targetA > 0) ? 1 : (targetA < 0 ? -1 : 0);
    int8_t dirB = (targetB > 0) ? 1 : (targetB < 0 ? -1 : 0);

    bool doneA = (absTargetA == 0);
    bool doneB = (absTargetB == 0);

    if (!doneA) setLeftMotor(dirA, moveSpeed);
    if (!doneB) setRightMotor(dirB, moveSpeed);

    while (!doneA || !doneB) {
        if (!doneA && labs(countA - startA) >= absTargetA) {
            setLeftMotor(0, 0);
            doneA = true;
        }

        if (!doneB && labs(countB - startB) >= absTargetB) {
            setRightMotor(0, 0);
            doneB = true;
        }
    }
}

void moveTo(double dx, double dy) {
    runAxis(dx, dx);
    runAxis(dy, -dy);
}

void setup() {
    Serial.begin(115200);
    pinMode(L_ENCA, INPUT_PULLUP);
    pinMode(L_ENCB, INPUT_PULLUP);
    pinMode(R_ENCA, INPUT_PULLUP);
    pinMode(R_ENCB, INPUT_PULLUP);

    pinMode(L_LIMIT, INPUT_PULLUP);
    pinMode(R_LIMIT, INPUT_PULLUP);
    pinMode(U_LIMIT, INPUT_PULLUP);
    pinMode(D_LIMIT, INPUT_PULLUP);

    pinMode(M1, OUTPUT);
    pinMode(M2, OUTPUT);
    pinMode(E1, OUTPUT);
    pinMode(E2, OUTPUT);

    cli();
    EIMSK |= (1 << INT0) | (1 << INT1) | (1 << INT2) | (1 << INT3);
    EICRA |= (1 << ISC00) | (1 << ISC10) | (1 << ISC20) | (1 << ISC30);
    sei();
}

void loop() {
    static bool hasMoved = false;

    if (!hasMoved) {
        moveTo(20, -10);
        hasMoved = true;
    }
}

ISR(INT0_vect) { updateB(); }
ISR(INT1_vect) { updateB(); }
ISR(INT2_vect) { updateA(); }
ISR(INT3_vect) { updateA(); }