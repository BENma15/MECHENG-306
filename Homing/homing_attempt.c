#include <Arduino.h>

#include "limitSwitch.h"
#include "motion.h"

const int E1 = 5;
const int M1 = 4;
const int E2 = 6;
const int M2 = 7;

const int HOMING_PWM = 150;

bool hasMovedUp = false;
bool hasBouncedDown = false;
bool hasMovedLeft = false;
bool hasBouncedRight = false;

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

    setupLimitSwitches(E1, E2);

    cli();
    EIMSK |= (1 << INT0) | (1 << INT1) | (1 << INT2) | (1 << INT3);
    EICRA |= (1 << ISC00) | (1 << ISC10) | (1 << ISC20) | (1 << ISC30);
    sei();
}

void driveUntilTopLimit() {
    resetLimitStops();

    digitalWrite(M1, HIGH);
    digitalWrite(M2, LOW);
    analogWrite(E1, HOMING_PWM);
    analogWrite(E2, HOMING_PWM);

    while (!isMotor1LimitStopped() && !isMotor2LimitStopped()) {

    }

    analogWrite(E1, 0);
    analogWrite(E2, 0);
}

void driveUntilLeftLimit() {
    resetLimitStops();

    digitalWrite(M1, LOW);
    digitalWrite(M2, LOW);
    analogWrite(E1, HOMING_PWM);
    analogWrite(E2, HOMING_PWM);

    while (!isMotor1LimitStopped() && !isMotor2LimitStopped()) {
    }

    analogWrite(E1, 0);
    analogWrite(E2, 0);
}

void loop() {
    if (!hasMovedUp) {
        driveUntilTopLimit();
        hasMovedUp = true;
    } else if (!hasBouncedDown) {
        moveTo(0, -10);
        hasBouncedDown = true;
    } else if (!hasMovedLeft) {
        driveUntilLeftLimit();
        hasMovedLeft = true;
    } else if (!hasBouncedRight) {
        moveTo(10, 0);
        hasBouncedRight = true;
    }
}