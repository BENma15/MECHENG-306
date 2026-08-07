#include <Arduino.h>

#include "limitSwitchDebounce.h"
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
    setupMotion();
    setupLimitSwitches();
}

void driveUntilTopLimit() {

    digitalWrite(M1, HIGH);
    digitalWrite(M2, LOW);
    analogWrite(E1, HOMING_PWM);
    analogWrite(E2, HOMING_PWM);

    while (true) { 
        updateLimitSwitches();

        if (isTopLimitPressed()) {
            analogWrite(E1, 0);
            analogWrite(E2, 0);
            return;
        }
    }
}

void driveUntilLeftLimit() {

    digitalWrite(M1, LOW);
    digitalWrite(M2, LOW);
    analogWrite(E1, HOMING_PWM);
    analogWrite(E2, HOMING_PWM);

    while (true) { 
        updateLimitSwitches();

        if (isLeftLimitPressed()) {
            analogWrite(E1, 0);
            analogWrite(E2, 0);
            return;
        }
    }
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