#include <Arduino.h>

#include "HomingDiagonal.h"
#include <LimitSwitchDebounce.h>
#include <Motion.h>

const int E1 = 5;
const int M1 = 4;
const int E2 = 6;
const int M2 = 7;

const int HOMING_PWM = 150;

bool hasHomed = false;
bool hasPulledOff = false;

void setupHoming() {
    Serial.begin(115200);
    setupMotion();
    setupLimitSwitches();
}

void driveUntilTopAndLeftLimits() {

    bool topReached = false;
    bool leftReached = false;

    while (!topReached || !leftReached) {
        updateLimitSwitches();

        if (isTopLimitPressed()) {
            topReached = true;
        }

        if (isLeftLimitPressed()) {
            leftReached = true;
        }

        if (!topReached && !leftReached) {
            // Move up and left together
            // Only motor 2 needs to move
            analogWrite(E1, 0);

            digitalWrite(M2, LOW);
            analogWrite(E2, HOMING_PWM);

        } else if (topReached && !leftReached) {
            // Top reached first, continue moving left
            digitalWrite(M1, LOW);
            digitalWrite(M2, LOW);

            analogWrite(E1, HOMING_PWM);
            analogWrite(E2, HOMING_PWM);

        } else if (leftReached && !topReached) {
            // Left reached first, continue moving up
            digitalWrite(M1, HIGH);
            digitalWrite(M2, LOW);

            analogWrite(E1, HOMING_PWM);
            analogWrite(E2, HOMING_PWM);
        }
    }

    analogWrite(E1, 0);
    analogWrite(E2, 0);
}

void homeMachine() {
    if (!hasHomed) {
        driveUntilTopAndLeftLimits();
        hasHomed = true;

    } else if (!hasPulledOff) {
        // Move 10 mm right and 10 mm down simultaneously
        moveTo(10, -10);
        hasPulledOff = true;
    }
}
