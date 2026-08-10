#include "Motor.h"


// Initialise Motor Pins
void Motor_Init() {
    pinMode(M1, OUTPUT);
    pinMode(M2, OUTPUT);
    pinMode(E1, OUTPUT);
    pinMode(E2, OUTPUT);
}

// Left motor movement
void setLeftMotor(int8_t dir, int pwm) {
    if (dir == 0) {
        analogWrite(E1, 0);
        return;
    }
    digitalWrite(M1, dir > 0 ? HIGH : LOW);
    analogWrite(E1, pwm);
}

// Right motor movement
void setRightMotor(int8_t dir, int pwm) {
    if (dir == 0) {
        analogWrite(E2, 0);
        return;
    }
    digitalWrite(M2, dir > 0 ? HIGH : LOW);
    analogWrite(E2, pwm);
}