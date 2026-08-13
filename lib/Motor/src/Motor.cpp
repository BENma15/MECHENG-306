#include "Motor.h"
#include <FSM.h>


// Initialise Motor Pins
void Motor_Init() {
    pinMode(M1, OUTPUT);
    pinMode(M2, OUTPUT);
    pinMode(E1, OUTPUT);
    pinMode(E2, OUTPUT);
}

// Left motor movement
String setLeftMotor(int8_t dir, int pwm) {

    if(FSM_getCurrentState() == STATE_FAULT) {
        return "FAULT";
    } else if (dir == 0) {
        analogWrite(E1, 0);
        return "STOP";
    }

    digitalWrite(M1, dir > 0 ? LOW : HIGH);
    analogWrite(E1, pwm);

    return "SUCCESS";
}

// Right motor movement
String setRightMotor(int8_t dir, int pwm) {

    if(FSM_getCurrentState() == STATE_FAULT) {
        return "FAULT";
    } else if (dir == 0) {
        analogWrite(E2, 0);
        return "STOP";
    }
    
    digitalWrite(M2, dir > 0 ? LOW : HIGH);
    analogWrite(E2, pwm);

    return "SUCCESS";
}