#ifndef MOTOR_MOVE_H
#define MOTOR_MOVE_H

#include <Arduino.h>

struct MotorMoveConfig {
    int m1Pin;
    int e1Pin;
    int e2Pin;
    int m2Pin;
    int motorSpeed;
    unsigned long moveTimeMs;
};

void motorMoveBegin(const MotorMoveConfig& config);
void motorMoveStop();
void motorMoveForDuration(int motor1Direction, int motor2Direction);
void motorMoveRight();
void motorMoveLeft();
void motorMoveUp();
void motorMoveDown();

#endif
