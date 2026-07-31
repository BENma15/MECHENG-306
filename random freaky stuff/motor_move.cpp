#include "motor_move.h"

namespace {
MotorMoveConfig g_config = {4, 5, 6, 7, 50, 500};
bool g_initialized = false;

const int A_POSITIVE = HIGH;
const int A_NEGATIVE = LOW;
const int B_POSITIVE = HIGH;
const int B_NEGATIVE = LOW;
}

void motorMoveBegin(const MotorMoveConfig& config)
{
    g_config = config;

    pinMode(g_config.m1Pin, OUTPUT);
    pinMode(g_config.e1Pin, OUTPUT);
    pinMode(g_config.m2Pin, OUTPUT);
    pinMode(g_config.e2Pin, OUTPUT);

    motorMoveStop();
    g_initialized = true;
}

void motorMoveStop()
{
    analogWrite(g_config.e1Pin, 0);
    analogWrite(g_config.e2Pin, 0);
}

void motorMoveForDuration(int motor1Direction, int motor2Direction)
{
    if (!g_initialized) {
        motorMoveBegin(g_config);
    }

    digitalWrite(g_config.m1Pin, motor1Direction);
    digitalWrite(g_config.m2Pin, motor2Direction);

    analogWrite(g_config.e1Pin, g_config.motorSpeed);
    analogWrite(g_config.e2Pin, g_config.motorSpeed);

    delay(g_config.moveTimeMs);

    motorMoveStop();
}

void motorMoveRight()
{
    motorMoveForDuration(A_POSITIVE, B_POSITIVE);
}

void motorMoveLeft()
{
    motorMoveForDuration(A_NEGATIVE, B_NEGATIVE);
}

void motorMoveUp()
{
    motorMoveForDuration(A_POSITIVE, B_NEGATIVE);
}

void motorMoveDown()
{
    motorMoveForDuration(A_NEGATIVE, B_POSITIVE);
}
