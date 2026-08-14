#include "Homing.h"

#include <Motor.h>
#include <LimitSwitch.h>


static const int HOMING_PWM = 150;


enum HomingState
{
    HOMING_UP,
    HOMING_BACKOFF_DOWN,
    HOMING_LEFT,
    HOMING_BACKOFF_RIGHT,
    HOMING_DONE
};

static HomingState homingState = HOMING_UP;


static void stopMotors()
{
    analogWrite(E1, 0);
    analogWrite(E2, 0);
}


// Up
static void driveUp()
{
    digitalWrite(M1, HIGH);
    digitalWrite(M2, LOW);

    analogWrite(E1, HOMING_PWM);
    analogWrite(E2, HOMING_PWM);
}


// Down
static void driveDown()
{
    digitalWrite(M1, LOW);
    digitalWrite(M2, HIGH);

    analogWrite(E1, HOMING_PWM);
    analogWrite(E2, HOMING_PWM);
}


// Left
static void driveLeft()
{
    digitalWrite(M1, LOW);
    digitalWrite(M2, LOW);

    analogWrite(E1, HOMING_PWM);
    analogWrite(E2, HOMING_PWM);
}


// Right
static void driveRight()
{
    digitalWrite(M1, HIGH);
    digitalWrite(M2, HIGH);

    analogWrite(E1, HOMING_PWM);
    analogWrite(E2, HOMING_PWM);
}


void homingStart()
{
    stopMotors();

    homingState = HOMING_UP;
}


HomingResult homingUpdate()
{
    switch (homingState)
    {
        // Home up
        case HOMING_UP:

            if (LimitSwitch_leftPressed() ||
                LimitSwitch_rightPressed() ||
                LimitSwitch_bottomPressed())
            {
                stopMotors();
                return HOMING_FAULT;
            }

            if (LimitSwitch_topPressed())
            {
                stopMotors();
                homingState = HOMING_BACKOFF_DOWN;
                return HOMING_RUNNING;
            }

            driveUp();
            return HOMING_RUNNING;

        // Back off down
        case HOMING_BACKOFF_DOWN:

            if (LimitSwitch_leftPressed() ||
                LimitSwitch_rightPressed() ||
                LimitSwitch_bottomPressed())
            {
                stopMotors();
                return HOMING_FAULT;
            }

            if (!LimitSwitch_topPressed())
            {
                stopMotors();
                homingState = HOMING_LEFT;
                return HOMING_RUNNING;
            }

            driveDown();
            return HOMING_RUNNING;

        // Home left
        case HOMING_LEFT:

            if (LimitSwitch_topPressed() ||
                LimitSwitch_rightPressed() ||
                LimitSwitch_bottomPressed())
            {
                stopMotors();
                return HOMING_FAULT;
            }

            if (LimitSwitch_leftPressed())
            {
                stopMotors();
                homingState = HOMING_BACKOFF_RIGHT;
                return HOMING_RUNNING;
            }

            driveLeft();
            return HOMING_RUNNING;

        // Back off right
        case HOMING_BACKOFF_RIGHT:

            if (LimitSwitch_topPressed() ||
                LimitSwitch_rightPressed() ||
                LimitSwitch_bottomPressed())
            {
                stopMotors();
                return HOMING_FAULT;
            }

            if (!LimitSwitch_leftPressed())
            {
                stopMotors();
                homingState = HOMING_DONE;
                return HOMING_COMPLETE;
            }

            driveRight();
            return HOMING_RUNNING;

        case HOMING_DONE:

            stopMotors();
            return HOMING_COMPLETE;
    }

    stopMotors();
    return HOMING_FAULT;
}
