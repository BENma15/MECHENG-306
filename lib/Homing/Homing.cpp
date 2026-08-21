#include "Homing.h"
#include <Motor.h>
#include <LimitSwitch.h>
#include <Encoder.h>

static const int HOMING_PWM = 150;
static const unsigned long HOMING_STATE_DELAY_MS = 20;

enum HomingState
{
    HOMING_DOWN,
    HOMING_BACKOFF_UP,
    HOMING_LEFT,
    HOMING_BACKOFF_RIGHT,
    HOMING_DONE
};

static HomingState homingState = HOMING_DOWN;

static bool waiting = false;
static unsigned long waitStart = 0;

static void stopMotors()
{
    analogWrite(E1, 0);
    analogWrite(E2, 0);
}

static void driveUp()
{
    digitalWrite(M1, HIGH);
    digitalWrite(M2, LOW);

    analogWrite(E1, HOMING_PWM);
    analogWrite(E2, HOMING_PWM);
}

static void driveDown()
{
    digitalWrite(M1, LOW);
    digitalWrite(M2, HIGH);

    analogWrite(E1, HOMING_PWM);
    analogWrite(E2, HOMING_PWM);
}

static void driveLeft()
{
    digitalWrite(M1, LOW);
    digitalWrite(M2, LOW);

    analogWrite(E1, HOMING_PWM);
    analogWrite(E2, HOMING_PWM);
}

static void driveRight()
{
    digitalWrite(M1, HIGH);
    digitalWrite(M2, HIGH);

    analogWrite(E1, HOMING_PWM);
    analogWrite(E2, HOMING_PWM);
}

static void startWait()
{
    stopMotors();

    waiting = true;
    waitStart = millis();
}

static bool waitFinished()
{
    return (millis() - waitStart) >= HOMING_STATE_DELAY_MS;
}

void homingStart()
{
    stopMotors();

    homingState = HOMING_DOWN;

    waiting = false;
    waitStart = 0;
}

HomingResult homingUpdate()
{
    switch (homingState)
    {
        case HOMING_DOWN:
        {
            if (waiting)
            {
                stopMotors();

                if (!waitFinished())
                {
                    return HOMING_RUNNING;
                }
                waiting = false;
                homingState = HOMING_BACKOFF_UP;
                return HOMING_RUNNING;
            }

            if (LimitSwitch_leftPressed() ||
                LimitSwitch_rightPressed() ||
                LimitSwitch_topPressed())
            {
                stopMotors();
                return HOMING_FAULT;
            }

            if (LimitSwitch_bottomPressed())
            {
                startWait();
                return HOMING_RUNNING;
            }
            driveDown();
            return HOMING_RUNNING;
        }
        
        case HOMING_BACKOFF_UP:
        {
            if (waiting)
            {
                stopMotors();
                
                if (!waitFinished())
                {
                    return HOMING_RUNNING;
                }

                waiting = false;
                homingState = HOMING_LEFT;
                return HOMING_RUNNING;
            }

            if (LimitSwitch_leftPressed() ||
                LimitSwitch_rightPressed() ||
                LimitSwitch_topPressed())
            {
                stopMotors();
                return HOMING_FAULT;
            }

            if (!LimitSwitch_bottomPressed())
            {
                startWait();
                return HOMING_RUNNING;
            }
            driveUp();
            return HOMING_RUNNING;
        }

        case HOMING_LEFT:
        {
            if (waiting)
            {
                stopMotors();
                
                if (!waitFinished())
                {
                    return HOMING_RUNNING;
                }
                waiting = false;
                homingState = HOMING_BACKOFF_RIGHT;
                return HOMING_RUNNING;
            }

            if (LimitSwitch_topPressed() ||
                LimitSwitch_rightPressed() ||
                LimitSwitch_bottomPressed())
            {
                stopMotors();
                return HOMING_FAULT;
            }

            if (LimitSwitch_leftPressed())
            {
                startWait();
                return HOMING_RUNNING;
            }
            driveLeft();
            return HOMING_RUNNING;
        }

        case HOMING_BACKOFF_RIGHT:
        {
            if (waiting)
            {
                stopMotors();
                
                if (!waitFinished())
                {
                    return HOMING_RUNNING;
                }
                waiting = false;
                homingState = HOMING_DONE;
                return HOMING_COMPLETE;
            }

            if (LimitSwitch_topPressed() ||
                LimitSwitch_rightPressed() ||
                LimitSwitch_bottomPressed())
            {
                stopMotors();
                return HOMING_FAULT;
            }

            if (!LimitSwitch_leftPressed())
            {
                startWait();
                return HOMING_RUNNING;
            }
            driveRight();
            return HOMING_RUNNING;
        }

        case HOMING_DONE:
        {
            stopMotors();
            Encoder_setLeftEncoderCountZero();
            Encoder_setRightEncoderCountZero();
            return HOMING_COMPLETE;
        }
    }
    stopMotors();
    return HOMING_FAULT;
}
