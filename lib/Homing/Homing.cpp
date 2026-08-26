#include "Homing.h"
#include <Motor.h>
#include <LimitSwitch.h>
#include <PID_FSM>

static const int homingFeedrate = 600;
static const int slowHomingFeedrate = 300;
static const int homingTravel = -99999;
static const int homingBackoffmm = 10;

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

static void stopMotors()
{
    analogWrite(E1, 0);
    analogWrite(E2, 0);
}

void homingStart()
{
    stopMotors();

    homingState = HOMING_DOWN;

    waiting = false;
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

            if (encoderCountsChanged())
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
            moveActive = false;
            moveStarted = false;

            startWait();
            waiting = true;

            return HOMING_RUNNING;
        }
        move_FSM(0, homingTravel, homingFeedrate);
        return HOMING_RUNNING;
    }

    case HOMING_BACKOFF_UP:
    {
        if (waiting)
        {
            stopMotors();

            if (encodercountsChanged())
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

        move_FSM(0, homingBackoffmm, homingFeedrate);

        if (moveFinished)
        {
            waiting = true;
        }
        return HOMING_RUNNING;
    }

    case HOMING_LEFT:
    {
        if (waiting)
        {
            stopMotors();

            if (encoderCountsChanged())
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
            moveActive = false;
            moveStarted = false;

            startWait();
            waiting = true;

            return HOMING_RUNNING;
        }
        move_FSM(homingTravel, 0, homingFeedrate);
        return HOMING_RUNNING;
    }

    case HOMING_BACKOFF_RIGHT:
    {
        if (waiting)
        {
            stopMotors();

            if (encodercountsChanged())
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

        move_FSM(homingBackoffmm, 0, homingFeedrate);

        if (moveFinished)
        {
            waiting = true;
        }
        return HOMING_RUNNING;
    }

    case HOMING_DONE:
    {
        stopMotors();
        return HOMING_COMPLETE;
    }
    }
    stopMotors();
    return HOMING_FAULT;
}
