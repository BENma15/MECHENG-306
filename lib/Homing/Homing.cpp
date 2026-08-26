#include "Homing.h"
#include <Motor.h>
#include <LimitSwitch.h>
#include <PID_FSM.h>
#include <Encoder.h>


// Homing movement settings
static const int homingFeedrate = 600;
static const int slowHomingFeedrate = 300;
static const int homingTravel = 99999;
static const int homingBackoffmm = 10;


enum HomingState
{
    HOMING_DOWN_FAST,
    HOMING_BACKOFF_UP_1,
    HOMING_DOWN_SLOW,
    HOMING_BACKOFF_UP_2,

    HOMING_LEFT_FAST,
    HOMING_BACKOFF_RIGHT_1,
    HOMING_LEFT_SLOW,
    HOMING_BACKOFF_RIGHT_2,

    HOMING_DONE
};

static HomingState homingState = HOMING_DOWN_FAST;

static bool waiting = false;

static void stopMotors()
{
    setLeftMotor(0, 0);
    setRightMotor(0, 0);
}

static HomingResult moveToLimit(int x, int y, int feedrate, bool targetSwitchPressed, bool invalidSwitchPressed, HomingState nextState)
    {
 
        if (waiting)
        {
            stopMotors();

            if (encoderCountsChanged())
            {
                return HOMING_RUNNING;
            }

            waiting = false;
            homingState = nextState;

            return HOMING_RUNNING;
        }

        if (invalidSwitchPressed)
        {
            stopMotors();

            moveActive = false;
            moveStarted = false;

            return HOMING_FAULT;
        }

        if (targetSwitchPressed)
        {
            moveActive = false;
            moveStarted = false;

            stopMotors();

            waiting = true;

            return HOMING_RUNNING;
        }
        move_FSM(x, y, feedrate);
        return HOMING_RUNNING;
    }


static HomingResult backoffMove(int x, int y, int feedrate, bool invalidSwitchPressed, HomingState nextState)
    {
        if (waiting)
        {
            stopMotors();

            if (encoderCountsChanged())
            {
                return HOMING_RUNNING;
            }

            waiting = false;
            homingState = nextState;

            if (nextState == HOMING_DONE)
            {
                return HOMING_COMPLETE;
            }
            return HOMING_RUNNING;
        }

        if (invalidSwitchPressed)
        {
            stopMotors();

            moveActive = false;
            moveStarted = false;

            return HOMING_FAULT;
        }

        move_FSM(x, y, feedrate);

        if (moveFinished)
        {
            waiting = true;
        }
        return HOMING_RUNNING;
    }


void homingStart()
{
    stopMotors();

    homingState = HOMING_DOWN_FAST;

    waiting = false;

    moveActive = false;
    moveStarted = false;
    moveFinished = false;
}


HomingResult homingUpdate()
{
    switch (homingState)
    {

    /*
        ============================================================
        Y AXIS - FIRST PASS
        ============================================================
    */

    case HOMING_DOWN_FAST:
    {
        return moveToLimit(
            0,
            -homingTravel,
            homingFeedrate,

            LimitSwitch_bottomPressed(),

            LimitSwitch_leftPressed() ||
            LimitSwitch_rightPressed() ||
            LimitSwitch_topPressed(),

            HOMING_BACKOFF_UP_1);
    }


    case HOMING_BACKOFF_UP_1:
    {
        return backoffMove(
            0,
            homingBackoffmm,
            homingFeedrate,

            LimitSwitch_leftPressed() ||
            LimitSwitch_rightPressed() ||
            LimitSwitch_topPressed(),

            HOMING_DOWN_SLOW);
    }


    /*
        ============================================================
        Y AXIS - SECOND / SLOW PASS
        ============================================================
    */

    case HOMING_DOWN_SLOW:
    {
        return moveToLimit(
            0,
            -homingTravel,
            slowHomingFeedrate,

            LimitSwitch_bottomPressed(),

            LimitSwitch_leftPressed() ||
            LimitSwitch_rightPressed() ||
            LimitSwitch_topPressed(),

            HOMING_BACKOFF_UP_2);
    }


    case HOMING_BACKOFF_UP_2:
    {
        return backoffMove(
            0,
            homingBackoffmm,
            homingFeedrate,

            LimitSwitch_leftPressed() ||
            LimitSwitch_rightPressed() ||
            LimitSwitch_topPressed(),

            HOMING_LEFT_FAST);
    }


    /*
        ============================================================
        X AXIS - FIRST PASS
        ============================================================
    */

    case HOMING_LEFT_FAST:
    {
        return moveToLimit(
            -homingTravel,
            0,
            homingFeedrate,

            LimitSwitch_leftPressed(),

            LimitSwitch_topPressed() ||
            LimitSwitch_rightPressed() ||
            LimitSwitch_bottomPressed(),

            HOMING_BACKOFF_RIGHT_1);
    }


    case HOMING_BACKOFF_RIGHT_1:
    {
        return backoffMove(
            homingBackoffmm,
            0,
            homingFeedrate,

            LimitSwitch_topPressed() ||
            LimitSwitch_rightPressed() ||
            LimitSwitch_bottomPressed(),

            HOMING_LEFT_SLOW);
    }


    /*
        ============================================================
        X AXIS - SECOND / SLOW PASS
        ============================================================
    */

    case HOMING_LEFT_SLOW:
    {
        return moveToLimit(
            -homingTravel,
            0,
            slowHomingFeedrate,

            LimitSwitch_leftPressed(),

            LimitSwitch_topPressed() ||
            LimitSwitch_rightPressed() ||
            LimitSwitch_bottomPressed(),

            HOMING_BACKOFF_RIGHT_2);
    }


    case HOMING_BACKOFF_RIGHT_2:
    {
        return backoffMove(
            homingBackoffmm,
            0,
            homingFeedrate,

            LimitSwitch_topPressed() ||
            LimitSwitch_rightPressed() ||
            LimitSwitch_bottomPressed(),

            HOMING_DONE);
    }


    /*
        ============================================================
        HOMING COMPLETE
        ============================================================
    */

        case HOMING_DONE:
        {
            stopMotors();

            return HOMING_COMPLETE;
        }
    }
    stopMotors();

    moveActive = false;
    moveStarted = false;

    return HOMING_FAULT;
}