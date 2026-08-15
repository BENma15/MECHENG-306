#include "LimitSwitch.h"
#include <FSM.h>
#include <avr/interrupt.h>

static const int LEFT_LIMIT_PIN   = 12;
static const int RIGHT_LIMIT_PIN  = 13;
static const int TOP_LIMIT_PIN    = 11;
static const int BOTTOM_LIMIT_PIN = 10;

static const unsigned long DEBOUNCE_TIME_MS = 10;

static volatile bool leftState   = LOW;
static volatile bool rightState  = LOW;
static volatile bool topState    = LOW;
static volatile bool bottomState = LOW;

static volatile unsigned long leftLastEdgeTime   = 0;
static volatile unsigned long rightLastEdgeTime  = 0;
static volatile unsigned long topLastEdgeTime    = 0;
static volatile unsigned long bottomLastEdgeTime = 0;

static void checkLimit(int pin, volatile bool &switchState, volatile unsigned long &lastEdgeTime) {
    unsigned long now = millis();

    if ((now - lastEdgeTime) < DEBOUNCE_TIME_MS)
    {
        return;
    }

    bool newState = digitalRead(pin);

    if (newState == switchState)
    {
        return;
    }

    switchState = newState;
    lastEdgeTime = now;

    if (switchState == HIGH)
    {
        if (FSM_getCurrentState() != STATE_HOMING)
        {
            FSM_triggerFault();
        }
    }
}

void setupLimitSwitches()
{
    pinMode(LEFT_LIMIT_PIN, INPUT);
    pinMode(RIGHT_LIMIT_PIN, INPUT);
    pinMode(TOP_LIMIT_PIN, INPUT);
    pinMode(BOTTOM_LIMIT_PIN, INPUT);

    leftState   = digitalRead(LEFT_LIMIT_PIN);
    rightState  = digitalRead(RIGHT_LIMIT_PIN);
    topState    = digitalRead(TOP_LIMIT_PIN);
    bottomState = digitalRead(BOTTOM_LIMIT_PIN);

    if (leftState || rightState || topState || bottomState) {
        FSM_triggerFault();
    }

    unsigned long now = millis();

    leftLastEdgeTime   = now - DEBOUNCE_TIME_MS;
    rightLastEdgeTime  = now - DEBOUNCE_TIME_MS;
    topLastEdgeTime    = now - DEBOUNCE_TIME_MS;
    bottomLastEdgeTime = now - DEBOUNCE_TIME_MS;

    cli();

    PCMSK0 |= (1 << PCINT4);
    PCMSK0 |= (1 << PCINT5);
    PCMSK0 |= (1 << PCINT6);
    PCMSK0 |= (1 << PCINT7);

    PCIFR |= (1 << PCIF0);
    PCICR |= (1 << PCIE0);

    sei();
}

bool LimitSwitch_leftPressed()
{
    return leftState;
}


bool LimitSwitch_rightPressed()
{
    return rightState;
}


bool LimitSwitch_topPressed()
{
    return topState;
}


bool LimitSwitch_bottomPressed()
{
    return bottomState;
}

ISR(PCINT0_vect)
{
    checkLimit(
        LEFT_LIMIT_PIN,
        leftState,
        leftLastEdgeTime
    );


    checkLimit(
        RIGHT_LIMIT_PIN,
        rightState,
        rightLastEdgeTime
    );


    checkLimit(
        TOP_LIMIT_PIN,
        topState,
        topLastEdgeTime
    );


    checkLimit(
        BOTTOM_LIMIT_PIN,
        bottomState,
        bottomLastEdgeTime
    );
}
