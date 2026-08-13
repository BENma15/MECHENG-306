#include "LimitSwitchDebounce.h"
#include <FSM.h>
#include <avr/interrupt.h>

static const int LEFT_LIMIT_PIN   = 12;
static const int RIGHT_LIMIT_PIN  = 13;
static const int TOP_LIMIT_PIN    = 11;
static const int BOTTOM_LIMIT_PIN = 10;

static const unsigned long DEBOUNCE_TIME_MS = 10;

static volatile bool leftPressed = false;
static volatile bool rightPressed = false;
static volatile bool topPressed = false;
static volatile bool bottomPressed = false;

static volatile unsigned long leftLastTriggerTime   = 0;
static volatile unsigned long rightLastTriggerTime  = 0;
static volatile unsigned long topLastTriggerTime    = 0;
static volatile unsigned long bottomLastTriggerTime = 0;

static void checkLimit(int pin, volatile unsigned long &lastTriggerTime, volatile bool &pressedState)
{
    bool rawState = digitalRead(pin);

    if (rawState == LOW) {
        pressedState = false;
        return;
    }
    
    unsigned long now = millis();

    if ((now - lastTriggerTime) >= DEBOUNCE_TIME_MS)
    {
        return;
    }

    lastTriggerTime = now;
    FSM_triggerFault();

    if (FSM_getCurrentState() != STATE_HOMING) {
        FSM_triggerFault();
    }
}

void setupLimitSwitches()
{
    pinMode(LEFT_LIMIT_PIN, INPUT);
    pinMode(RIGHT_LIMIT_PIN, INPUT);
    pinMode(TOP_LIMIT_PIN, INPUT);
    pinMode(BOTTOM_LIMIT_PIN, INPUT);

    leftPressed = digitalRead(LEFT_LIMIT_PIN);
    rightPressed = digitalRead(RIGHT_LIMIT_PIN);
    topPressed = digitalRead(TOP_LIMIT_PIN);
    bottomPressed = digitalRead(BOTTOM_LIMIT_PIN);

    unsigned long currentTime = millis();

    leftLastTriggerTime   = currentTime - DEBOUNCE_TIME_MS;
    rightLastTriggerTime  = currentTime - DEBOUNCE_TIME_MS;
    topLastTriggerTime    = currentTime - DEBOUNCE_TIME_MS;
    bottomLastTriggerTime = currentTime - DEBOUNCE_TIME_MS;

    cli();

    PCMSK0 |= (1 << PCINT4);
    PCMSK0 |= (1 << PCINT5);
    PCMSK0 |= (1 << PCINT6);
    PCMSK0 |= (1 << PCINT7);

    // Clear any pending pin change interrupt.
    PCIFR |= (1 << PCIF0);

    // Enable the PCINT0 interrupt group.
    PCICR |= (1 << PCIE0);

    sei();
}

bool LimitSwitch_leftPressed() {
    return leftPressed;
}

bool LimitSwitch_rightPressed() {
    return rightPressed;
}

bool LimitSwitch_topPressed() {
    return topPressed;
}

bool LimitSwitch_bottomPressed() {
    return bottomPressed;
}

ISR(PCINT0_vect)
{
    checkLimit(LEFT_LIMIT_PIN,   leftLastTriggerTime, leftPressed);
    checkLimit(RIGHT_LIMIT_PIN,  rightLastTriggerTimer, rightPressed);
    checkLimit(TOP_LIMIT_PIN,    topLastTriggerTime, topPressed);
    checkLimit(BOTTOM_LIMIT_PIN, bottomLastTriggerTime, bottomPressed);
}
