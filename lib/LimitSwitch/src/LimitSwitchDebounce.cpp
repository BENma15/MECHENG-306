#include "LimitSwitchDebounce.h"
#include <FSM.h>
#include <avr/interrupt.h>

static const int LEFT_LIMIT_PIN = 12;
static const int RIGHT_LIMIT_PIN = 13;
static const int TOP_LIMIT_PIN = 11;
static const int BOTTOM_LIMIT_PIN = 10;

static const unsigned long DEBOUNCE_TIME_MS = 10;

static bool leftDebouncedState = LOW;
static bool rightDebouncedState = LOW;
static bool topDebouncedState = LOW;
static bool bottomDebouncedState = LOW;

static unsigned long leftLastEdgeTime = 0;
static unsigned long rightLastEdgeTime = 0;
static unsigned long topLastEdgeTime = 0;
static unsigned long bottomLastEdgeTime = 0;

static void checkLimit(int pin, unsigned long &lastEdgeTime, bool &debouncedState)
{
    unsigned long now = millis();
    bool raw = digitalRead(pin);

    if (raw != debouncedState && (now - lastEdgeTime) >= DEBOUNCE_TIME_MS)
    {
        lastEdgeTime = now;
        debouncedState = raw;

        if (debouncedState == HIGH)
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

    leftDebouncedState = digitalRead(LEFT_LIMIT_PIN);
    rightDebouncedState = digitalRead(RIGHT_LIMIT_PIN);
    topDebouncedState = digitalRead(TOP_LIMIT_PIN);
    bottomDebouncedState = digitalRead(BOTTOM_LIMIT_PIN);

    unsigned long currentTime = millis();
    leftLastEdgeTime = currentTime;
    rightLastEdgeTime = currentTime;
    topLastEdgeTime = currentTime;
    bottomLastEdgeTime = currentTime;

    cli();

    PCMSK0 |= (1 << PCINT4);
    PCMSK0 |= (1 << PCINT5);
    PCMSK0 |= (1 << PCINT6);
    PCMSK0 |= (1 << PCINT7);
    PCIFR  |= (1 << PCIF0);
    PCICR  |= (1 << PCIE0);

    sei();
}

ISR(PCINT0_vect)
{
    checkLimit(LEFT_LIMIT_PIN, leftLastEdgeTime, leftDebouncedState);
    checkLimit(RIGHT_LIMIT_PIN, rightLastEdgeTime, rightDebouncedState);
    checkLimit(TOP_LIMIT_PIN, topLastEdgeTime, topDebouncedState);
    checkLimit(BOTTOM_LIMIT_PIN, bottomLastEdgeTime, bottomDebouncedState);
}
