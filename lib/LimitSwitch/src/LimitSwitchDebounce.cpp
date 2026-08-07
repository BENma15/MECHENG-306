#include "limitSwitchDebounce.h"
#include <avr/interrupt.h>

const int LEFT_LIMIT_PIN = 12;
const int RIGHT_LIMIT_PIN = 13;
const int TOP_LIMIT_PIN = 11;
const int BOTTOM_LIMIT_PIN = 10;

const unsigned long DEBOUNCE_TIME_MS = 10;

volatile bool leftLimitInterruptOccurred = false;
volatile bool rightLimitInterruptOccurred = false;
volatile bool topLimitInterruptOccurred = false;
volatile bool bottomLimitInterruptOccurred = false;

bool leftRawState = LOW;
bool rightRawState = LOW;
bool topRawState = LOW;
bool bottomRawState = LOW;

bool leftDebouncedState = LOW;
bool rightDebouncedState = LOW;
bool topDebouncedState = LOW;
bool bottomDebouncedState = LOW;

bool leftDebounceActive = false;
bool rightDebounceActive = false;
bool topDebounceActive = false;
bool bottomDebounceActive = false;

unsigned long leftLastChangeTime = 0;
unsigned long rightLastChangeTime = 0;
unsigned long topLastChangeTime = 0;
unsigned long bottomLastChangeTime = 0;

void updateLeftLimit()
{
    unsigned long currentTime = millis();

    if (leftLimitInterruptOccurred)
    {
        leftLimitInterruptOccurred = false;
        leftDebounceActive = true;
    }

    if (!leftDebounceActive)
    {
        return;
    }

    bool currentRawState = digitalRead(LEFT_LIMIT_PIN);

    if (currentRawState != leftRawState)
    {
        leftRawState = currentRawState;
        leftLastChangeTime = currentTime;
    }

    if ((currentTime - leftLastChangeTime) >= DEBOUNCE_TIME_MS)
    {
        leftDebouncedState = leftRawState;
        leftDebounceActive = false;
    }
}

void updateRightLimit()
{
    unsigned long currentTime = millis();

    if (rightLimitInterruptOccurred)
    {
        rightLimitInterruptOccurred = false;
        rightDebounceActive = true;
    }

    if (!rightDebounceActive)
    {
        return;
    }

    bool currentRawState = digitalRead(RIGHT_LIMIT_PIN);

    if (currentRawState != rightRawState)
    {
        rightRawState = currentRawState;
        rightLastChangeTime = currentTime;
    }

    if ((currentTime - rightLastChangeTime) >= DEBOUNCE_TIME_MS)
    {
        rightDebouncedState = rightRawState;
        rightDebounceActive = false;
    }
}

void updateTopLimit()
{
    unsigned long currentTime = millis();

    if (topLimitInterruptOccurred)
    {
        topLimitInterruptOccurred = false;
        topDebounceActive = true;
    }

    if (!topDebounceActive)
    {
        return;
    }

    bool currentRawState = digitalRead(TOP_LIMIT_PIN);

    if (currentRawState != topRawState)
    {
        topRawState = currentRawState;
        topLastChangeTime = currentTime;
    }

    if ((currentTime - topLastChangeTime) >= DEBOUNCE_TIME_MS)
    {
        topDebouncedState = topRawState;
        topDebounceActive = false;
    }
}

void updateBottomLimit()
{
    unsigned long currentTime = millis();

    if (bottomLimitInterruptOccurred)
    {
        bottomLimitInterruptOccurred = false;
        bottomDebounceActive = true;
    }

    if (!bottomDebounceActive)
    {
        return;
    }

    bool currentRawState = digitalRead(BOTTOM_LIMIT_PIN);

    if (currentRawState != bottomRawState)
    {
        bottomRawState = currentRawState;
        bottomLastChangeTime = currentTime;
    }

    if ((currentTime - bottomLastChangeTime) >= DEBOUNCE_TIME_MS)
    {
        bottomDebouncedState = bottomRawState;
        bottomDebounceActive = false;
    }
}

void setupLimitSwitches()
{
    pinMode(LEFT_LIMIT_PIN, INPUT);
    pinMode(RIGHT_LIMIT_PIN, INPUT);
    pinMode(TOP_LIMIT_PIN, INPUT);
    pinMode(BOTTOM_LIMIT_PIN, INPUT);

    leftRawState = digitalRead(LEFT_LIMIT_PIN);
    rightRawState = digitalRead(RIGHT_LIMIT_PIN);
    topRawState = digitalRead(TOP_LIMIT_PIN);
    bottomRawState = digitalRead(BOTTOM_LIMIT_PIN);

    leftDebouncedState = leftRawState;
    rightDebouncedState = rightRawState;
    topDebouncedState = topRawState;
    bottomDebouncedState = bottomRawState;

    unsigned long currentTime = millis();

    leftLastChangeTime = currentTime;
    rightLastChangeTime = currentTime;
    topLastChangeTime = currentTime;
    bottomLastChangeTime = currentTime;

    cli();

    PCMSK0 |= (1 << PCINT4);
    PCMSK0 |= (1 << PCINT5);
    PCMSK0 |= (1 << PCINT6);
    PCMSK0 |= (1 << PCINT7);

    PCIFR |= (1 << PCIF0);
    PCICR |= (1 << PCIE0);

    sei();
}

void updateLimitSwitches()
{
    updateLeftLimit();
    updateRightLimit();
    updateTopLimit();
    updateBottomLimit();
}

bool isLeftLimitPressed()
{
    return leftDebouncedState == HIGH;
}

bool isRightLimitPressed()
{
    return rightDebouncedState == HIGH;
}

bool isTopLimitPressed()
{
    return topDebouncedState == HIGH;
}

bool isBottomLimitPressed()
{
    return bottomDebouncedState == HIGH;
}

bool isAnyLimitPressed()
{
    return isLeftLimitPressed() ||
           isRightLimitPressed() ||
           isTopLimitPressed() ||
           isBottomLimitPressed();
}

ISR(PCINT0_vect)
{
    leftLimitInterruptOccurred = true;
    rightLimitInterruptOccurred = true;
    topLimitInterruptOccurred = true;
    bottomLimitInterruptOccurred = true;
}
