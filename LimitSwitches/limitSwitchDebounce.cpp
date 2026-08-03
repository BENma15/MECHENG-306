#include "limitSwitch.h"
#include <avr/interrupt.h>

const int LEFT_LIMIT_PIN = 13;
const unsigned long DEBOUNCE_TIME_MS = 10;

volatile bool leftLimitInterruptOccurred = false;

bool leftRawState = HIGH;
bool leftDebouncedState = HIGH;
bool leftDebounceActive = false;

unsigned long leftLastChangeTime = 0;

void setupLimitSwitch()
{
    pinMode(LEFT_LIMIT_PIN, INPUT_PULLUP);

    leftRawState = digitalRead(LEFT_LIMIT_PIN);
    leftDebouncedState = leftRawState;
    leftLastChangeTime = millis();

    cli();

    PCMSK0 |= (1 << PCINT7);
    PCIFR |= (1 << PCIF0);
    PCICR |= (1 << PCIE0);

    sei();
}

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

bool isLeftLimitPressed()
{
    return leftDebouncedState == LOW;
}

ISR(PCINT0_vect)
{
    leftLimitInterruptOccurred = true;
}