#include "LimitSwitchDebounce.h"
#include <FSM.h>
#include <avr/interrupt.h>

static const int LEFT_LIMIT_PIN = 12;
static const int RIGHT_LIMIT_PIN = 13;
static const int TOP_LIMIT_PIN = 11;
static const int BOTTOM_LIMIT_PIN = 10;

static const unsigned long DEBOUNCE_TIME_MS = 10;

static volatile bool leftLimitInterruptOccurred = false;
static volatile bool rightLimitInterruptOccurred = false;
static volatile bool topLimitInterruptOccurred = false;
static volatile bool bottomLimitInterruptOccurred = false;

static bool leftRawState = LOW;
static bool rightRawState = LOW;
static bool topRawState = LOW;
static bool bottomRawState = LOW;

static bool leftDebouncedState = LOW;
static bool rightDebouncedState = LOW;
static bool topDebouncedState = LOW;
static bool bottomDebouncedState = LOW;

static bool leftDebounceActive = false;
static bool rightDebounceActive = false;
static bool topDebounceActive = false;
static bool bottomDebounceActive = false;

static unsigned long leftLastChangeTime = 0;
static unsigned long rightLastChangeTime = 0;
static unsigned long topLastChangeTime = 0;
static unsigned long bottomLastChangeTime = 0;


static void updateLimit(int pin, volatile bool &interruptOccurred, bool &rawState, bool &debouncedState, bool &debounceActive, unsigned long &lastChangeTime)
{
    unsigned long currentTime = millis();

    if (interruptOccurred)
    {
        interruptOccurred = false;
        debounceActive = true;
    }

    if (!debounceActive)
    {
        return;
    }

    bool currentRawState = digitalRead(pin);

    if (currentRawState != rawState)
    {
        rawState = currentRawState;
        lastChangeTime = currentTime;
    }

    if ((currentTime - lastChangeTime) >= DEBOUNCE_TIME_MS)
    {
        bool previousDebouncedState = debouncedState;
        debouncedState = rawState;
        debounceActive = false;

        if (debouncedState == HIGH && previousDebouncedState != HIGH)
        {
            FSM_triggerFault();
        }
    }
}

static void updateLimitSwitches()
{
    updateLimit(LEFT_LIMIT_PIN, leftLimitInterruptOccurred, leftRawState, leftDebouncedState, leftDebounceActive, leftLastChangeTime);
    updateLimit(RIGHT_LIMIT_PIN, rightLimitInterruptOccurred, rightRawState, rightDebouncedState, rightDebounceActive, rightLastChangeTime);
    updateLimit(TOP_LIMIT_PIN, topLimitInterruptOccurred, topRawState, topDebouncedState, topDebounceActive, topLastChangeTime);
    updateLimit(BOTTOM_LIMIT_PIN, bottomLimitInterruptOccurred, bottomRawState, bottomDebouncedState, bottomDebounceActive, bottomLastChangeTime);
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

    // Pin-change interrupts: wake the debounce state machine on any edge
    PCMSK0 |= (1 << PCINT4);
    PCMSK0 |= (1 << PCINT5);
    PCMSK0 |= (1 << PCINT6);
    PCMSK0 |= (1 << PCINT7);
    PCIFR  |= (1 << PCIF0);
    PCICR  |= (1 << PCIE0);

    // Timer1 CTC, 1kHz: drives updateLimitSwitches() without needing loop()
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10); // CTC, prescaler 64
    OCR1A  = 249;
    TIMSK1 = (1 << OCIE1A);

    sei();
}

ISR(PCINT0_vect)
{
    leftLimitInterruptOccurred = true;
    rightLimitInterruptOccurred = true;
    topLimitInterruptOccurred = true;
    bottomLimitInterruptOccurred = true;
}

ISR(TIMER1_COMPA_vect)
{
    updateLimitSwitches();
}