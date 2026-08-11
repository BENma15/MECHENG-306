#include "LimitSwitch.h"
#include <avr/interrupt.h>
#include <Motor.h>

const uint8_t LEFT_LIMIT_PIN   = 10;  // PB4 / PCINT4
const uint8_t RIGHT_LIMIT_PIN  = 11;  // PB5 / PCINT5
const uint8_t BOTTOM_LIMIT_PIN = 12;  // PB6 / PCINT6
const uint8_t TOP_LIMIT_PIN    = 13;  // PB7 / PCINT7

static uint8_t motor1PwmPin;
static uint8_t motor2PwmPin;

static volatile bool motor1LimitStopped = false;
static volatile bool motor2LimitStopped = false;

static volatile uint8_t limitEvents = LIMIT_NONE;


void setupLimitSwitches(uint8_t suppliedMotor1PwmPin,
                        uint8_t suppliedMotor2PwmPin)
{
    motor1PwmPin = suppliedMotor1PwmPin;
    motor2PwmPin = suppliedMotor2PwmPin;

    pinMode(LEFT_LIMIT_PIN, INPUT_PULLUP);
    pinMode(RIGHT_LIMIT_PIN, INPUT_PULLUP);
    pinMode(BOTTOM_LIMIT_PIN, INPUT_PULLUP);
    pinMode(TOP_LIMIT_PIN, INPUT_PULLUP);

    noInterrupts();

    // Enable D10-D13 in the PCINT0 group
    PCMSK0 |= (1 << PCINT4);
    PCMSK0 |= (1 << PCINT5);
    PCMSK0 |= (1 << PCINT6);
    PCMSK0 |= (1 << PCINT7);

    // Clear any existing pin-change interrupt flag
    PCIFR |= (1 << PCIF0);

    // Enable PCINT group 0
    PCICR |= (1 << PCIE0);

    interrupts();
}


bool isMotor1LimitStopped()
{
    return motor1LimitStopped;
}


bool isMotor2LimitStopped()
{
    return motor2LimitStopped;
}


uint8_t getLimitEvents()
{
    noInterrupts();

    uint8_t events = limitEvents;
    limitEvents = LIMIT_NONE;

    interrupts();

    return events;
}


ISR(PCINT0_vect)
{
    // INPUT_PULLUP wiring:
    // released = HIGH
    // pressed  = LOW

    if (digitalRead(LEFT_LIMIT_PIN) == LOW)
    {
        analogWrite(motor1PwmPin, 0);
        motor1LimitStopped = true;
        limitEvents |= LIMIT_LEFT;
    }

    if (digitalRead(RIGHT_LIMIT_PIN) == LOW)
    {
        analogWrite(motor1PwmPin, 0);
        motor1LimitStopped = true;
        limitEvents |= LIMIT_RIGHT;
    }

    if (digitalRead(BOTTOM_LIMIT_PIN) == LOW)
    {
        analogWrite(motor2PwmPin, 0);
        motor2LimitStopped = true;
        limitEvents |= LIMIT_BOTTOM;
    }

    if (digitalRead(TOP_LIMIT_PIN) == LOW)
    {
        analogWrite(motor2PwmPin, 0);
        motor2LimitStopped = true;
        limitEvents |= LIMIT_TOP;
    }
}