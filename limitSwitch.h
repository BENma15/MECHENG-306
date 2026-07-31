#ifndef LIMIT_SWITCH_H
#define LIMIT_SWITCH_H

#include <Arduino.h>

// Identifies which limit switch generated an event
enum LimitFlag : uint8_t
{
    LIMIT_NONE   = 0,
    LIMIT_LEFT   = 1 << 0,
    LIMIT_RIGHT  = 1 << 1,
    LIMIT_BOTTOM = 1 << 2,
    LIMIT_TOP    = 1 << 3
};

void setupLimitSwitches(uint8_t motor1PwmPin, uint8_t motor2PwmPin);

// Returns true after the corresponding motor has hit a limit
bool isMotor1LimitStopped();
bool isMotor2LimitStopped();

// Returns and clears the recorded limit events
uint8_t getLimitEvents();

#endif