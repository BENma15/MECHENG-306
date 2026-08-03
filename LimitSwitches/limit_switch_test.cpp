#include <Arduino.h>
#include "limitSwitchDebounce.h"

bool previousLeftLimitState = false;

void setup()
{
    Serial.begin(115200);

    pinMode(LED_BUILTIN, OUTPUT);

    setupLimitSwitch();

    previousLeftLimitState = isLeftLimitPressed();
    digitalWrite(LED_BUILTIN, previousLeftLimitState);

    Serial.println("Left limit switch debounce test");

    if (previousLeftLimitState)
    {
        Serial.println("Initial state: PRESSED");
    }
    else
    {
        Serial.println("Initial state: RELEASED");
    }
}

void loop()
{
    updateLeftLimit();

    bool currentLeftLimitState = isLeftLimitPressed();

    if (currentLeftLimitState != previousLeftLimitState)
    {
        previousLeftLimitState = currentLeftLimitState;

        digitalWrite(LED_BUILTIN, currentLeftLimitState);

        Serial.print(millis());
        Serial.print(" ms: Left limit ");

        if (currentLeftLimitState)
        {
            Serial.println("PRESSED");
        }
        else
        {
            Serial.println("RELEASED");
        }
    }
}