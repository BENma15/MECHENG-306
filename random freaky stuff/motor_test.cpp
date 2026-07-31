#include <Arduino.h>
#include "motor_move.h"


void setup()
{
    Serial.begin(115200);

    MotorMoveConfig config = {4, 5, 6, 7, 50, 500};
    motorMoveBegin(config);

    Serial.println("Enter: up, down, left, or right");
}


void loop()
{
    if (Serial.available() > 0)
    {
        String command = Serial.readStringUntil('\n');

        command.trim();
        command.toLowerCase();

        if (command == "up")
        {
            Serial.println("Moving up for 0.5 seconds");
            motorMoveUp();
        }
        else if (command == "down")
        {
            Serial.println("Moving down for 0.5 seconds");
            motorMoveDown();
        }
        else if (command == "left")
        {
            Serial.println("Moving left for 0.5 seconds");
            motorMoveLeft();
        }
        else if (command == "right")
        {
            Serial.println("Moving right for 0.5 seconds");
            motorMoveRight();
        }
        else
        {
            Serial.println("Unknown command");
        }
    }
}