#include <Arduino.h>

// Motor 1 = left motor = Motor A
const int M1 = 4;
const int E1 = 5;

// Motor 2 = right motor = Motor B
const int E2 = 6;
const int M2 = 7;

const int MOTOR_SPEED = 50;
const unsigned long MOVE_TIME = 500; // 500 ms = 0.5 seconds

const int A_POSITIVE = HIGH;
const int A_NEGATIVE = LOW;

const int B_POSITIVE = HIGH;
const int B_NEGATIVE = LOW;


void stopMotors()
{
    analogWrite(E1, 0);
    analogWrite(E2, 0);
}


void moveRight()
{
    digitalWrite(M1, A_POSITIVE);
    digitalWrite(M2, B_POSITIVE);

    analogWrite(E1, MOTOR_SPEED);
    analogWrite(E2, MOTOR_SPEED);
}


void moveLeft()
{
    digitalWrite(M1, A_NEGATIVE);
    digitalWrite(M2, B_NEGATIVE);

    analogWrite(E1, MOTOR_SPEED);
    analogWrite(E2, MOTOR_SPEED);
}


void moveUp()
{
    digitalWrite(M1, A_POSITIVE);
    digitalWrite(M2, B_NEGATIVE);

    analogWrite(E1, MOTOR_SPEED);
    analogWrite(E2, MOTOR_SPEED);
}


void moveDown()
{
    digitalWrite(M1, A_NEGATIVE);
    digitalWrite(M2, B_POSITIVE);

    analogWrite(E1, MOTOR_SPEED);
    analogWrite(E2, MOTOR_SPEED);
}


void moveForHalfSecond(void (*movementFunction)())
{
    movementFunction();

    delay(MOVE_TIME);

    stopMotors();
}


void setup()
{
    Serial.begin(115200);

    pinMode(M1, OUTPUT);
    pinMode(E1, OUTPUT);
    pinMode(M2, OUTPUT);
    pinMode(E2, OUTPUT);

    stopMotors();

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
            moveForHalfSecond(moveUp);
        }
        else if (command == "down")
        {
            Serial.println("Moving down for 0.5 seconds");
            moveForHalfSecond(moveDown);
        }
        else if (command == "left")
        {
            Serial.println("Moving left for 0.5 seconds");
            moveForHalfSecond(moveLeft);
        }
        else if (command == "right")
        {
            Serial.println("Moving right for 0.5 seconds");
            moveForHalfSecond(moveRight);
        }
        else if (command == "stop")
        {
            stopMotors();
            Serial.println("Motors stopped");
        }
        else
        {
            Serial.println("Unknown command");
        }
    }
}