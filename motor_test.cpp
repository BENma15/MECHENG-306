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


void moveForDuration(int motor1Direction, int motor2Direction)
{
    digitalWrite(M1, motor1Direction);
    digitalWrite(M2, motor2Direction);

    analogWrite(E1, MOTOR_SPEED);
    analogWrite(E2, MOTOR_SPEED);

    delay(MOVE_TIME);

    analogWrite(E1, 0);
    analogWrite(E2, 0);
}


void moveRight()
{
    moveForDuration(A_POSITIVE, B_POSITIVE);
}


void moveLeft()
{
    moveForDuration(A_NEGATIVE, B_NEGATIVE);
}


void moveUp()
{
    moveForDuration(A_POSITIVE, B_NEGATIVE);
}


void moveDown()
{
    moveForDuration(A_NEGATIVE, B_POSITIVE);
}


void setup()
{
    Serial.begin(115200);

    pinMode(M1, OUTPUT);
    pinMode(E1, OUTPUT);
    pinMode(M2, OUTPUT);
    pinMode(E2, OUTPUT);

    analogWrite(E1, 0);
    analogWrite(E2, 0);

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
            moveUp();
        }
        else if (command == "down")
        {
            Serial.println("Moving down for 0.5 seconds");
            moveDown();
        }
        else if (command == "left")
        {
            Serial.println("Moving left for 0.5 seconds");
            moveLeft();
        }
        else if (command == "right")
        {
            Serial.println("Moving right for 0.5 seconds");
            moveRight();
        }
        else
        {
            Serial.println("Unknown command");
        }
    }
}