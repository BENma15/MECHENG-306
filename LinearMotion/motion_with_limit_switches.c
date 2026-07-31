#include <Arduino.h>
#include <avr/interrupt.h>
#include "LimitSwitch.h"

// Motor A encoder pins
const int L_ENCA = 18;
const int L_ENCB = 19;

// Motor B encoder pins
const int R_ENCA = 20;
const int R_ENCB = 21;

// Motor pins
const int E1 = 5;
const int M1 = 4;
const int E2 = 6;
const int M2 = 7;

volatile long countA = 0;
volatile long countB = 0;
volatile uint8_t stateA = 0;
volatile uint8_t stateB = 0;

const double COUNTS_PER_REV = 8256.0;
const double WHEEL_RADIUS_MM = 4.0;
const double LIMIT_BACKOFF_MM = 10.0;

int moveSpeed = 150;

const int8_t encTable[16] = {
    0, -1, +1,  0,
    +1,  0,  0, -1,
    -1,  0,  0, +1,
    0, +1, -1,  0
};

void updateA()
{
    uint8_t a = digitalRead(L_ENCA);
    uint8_t b = digitalRead(L_ENCB);

    uint8_t newState = (a << 1) | b;
    uint8_t index = (stateA << 2) | newState;

    countA += encTable[index];
    stateA = newState;
}

void updateB()
{
    uint8_t a = digitalRead(R_ENCA);
    uint8_t b = digitalRead(R_ENCB);

    uint8_t newState = (a << 1) | b;
    uint8_t index = (stateB << 2) | newState;

    countB += encTable[index];
    stateB = newState;
}

void setLeftMotor(int8_t dir, int pwm)
{
    if (dir == 0)
    {
        analogWrite(E1, 0);
        return;
    }

    digitalWrite(M1, dir > 0 ? HIGH : LOW);
    analogWrite(E1, pwm);
}

void setRightMotor(int8_t dir, int pwm)
{
    if (dir == 0)
    {
        analogWrite(E2, 0);
        return;
    }

    digitalWrite(M2, dir > 0 ? HIGH : LOW);
    analogWrite(E2, pwm);
}

long distanceToCounts(double distance_mm)
{
    return (long)round(
        COUNTS_PER_REV * distance_mm /
        (2.0 * PI * WHEEL_RADIUS_MM)
    );
}

void runAxis(double deltaA_mm, double deltaB_mm)
{
    long targetA = distanceToCounts(deltaA_mm);
    long targetB = distanceToCounts(deltaB_mm);

    long startA = countA;
    long startB = countB;

    long absTargetA = labs(targetA);
    long absTargetB = labs(targetB);

    int8_t dirA =
        (targetA > 0) ? 1 :
        (targetA < 0) ? -1 : 0;

    int8_t dirB =
        (targetB > 0) ? 1 :
        (targetB < 0) ? -1 : 0;

    bool doneA = (absTargetA == 0);
    bool doneB = (absTargetB == 0);

    long largerTarget =
        (absTargetA > absTargetB) ? absTargetA : absTargetB;

    int pwmA = moveSpeed;
    int pwmB = moveSpeed;

    if (largerTarget > 0)
    {
        pwmA = (int)round(
            moveSpeed * ((double)absTargetA / largerTarget)
        );

        pwmB = (int)round(
            moveSpeed * ((double)absTargetB / largerTarget)
        );
    }

    if (!doneA)
    {
        setLeftMotor(dirA, pwmA);
    }

    if (!doneB)
    {
        setRightMotor(dirB, pwmB);
    }

    while (!doneA || !doneB)
    {
        if (!doneA &&
            labs(countA - startA) >= absTargetA)
        {
            setLeftMotor(0, 0);
            doneA = true;
        }

        if (!doneB &&
            labs(countB - startB) >= absTargetB)
        {
            setRightMotor(0, 0);
            doneB = true;
        }
    }
}

void moveTo(double dx, double dy)
{
    runAxis(dx + dy, dx - dy);
}

void setup()
{
    Serial.begin(115200);

    pinMode(L_ENCA, INPUT_PULLUP);
    pinMode(L_ENCB, INPUT_PULLUP);
    pinMode(R_ENCA, INPUT_PULLUP);
    pinMode(R_ENCB, INPUT_PULLUP);

    pinMode(M1, OUTPUT);
    pinMode(M2, OUTPUT);
    pinMode(E1, OUTPUT);
    pinMode(E2, OUTPUT);

    // Limit-switch pins and PCINT are configured by LimitSwitch.cpp
    setupLimitSwitches(E1, E2);

    cli();

    EIMSK |=
        (1 << INT0) |
        (1 << INT1) |
        (1 << INT2) |
        (1 << INT3);

    EICRA |=
        (1 << ISC00) |
        (1 << ISC10) |
        (1 << ISC20) |
        (1 << ISC30);

    sei();
}

void loop()
{
    uint8_t event = getLimitEvents();

    if (event & LIMIT_LEFT)
    {
        Serial.println("LEFT limit switch hit");

        // Allow the motors to move again during recovery
        clearMotorLimitStops();

        moveTo(+LIMIT_BACKOFF_MM, 0.0);
    }
    else if (event & LIMIT_RIGHT)
    {
        Serial.println("RIGHT limit switch hit");

        clearMotorLimitStops();

        moveTo(-LIMIT_BACKOFF_MM, 0.0);
    }
    else if (event & LIMIT_BOTTOM)
    {
        Serial.println("BOTTOM limit switch hit");

        clearMotorLimitStops();

        moveTo(0.0, +LIMIT_BACKOFF_MM);
    }
    else if (event & LIMIT_TOP)
    {
        Serial.println("TOP limit switch hit");

        clearMotorLimitStops();

        moveTo(0.0, -LIMIT_BACKOFF_MM);
    }

    // Your temporary test movement
    digitalWrite(M1, HIGH);
    digitalWrite(M2, HIGH);

    if (!isMotor1LimitStopped())
    {
        analogWrite(E1, 128);
    }

    if (!isMotor2LimitStopped())
    {
        analogWrite(E2, 128);
    }
}

ISR(INT0_vect) { updateB(); }
ISR(INT1_vect) { updateB(); }
ISR(INT2_vect) { updateA(); }
ISR(INT3_vect) { updateA(); }
