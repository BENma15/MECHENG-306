#include <Arduino.h>
#include <avr/interrupt.h>
#include "limitSwitch.h"

#include "motion.h"

// Motor A encoder pins
const int L_ENCA = 18; // INT3
const int L_ENCB = 19; // INT2
// Motor B encoder pins
const int R_ENCA = 20; // INT1
const int R_ENCB = 21; // INT0

// Limit switch pins
const int L_LIMIT = 13;
const int R_LIMIT = 12;
const int U_LIMIT = 10;
const int D_LIMIT = 11;

// Motors pins
const int E1 = 5;
const int M1 = 4;
const int E2 = 6;
const int M2 = 7;

// Encoder variables to keep track of encoder count and encoder reading
volatile long countA = 0;
volatile long countB = 0;
volatile uint8_t stateA = 0;
volatile uint8_t stateB = 0;

// Distance to encoder count equation variables
const double COUNTS_PER_REV = 8256.0;
const double WHEEL_RADIUS_MM = 8.0;

// Current movement speed (TO BE CHANGED)
int moveSpeed = 150;

// Predetermined tables to see which way the motor is spinning
const int8_t encTable[16] = {
    0, -1, +1, 0,
    +1, 0, 0, -1,
    -1, 0, 0, +1,
    0, +1, -1, 0};

// Left encoder reading function
void updateLeft()
{
    // Read both values from the two left encoder channels
    uint8_t a = digitalRead(L_ENCA);
    uint8_t b = digitalRead(L_ENCB);

    // Left shift a encoder value and insert b into LSB to store both
    uint8_t newState = (a << 1) | b;
    // Left shift the current state by 2 bits and insert new state in the 2 LSB's
    uint8_t index = (stateA << 2) | newState;
    // Looking at table to figure out which way the motor is spinning
    countA += encTable[index];
    // Set current state to previous state
    stateA = newState;
}

// Right encoder reading function
void updateRight()
{
    uint8_t a = digitalRead(R_ENCA);
    uint8_t b = digitalRead(R_ENCB);

    uint8_t newState = (a << 1) | b;
    uint8_t index = (stateB << 2) | newState;
    countB += encTable[index];
    stateB = newState;
}

// Left motor movement
void setLeftMotor(int8_t dir, int pwm)
{
    //
    if (dir == 0)
    {
        analogWrite(E1, 0);
        return;
    }
    digitalWrite(M1, dir > 0 ? HIGH : LOW);
    analogWrite(E1, pwm);
}

// Right motor movement
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
    return (long)round(COUNTS_PER_REV * distance_mm / (2.0 * PI * WHEEL_RADIUS_MM));
}

void moveToPID(double dx, double dy, int speed)
{
    long dx_enc = distanceToCounts(dx);
    long dy_enc = distanceToCounts(dy);

    double theta = atan2(dy, dx);

    double x_speed_mm_s = speed * cos(theta);
    double y_speed_mm_s = speed * sin(theta);

    cli();
    long a_start = countA;
    long b_start = countB;
    sei();

    // Motor A variables
    long a_move_target = a_start + dx_enc + dy_enc;
    long a_move_current = a_start;
    long a_move_prev = a_start;

    double a_speed_mm_s = x_speed_mm_s + y_speed_mm_s;
    long a_speed_target = abs(distanceToCounts(a_speed_mm_s));
    double a_speed_current = 0.0;
    double a_speed_error = 0.0;
    int a_pwm = 0;

    // Motor B variables
    long b_move_target = b_start + dx_enc - dy_enc;
    long b_move_current = b_start;
    long b_move_prev = b_start;

    double b_speed_mm_s = x_speed_mm_s - y_speed_mm_s;
    long b_speed_target = abs(distanceToCounts(b_speed_mm_s));
    double b_speed_current = 0.0;
    double b_speed_error = 0.0;
    int b_pwm = 0;

    long tolerance = 10;
    const unsigned long period = 10; // milliseconds

    int8_t a_direction = 0;
    int8_t b_direction = 0;

    unsigned long prev_time = millis();

    // introduce velocity gradient here to avoid sudden acceleration

    while (abs(a_move_target - a_move_current) > tolerance ||
           abs(b_move_target - b_move_current) > tolerance)
    {
        cli();
        a_move_current = countA;
        b_move_current = countB;
        sei();

        unsigned long current_time = millis();
        unsigned long elapsed_ms = current_time - prev_time;
        
        // velocity calculator
        if (elapsed_ms >= period)
        {

            double time_elapsed = elapsed_ms / 1000.0; // seconds
            prev_time = current_time;

            a_speed_current = fabs((a_move_current - a_move_prev) / time_elapsed);
            b_speed_current = fabs((b_move_current - b_move_prev) / time_elapsed);
            a_move_prev = a_move_current;
            b_move_prev = b_move_current;

            a_speed_error = a_speed_target - a_speed_current;
            b_speed_error = b_speed_target - b_speed_current;
        }

        a_pwm = constrain((int)(a_speed_error * 0.02), 0, 255);
        b_pwm = constrain((int)(b_speed_error * 0.02), 0, 255);

        bool a_finished = false;

        if (a_move_target > a_move_current)
        {
            a_direction = 1;
        }
        else if (a_move_target < a_move_current)
        {
            a_direction = -1;
        }

        if (b_move_target > b_move_current)
        {
            b_direction = 1;
        }
        else if (b_move_target < b_move_current)
        {
            b_direction = -1;
        }

        if (abs(a_move_target - a_move_current) <= tolerance)
        {
            a_finished = true;
        }
        else
        {
            a_finished = false;
        }

        bool b_finished = false;
        if (abs(b_move_target - b_move_current) <= tolerance)
        {
            b_finished = true;
        }
        else
        {
            b_finished = false;
        }

        if (a_finished)
        {
            setLeftMotor(0, 0);
        }
        else
        {
            setLeftMotor(a_direction, a_pwm);
        }

        if (b_finished)
        {
            setRightMotor(0, 0);
        }
        else
        {
            setRightMotor(b_direction, b_pwm);
        }
    }

    // introduce velocity gradient here to avoid sudden deceleration

    setLeftMotor(0, 0);
    setRightMotor(0, 0);
}

void setup()
{
    Serial.begin(115200);
    pinMode(L_ENCA, INPUT_PULLUP);
    pinMode(L_ENCB, INPUT_PULLUP);
    pinMode(R_ENCA, INPUT_PULLUP);
    pinMode(R_ENCB, INPUT_PULLUP);

    pinMode(L_LIMIT, INPUT_PULLUP);
    pinMode(R_LIMIT, INPUT_PULLUP);
    pinMode(U_LIMIT, INPUT_PULLUP);
    pinMode(D_LIMIT, INPUT_PULLUP);

    pinMode(M1, OUTPUT);
    pinMode(M2, OUTPUT);
    pinMode(E1, OUTPUT);
    pinMode(E2, OUTPUT);

    setupLimitSwitches(E1, E2);

    cli();
    EIMSK |= (1 << INT0) | (1 << INT1) | (1 << INT2) | (1 << INT3);
    EICRA |= (1 << ISC00) | (1 << ISC10) | (1 << ISC20) | (1 << ISC30);
    sei();
}

void loop()
{
    moveToPID(10, 20, 20);
}

ISR(INT0_vect) { updateRight(); }
ISR(INT1_vect) { updateRight(); }
ISR(INT2_vect) { updateLeft(); }
ISR(INT3_vect) { updateLeft(); }