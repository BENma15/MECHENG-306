#include <Arduino.h>
#include <avr/interrupt.h>

#include <Motion.h>
#include <HelperFunctions.h>
#include <Encoder.h>
#include <Graph.h>

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

// long distanceToCounts(double distance_mm)
// {
//     return (long)round(COUNTS_PER_REV * distance_mm / (2.0 * PI * WHEEL_RADIUS_MM));
// }

void moveToPID(double dx, double dy, int speed)
{
    double kp = 0.1; 
    double ki = 0.01; 
    double kd = 0.005; 

    double a_integral_error = 0;
    double b_integral_error = 0;

    double a_derivative_error = 0;
    double b_derivative_error = 0;

    long dx_enc = distanceToCounts(dx);
    long dy_enc = distanceToCounts(dy);

    double theta = atan2(dy, dx);

    double x_speed_mm_s = speed * cos(theta);
    double y_speed_mm_s = speed * sin(theta);

    cli();
    long a_start = Encoder_getLeftEncoderCount();
    long b_start = Encoder_getRightEncoderCount();
    sei();

    // Motor A variables
    long a_move_target = a_start + dx_enc + dy_enc;
    long a_move_current = a_start;
    long a_move_prev = a_start;

    double a_speed_mm_s = x_speed_mm_s + y_speed_mm_s;
    long a_speed_target = abs(distanceToCounts(a_speed_mm_s));
    double a_speed_current = 0.0;
    double a_speed_error = 0.0;
    double a_speed_prev = 0.0;
    int a_pwm = 0;

    // Motor B variables
    long b_move_target = b_start + dx_enc - dy_enc;
    long b_move_current = b_start;
    long b_move_prev = b_start;

    double b_speed_mm_s = x_speed_mm_s - y_speed_mm_s;
    long b_speed_target = abs(distanceToCounts(b_speed_mm_s));
    double b_speed_current = 0.0;
    double b_speed_error = 0.0;
    double b_speed_prev = 0.0;
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
        a_move_current = Encoder_getLeftEncoderCount();
        b_move_current = Encoder_getRightEncoderCount();
        sei();

        unsigned long current_time = millis();
        unsigned long elapsed_ms = current_time - prev_time;

        if (elapsed_ms >= period)
        {
            double elapsed_seconds = elapsed_ms / 1000.0; // seconds
            prev_time = current_time;
              
            a_speed_prev = a_speed_current;
            b_speed_prev = b_speed_current;

            a_speed_current = fabs((a_move_current - a_move_prev) / elapsed_seconds);
            b_speed_current = fabs((b_move_current - b_move_prev) / elapsed_seconds);

            a_move_prev = a_move_current;
            b_move_prev = b_move_current;


            a_speed_error = a_speed_target - a_speed_current;
            b_speed_error = b_speed_target - b_speed_current;

            a_integral_error = a_integral_error + a_speed_error * elapsed_seconds;
            b_integral_error = b_integral_error + b_speed_error * elapsed_seconds;

            a_derivative_error = (a_speed_current - a_speed_prev) / elapsed_seconds;
            b_derivative_error = (b_speed_current - b_speed_prev) / elapsed_seconds;

            a_pwm = a_speed_error * kp + a_integral_error * ki - a_derivative_error * kd;
            b_pwm = b_speed_error * kp + b_integral_error * ki - b_derivative_error * kd;

            a_pwm = constrain((int)(a_pwm), 0, 255);
            b_pwm = constrain((int)(b_pwm), 0, 255);

            addDataPoint(current_time, a_speed_current);
        }

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
    exportData();
    clearData();
}

void PID_Init() {

    pinMode(L_LIMIT, INPUT_PULLUP);
    pinMode(R_LIMIT, INPUT_PULLUP);
    pinMode(U_LIMIT, INPUT_PULLUP);
    pinMode(D_LIMIT, INPUT_PULLUP);

    pinMode(M1, OUTPUT);
    pinMode(M2, OUTPUT);
    pinMode(E1, OUTPUT);
    pinMode(E2, OUTPUT);
}

void testlLoop()
{
    delay(1000);
    moveToPID(30, 0, 25);
    delay(1000);
    moveToPID(-30, 0, 25);
}