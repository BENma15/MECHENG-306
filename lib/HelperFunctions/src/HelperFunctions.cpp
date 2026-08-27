#include "HelperFunctions.h"

const double COUNTS_PER_MM = 185; //  167.37 counts/mm

long distanceToCounts(double distance_mm)
{
    return (long)round(distance_mm * COUNTS_PER_MM);
}

double countsToDistance(long counts)
{
    return counts / COUNTS_PER_MM;
}

int8_t outputToDirection(double output)
{
    if (output > 0)
    {
        return 1;
    }
    if (output < 0)
    {
        return -1;
    }
    return 0;
}

int applyMotorPwmLimits(int pwm, int8_t direction, int minimumDrivePwm)
{
    if (direction != 0)
    {
        pwm += minimumDrivePwm;
    }
    return min(pwm, 255);
}

Timer::Timer()
{
    startTime = 0;
    timerActive = false;
}

bool Timer::startTimer(unsigned long waitTime)
{
    if (timerActive == false)
    {
        startTime = millis();
        timerActive = true;

        return false;
    }

    unsigned long currentTime = millis();

    if ((currentTime - startTime) >= waitTime)
    {
        timerActive = false;

        return true;
    }

    return false;
}

void Timer::reset()
{
    startTime = 0;
    timerActive = false;
}
