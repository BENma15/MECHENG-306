#include "HelperFunctions.h"

const double COUNTS_PER_MM = 167.37; //  167.37 counts/mm
unsigned long startTime = 0;
unsigned long currentTime = 0;
bool timerActive = false;

long distanceToCounts(double distance_mm)
{
    return (long)round(distance_mm * COUNTS_PER_MM);
}

double countsToDistance(long counts)
{
    return counts / COUNTS_PER_MM;
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
