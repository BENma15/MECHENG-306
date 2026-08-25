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

bool timer(unsigned long waitTime)
{
    if (!timerActive)
    {
        startTime = millis();
        timerActive = true;
        return false;
    }
    currentTime = millis();
    if ((currentTime - startTime) >= waitTime)
    {
        timerActive = false;
        return true;
    }
    return false;
}

// Wrapper so calls that pass an int still link
bool timer(int waitTime)
{
    return timer((unsigned long)waitTime);
}