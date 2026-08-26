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

bool encoderCountsChanged()
{
    static Timer encoderTimer;
    static long previousLeft = 0;
    static long previousRight = 0;
    static bool initialized = false;

    if (!initialized)
    {
        previousLeft = Encoder_getLeftEncoderCount();
        previousRight = Encoder_getRightEncoderCount();
        encoderTimer.startTimer(10);
        initialized = true;
        return true;
    }

    // Don't compare until 10 ms has passed
    if (!encoderTimer.startTimer(10))
    {
        return true;
    }

    long currentLeft = Encoder_getLeftEncoderCount();
    long currentRight = Encoder_getRightEncoderCount();

    bool changed = (currentLeft != previousLeft) || (currentRight != previousRight);

    previousLeft = currentLeft;
    previousRight = currentRight;

    return changed;
}
