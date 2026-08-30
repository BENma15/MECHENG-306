#include "Graph.h"
#include <Arduino.h>

double dataA[MAX_SAMPLES] = {0};
double dataB[MAX_SAMPLES] = {0};

unsigned long timeData[MAX_SAMPLES] = {0};

int sampleIndex = 0;

// Store a data point for later export.
void addDataPoint(double dataPointA, double dataPointB, unsigned long timePoint)
{
    if (sampleIndex < MAX_SAMPLES)
    {
        dataA[sampleIndex] = dataPointA;
        dataB[sampleIndex] = dataPointB;
        timeData[sampleIndex] = timePoint;
        sampleIndex++;
    }
}

// Print captured data to the serial monitor.
void exportData()
{
    Serial.println("START_DATA");
    Serial.println("Time,DataA,DataB");

    for (int i = 0; i < sampleIndex; i++)
    {
        Serial.print(timeData[i]);
        Serial.print(",");

        Serial.print(dataA[i], 3);
        Serial.print(",");

        Serial.print(dataB[i], 3);
        Serial.println();
    }

    Serial.println("END_DATA");
}

// Clear stored motion samples.
void clearData()
{
    sampleIndex = 0;
}