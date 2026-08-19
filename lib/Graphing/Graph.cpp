#include "Graph.h"
#include <Arduino.h>

long dataA[MAX_SAMPLES] = {0};
long dataB[MAX_SAMPLES] = {0};

int timeData[MAX_SAMPLES] = {0};

int sampleIndex = 0;

void addDataPoint(long dataPointA, long dataPointB, unsigned int timePoint)
{
    if (sampleIndex < MAX_SAMPLES)
    {
        dataA[sampleIndex] = dataPointA;
        dataB[sampleIndex] = dataPointB;
        timeData[sampleIndex] = timePoint;
        sampleIndex++;
    }
}

void exportData()
{
    Serial.println("START_DATA");
    Serial.println("Time,DataA,DataB");

    for (int i = 0; i < sampleIndex; i++)
    {
        Serial.print(timeData[i]);
        Serial.print(",");
        Serial.print(dataA[i]);
        Serial.print(",");
        Serial.print(dataB[i]);
        Serial.println();
    }

    Serial.println("END_DATA");
}

void clearData()
{
    sampleIndex = 0;
}