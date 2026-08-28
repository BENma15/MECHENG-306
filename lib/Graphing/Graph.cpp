#include "Graph.h"
#include <Arduino.h>

double dataA[MAX_SAMPLES] = {0};
double dataB[MAX_SAMPLES] = {0};

unsigned long timeData[MAX_SAMPLES] = {0};

int sampleIndex = 0;

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

void clearData()
{
    sampleIndex = 0;
}