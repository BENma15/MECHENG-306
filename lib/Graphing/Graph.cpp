#include "Graph.h"

const int MAX_SAMPLES = 500;

long data[MAX_SAMPLES];
unsigned int timeData[MAX_SAMPLES];

int sampleIndex = 0;

void addDataPoint(long dataPoint, unsigned int timePoint)
{
    if (sampleIndex < MAX_SAMPLES)
    {
        data[sampleIndex] = dataPoint;
        timeData[sampleIndex] = timePoint;
        sampleIndex++;
    }
}

void exportData()
{
    Serial.println("START_DATA");
    Serial.println("Time,Data");

    for (int i = 0; i < sampleIndex; i++)
    {
        Serial.print(timeData[i]);
        Serial.print(",");
        Serial.println(data[i]);
    }

    Serial.println("END_DATA");
}

void clearData()
{
    sampleIndex = 0;
}