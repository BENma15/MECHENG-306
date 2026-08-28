#ifndef GRAPH_H
#define GRAPH_H

#define MAX_SAMPLES 500

// Global buffers (defined in Graph.cpp)

extern double dataA[MAX_SAMPLES];

extern double dataB[MAX_SAMPLES];

extern unsigned long timeData[MAX_SAMPLES];

void addDataPoint(double dataPointA, double dataPointB, unsigned long timePoint);

void exportData();

void clearData();

#endif