#ifndef GRAPH_H
#define GRAPH_H

#define MAX_SAMPLES 500
// Global buffers (defined in Graph.cpp)
extern long data[MAX_SAMPLES];
extern int timeData[MAX_SAMPLES];

void addDataPoint(long dataPoint, unsigned int timePoint);
void exportData();
void clearData();

#endif