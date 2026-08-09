#ifndef GRAPH_H
#define GRAPH_H

const int MAX_SAMPLES = 500;

extern long data[MAX_SAMPLES];
extern unsigned int timeData[MAX_SAMPLES];
extern int sampleIndex;

void addDataPoint(long dataPoint, unsigned int timePoint);
void exportData();
void clearData();

#endif