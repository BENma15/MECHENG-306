#include "HelperFunctions.h"

const double COUNTS_PER_MM = 2367.0 / 10.0;  // 236.7 counts/mm

long distanceToCounts(double distance_mm)
{
    return (long) round(distance_mm * COUNTS_PER_MM);
}

double countsToDistance(long counts)
{
    return counts / COUNTS_PER_MM;
}