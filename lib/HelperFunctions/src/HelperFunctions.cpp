#include "HelperFunctions.h"

const double COUNTS_PER_MM = 167.37;  //  167.37 counts/mm

long distanceToCounts(double distance_mm)
{
    return (long) round(distance_mm * COUNTS_PER_MM);
}

double countsToDistance(long counts)
{
    return counts / COUNTS_PER_MM;
}