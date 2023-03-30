#ifndef POSWITHGT_H
#define POSWITHGT_H

#include "GPSPos.h"

struct PosWithGT
{
    bool isTrackingOK;
    double x;
    double y;
    double z;
    GPSPos gps;
};

#endif // POSWITHGT_H
