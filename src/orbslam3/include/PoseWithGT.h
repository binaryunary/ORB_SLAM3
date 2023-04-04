
#include "GPSPos.h"

#ifndef POSEWITHGT_H
#define POSEWITHGT_H

struct PoseWithGT
{
    bool isTrackingOK;
    double timestamp;
    double x;
    double y;
    double z;
    double qx;
    double qy;
    double qz;
    double qw;
    GPSPos gps;
};

#endif // POSEWITHGT_H

