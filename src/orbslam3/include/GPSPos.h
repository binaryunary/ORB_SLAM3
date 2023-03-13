#ifndef GPSPOS_H
#define GPSPOS_H

#include <boost/serialization/serialization.hpp>

struct GPSPos
{
    double lat;
    double lon;
    double alt;

    template <class Archive> void serialize(Archive &ar, const unsigned int version)
    {
        ar &lat;
        ar &lon;
        ar &alt;
    };
};

#endif
