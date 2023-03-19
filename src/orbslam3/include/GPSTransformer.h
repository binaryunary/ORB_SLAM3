#ifndef GPS_TRANSFORMER_H
#define GPS_TRANSFORMER_H

#include "Settings.h"
#include <Eigen/Dense>
#include <proj_api.h>

class GPSTransformer
{

  public:
    GPSTransformer(ORB_SLAM3::Settings *settings);
    ~GPSTransformer();

    void wgs84ToMercator(double &lat, double &lon);
    void mercatorToWGS84(double &x, double &y);

    // TODO: Move to a separate class
    /**
     * Transforms SLAM coordinagtes to another coordinate system using transformation parameters from the settings file.
    */
    Eigen::Vector3f transformSLAM(Eigen::Vector3f point);

  private:
    projPJ pjWGS84;
    projPJ pjMercator;
    Eigen::Matrix3f R;
    Eigen::Vector3f t;
    float c;
};

#endif // GPS_TRANSFORMER_H
