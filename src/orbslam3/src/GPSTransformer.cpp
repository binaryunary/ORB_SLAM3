#include "GPSTransformer.h"
#include <proj_api.h>
#include <Eigen/Dense>

using namespace ORB_SLAM3;

GPSTransformer::GPSTransformer(Settings *settings)
    : R(settings->gpsTransformRotation()), t(settings->gpsTransformTranslation()), c(settings->gpsTransformScale())
{
    projPJ pjMercator = pj_init_plus("+proj=merc +ellps=WGS84 +datum=WGS84 +units=m");
    projPJ pjWGS84 = pj_init_plus("+proj=longlat +ellps=WGS84 +datum=WGS84 +no_defs");
}

GPSTransformer::~GPSTransformer()
{
    pj_free(pjMercator);
    pj_free(pjWGS84);
}

void GPSTransformer::wgs84ToMercator(double &lat, double &lon)
{
    pj_transform(pjWGS84, pjMercator, 1, 1, &lat, &lon, nullptr);
}

void GPSTransformer::mercatorToWGS84(double &x, double &y)
{
    double radX = x;
    double radY = y;
    pj_transform(pjMercator, pjWGS84, 1, 1, &radX, &radY, nullptr);

    x = radX * RAD_TO_DEG;
    y = radY * RAD_TO_DEG;
}

Eigen::Vector3f GPSTransformer::slamToMercator(Eigen::Vector3f point)
{
    return t + c * R * point;
}
