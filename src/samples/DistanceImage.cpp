#include "DistanceImage.hpp"

#include <limits>
#include <Eigen/Core>

namespace base { namespace samples {

void DistanceImage::clear()
{
    std::fill( data.begin(), data.end(), std::numeric_limits<scalar>::quiet_NaN() );
}

Pointcloud DistanceImage::getPointCloud() const
{
    Pointcloud pointCloud;
    pointCloud.time = time;
    Eigen::Vector3d point;
    for(size_t y = 0; y < this->height ; ++y)
    {
        for(size_t x = 0; x < this->width ; ++x)
        {
            if (this->getScenePoint(x, y, point))
            {
                pointCloud.points.push_back(point);
            }
        }
    }
    return pointCloud;
}

void DistanceImage::setIntrinsic(double f_x, double f_y, double c_x, double c_y)
{
    scale_x = 1.0 / f_x;
    scale_y = 1.0 / f_y;
    center_x = -c_x / f_x;
    center_y = -c_y / f_y;
}

void DistanceImage::setSize(uint16_t width, uint16_t height)
{
    this->width = width;
    this->height = height;
    data.resize( (size_t)width * (size_t)height );
}

}} //end namespace base::samples
