#include "DistanceImage.hpp"

#include <limits>
#include <Eigen/Core>

void base::samples::DistanceImage::clear()
{
    std::fill( data.begin(), data.end(), std::numeric_limits<float>::quiet_NaN() );
}

base::samples::Pointcloud base::samples::DistanceImage::getPointCloud() const
{
    base::samples::Pointcloud pointCloud;
    Eigen::Vector3d point;
    for(size_t y = 0; y < height ; y++)
    {
        for(size_t x = 0; x < width ; x++)
        {
            pointCloud.points.push_back(point);
        }
    }
    return pointCloud;
}

void base::samples::DistanceImage::setIntrinsic(double f_x, double f_y, double c_x, double c_y)
{
    scale_x = 1.0 / f_x;
    scale_y = 1.0 / f_y;
    center_x = -c_x / f_x;
    center_y = -c_y / f_y;
}

void base::samples::DistanceImage::setSize(double width, double height)
{
    this->width = width;
    this->height = height;
    data.resize( width * height );
}
