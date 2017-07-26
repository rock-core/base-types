#include "RigidBodyAcceleration.hpp"

namespace base { namespace samples {

void RigidBodyAcceleration::invalidateOrientation()
{
    cov_acceleration = Eigen::Matrix3d::Identity();
    cov_acceleration *= INFINITY;
    cov_angular_acceleration = Eigen::Matrix3d::Identity();
    cov_angular_acceleration *= INFINITY;
}

}} //end namespace base::samples
