#include "RigidBodyAcceleration.hpp"

void base::samples::RigidBodyAcceleration::invalidateOrientation()
{
    cov_acceleration = Eigen::Matrix3d::Identity();
    cov_acceleration *= INFINITY;
}
