#include "dfkiToolkitUser.hpp"


void dfki::to_intermediate(DFKI::Quaternion& intermediate, Eigen::Quaterniond const& real_type)
{
    intermediate.x() = real_type.x();
    intermediate.y() = real_type.y();
    intermediate.z() = real_type.z();
    intermediate.w() = real_type.w();
}
void dfki::from_intermediate(Eigen::Quaterniond& real_type, DFKI::Quaternion& intermediate)
{
    real_type.x() = intermediate.x();
    real_type.y() = intermediate.y();
    real_type.z() = intermediate.z();
    real_type.w() = intermediate.w();
}

void dfki::to_intermediate(DFKI::Vector3& intermediate, Eigen::Vector3d const& real_type)
{
    intermediate.x() = real_type.x();
    intermediate.y() = real_type.y();
    intermediate.z() = real_type.z();
}

void dfki::from_intermediate(Eigen::Vector3d& real_type, DFKI::Vector3& intermediate)
{
    real_type.x() = intermediate.x();
    real_type.y() = intermediate.y();
    real_type.z() = intermediate.z();
}



