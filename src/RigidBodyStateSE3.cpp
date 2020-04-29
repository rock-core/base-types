#include "RigidBodyStateSE3.hpp"

namespace base {

void RigidBodyStateSE3::setNaN()
{
    pose.position.setConstant(std::numeric_limits<double>::quiet_NaN());
    pose.orientation.coeffs().setConstant(std::numeric_limits<double>::quiet_NaN());
    twist.setNaN();
    acceleration.setNaN();
    wrench.force.setConstant(std::numeric_limits<double>::quiet_NaN());
    wrench.torque.setConstant(std::numeric_limits<double>::quiet_NaN());
}

bool RigidBodyStateSE3::hasValidPose() const
{
    return base::isnotnan(pose.position) && base::isnotnan(pose.orientation.coeffs()) &&
           (abs(pose.orientation.squaredNorm()-1.0) < 1e-6);
}

bool RigidBodyStateSE3::hasValidTwist() const
{
    return twist.isValid();
}

bool RigidBodyStateSE3::hasValidAcceleration() const
{
    return acceleration.isValid();
}

bool RigidBodyStateSE3::hasValidWrench() const
{
    return base::isnotnan(wrench.force) && base::isnotnan(wrench.torque);
}

Twist operator*(const Pose& transform, const Twist& twist_in)
{
    Twist twist_out;
    twist_out.angular=transform.orientation*twist_in.angular;
    twist_out.linear=transform.orientation*twist_in.linear - transform.position.cross(twist_out.angular);
    return twist_out;
}


Acceleration operator*(const Pose& transform, const Acceleration& acc_in)
{
    Acceleration acc_out;
    acc_out.angular = transform.orientation*acc_in.angular;
    acc_out.linear  = transform.orientation*acc_in.linear - transform.position.cross(acc_out.angular);
    return acc_out;
}

Wrench operator*(const Pose& transform, const Wrench& wrench_in)
{
    Wrench wrench_out;
    wrench_out.force  = transform.orientation*wrench_in.force;
    wrench_out.torque = transform.orientation*wrench_in.torque - transform.position.cross(wrench_out.force);
    return wrench_out;
}
}
