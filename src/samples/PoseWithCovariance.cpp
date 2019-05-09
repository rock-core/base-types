#include "PoseWithCovariance.hpp"

namespace base { namespace samples {

PoseWithCovariance::PoseWithCovariance()
{

}

PoseWithCovariance::PoseWithCovariance(const TransformWithCovariance& transform) :
        transform(transform.translation, transform.orientation, transform.cov)
{

}

PoseWithCovariance::PoseWithCovariance(const RigidBodyState& rbs) : time(rbs.time),
    frame_id(rbs.targetFrame), object_frame_id(rbs.sourceFrame), transform(rbs.position, rbs.orientation)
{
    transform.cov << rbs.cov_position, Eigen::Matrix3d::Zero(),
                        Eigen::Matrix3d::Zero(), rbs.cov_orientation;
}

PoseWithCovariance PoseWithCovariance::operator*(const PoseWithCovariance& pose) const
{
    // compose new pose
    PoseWithCovariance new_pose;
    new_pose.time.microseconds = std::max(this->time.microseconds, pose.time.microseconds);
    new_pose.object_frame_id = pose.object_frame_id;
    new_pose.frame_id = this->frame_id;
    new_pose.transform = this->transform * pose.transform;
    return new_pose;
}

void PoseWithCovariance::setTransform(const TransformWithCovariance& transform)
{
    this->transform = transform;
}

void PoseWithCovariance::setTransform(const Eigen::Affine3d& transform)
{
    this->transform.setTransform(transform);
}

const TransformWithCovariance& PoseWithCovariance::getTransformWithCovariance() const
{
    return transform;
}

Eigen::Affine3d PoseWithCovariance::getTransform() const
{
    return transform.getTransform();
}

const TransformWithCovariance::Covariance& PoseWithCovariance::getCovariance() const
{
    return transform.getCovariance();
}

RigidBodyState PoseWithCovariance::toRigidBodyState() const
{
    base::samples::RigidBodyState rbs;
    rbs.time = time;
    rbs.targetFrame = frame_id;
    rbs.sourceFrame = object_frame_id;
    rbs.position = transform.translation;
    rbs.orientation = transform.orientation;
    if(transform.hasValidCovariance())
    {
        rbs.cov_position = transform.getTranslationCov();
        rbs.cov_orientation = transform.getOrientationCov();
    }
    return rbs;
}

}}