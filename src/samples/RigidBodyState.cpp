#include "RigidBodyState.hpp"

base::samples::RigidBodyState::RigidBodyState(bool doInvalidation)
{
    if(doInvalidation)
        invalidate();
}

void base::samples::RigidBodyState::setTransform(const Eigen::Affine3d& transform)
{
    position = transform.translation();
    orientation = Eigen::Quaterniond( transform.linear() );
}

Eigen::Affine3d base::samples::RigidBodyState::getTransform() const
{
    Eigen::Affine3d ret;
    ret.setIdentity();
    ret.rotate(this->orientation);
    ret.translation() = this->position;
    return ret;
}

void base::samples::RigidBodyState::setPose(const base::Pose& pose)
{
    orientation = pose.orientation;
    position = pose.position;
}

base::Pose base::samples::RigidBodyState::getPose() const
{
    return base::Pose( position, orientation );
}

double base::samples::RigidBodyState::getYaw() const
{
    return base::getYaw(orientation);
}

double base::samples::RigidBodyState::getPitch() const
{
    return base::getPitch(orientation);
}

double base::samples::RigidBodyState::getRoll() const
{
    return base::getRoll(orientation);
}

base::samples::RigidBodyState base::samples::RigidBodyState::unknown()
{
    RigidBodyState result(false);
    result.initUnknown();
    return result;
}

base::samples::RigidBodyState base::samples::RigidBodyState::invalid()
{
    RigidBodyState result(true);
    return result;
}

void base::samples::RigidBodyState::initSane()
{
    invalidate();
}

void base::samples::RigidBodyState::invalidate()
{
    invalidateOrientation();
    invalidateOrientationCovariance();
    invalidatePosition();
    invalidatePositionCovariance();
    invalidateVelocity();
    invalidateVelocityCovariance();
    invalidateAngularVelocity();
    invalidateAngularVelocityCovariance();
}

void base::samples::RigidBodyState::initUnknown()
{
    position.setZero();
    velocity.setZero();
    orientation = Eigen::Quaterniond::Identity();
    angular_velocity.setZero();
    cov_position = setValueUnknown();
    cov_orientation = setValueUnknown();
    cov_velocity = setValueUnknown();
    cov_angular_velocity = setValueUnknown();
}

bool base::samples::RigidBodyState::isValidValue(const base::Vector3d& vec)
{
    return !base::isNaN(vec(0)) &&
        !base::isNaN(vec(1)) &&
        !base::isNaN(vec(2));
}

bool base::samples::RigidBodyState::isValidValue(const base::Orientation& ori)
{
    return !base::isNaN(ori.w()) &&
        !base::isNaN(ori.x()) &&
        !base::isNaN(ori.y()) &&
        !base::isNaN(ori.z()) &&
        fabs(ori.squaredNorm()-1.0) < 1e-6;     //assuming at least single precision 
}

bool base::samples::RigidBodyState::isKnownValue(const base::Matrix3d& cov)
{
    return !base::isInfinity(cov(0,0)) &&
        !base::isInfinity(cov(1,1)) &&
        !base::isInfinity(cov(2,2));
}

bool base::samples::RigidBodyState::isValidCovariance(const base::Matrix3d& cov)
{
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            if (base::isNaN(cov(i, j)))
                return false;
    return true;
}

bool base::samples::RigidBodyState::isValidValue(const base::Vector3d& vec, int dim)
{
    return !base::isNaN(vec(dim));
}

bool base::samples::RigidBodyState::isValidCovariance(const base::Matrix3d& cov, int dim)
{
    return !base::isNaN(cov(dim,dim));
}

bool base::samples::RigidBodyState::isKnownValue(const base::Matrix3d& cov, int dim)
{
    return !base::isInfinity(cov(dim,dim));
}

base::Vector3d base::samples::RigidBodyState::invalidValue()
{
    return base::Vector3d::Ones() * base::NaN<double>();
}

base::Orientation base::samples::RigidBodyState::invalidOrientation()
{
    return base::Orientation(Eigen::Vector4d::Ones() * base::NaN<double>());
}

base::Matrix3d base::samples::RigidBodyState::setValueUnknown()
{
    return base::Matrix3d::Ones() * base::infinity<double>();
}

base::Matrix3d base::samples::RigidBodyState::invalidCovariance()
{
    return base::Matrix3d::Ones() * base::NaN<double>();
}

bool base::samples::RigidBodyState::hasValidPosition() const
{
    return isValidValue(position);
}

bool base::samples::RigidBodyState::hasValidPosition(int idx) const
{
    return isValidValue(position, idx);
}

bool base::samples::RigidBodyState::hasValidPositionCovariance() const
{
    return isValidCovariance(cov_position);
}

void base::samples::RigidBodyState::invalidatePosition()
{
    position = invalidValue();
}

void base::samples::RigidBodyState::invalidatePositionCovariance()
{
    cov_position = invalidCovariance();
}

bool base::samples::RigidBodyState::hasValidOrientation() const
{
    return isValidValue(orientation);
}

bool base::samples::RigidBodyState::hasValidOrientationCovariance() const
{
    return isValidCovariance(cov_orientation);
}

void base::samples::RigidBodyState::invalidateOrientation()
{
    orientation = invalidOrientation(); 
}

void base::samples::RigidBodyState::invalidateOrientationCovariance()
{
     cov_orientation = invalidCovariance();
}

bool base::samples::RigidBodyState::hasValidVelocity() const
{
    return isValidValue(velocity);
}

bool base::samples::RigidBodyState::hasValidVelocity(int idx) const
{
    return isValidValue(velocity, idx);
}

bool base::samples::RigidBodyState::hasValidVelocityCovariance() const
{
    return isValidCovariance(cov_velocity);
}

void base::samples::RigidBodyState::invalidateVelocity()
{
    velocity = invalidValue();
}

void base::samples::RigidBodyState::invalidateVelocityCovariance()
{
    cov_velocity = invalidCovariance();
}

bool base::samples::RigidBodyState::hasValidAngularVelocity() const
{
    return isValidValue(angular_velocity);
}

bool base::samples::RigidBodyState::hasValidAngularVelocity(int idx) const
{
     return isValidValue(angular_velocity, idx);
}

bool base::samples::RigidBodyState::hasValidAngularVelocityCovariance() const
{
    return isValidCovariance(cov_angular_velocity);
}

void base::samples::RigidBodyState::invalidateAngularVelocity()
{
    angular_velocity = invalidValue();
}

void base::samples::RigidBodyState::invalidateAngularVelocityCovariance()
{
    cov_angular_velocity = invalidCovariance(); 
}

void base::samples::RigidBodyState::invalidateValues(bool invPos, bool invOri, bool invVel, bool invAngVel)
{
    if (invPos) invalidatePosition();
    if (invOri) invalidateOrientation();
    if (invVel) invalidateVelocity();
    if (invAngVel) invalidateAngularVelocity();
}

void base::samples::RigidBodyState::invalidateCovariances(bool invPos, bool invOri, bool invVel, bool invAngVel)
{
    if (invPos) invalidatePositionCovariance();
    if (invOri) invalidateOrientationCovariance();
    if (invVel) invalidateVelocityCovariance();
    if (invAngVel) invalidateAngularVelocityCovariance();
}






































