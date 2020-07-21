#include "RigidBodyState.hpp"

namespace base {

Vector3d angularVelocity2EulerRate(const Vector3d& angular_velocity,
                                   const Orientation& orientation)
{
    double roll = base::getRoll(orientation);
    double pitch = base::getPitch(orientation);

    double sr = sin(roll);
    double cr = cos(roll);
    double sp = sin(pitch);
    double cp = cos(pitch);

    // This is the transformation matrix that maps the angular velocity into an Euler
    // angle rate vector following the ZYX-order (yaw-pitch-roll).
    Eigen::Matrix3d ypr_jacobian;
    ypr_jacobian << 0,      sr / cp,      cr / cp,
                    0,           cr,          -sr,
                    1, sr * sp / cp, cr * sp / cp;

    return ypr_jacobian * angular_velocity;
}

Vector3d eulerRate2AngularVelocity(const Vector3d& euler_rate,
                                   const Orientation& orientation)
{
    double roll = base::getRoll(orientation);
    double pitch = base::getPitch(orientation);

    double sr = sin(roll);
    double cr = cos(roll);
    double sp = sin(pitch);
    double cp = cos(pitch);

    // This is the transformation matrix that maps the Euler angles rate vector
    // following the ZYX-order (yaw-pitch-roll) to the angular velocity.
    Eigen::Matrix3d ypr_jacobian_inv;
    ypr_jacobian_inv <<     -sp,   0, 1,
                        cp * sr,  cr, 0,
                        cp * cr, -sr, 0;

    return ypr_jacobian_inv * euler_rate;
}

namespace samples {

RigidBodyState::RigidBodyState(bool doInvalidation)
{
    if (doInvalidation) { invalidate(); }
}

void RigidBodyState::setTransform(const Eigen::Affine3d& transform)
{
    position = transform.translation();
    orientation = Eigen::Quaterniond(transform.linear());
}

Eigen::Affine3d RigidBodyState::getTransform() const
{
    Eigen::Affine3d ret;
    ret.setIdentity();
    ret.rotate(this->orientation);
    ret.translation() = this->position;
    return ret;
}

void RigidBodyState::setPose(const Pose& pose)
{
    orientation = pose.orientation;
    position = pose.position;
}

Pose RigidBodyState::getPose() const
{
    return Pose(position, orientation);
}

double RigidBodyState::getYaw() const
{
    return base::getYaw(orientation);
}

double RigidBodyState::getPitch() const
{
    return base::getPitch(orientation);
}

double RigidBodyState::getRoll() const
{
    return base::getRoll(orientation);
}

Vector3d RigidBodyState::getEulerRate() const
{
    return base::angularVelocity2EulerRate(angular_velocity, orientation);
}

double RigidBodyState::getYawRate() const
{
    return getEulerRate()[0];
}

double RigidBodyState::getPitchRate() const
{
    return getEulerRate()[1];
}

double RigidBodyState::getRollRate() const
{
    return getEulerRate()[2];
}

void RigidBodyState::setAngularVelocity(const Vector3d& euler_rate)
{
    angular_velocity = base::eulerRate2AngularVelocity(euler_rate, orientation);
}

RigidBodyState RigidBodyState::unknown()
{
    RigidBodyState result(false);
    result.initUnknown();
    return result;
}

RigidBodyState RigidBodyState::invalid()
{
    RigidBodyState result(true);
    return result;
}

void RigidBodyState::initSane()
{
    invalidate();
}

void RigidBodyState::invalidate()
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

void RigidBodyState::initUnknown()
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

bool RigidBodyState::isValidValue(const Vector3d& vec)
{
    return !isNaN(vec(0)) &&
        !isNaN(vec(1)) &&
        !isNaN(vec(2));
}

bool RigidBodyState::isValidValue(const Orientation& ori)
{
    return !isNaN(ori.w()) &&
           !isNaN(ori.x()) &&
           !isNaN(ori.y()) &&
           !isNaN(ori.z()) &&
           fabs(ori.squaredNorm() - 1.0) < 1e-6;  //assuming at least single precision
}

bool RigidBodyState::isKnownValue(const Matrix3d& cov)
{
    return !isInfinity(cov(0, 0)) &&
           !isInfinity(cov(1, 1)) &&
           !isInfinity(cov(2, 2));
}

bool RigidBodyState::isValidCovariance(const Matrix3d& cov)
{
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            if (isNaN(cov(i, j))) {
                return false;
            }
        }
    }
    return true;
}

bool RigidBodyState::isValidValue(const Vector3d& vec, int dim)
{
    return !isNaN(vec(dim));
}

bool RigidBodyState::isValidCovariance(const Matrix3d& cov, int dim)
{
    return !isNaN(cov(dim, dim));
}

bool RigidBodyState::isKnownValue(const Matrix3d& cov, int dim)
{
    return !isInfinity(cov(dim, dim));
}

Vector3d RigidBodyState::invalidValue()
{
    return Vector3d::Ones() * NaN<double>();
}

Orientation RigidBodyState::invalidOrientation()
{
    return Orientation(Eigen::Vector4d::Ones() * NaN<double>());
}

Matrix3d RigidBodyState::setValueUnknown()
{
    return Matrix3d::Ones() * infinity<double>();
}

Matrix3d RigidBodyState::invalidCovariance()
{
    return Matrix3d::Ones() * NaN<double>();
}

bool RigidBodyState::hasValidPosition() const
{
    return isValidValue(position);
}

bool RigidBodyState::hasValidPosition(int idx) const
{
    return isValidValue(position, idx);
}

bool RigidBodyState::hasValidPositionCovariance() const
{
    return isValidCovariance(cov_position);
}

void RigidBodyState::invalidatePosition()
{
    position = invalidValue();
}

void RigidBodyState::invalidatePositionCovariance()
{
    cov_position = invalidCovariance();
}

bool RigidBodyState::hasValidOrientation() const
{
    return isValidValue(orientation);
}

bool RigidBodyState::hasValidOrientationCovariance() const
{
    return isValidCovariance(cov_orientation);
}

void RigidBodyState::invalidateOrientation()
{
    orientation = invalidOrientation();
}

void RigidBodyState::invalidateOrientationCovariance()
{
     cov_orientation = invalidCovariance();
}

bool RigidBodyState::hasValidVelocity() const
{
    return isValidValue(velocity);
}

bool RigidBodyState::hasValidVelocity(int idx) const
{
    return isValidValue(velocity, idx);
}

bool RigidBodyState::hasValidVelocityCovariance() const
{
    return isValidCovariance(cov_velocity);
}

void RigidBodyState::invalidateVelocity()
{
    velocity = invalidValue();
}

void RigidBodyState::invalidateVelocityCovariance()
{
    cov_velocity = invalidCovariance();
}

bool RigidBodyState::hasValidAngularVelocity() const
{
    return isValidValue(angular_velocity);
}

bool RigidBodyState::hasValidAngularVelocity(int idx) const
{
     return isValidValue(angular_velocity, idx);
}

bool RigidBodyState::hasValidAngularVelocityCovariance() const
{
    return isValidCovariance(cov_angular_velocity);
}

void RigidBodyState::invalidateAngularVelocity()
{
    angular_velocity = invalidValue();
}

void RigidBodyState::invalidateAngularVelocityCovariance()
{
    cov_angular_velocity = invalidCovariance();
}

void RigidBodyState::invalidateValues(bool invPos, bool invOri, bool invVel,
                                      bool invAngVel)
{
    if (invPos) { invalidatePosition(); }
    if (invOri) { invalidateOrientation(); }
    if (invVel) { invalidateVelocity(); }
    if (invAngVel) { invalidateAngularVelocity(); }
}

void RigidBodyState::invalidateCovariances(bool invPos, bool invOri, bool invVel,
                                           bool invAngVel)
{
    if (invPos) { invalidatePositionCovariance(); }
    if (invOri) { invalidateOrientationCovariance(); }
    if (invVel) { invalidateVelocityCovariance(); }
    if (invAngVel) { invalidateAngularVelocityCovariance(); }
}
}  // end namespace samples
}  // end namespace base
