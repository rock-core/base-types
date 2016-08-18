#include "BodyState.hpp"

#include <Eigen/Core>
#include <Eigen/LU>

namespace base { namespace samples {

BodyState::BodyState(bool doInvalidation)
{
    if(doInvalidation)
        invalidate();
}

void BodyState::setPose(const Affine3d& pose)
{
    this->pose.setTransform(pose);
}

const Affine3d BodyState::getPose() const
{
    return this->pose.getTransform();
}

double BodyState::getYaw() const
{
    return base::getYaw(this->pose.orientation);
}

double BodyState::getPitch() const
{
    return base::getPitch(this->pose.orientation);
}

double BodyState::getRoll() const
{
    return base::getRoll(this->pose.orientation);
}

const Position& BodyState::position() const
{
    return this->pose.translation;
}

Position& BodyState::position()
{
    return this->pose.translation;
}

const Quaterniond& BodyState::orientation() const
{
    return this->pose.orientation;
}

Quaterniond& BodyState::orientation()
{
    return this->pose.orientation;
}

const Vector3d& BodyState::linear_velocity() const
{
    return this->velocity.vel;
}

const Vector3d& BodyState::angular_velocity() const
{
    return this->velocity.rot;
}

Position& BodyState::linear_velocity()
{
    return this->velocity.vel;
}

Position& BodyState::angular_velocity()
{
    return this->velocity.rot;
}

const Matrix6d& BodyState::cov_pose() const
{
    return this->pose.cov;
}

Matrix6d& BodyState::cov_pose()
{
    return this->pose.cov;
}

const Matrix3d BodyState::cov_orientation() const
{
    return this->pose.getOrientationCov();
}

void BodyState::cov_orientation(const Matrix3d& cov)
{
    return this->pose.setOrientationCov(cov);
}

const Matrix3d BodyState::cov_position() const
{
    return this->pose.getTranslationCov();
}

void BodyState::cov_position(const Matrix3d& cov)
{
    return this->pose.setTranslationCov(cov);
}

const Matrix6d& BodyState::cov_velocity() const
{
    return this->velocity.cov;
}

Matrix6d& BodyState::cov_velocity()
{
    return this->velocity.cov;
}

const Matrix3d BodyState::cov_linear_velocity() const
{
    return this->velocity.getLinearVelocityCov();
}

void BodyState::cov_linear_velocity(const Matrix3d& cov)
{
    return this->velocity.setLinearVelocityCov(cov);
}

const Matrix3d BodyState::cov_angular_velocity() const
{
    return this->velocity.getAngularVelocityCov();
}

void BodyState::cov_angular_velocity(const Matrix3d& cov)
{
    return this->velocity.setAngularVelocityCov(cov);
}

BodyState BodyState::Unknown()
{
    BodyState result(false);
    result.initUnknown();
    return result;
}

BodyState BodyState::Invalid()
{
    BodyState result(true);
    return result;
}

void BodyState::initSane()
{
    invalidate();
}

void BodyState::invalidate()
{
    invalidatePose();
    invalidatePoseCovariance();
    invalidateVelocity();
    invalidateVelocityCovariance();
}

void BodyState::initUnknown()
{
    this->pose.setTransform(Affine3d::Identity());
    this->pose.invalidateCovariance();
    this->velocity.setVelocity(Vector6d::Zero());
    this->velocity.invalidateCovariance();
}

bool BodyState::hasValidPose() const
{
    return this->pose.hasValidTransform();
}

bool BodyState::hasValidPoseCovariance() const
{
    return this->pose.hasValidCovariance();
}

void BodyState::invalidatePose()
{
    this->pose.invalidateTransform(); 
}

void BodyState::invalidatePoseCovariance()
{
    this->pose.invalidateCovariance(); 
}

bool BodyState::hasValidVelocity() const
{
    return this->velocity.hasValidVelocity();
}

bool BodyState::hasValidVelocityCovariance() const
{
    return this->velocity.hasValidCovariance();
}

void BodyState::invalidateVelocity()
{
    this->velocity.invalidateVelocity();
}

void BodyState::invalidateVelocityCovariance()
{
    this->velocity.invalidateCovariance();
}

void BodyState::invalidateValues(bool pose, bool velocity)
{
    if (pose) this->invalidatePose();
    if (velocity) this->invalidateVelocity();
}

void BodyState::invalidateCovariances(bool pose, bool velocity)
{
    if (pose) this->invalidatePoseCovariance();
    if (velocity) this->invalidateVelocityCovariance();
}

BodyState& BodyState::operator=(const RigidBodyState& rbs)
{
    /** extract the transformation **/
    this->pose.setTransform(rbs.getTransform());

    /** and the transformation covariance **/
    this->pose.cov << rbs.cov_orientation, Eigen::Matrix3d::Zero(),
                        Eigen::Matrix3d::Zero(), rbs.cov_position;

    /** Extract the velocity **/
    this->velocity.vel = rbs.velocity;
    this->velocity.rot = rbs.angular_velocity;

    /** Extract the velocity covariance **/
    this->velocity.cov << rbs.cov_angular_velocity, Eigen::Matrix3d::Zero(),
                        Eigen::Matrix3d::Zero(), rbs.cov_velocity;

    return *this;
}

BodyState BodyState::composition(const BodyState& bs) const
{
    return this->operator*( bs );
}

BodyState BodyState::operator*(const BodyState& bs) const
{
    const BodyState &bs2(*this);
    const BodyState &bs1(bs);

    /** The composition of two body states is here defined as the
        * composition of the pose transformation as it is defined in the
        * TransformWithCovariance. The velocity held by the last body
        * state (here bs1) is assumed to be the current "instantaneous"
        * body state velocity and of the resulting body state.
        **/
    TransformWithCovariance result_pose (static_cast<TransformWithCovariance>(bs2.pose * bs1.pose));
    TwistWithCovariance result_velocity (bs1.velocity);

    /** Resulting velocity and covariance with respect to the pose base frame **/
    result_velocity.vel = result_pose.orientation * result_velocity.vel;
    result_velocity.rot = result_pose.orientation * result_velocity.rot;
    if (result_velocity.hasValidCovariance())
    {
        /** Uncertainty propagation through assuming linear transformation. From R. Astudillo and R.Kolossa Chapter 3 Uncertainty Propagation **/
        /** Improvement at this point(TO-DO): change to Jacobian derivation since the propagation is not linear **/
        Eigen::Matrix3d rot_matrix(result_pose.orientation.toRotationMatrix());
        result_velocity.cov.block<3,3>(0,0) = (rot_matrix * result_velocity.cov.block<3,3>(0,0) * rot_matrix.transpose());
        result_velocity.cov.block<3,3>(3,3) = (rot_matrix * result_velocity.cov.block<3,3>(3,3) * rot_matrix.transpose());
        result_velocity.cov.block<3,3>(3,0) = (rot_matrix * result_velocity.cov.block<3,3>(3,0) * rot_matrix.transpose());
        result_velocity.cov.block<3,3>(0,3) = (rot_matrix * result_velocity.cov.block<3,3>(0,3) * rot_matrix.transpose());
    }

    /* Result Body State **/
    return BodyState(result_pose, result_velocity);
}

std::ostream& operator<<(std::ostream& out, const BodyState& bs)
{
    out << bs.pose << "\n";
    out << bs.velocity << "\n";
    return out;
}

}} //end namespace base::samples







































