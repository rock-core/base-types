#include "BodyState.hpp"

#include <Eigen/Core>
#include <Eigen/LU>

base::samples::BodyState::BodyState(bool doInvalidation)
{
    if(doInvalidation)
        invalidate();
}

void base::samples::BodyState::setPose(const base::Affine3d& pose)
{
    this->pose.setTransform(pose);
}

const base::Affine3d base::samples::BodyState::getPose() const
{
    return this->pose.getTransform();
}

double base::samples::BodyState::getYaw() const
{
    return base::getYaw(this->pose.orientation);
}

double base::samples::BodyState::getPitch() const
{
    return base::getPitch(this->pose.orientation);
}

double base::samples::BodyState::getRoll() const
{
    return base::getRoll(this->pose.orientation);
}

const base::Position& base::samples::BodyState::position() const
{
    return this->pose.translation;
}

base::Position& base::samples::BodyState::position()
{
    return this->pose.translation;
}

const base::Quaterniond& base::samples::BodyState::orientation() const
{
    return this->pose.orientation;
}

base::Quaterniond& base::samples::BodyState::orientation()
{
    return this->pose.orientation;
}

const base::Vector3d& base::samples::BodyState::linear_velocity() const
{
    return this->velocity.vel;
}

const base::Vector3d& base::samples::BodyState::angular_velocity() const
{
    return this->velocity.rot;
}

base::Position& base::samples::BodyState::linear_velocity()
{
    return this->velocity.vel;
}

base::Position& base::samples::BodyState::angular_velocity()
{
    return this->velocity.rot;
}

const base::Matrix6d& base::samples::BodyState::cov_pose() const
{
    return this->pose.cov;
}

base::Matrix6d& base::samples::BodyState::cov_pose()
{
    return this->pose.cov;
}

const base::Matrix3d base::samples::BodyState::cov_orientation() const
{
    return this->pose.getOrientationCov();
}

void base::samples::BodyState::cov_orientation(const base::Matrix3d& cov)
{
    return this->pose.setOrientationCov(cov);
}

const base::Matrix3d base::samples::BodyState::cov_position() const
{
    return this->pose.getTranslationCov();
}

void base::samples::BodyState::cov_position(const base::Matrix3d& cov)
{
    return this->pose.setTranslationCov(cov);
}

const base::Matrix6d& base::samples::BodyState::cov_velocity() const
{
    return this->velocity.cov;
}

base::Matrix6d& base::samples::BodyState::cov_velocity()
{
    return this->velocity.cov;
}

const base::Matrix3d base::samples::BodyState::cov_linear_velocity() const
{
    return this->velocity.getLinearVelocityCov();
}

void base::samples::BodyState::cov_linear_velocity(const base::Matrix3d& cov)
{
    return this->velocity.setLinearVelocityCov(cov);
}

const base::Matrix3d base::samples::BodyState::cov_angular_velocity() const
{
    return this->velocity.getAngularVelocityCov();
}

void base::samples::BodyState::cov_angular_velocity(const base::Matrix3d& cov)
{
    return this->velocity.setAngularVelocityCov(cov);
}

base::samples::BodyState base::samples::BodyState::Unknown()
{
    BodyState result(false);
    result.initUnknown();
    return result;
}

base::samples::BodyState base::samples::BodyState::Invalid()
{
    BodyState result(true);
    return result;
}

void base::samples::BodyState::initSane()
{
    invalidate();
}

void base::samples::BodyState::invalidate()
{
    invalidatePose();
    invalidatePoseCovariance();
    invalidateVelocity();
    invalidateVelocityCovariance();
}

void base::samples::BodyState::initUnknown()
{
    this->pose.setTransform(base::Affine3d::Identity());
    this->pose.invalidateCovariance();
    this->velocity.setVelocity(base::Vector6d::Zero());
    this->velocity.invalidateCovariance();
}

bool base::samples::BodyState::hasValidPose() const
{
    return this->pose.hasValidTransform();
}

bool base::samples::BodyState::hasValidPoseCovariance() const
{
    return this->pose.hasValidCovariance();
}

void base::samples::BodyState::invalidatePose()
{
    this->pose.invalidateTransform(); 
}

void base::samples::BodyState::invalidatePoseCovariance()
{
    this->pose.invalidateCovariance(); 
}

bool base::samples::BodyState::hasValidVelocity() const
{
    return this->velocity.hasValidVelocity();
}

bool base::samples::BodyState::hasValidVelocityCovariance() const
{
    return this->velocity.hasValidCovariance();
}

void base::samples::BodyState::invalidateVelocity()
{
    this->velocity.invalidateVelocity();
}

void base::samples::BodyState::invalidateVelocityCovariance()
{
    this->velocity.invalidateCovariance();
}

void base::samples::BodyState::invalidateValues(bool pose, bool velocity)
{
    if (pose) this->invalidatePose();
    if (velocity) this->invalidateVelocity();
}

void base::samples::BodyState::invalidateCovariances(bool pose, bool velocity)
{
    if (pose) this->invalidatePoseCovariance();
    if (velocity) this->invalidateVelocityCovariance();
}

base::samples::BodyState& base::samples::BodyState::operator=(const base::samples::RigidBodyState& rbs)
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

base::samples::BodyState base::samples::BodyState::composition(const base::samples::BodyState& bs) const
{
    return this->operator*( bs );
}

base::samples::BodyState base::samples::BodyState::operator*(const base::samples::BodyState& bs) const
{
    const BodyState &bs2(*this);
    const BodyState &bs1(bs);

    /** The composition of two body states is here defined as the
        * composition of the pose transformation as it is defined in the
        * TransformWithCovariance. The velocity held by the last body
        * state (here bs1) is assumed to be the current "instantaneous"
        * body state velocity and of the resulting body state.
        **/
    base::TransformWithCovariance result_pose (static_cast<base::TransformWithCovariance>(bs2.pose * bs1.pose));
    base::TwistWithCovariance result_velocity (bs1.velocity);

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

std::ostream& base::samples::operator<<(std::ostream& out, const base::samples::BodyState& bs)
{
    out << bs.pose << "\n";
    out << bs.velocity << "\n";
    return out;
}








































