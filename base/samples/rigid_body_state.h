#ifndef __BASE_SAMPLES_RIGID_BODY_STATE_HH
#define __BASE_SAMPLES_RIGID_BODY_STATE_HH

#ifdef __GCCXML__
#define EIGEN_DONT_VECTORIZE
#endif

#ifdef __orogen
#error "this header cannot be used in orogen-parsed code. Use wrappers/samples/rigid_body_state.h and wrappers::samples::RigidBodyState instead"
#endif

#include <base/pose.h>
#include <base/time.h>
#include <base/float.h>

#include <Eigen/Core>
#include <Eigen/LU>

namespace base { namespace samples {
    struct RigidBodyState
    {
        base::Time time;

	/** name of the source reference frame */
	std::string sourceFrame;

	/** name of the target reference frame */
	std::string targetFrame;

        /** Position in m, world fixed frame of reference (East-North-Up) */
        Position   position;
	/** Covariance matrix of the position
	 */
        base::Matrix3d cov_position;

        /** Orientation as a body->world transformation */
        Orientation orientation;
        /** Covariance matrix of the orientation as an axis/angle manifold in
         * body coordinates
	 */
        base::Matrix3d cov_orientation;

        /** Velocity in m/s with respect to world fixed frame, in body fixed
         * frame (Right-Front-Up) */
        base::Vector3d velocity;
	/** Covariance of the velocity 
	 */
        base::Matrix3d cov_velocity;

        /** Angular Velocity as an axis-angle representation in body fixed frame
         * (Right-Front-Up)
         *
         * The direction of the vector is the axis, its length the speed */
        base::Vector3d angular_velocity;
        /** Covariance of the angular velocity
	 */
        base::Matrix3d cov_angular_velocity;

	void setTransform(const Eigen::Affine3d& transform)
	{
	    position = transform.translation();
	    orientation = Eigen::Quaterniond( transform.linear() );
	}

	 Eigen::Affine3d getTransform() const 
	 {
	    Eigen::Affine3d ret;
	    ret.setIdentity();
	    ret.rotate(this->orientation);
	    ret.translation() = this->position;
	    return ret;
	 }

	void setPose(const base::Pose& pose)
	{
	    orientation = pose.orientation;
	    position = pose.position;
	}

	base::Pose getPose() const
	{
	    return base::Pose( position, orientation );
	}

        double getYaw() const
        {
            return base::getYaw(orientation);
        }
	
        double getPitch() const
        {
            return base::getPitch(orientation);
        }
	
        double getRoll() const
        {
            return base::getRoll(orientation);
        }
	
	operator Eigen::Affine3d() const
	{
	    Eigen::Affine3d ret;
	    ret.setIdentity();
	    ret.rotate(this->orientation);
	    ret.translation() = this->position;
	    return ret;
	}

        static RigidBodyState invalid() {
            RigidBodyState result;
            result.invalidate();
            return result;
        }
	
        /** For backward compatibility only. Use invalidate() */
        void initSane() {
            invalidate();
        }

        /** Initializes the rigid body state with arbitrary values for the
         * position, velocity, orientation and angular velocity, and with
         * infinite covariance matrices
         */
	void invalidate() {
	    invalidateOrientation();
	    invalidatePosition();
	    invalidateVelocity();
	    invalidateAngularVelocity();
	    
	    position.setZero();
	    velocity.setZero();
	    orientation = Eigen::Quaterniond::Identity();
	    angular_velocity.setZero();
	}

        /** Helper method that checks if the value whose covariance is
         * represented by the given matrix is a valid value
         */
        static bool isValidValue(Eigen::Matrix3d const& cov)
        {
            return !base::isInfinity(cov(0,0)) &&
                !base::isInfinity(cov(1,1)) &&
                !base::isInfinity(cov(2,2));
        }

        /** Helper method that checks if the covariance represented by the given
         * matrix is valid
         */
        static bool isValidCovariance(base::Matrix3d const& cov)
        {
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    if (base::isNaN(cov(i, j)))
                        return false;
            return true;
        }

        /** Helper method that checks if the dimension of the value whose
         * covariance is represented by the given matrix is a valid value
         */
        static bool isValidValue(base::Matrix3d const& cov, int dim)
        {
            return !base::isInfinity(cov(dim,dim));
        }

        static base::Matrix3d invalidValue()
        {
            return Eigen::Matrix3d::Ones() * base::infinity<double>();
        }

        static base::Matrix3d invalidCovariance()
        {
            return base::Matrix3d::Ones() * base::NaN<double>();
        }
	
	bool hasValidPosition() const { return isValidValue(cov_position); }
        bool hasValidPosition(int idx) const { return isValidValue(cov_position, idx); }
	bool hasValidPositionCovariance() const { return isValidCovariance(cov_position); }
	void invalidatePosition() { cov_position = invalidValue(); }
	void invalidatePositionCovariance() { cov_position = invalidCovariance(); }
	
	bool hasValidOrientation() const { return isValidValue(cov_orientation); }
        bool hasValidOrientation(int idx) const { return isValidValue(cov_orientation, idx); }
	bool hasValidOrientationCovariance() const { return isValidCovariance(cov_orientation); }
	void invalidateOrientation() { cov_orientation = invalidValue(); }
	void invalidateOrientationCovariance() { cov_orientation = invalidCovariance(); }
	
	bool hasValidVelocity() const { return isValidValue(cov_velocity); }
        bool hasValidVelocity(int idx) const { return isValidValue(cov_velocity, idx); }
	bool hasValidVelocityCovariance() const { return isValidCovariance(cov_velocity); }
	void invalidateVelocity() { cov_velocity = invalidValue(); }
	void invalidateVelocityCovariance() { cov_velocity = invalidCovariance(); }
	
	bool hasValidAngularVelocity() const { return isValidValue(cov_angular_velocity); }
        bool hasValidAngularVelocity(int idx) const { return isValidValue(cov_angular_velocity, idx); }
	bool hasValidAngularVelocityCovariance() const { return isValidCovariance(cov_angular_velocity); }
	void invalidateAngularVelocity() { cov_angular_velocity = invalidValue(); }
	void invalidateAngularVelocityCovariance() { cov_angular_velocity = invalidCovariance(); }
    };
}}

#endif

