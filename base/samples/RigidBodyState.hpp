#ifndef __BASE_SAMPLES_RIGID_BODY_STATE_HH
#define __BASE_SAMPLES_RIGID_BODY_STATE_HH

#include <base/Pose.hpp>
#include <base/Time.hpp>
#include <base/Float.hpp>

#include <Eigen/Core>
#include <Eigen/LU>

namespace base { namespace samples {
    /** Representation of the state of a rigid body
     *
     * This is among other things used to express frame transformations by
     * Rock's transformer
     * 
     * Given a source and target frame, this structure expresses the _frame
     * change_ between these two frames. In effect, it represents the state of
     * the source frame expressed in the target frame.
     *
     * Per [Rock's conventions](http://rock.opendfki.de/wiki/WikiStart/Standards), you
     * should use a X-forward, right handed coordinate system when assigning
     * frames to bodies (i.e.  X=forward, Y=left, Z=up). In addition,
     * world-fixed frames should be aligned to North (North-West-Up, aka NWU)
     *
     * For instance, if sourceFrame is "body" and targetFrame is "world", then
     * the RigidBodyState object is the state of body in the world frame
     * (usually, the world frame has an arbitrary origin and a North-West-Up
     * orientation).
     */
    struct RigidBodyState
    {

        RigidBodyState(bool doInvalidation=true){
            if(doInvalidation)
                    invalidate();
        };

        base::Time time;

	/** Name of the source reference frame */
	std::string sourceFrame;

	/** Name of the target reference frame */
	std::string targetFrame;

        /** Position in m of sourceFrame's origin expressed in targetFrame
         */
        Position   position;
	/** Covariance matrix of the position
	 */
        base::Matrix3d cov_position;

        /** Orientation of targetFrame expressed in sourceFrame */
        Orientation orientation;
        /** Covariance matrix of the orientation as an axis/angle manifold in
         * body coordinates
	 */
        base::Matrix3d cov_orientation;

        /** Velocity in m/s of sourceFrame expressed in targetFrame */
        base::Vector3d velocity;
	/** Covariance of the velocity 
	 */
        base::Matrix3d cov_velocity;

        /** Angular Velocity as an axis-angle representation in body fixed frame
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
	
	template <int _Options>
	operator Eigen::Transform<double, 3, Eigen::Affine, _Options>() const
	{
	    Eigen::Transform<double, 3, Eigen::Affine, _Options> ret;
	    ret.setIdentity();
	    ret.rotate(this->orientation);
	    ret.translation() = this->position;
	    return ret;
	}

        static RigidBodyState unknown(){
            RigidBodyState result(false);
            result.initUnknown();
            return result;
        };

        static RigidBodyState invalid() {
            RigidBodyState result(true);
            return result;
        };
	
        /** For backward compatibility only. Use invalidate() */
        void initSane() {
            invalidate();
        }

        /** Initializes the rigid body state with NaN for the
         * position, velocity, orientation and angular velocity.
         */
	void invalidate() {
	    invalidateOrientation();
            invalidateOrientationCovariance();
	    invalidatePosition();
            invalidatePositionCovariance();
	    invalidateVelocity();
            invalidateVelocityCovariance();
	    invalidateAngularVelocity();
            invalidateAngularVelocityCovariance();
	}
	
	/**
         * Initializes the rigid body state unknown with Zero for the
         * position, velocity and angular velocity, Identity for the orientation
         * and infinity for all covariances.
         */
	void initUnknown()
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

        /** Helper method that checks if a value is valid (not NaN anywhere). */
        static bool isValidValue(base::Vector3d const& vec)
        {
            return !base::isNaN(vec(0)) &&
                !base::isNaN(vec(1)) &&
                !base::isNaN(vec(2));
        }
        
        /** Helper method that checks if an orientation is valid (not NaN anywhere)
         *  and that the orientation is an unit quaternion. */
        static bool isValidValue(base::Orientation const& ori)
        {
            return !base::isNaN(ori.w()) &&
                !base::isNaN(ori.x()) &&
                !base::isNaN(ori.y()) &&
                !base::isNaN(ori.z()) &&
                fabs(ori.squaredNorm()-1.0) < 1e-6;     //assuming at least single precision 
        }
        
        /** Helper method that checks if the value whose covariance is
         * represented by the given matrix is a known value (not Inf in the diagonal).
         */
        static bool isKnownValue(base::Matrix3d const& cov)
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

        /** Checks if a given dimension of a vector is valid. */
        static bool isValidValue(base::Vector3d const& vec, int dim)
        {
            return !base::isNaN(vec(dim));
        }
        
        /** Checks if a given dimension of a covariance matrix is valid. */
        static bool isValidCovariance(base::Matrix3d const& cov, int dim)
        {
            return !base::isNaN(cov(dim,dim));
        }
        
        /** Checks if the dimension of a vector given by a covariance is known. */
        static bool isKnownValue(base::Matrix3d const& cov, int dim)
        {
            return !base::isInfinity(cov(dim,dim));
        }

        static base::Vector3d invalidValue()
        {
            return base::Vector3d::Ones() * base::NaN<double>();
        }
        
        static base::Orientation invalidOrientation()
        {
            return base::Orientation(Eigen::Vector4d::Ones() * base::NaN<double>());
        }

        static base::Matrix3d setValueUnknown()
        {
            return base::Matrix3d::Ones() * base::infinity<double>();
        }

        static base::Matrix3d invalidCovariance()
        {
            return base::Matrix3d::Ones() * base::NaN<double>();
        }
	
        bool hasValidPosition() const { return isValidValue(position); }
        bool hasValidPosition(int idx) const { return isValidValue(position, idx); }
        bool hasValidPositionCovariance() const { return isValidCovariance(cov_position); }
        void invalidatePosition() { position = invalidValue(); }
        void invalidatePositionCovariance() { cov_position = invalidCovariance(); }
        
        bool hasValidOrientation() const { return isValidValue(orientation); }
        bool hasValidOrientationCovariance() const { return isValidCovariance(cov_orientation); }
        void invalidateOrientation() { orientation = invalidOrientation(); }
        void invalidateOrientationCovariance() { cov_orientation = invalidCovariance(); }
        
        bool hasValidVelocity() const { return isValidValue(velocity); }
        bool hasValidVelocity(int idx) const { return isValidValue(velocity, idx); }
        bool hasValidVelocityCovariance() const { return isValidCovariance(cov_velocity); }
        void invalidateVelocity() { velocity = invalidValue(); }
        void invalidateVelocityCovariance() { cov_velocity = invalidCovariance(); }
        
        bool hasValidAngularVelocity() const { return isValidValue(angular_velocity); }
        bool hasValidAngularVelocity(int idx) const { return isValidValue(angular_velocity, idx); }
        bool hasValidAngularVelocityCovariance() const { return isValidCovariance(cov_angular_velocity); }
        void invalidateAngularVelocity() { angular_velocity = invalidValue(); }
        void invalidateAngularVelocityCovariance() { cov_angular_velocity = invalidCovariance(); }

        void invalidateValues(bool invPos, bool invOri, bool invVel = true,
                              bool invAngVel = true) {
            if (invPos) invalidatePosition();
            if (invOri) invalidateOrientation();
            if (invVel) invalidateVelocity();
            if (invAngVel) invalidateAngularVelocity();
        }

        void invalidateCovariances(bool invPos = true, bool invOri = true,
                                   bool invVel = true, bool invAngVel = true) {
            if (invPos) invalidatePositionCovariance();
            if (invOri) invalidateOrientationCovariance();
            if (invVel) invalidateVelocityCovariance();
            if (invAngVel) invalidateAngularVelocityCovariance();
        }
    };
}}

#endif
