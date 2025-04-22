#ifndef __BASE_SAMPLES_RIGID_BODY_STATE_HH
#define __BASE_SAMPLES_RIGID_BODY_STATE_HH

#include <base/Pose.hpp>
#include <base/Time.hpp>
#include <base/Float.hpp>

#include <Eigen/Core>
#include <Eigen/LU>

namespace base {
/**
 * Maps angular velocity vector into Euler angles rate vector.
 *
 * It takes the provided orientation into account in order to map the provided angular
 * velocity (represented by axis-angle) into Euler angles rate vector following the
 * ZYX-order (yaw-pitch-roll).
 *
 * It considers that "orientation" represents the orientation of the targetFrame
 * expressed in the sourceFrame, the same way it is defined in RigidBodyState. The
 * input angular_velocity is, then, considered to be expressed in the sourceFrame.
 *
 * For instance, if sourceFrame is "body" and targetFrame is "world", the input
 * angular_velocity will be considered as being expressed in the "body" frame.
 */
Vector3d angularVelocity2EulerRate(const Vector3d& angular_velocity,
                                   const Orientation& orientation);

/**
 * Maps Euler angles rate vector into angular velocity vector.
 *
 * It takes the provided orientation into account in order to map the provided Euler
 * rate vector following the ZYX-order (yaw-pitch-roll) into an angular velocity
 * (represented by axis-angle).
 *
 * It considers that "orientation" represents the orientation of the targetFrame
 * expressed in the sourceFrame, the same way it is defined in RigidBodyState. The
 * resulting angular_velocity is, then, expressed in the sourceFrame.
 *
 * For instance, if sourceFrame is "body" and targetFrame is "world", the resulting
 * angular velocity will be expressed in the "body" frame.
 */
Vector3d eulerRate2AngularVelocity(const Vector3d& euler_rate,
                                   const Orientation& orientation);

namespace samples {
    /** Representation of the state of a rigid body
     *
     * This is among other things used to express frame transformations by
     * Rock's transformer
     *
     * Given a source and target frame, this structure expresses the _frame
     * change_ between these two frames. In effect, it represents the state of
     * the source frame expressed in the target frame.
     *
     * Per [Rock's conventions](http://rock.opendfki.de/wiki/WikiStart/Standards),
     * you should use a X-forward, right handed coordinate system when assigning
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
        RigidBodyState(bool doInvalidation=true);

        /** @meta role logical_time */
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

        /** Velocity in m/s of sourceFrame relative to targetFrame,
         * expressed in targetFrame */
        base::Vector3d velocity;
        /** Covariance of the velocity
         */
        base::Matrix3d cov_velocity;

        /** Angular Velocity of sourceFrame relative to targetFrame,
         * expressed in sourceFrame, as an axis-angle representation
         *
         * The direction of the vector is the axis, its length the speed */
        base::Vector3d angular_velocity;
        /** Covariance of the angular velocity
         */
        base::Matrix3d cov_angular_velocity;

        void setTransform(const Eigen::Affine3d& transform);

        Eigen::Affine3d getTransform() const;

        void setPose(const base::Pose& pose);

        base::Pose getPose() const;

        double getYaw() const;

        double getPitch() const;

        double getRoll() const;

        /** Gets the time derivative of the Euler angles ZYX (yaw-pitch-roll) */
        base::Vector3d getEulerRate() const;

        /** Gets yaw rate from getEulerRate()*/
        double getYawRate() const;

        /** Gets pitch rate from getEulerRate()*/
        double getPitchRate() const;

        /** Gets roll rate from getEulerRate()*/
        double getRollRate() const;

        /** Sets the angular velocity given the time derivative of Euler angles ZYX
         *  (yaw-pitch-roll) */
        void setAngularVelocity(const base::Vector3d& euler_rate);

        template <int _Options>
        operator Eigen::Transform<double, 3, Eigen::Affine, _Options>() const
        {
            Eigen::Transform<double, 3, Eigen::Affine, _Options> ret;
            ret.setIdentity();
            ret.rotate(this->orientation);
            ret.translation() = this->position;
            return ret;
        }

        static RigidBodyState unknown();

        static RigidBodyState invalid();

        /** For backward compatibility only. Use invalidate() */
        void initSane();

        /** Initializes the rigid body state with NaN for the
         * position, velocity, orientation and angular velocity.
         */
        void invalidate();

        /**
         * Initializes the rigid body state unknown with Zero for the
         * position, velocity and angular velocity, Identity for the orientation
         * and infinity for all covariances.
         */
        void initUnknown();

        /** Helper method that checks if a value is valid (not NaN anywhere). */
        static bool isValidValue(base::Vector3d const& vec);
        
        /** Helper method that checks if an orientation is valid (not NaN anywhere)
         *  and that the orientation is an unit quaternion. */
        static bool isValidValue(base::Orientation const& ori);
        
        /** Helper method that checks if the value whose covariance is
         * represented by the given matrix is a known value (not Inf in the diagonal).
         */
        static bool isKnownValue(base::Matrix3d const& cov);

        /** Helper method that checks if the covariance represented by the given
         * matrix is valid
         */
        static bool isValidCovariance(base::Matrix3d const& cov);

        /** Checks if a given dimension of a vector is valid. */
        static bool isValidValue(base::Vector3d const& vec, int dim);
        
        /** Checks if a given dimension of a covariance matrix is valid. */
        static bool isValidCovariance(base::Matrix3d const& cov, int dim);
        
        /** Checks if the dimension of a vector given by a covariance is known. */
        static bool isKnownValue(base::Matrix3d const& cov, int dim);

        static base::Vector3d invalidValue();
        
        static base::Orientation invalidOrientation();

        static base::Matrix3d setValueUnknown();

        static base::Matrix3d invalidCovariance();

        bool hasValidPosition() const;
        bool hasValidPosition(int idx) const;
        bool hasValidPositionCovariance() const;
        void invalidatePosition();
        void invalidatePositionCovariance();
        
        bool hasValidOrientation() const;
        bool hasValidOrientationCovariance() const;
        void invalidateOrientation();
        void invalidateOrientationCovariance();
        
        bool hasValidVelocity() const;
        bool hasValidVelocity(int idx) const;
        bool hasValidVelocityCovariance() const;
        void invalidateVelocity();
        void invalidateVelocityCovariance();
        
        bool hasValidAngularVelocity() const;
        bool hasValidAngularVelocity(int idx) const;
        bool hasValidAngularVelocityCovariance() const;
        void invalidateAngularVelocity();
        void invalidateAngularVelocityCovariance();

        void invalidateValues(bool invPos, bool invOri, bool invVel = true,
                              bool invAngVel = true);

        void invalidateCovariances(bool invPos = true, bool invOri = true,
                                   bool invVel = true, bool invAngVel = true);
    };
}  // end namespace samples
}  // end namespace base

#endif
