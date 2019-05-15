#ifndef __BASE_SAMPLES_RIGID_BODY_ACCELERATION_HH
#define __BASE_SAMPLES_RIGID_BODY_ACCELERATION_HH

#include <base/Eigen.hpp>
#include <base/Time.hpp>

namespace base { namespace samples {
    /**
     * Representation of accelerations of a body in a given (unspecified) fixed
     * frame of reference
     *
     * While the frame of reference is unspecified, it will usually be the
     * inertial frame. We encourage Rock code to use the body-fixed frame
     * as the frame of expression.
     *
     * Indeed, Sensors (e.g. gyros, accelerometers) and models (e.g.
     * hydrodynamic models) give access to velocities and accelerations w.r.t.
     * the inertial frame, but expressed in the body frame. However, expressing
     * the velocities/accelerations in the world frame from these
     * sensing/estimation methods would require to have an estimate
     * of the system's pose in the world, which is a harder thing to get.
     */
    struct RigidBodyAcceleration
    {
        RigidBodyAcceleration();

        base::Time time;

        /** Linear acceleration in m/s2 */
        base::Vector3d acceleration;
        /** Covariance matrix of the linear acceleration
         */
        base::Matrix3d cov_acceleration;

        /** Angular acceleration in rad/s2 */
        base::Vector3d angular_acceleration;
        /** Covariance matrix of the angular acceleration
         */
        base::Matrix3d cov_angular_acceleration;

        void invalidate();

    };
}}

#endif

