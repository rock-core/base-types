#ifndef __BASE_SAMPLES_BODY_STATE_HH
#define __BASE_SAMPLES_BODY_STATE_HH

#include <base/Time.hpp>
#include <base/Float.hpp>
#include <base/Pose.hpp>
#include <base/TransformWithCovariance.hpp>
#include <base/TwistWithCovariance.hpp>
#include <base/samples/RigidBodyState.hpp> /** For backward compatibility with RBS **/

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
     * world-fixed frames should be aligned to North (North-East-Up)
     *
     * For instance, if sourceFrame is "body" and targetFrame is "world", then
     * the BodyState object is the state of body in the world frame
     * (usually, the world frame has an arbitrary origin and a North-East-Up
     * orientation).
     */
    struct BodyState
    {

        BodyState(bool doInvalidation=true)
        {
            if(doInvalidation)
                invalidate();
        };

        BodyState(const base::TransformWithCovariance& pose, const base::TwistWithCovariance& velocity):
            pose(pose), velocity(velocity) {};

        /** Time-stamp **/
        base::Time time;

        /** Robot pose: rotation in radians and translation in meters */
        base::TransformWithCovariance pose;

        /** TwistWithCovariance: Linear[m/s] and Angular[rad/s] Velocity of the Body */
        /** It is assumed here that velocity is the derivative of a delta pose for
         * a given interval. When such interval tends to zero, one could talk
         * of instantaneous velocity **/
        base::TwistWithCovariance velocity;

        void setPose(const base::Affine3d& pose)
        {
            this->pose.setTransform(pose);
        }

        const base::Affine3d getPose() const
        {
            return this->pose.getTransform();
        }

        double getYaw() const
        {
            return base::getYaw(this->pose.orientation);
        }
	
        double getPitch() const
        {
            return base::getPitch(this->pose.orientation);
        }
	
        double getRoll() const
        {
            return base::getRoll(this->pose.orientation);
        }

        inline const base::Position& position() const
        {
            return this->pose.translation;
        }

        inline base::Position& position()
        {
            return this->pose.translation;
        }

        /** A read-only expression of the rotation **/
        inline const base::AngleAxisd& orientation() const
        {
            return this->pose.orientation;
        }

        inline base::AngleAxisd& orientation()
        {
            return this->pose.orientation;
        }

        inline const base::Vector3d& linear_velocity() const
        {
            return this->velocity.vel;
        }

        inline base::Position& linear_velocity()
        {
            return this->velocity.vel;
        }

        inline const base::Vector3d& angular_velocity() const
        {
            return this->velocity.rot;
        }

        inline base::Position& angular_velocity()
        {
            return this->velocity.rot;
        }


        /** A read-only expression of the pose covariance **/
        inline const base::Matrix6d& cov_pose() const
        {
            return this->pose.cov;
        }

        inline base::Matrix6d& cov_pose()
        {
            return this->pose.cov;
        }

        /** A read-only expression of the covariance rotation **/
        inline const base::Matrix3d cov_orientation() const
        {
            return this->pose.getOrientationCov();
        }

        inline void cov_orientation(const base::Matrix3d& cov)
        {
            return this->pose.setOrientationCov(cov);
        }

        /** A read-only expression of the rotation in quaternion **/
        inline const base::Matrix3d cov_position() const
        {
            return this->pose.getTranslationCov();
        }

        inline void cov_position(const base::Matrix3d& cov)
        {
            return this->pose.setTranslationCov(cov);
        }

        /** A read-only expression of the velocity covariance **/
        inline const base::Matrix6d& cov_velocity() const
        {
            return this->velocity.cov;
        }

        inline base::Matrix6d& cov_velocity()
        {
            return this->velocity.cov;
        }

        static BodyState unknown()
        {
            BodyState result(false);
            result.initUnknown();
            return result;
        };

        static BodyState invalid()
        {
            BodyState result(true);
            return result;
        };
	
        /** For backward compatibility only. Use invalidate() */
        void initSane()
        {
            invalidate();
        }

        /** Initializes the rigid body state with NaN for the
         * position, velocity, orientation and angular velocity.
         */
        void invalidate()
        {
            invalidatePose();
            invalidatePoseCovariance();
            invalidateVelocity();
            invalidateVelocityCovariance();
        }
	
        /**
         * Initializes the rigid body state unknown with Zero for the
         * position, velocity and angular velocity, Identity for the orientation
         * and infinity for all covariances.
         */
    	void initUnknown()
        {
            this->pose.setTransform(base::Affine3d::Identity());
            this->pose.invalidateCovariance();
            this->velocity.setVelocity(base::Vector6d::Zero());
            this->velocity.invalidateCovariance();
        }

        bool hasValidPose() const  { return this->pose.hasValidTransform(); }
        bool hasValidPoseCovariance() const  { return this->pose.hasValidCovariance(); }
        void invalidatePose() { this->pose.invalidateTransform(); }
        void invalidatePoseCovariance() { this->pose.invalidateCovariance(); }

        bool hasValidVelocity() const  { return this->velocity.hasValidVelocity(); }
        bool hasValidVelocityCovariance() const  { return this->velocity.hasValidCovariance(); }
        void invalidateVelocity() { this->velocity.invalidateVelocity(); }
        void invalidateVelocityCovariance() { this->velocity.invalidateCovariance(); }

        void invalidateValues ( bool pose = true, bool velocity = true)
        {
            if (pose) this->invalidatePose();
            if (velocity) this->invalidateVelocity();
        }

        void invalidateCovariances ( bool pose = true, bool velocity = true)
        {
            if (pose) this->invalidatePoseCovariance();
            if (velocity) this->invalidateVelocityCovariance();
        }

        /** For backward compatibility with RBS **/
        BodyState& operator=( const base::samples::RigidBodyState& rbs )
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
        };


        /** Default std::cout function
        */
        friend std::ostream & operator<<(std::ostream &out, const TransformWithCovariance& trans);

        /** performs a composition of this Body State with the Body State given.
         * The result is another Body State with result = this * trans
         */
        BodyState composition( const BodyState& bs ) const
        {
            return this->operator*( bs );
        };

        /** alias for the composition of two body states
         */
        BodyState operator*( const BodyState& bs ) const
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
                Eigen::Matrix3d rot_matrix(result_pose.orientation.toRotationMatrix());
                result_velocity.cov.block<3,3>(0,0) = (rot_matrix.transpose() * result_velocity.cov.block<3,3>(0,0) * rot_matrix);
                result_velocity.cov.block<3,3>(3,3) = (rot_matrix.transpose() * result_velocity.cov.block<3,3>(3,3) * rot_matrix);
                result_velocity.cov.block<3,3>(3,0) = (rot_matrix.transpose() * result_velocity.cov.block<3,3>(3,0) * rot_matrix);
                result_velocity.cov.block<3,3>(0,3) = (rot_matrix.transpose() * result_velocity.cov.block<3,3>(0,3) * rot_matrix);
            }

            /* Result Body State **/
            return BodyState(result_pose, result_velocity);
        }
    };

    /** Default std::cout function
    */
    inline std::ostream & operator<<(std::ostream &out, const base::samples::BodyState& bs)
    {
        out << bs.pose << "\n";
        out << bs.velocity << "\n";
        return out;
    };

}}//end namespace base::samples
#endif

