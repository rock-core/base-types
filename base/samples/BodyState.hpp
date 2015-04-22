#ifndef __BASE_SAMPLES_BODY_STATE_HH
#define __BASE_SAMPLES_BODY_STATE_HH

#include <base/Time.hpp>
#include <base/Float.hpp>
#include <base/Pose.hpp>
#include <base/Transformation.hpp>
#include <base/Twist.hpp>

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

        BodyState(const base::Transformation& pose, const base::Twist& velocity):
            pose(pose), velocity(velocity) {};

        /** Time-stamp **/
        base::Time time;

	    /** Name of the source reference frame */
    	std::string sourceFrame;

    	/** Name of the target reference frame */
    	std::string targetFrame;

        /** Robot pose: translation in meters */
        base::Transformation pose;

        /** Twist: Linear and Angular Velocity of the Pose m/s and rad/s */
        base::Twist velocity;

        void setPose(const base::Transformation& pose)
        {
            this->pose = pose;
        }

        const base::Transformation& getPose() const
        {
            return this->pose;
        }

        double getYaw() const
        {
            base::Orientation orientation(this->pose.getTransform().rotation());
            return base::getYaw(orientation);
        }
	
        double getPitch() const
        {
            base::Orientation orientation(this->pose.getTransform().rotation());
            return base::getPitch(orientation);
        }
	
        double getRoll() const
        {
            base::Orientation orientation(this->pose.getTransform().rotation());
            return base::getRoll(orientation);
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
            this->pose.invalidateUncertainty();
            this->velocity.setVelocity(base::Vector6d::Zero());
            this->velocity.invalidateUncertainty();
        }

        bool hasValidPose() const  { return this->pose.hasValidTransform(); }
        bool hasValidPoseCovariance() const  { return this->pose.hasValidUncertainty(); }
        void invalidatePose() { this->pose.invalidateTransform(); }
        void invalidatePoseCovariance() { this->pose.invalidateUncertainty(); }

        bool hasValidVelocity() const  { return this->velocity.hasValidVelocity(); }
        bool hasValidVelocityCovariance() const  { return this->velocity.hasValidUncertainty(); }
        void invalidateVelocity() { this->velocity.invalidateVelocity(); }
        void invalidateVelocityCovariance() { this->velocity.invalidateUncertainty(); }

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

        /** Default std::cout function
        */
        friend std::ostream & operator<<(std::ostream &out, const Transformation& trans);

        /** performs a composition of this Body State with the Body State given.
         * The result is another Body State with result = this * trans
         */
        BodyState composition( const BodyState& bs ) const
        {
            return this->operator*( bs );
        };

        /** alias for the composition of two transforms
         */
        BodyState operator*( const BodyState& bs ) const
        {
            const BodyState &bs2(*this);
            const BodyState &bs1(bs);

            /* Result Body State **/
            return BodyState(static_cast<base::Transformation>(bs2.pose * bs1.pose),
                    static_cast<base::Twist>(bs2.velocity + bs1.velocity));
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

