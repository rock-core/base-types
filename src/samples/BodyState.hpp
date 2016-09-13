#ifndef __BASE_SAMPLES_BODY_STATE_HH
#define __BASE_SAMPLES_BODY_STATE_HH

#include <base/Time.hpp>
#include <base/Float.hpp>
#include <base/TransformWithCovariance.hpp>
#include <base/TwistWithCovariance.hpp>
#include <base/samples/RigidBodyState.hpp> /** For backward compatibility with RBS **/

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

        BodyState(bool doInvalidation=true);

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

        void setPose(const base::Affine3d& pose);

        const base::Affine3d getPose() const;

        double getYaw() const;
	
        double getPitch() const;
	
        double getRoll() const;

        const base::Position& position() const;

        base::Position& position();

        /** A read-only expression of the rotation **/
        const base::Quaterniond& orientation() const;

        base::Quaterniond& orientation();

        const base::Vector3d& linear_velocity() const;
        
        base::Position& linear_velocity();

        const base::Vector3d& angular_velocity() const;

        base::Position& angular_velocity();


        /** A read-only expression of the pose covariance **/
        const base::Matrix6d& cov_pose() const;

        base::Matrix6d& cov_pose();

        /** A read-only expression of the rotation covariance **/
        const base::Matrix3d cov_orientation() const;

        void cov_orientation(const base::Matrix3d& cov);

        /** A read-only expression of the position covariance **/
        const base::Matrix3d cov_position() const;

        void cov_position(const base::Matrix3d& cov);

        /** A read-only expression of the velocity covariance **/
        const base::Matrix6d& cov_velocity() const;

        base::Matrix6d& cov_velocity();

        /** A read-only expression of the linear velocity covariance **/
        const base::Matrix3d cov_linear_velocity() const;

        void cov_linear_velocity(const base::Matrix3d& cov);

        /** A read-only expression of the angular velocity covariance **/
        const base::Matrix3d cov_angular_velocity() const;

        void cov_angular_velocity(const base::Matrix3d& cov);

        static BodyState Unknown();

        static BodyState Invalid();
	
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

        bool hasValidPose() const;
        bool hasValidPoseCovariance() const;
        void invalidatePose();
        void invalidatePoseCovariance();

        bool hasValidVelocity() const;
        bool hasValidVelocityCovariance() const;
        void invalidateVelocity();
        void invalidateVelocityCovariance();

        void invalidateValues ( bool pose = true, bool velocity = true);

        void invalidateCovariances ( bool pose = true, bool velocity = true);

        /** For backward compatibility with RBS **/
        BodyState& operator=( const base::samples::RigidBodyState& rbs );


        /** Default std::cout function
        */
        friend std::ostream & operator<<(std::ostream &out, const TransformWithCovariance& trans);

        /** performs a composition of this Body State with the Body State given.
         * The result is another Body State with result = this * trans
         */
        BodyState composition( const BodyState& bs ) const;

        /** alias for the composition of two body states
         */
        BodyState operator*( const BodyState& bs ) const;
    };

    /** Default std::cout function
    */
    std::ostream & operator<<(std::ostream &out, const base::samples::BodyState& bs);


}}//end namespace base::samples
#endif

