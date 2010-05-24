#ifndef __BASE_ODOMETRY_HPP__
#define __BASE_ODOMETRY_HPP__

#include <Eigen/Core>
#include <base/time.h>
#include <base/pose.h>

namespace base
{
    /** Class which provides an abstract odometry model for a robot
     */
    template <class BodyState_, class Pose_>
    class Odometry
    {
    public:
	Odometry() 
	    : update_counter(0) {}

	/** 
	 * will set the current body state and store the previous state
	 * to perform the odometry calculations.
	 */
	virtual void updateBodyState( const BodyState_& state )
	{
	    // make the current configuration the previous configuration
	    state_k = state_kp;
	    // set the current configuration with the new values
	    state_kp = state;
	    update_counter++;
	}

	/** 
	 * return true if both a current and a previous Bodystate are set,
	 * so updateBodyState has been called at least twice.
	 */
	bool isValid()
	{
	    return update_counter > 1;
	}

	/** 
	 * returns the time difference between the current and the previous
	 * state
	 */
	base::Time getTimeDelta() const
	{
	    return state_kp.time - state_k.time;
	}

	const BodyState_& getCurrentBodyState() const
	{
	    return state_kp;
	}

	const BodyState_& getPreviousBodyState() const
	{
	    return state_k;
	}

    protected:
	size_t update_counter;

	/** Body state at time k */ 
	BodyState_ state_k, state_kp;
    };

    /** 
     * base class an odometry model with a gaussian error model
     */
    template <class BodyState_>
    class GaussianOdometry : public Odometry<BodyState_, Pose_>
    {
	/**
	 * return the pose delta in the body fixed frame of the previous
	 * state. 
	 */
	virtual Pose_ getPoseDelta() = 0;
	
	/**
	 * returns the covariance matrix of the linear velocity 
	 */
	virtual Eigen::Matrix3d getPositionError() = 0;

	/**
	 * returns the covariance matrix of the orientation error
	 * as an axis angle vector on a manifold. 
	 */
	virtual Eigen::Matrix3d getOrientationError() = 0;
    };

    /**
     * base class of an odometry with an arbitrary error model
     * which can be sampled.
     */
    template <class BodyState_, class Pose_>
    class SamplingOdometry : public Odometry<BodyState_, Pose_>
    {
	virtual Pose_ getPoseDeltaSample() = 0;
    };

    template <class BodyState_>
    class GaussianOdometry3D : public GaussianOdometry<BodyState_, base::Pose>
    {
    };

    template <class BodyState_>
    class SamplingOdometry2D : public SamplingOdometry<BodyState_, base::Pose2D>
    {
    };
}

#endif
