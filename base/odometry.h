#ifndef __BASE_ODOMETRY_HPP__
#define __BASE_ODOMETRY_HPP__

#ifdef __GCCXML__
#define EIGEN_DONT_VECTORIZE
#endif

#include <Eigen/Core>
#include <base/time.h>
#include <base/pose.h>

//namespace base 
//{
namespace odometry
{

    /** Class which provides common methods for bodystate handling which can be
     * used in the odometry models 
     */
    template <class BodyState_>
    class State
    {
    public:
	State() 
	    : update_counter(0) {}

	/** 
	 * will set the current body state and store the previous state
	 * to perform the odometry calculations.
	 */
	virtual void update( const BodyState_& state )
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

	const BodyState_& getCurrent() const
	{
	    return state_kp;
	}

	const BodyState_& getPrevious() const
	{
	    return state_k;
	}

    protected:
	size_t update_counter;

	/** Body state at time k */ 
	BodyState_ state_k, state_kp;
    };

    /** 
     * base class of a 3d odometry model with a gaussian error model
     */
    class Gaussian3D
    {
    public:
	/**
	 * return the pose delta in the body fixed frame of the previous
	 * state. 
	 */
	virtual base::Pose getPoseDelta() = 0;
	
	/**
	 * returns the covariance matrix of the linear velocity 
	 */
	virtual Eigen::Matrix3d getPositionError() = 0;

	/**
	 * returns the covariance matrix of the orientation error
	 * as an axis angle vector on a manifold. 
	 */
	virtual Eigen::Matrix3d getOrientationError() = 0;

        base::Matrix6d getPoseError()
        {
            base::Matrix6d cov;
            cov << 
                getOrientationError(), Eigen::Matrix3d::Zero(),
                Eigen::Matrix3d::Zero(), getPositionError();

            return cov; 
        }

    };

    class Gaussian2D
    {
    public:
	virtual base::Pose2D getPoseDelta2D() = 0;
	virtual Eigen::Matrix2d getPositionError2D() = 0;
	virtual double getOrientationError2D() = 0;
    };

    /** 
     * base class of a 2d odometry model with an arbitrary error model, which
     * can be sampled from
     */
    class Sampling2D
    {
    public:
	virtual base::Pose2D getPoseDeltaSample2D() = 0;
    };

    /** 
     * base class of a 3d odometry model with an arbitrary error model, which
     * can be sampled from
     */
    class Sampling3D
    {
    public:
	virtual base::Pose getPoseDeltaSample() = 0;
    };

}
//}

#endif
