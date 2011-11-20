#ifndef __BASE_POSE_HH__
#define __BASE_POSE_HH__

#ifdef __GCCXML__
#define EIGEN_DONT_VECTORIZE
#endif

#ifdef __orogen
#error "this header cannot be used in orogen-parsed code. Use wrappers/pose.h and wrappers::Pose instead"
#endif

#include <base/eigen.h>

namespace base
{
    typedef base::Vector3d    Position;
    typedef base::Quaterniond Orientation;

    typedef base::Vector2d    Position2D;
    typedef double            Orientation2D;

    static double getYaw(const base::Orientation& orientation)
    {
        return orientation.toRotationMatrix().eulerAngles(2,1,0)[0];
    }

    static inline base::Orientation removeYaw(const base::Orientation& orientation)
    {
	return Eigen::AngleAxisd( -getYaw(orientation), Eigen::Vector3d::UnitZ()) * orientation;
    }


    /** 
     * Represents a pose update threshold, with a number of test methods to see
     * of the threshold was met.
     */
    struct PoseUpdateThreshold
    {
	PoseUpdateThreshold() {};

	/** 
	 * constructor with distance and angle thresholds
	 */
	PoseUpdateThreshold( double distance, double angle )
	    : distance( distance ), angle( angle ) {};

	/** 
	 * test if distance or angle is greater than the 
	 * stored threshold.
	 */
	bool test( double distance, double angle )
	{
	    return distance > this->distance || angle > this->angle;
	}

	/** 
	 * test if the provided delta transformation is greater in 
	 * either distance or angle than the threshold
	 */
	bool test( const Eigen::Affine3d& pdelta )
	{
	    return test( Eigen::AngleAxisd( pdelta.linear() ).angle(), pdelta.translation().norm() );
	}

	/** 
	 * test if the delta of the provided transformations is greater in 
	 * either distance or angle than the threshold.
	 *
	 * @param a2b the initial transformation from A to B
	 * @param aprime2b the next transformation from A' to B
	 *
	 * @result true if the transformation A' to A is greater than the stored thresholds
	 */
	bool test( const Eigen::Affine3d& a2b, const Eigen::Affine3d& aprime2b )
	{
	    return test( a2b.inverse() * aprime2b );
	}

	double distance;
	double angle;
    };

    /**
     * Representation for a pose in 3D
     */
    struct Pose
    {
        Position    position;
        Orientation orientation;

        Pose()
            : position(Position::Zero()), orientation(Orientation::Identity()) {}

        Pose(Position const& p, Orientation const& o)
            : position(p), orientation(o) {}

	Pose(const Eigen::Affine3d &t)
	{
	    position = t.translation();
	    orientation = t.linear();
	}

	Eigen::Affine3d toTransform() const
	{
	    Eigen::Affine3d t;
	    t = orientation;
	    t.pretranslate( position );
	    return t;
	}

        double getYaw() const
        {
            return base::getYaw(orientation);
        }
    };

    /**
     * Representation for a pose in 2D
     */
    struct Pose2D
    {
        Position2D    position;
        Orientation2D orientation;

        Pose2D()
            : position(Position2D::Zero()), orientation(0) {}

        Pose2D(Position2D const& p, Orientation2D const& o)
            : position(p), orientation(o) {}
    };
}

#endif

