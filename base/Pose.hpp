#ifndef __BASE_POSE_HH__
#define __BASE_POSE_HH__

#include <base/Eigen.hpp>

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
    
    static double getPitch(const base::Orientation& orientation)
    {
        return orientation.toRotationMatrix().eulerAngles(2,1,0)[1];
    }
    
    static double getRoll(const base::Orientation& orientation)
    {
        return orientation.toRotationMatrix().eulerAngles(2,1,0)[2];
    }


    static inline base::Orientation removeYaw(const base::Orientation& orientation)
    {
	return Eigen::AngleAxisd( -getYaw(orientation), Eigen::Vector3d::UnitZ()) * orientation;
    }
    
    static inline base::Orientation removePitch(const base::Orientation& orientation)
    {
	return Eigen::AngleAxisd( -getPitch(orientation), Eigen::Vector3d::UnitY()) * orientation;
    }

    static inline base::Orientation removeRoll(const base::Orientation& orientation)
    {
	return Eigen::AngleAxisd( -getRoll(orientation), Eigen::Vector3d::UnitX()) * orientation;
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
     * @brief Representation for a pose in 3D
     *
     * Stores the position and orientation part separately
     * and is very well suited for storage and interchange,
     * since it is a compact representation.
     */
    struct Pose
    {
        Position    position;
        Orientation orientation;

	/** 
	 * @brief default constructor will initialize to zero
	 */
        Pose()
            : position(Position::Zero()), orientation(Orientation::Identity()) {}

        Pose(Position const& p, Orientation const& o)
            : position(p), orientation(o) {}

	/** 
	 * @brief constructor based on a 4x4 transform matrix
	 *
	 * @param t 4x4 homogenous transform matrix, for which the upper-left
	 *	3x3 matrix needs to be a rotation matrix.
	 */
	Pose(const Eigen::Affine3d &t)
	{
	    fromTransform( t );
	}

	Pose(const Vector6d &v)
	{
	    fromVector6d( v );
	}

	/** 
	 * @brief set the pose based on a 4x4 matrix
	 *
	 * @param t 4x4 homogenous transform matrix, for which the upper-left
	 *	3x3 matrix needs to be a rotation matrix.
	 */
	void fromTransform( const Eigen::Affine3d &t )
	{
	    position = t.translation();
	    orientation = t.linear();
	}

	/**
	 * @brief transform matrix which represents the pose transform as a 4x4
	 *  homogenous matrix
	 *
	 * @result 4x4 homogenous transform matrix
	 */
	Eigen::Affine3d toTransform() const
	{
	    Eigen::Affine3d t;
	    t = orientation;
	    t.pretranslate( position );
	    return t;
	}

	/** 
	 * @brief set pose based on compact scaled-axis representation
	 *
	 * @param v compact 6 vector [r t], where r is a 3 vector representing
	 *  the rotation in scaled axis form, and t is the translation 3 vector.
	 */
	void fromVector6d( const Vector6d &v )
	{
	    const Eigen::Vector3d saxis = v.head<3>();
	    if( saxis.norm() > 1e-9 )
		orientation = Eigen::AngleAxisd( saxis.norm(), saxis.normalized() );
	    else
		orientation = Eigen::Quaterniond::Identity();

	    position = v.tail<3>();
	}

	/**
	 * @brief get compact scaled-axis representation of pose
	 *
	 * @result compact 6 vector [r t], where r is a 3 vector representing
	 *  the rotation in scaled axis form, and t is the translation 3 vector.
	 */
	Vector6d toVector6d() const
	{
	    Vector6d res;
	    Eigen::AngleAxisd aa(orientation);
	    res.head<3>() = aa.axis() * aa.angle();
	    res.tail<3>() = position;

	    return res;
	}

	/**
	 * @result yaw (rotation around z-axis) part of the rotational part of the pose
	 */
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

