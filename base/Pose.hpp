#ifndef __BASE_POSE_HH__
#define __BASE_POSE_HH__

#include <base/Eigen.hpp>
#include "Angle.hpp"

namespace base
{
    /** Pose in a 3D-Space **/
    typedef base::Vector3d    Position;
    typedef base::Quaterniond Orientation;

    /** Pose in a 2D-Space **/
    typedef base::Vector2d    Position2D;
    typedef double            Orientation2D;

    /*
     * Decomposes the orientation in euler angles (non-proper, Tait-Bryan angles)
     * so that this can be obtained by applying the following rotations in order:
     *
     *  rotation of a2 around x-axis
     *  rotation of a1 around y-axis
     *  rotation of a0 around z-axis
     *
     * assuming angles in range of: a0:(-pi,pi), a1:(-pi/2,pi/2), a2:(-pi/2,pi/2)
     *
     */
    static base::Vector3d getEuler(const base::Orientation &orientation){
        const Eigen::Matrix3d m = orientation.toRotationMatrix();
        double x = base::Vector2d(m.coeff(2,2) , m.coeff(2,1)).norm();
        base::Vector3d res(0,::atan2(-m.coeff(2,0), x),0);
        if (x > Eigen::NumTraits<double>::dummy_precision()){
            res[0] = ::atan2(m.coeff(1,0), m.coeff(0,0));
            res[2] = ::atan2(m.coeff(2,1), m.coeff(2,2));
        }else{
            res[0] = 0;
            res[2] = (m.coeff(2,0)>0?1:-1)* ::atan2(-m.coeff(0,1), m.coeff(1,1));
        }
        return res;
    }

    /*
     * Decomposes the orientation in euler angles so that this can be
     * obtained by applying the following rotations in order:
     *
     *  rotation of a2 around x-axis
     *  rotation of a1 around y-axis
     *  rotation of a0 around z-axis
     *
     * assuming angles in range of: a0:(-pi,pi), a1:(-pi/2,pi/2), a2:(-pi/2,pi/2)
     *
     */
    static base::Vector3d getEuler(const base::AngleAxisd &orientation){
        const Eigen::Matrix3d m = orientation.toRotationMatrix();
        double x = base::Vector2d(m.coeff(2,2) , m.coeff(2,1)).norm();
        base::Vector3d res(0,::atan2(-m.coeff(2,0), x),0);
        if (x > Eigen::NumTraits<double>::dummy_precision()){
            res[0] = ::atan2(m.coeff(1,0), m.coeff(0,0));
            res[2] = ::atan2(m.coeff(2,1), m.coeff(2,2));
        }else{
            res[0] = 0;
            res[2] = (m.coeff(2,0)>0?1:-1)* ::atan2(-m.coeff(0,1), m.coeff(1,1));
        }
        return res;
    }

    static double getYaw(const base::Orientation& orientation)
    {
        return base::getEuler(orientation)[0];
    }

    static double getYaw(const base::AngleAxisd& orientation)
    {
        return base::getEuler(orientation)[0];
    }

    static double getPitch(const base::Orientation& orientation)
    {
        return base::getEuler(orientation)[1];
    }

    static double getPitch(const base::AngleAxisd& orientation)
    {
        return base::getEuler(orientation)[1];
    }

    static double getRoll(const base::Orientation& orientation)
    {
        return base::getEuler(orientation)[2];
    }

    static double getRoll(const base::AngleAxisd& orientation)
    {
        return base::getEuler(orientation)[2];
    }

    static inline base::Orientation removeYaw(const base::Orientation& orientation)
    {
	    return Eigen::AngleAxisd( -getYaw(orientation), Eigen::Vector3d::UnitZ()) * orientation;
    }

    static inline base::Orientation removeYaw(const base::AngleAxisd& orientation)
    {
	    return Eigen::AngleAxisd( -getYaw(orientation), Eigen::Vector3d::UnitZ()) * orientation;
    }

    static inline base::Orientation removePitch(const base::Orientation& orientation)
    {
	    return Eigen::AngleAxisd( -getPitch(orientation), Eigen::Vector3d::UnitY()) * orientation;
    }

    static inline base::Orientation removePitch(const base::AngleAxisd& orientation)
    {
	    return Eigen::AngleAxisd( -getPitch(orientation), Eigen::Vector3d::UnitY()) * orientation;
    }

    static inline base::Orientation removeRoll(const base::Orientation& orientation)
    {
    	return Eigen::AngleAxisd( -getRoll(orientation), Eigen::Vector3d::UnitX()) * orientation;
    }

    static inline base::Orientation removeRoll(const base::AngleAxisd& orientation)
    {
    	return Eigen::AngleAxisd( -getRoll(orientation), Eigen::Vector3d::UnitX()) * orientation;
    }

    /** 
     * Represents a pose update threshold, with a number of test methods to see
     * if the threshold was met.
     */
    struct PoseUpdateThreshold
    {
	PoseUpdateThreshold() {};

	/** 
	 * Constructor with distance and angle thresholds
	 */
	PoseUpdateThreshold( double _distance, double _angle )
	    : distance( _distance ), angle( _angle ) {};

	/** 
	 * Test if distance or angle is greater than the 
	 * stored threshold.
	 */
	bool test( double other_distance, double other_angle )
	{
	    return other_distance > distance || other_angle > angle;
	}

	/** 
	 * Test if the provided delta transformation is greater in 
	 * either distance or angle than the threshold
	 */
	bool test( const Eigen::Affine3d& pdelta )
	{
	    return test( pdelta.translation().norm(), Eigen::AngleAxisd( pdelta.linear() ).angle() );
	}

	/** 
	 * Test if the delta of the provided transformations is greater in 
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
	 * @brief Default constructor will initialize to zero
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

    inline std::ostream& operator << (std::ostream& io, base::Pose const& pose)
    {
        io << "Position "
           << pose.position.transpose()
           << " Orientation (RPY)" 
           << getRoll(pose.orientation) << " " 
           << getPitch(pose.orientation) << " " 
           << getYaw(pose.orientation);
;
        return io;
    }

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

        Pose2D(Position const& p, Orientation const& o)
            : position(Vector2d(p.x(), p.y())), orientation(base::getYaw(o)) {}

        Pose2D( const Pose &p)
            : position(Vector2d(p.position.x(), p.position.y())), orientation(p.getYaw()) {}

        bool isApprox(const Pose2D &other, double distPecision, double anglePrecision) const
        {
            return ((other.position - position).norm() < distPecision) && 
                   (Angle::fromRad(other.orientation).isApprox(Angle::fromRad(orientation), anglePrecision));
        }

    };

    inline std::ostream& operator << (std::ostream& io, base::Pose2D const& pose)
    {

        io << "Position "
           << pose.position.transpose()
           << " Orientation (Theta) " 
           << pose.orientation ;
;
        return io;
    }
}

#endif

