#ifndef __BASE_POSE_HH__
#define __BASE_POSE_HH__

#include "Eigen.hpp"
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
    base::Vector3d getEuler(const base::Orientation &orientation);

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
    base::Vector3d getEuler(const base::AngleAxisd &orientation);

    double getYaw(const base::Orientation& orientation);
    

    double getYaw(const base::AngleAxisd& orientation);

    double getPitch(const base::Orientation& orientation);

    double getPitch(const base::AngleAxisd& orientation);

    double getRoll(const base::Orientation& orientation);

    double getRoll(const base::AngleAxisd& orientation);

    base::Orientation removeYaw(const base::Orientation& orientation);

    base::Orientation removeYaw(const base::AngleAxisd& orientation);

    base::Orientation removePitch(const base::Orientation& orientation);

    base::Orientation removePitch(const base::AngleAxisd& orientation);

    base::Orientation removeRoll(const base::Orientation& orientation);

    base::Orientation removeRoll(const base::AngleAxisd& orientation);

    /** 
     * Represents a pose update threshold, with a number of test methods to see
     * if the threshold was met.
     */
    struct PoseUpdateThreshold
    {
	PoseUpdateThreshold();

	/** 
	 * Constructor with distance and angle thresholds
	 */
	PoseUpdateThreshold( double _distance, double _angle );

	/** 
	 * Test if distance or angle is greater than the 
	 * stored threshold.
	 */
	bool test( double other_distance, double other_angle );

	/** 
	 * Test if the provided delta transformation is greater in 
	 * either distance or angle than the threshold
	 */
	bool test( const Eigen::Affine3d& pdelta );

	/** 
	 * Test if the delta of the provided transformations is greater in 
	 * either distance or angle than the threshold.
	 *
	 * @param a2b the initial transformation from A to B
	 * @param aprime2b the next transformation from A' to B
	 *
	 * @result true if the transformation A' to A is greater than the stored thresholds
	 */
	bool test( const Eigen::Affine3d& a2b, const Eigen::Affine3d& aprime2b );

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
        void fromVector6d( const Vector6d &v );

        /**
         * @brief get compact scaled-axis representation of pose
         *
         * @result compact 6 vector [r t], where r is a 3 vector representing
         *  the rotation in scaled axis form, and t is the translation 3 vector.
         */
        Vector6d toVector6d() const;

        /**
        * @result yaw (rotation around z-axis) part of the rotational part of the pose
        */
        double getYaw() const
        {
            return base::getYaw(orientation);
        }
    };

    std::ostream& operator << (std::ostream& io, base::Pose const& pose);

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

    std::ostream& operator << (std::ostream& io, base::Pose2D const& pose);
}

#endif

