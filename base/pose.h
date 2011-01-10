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

	Pose(const Eigen::Transform3d &t)
	{
	    position = t.translation();
	    orientation = t.rotation();
	}

	Eigen::Transform3d toTransform() const
	{
	    Eigen::Transform3d t;
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

