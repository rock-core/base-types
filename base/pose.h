#ifndef __BASE_POSE_HH__
#define __BASE_POSE_HH__

#ifdef __orogen
#error "this header cannot be used in orogen-parsed code. Use wrappers/pose.h and wrappers::Pose instead"
#endif

#include <Eigen/Core>
#include <Eigen/Geometry> 

namespace base
{
    typedef Eigen::Vector3d    Position;
    typedef Eigen::Quaterniond Orientation;

    /**
     * Representation for a pose
     */
    struct Pose
    {
        Position    position;
        Orientation orientation;

        Pose()
            : position(Position::Identity()), orientation(Orientation::Identity()) {}

        Pose(Position const& p, Orientation const& o)
            : position(p), orientation(o) {}
    };
}

#endif

