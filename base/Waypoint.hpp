#ifndef __BASE_WAYPOINT_HH__
#define __BASE_WAYPOINT_HH__

#include <base/Pose.hpp>
#include <base/Eigen.hpp>

namespace base
{
    /**
     * Representation for a pose
     */
    struct Waypoint
    {
        base::Vector3d position;
        //heading in radians
        double heading;

        //tollerance of the position in m
        double tol_position;
        //tollerance of the heading in rad
        double tol_heading;

        // default: initializing with identity and zero
        Waypoint()
            : position(Position::Identity()), heading(0), tol_position(0),
              tol_heading(0){}
        // use base::Vector3d
        Waypoint(base::Vector3d const &_position, double _heading,
                 double _tol_position, double _tol_heading)
            : position(_position), heading(_heading),
              tol_position(_tol_position), tol_heading(_tol_heading){}
        // convenience: same for Eigen::Vector3d
        Waypoint(Eigen::Vector3d const &_position, double _heading,
                 double _tol_position, double _tol_heading)
            : position(_position), heading(_heading),
              tol_position(_tol_position), tol_heading(_tol_heading){}
    };
}

#endif

