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
      

        Waypoint()
	  : position(Position::Identity()), heading(0), tol_position(0), tol_heading(0)  {}
      
	Waypoint(base::Vector3d const& position, double heading, double tol_position, double tol_heading):
	    position(position), heading(heading), tol_position(tol_position), tol_heading(tol_heading) {};
	Waypoint(Eigen::Vector3d const& position, double heading, double tol_position, double tol_heading):
	    position(position), heading(heading), tol_position(tol_position), tol_heading(tol_heading) {};

    };
}

#endif

