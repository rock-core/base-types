#include "Waypoint.hpp"

namespace base {

Waypoint::Waypoint() : position(Position::Identity()), heading(0), tol_position(0), tol_heading(0)
{
    
}

Waypoint::Waypoint(const Vector3d& _position, double _heading, double _tol_position, double _tol_heading)
    : position(_position), heading(_heading), tol_position(_tol_position), tol_heading(_tol_heading)
{

}

Waypoint::Waypoint(const Eigen::Vector3d& _position, double _heading, double _tol_position, double _tol_heading)
     : position(_position), heading(_heading), tol_position(_tol_position), tol_heading(_tol_heading)
{

}

} //end namespace base