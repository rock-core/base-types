#include "Waypoint.hpp"

base::Waypoint::Waypoint() : position(Position::Identity()), heading(0), tol_position(0), tol_heading(0)
{
    
}

base::Waypoint::Waypoint(const base::Vector3d& _position, double _heading, double _tol_position, double _tol_heading)
    : position(_position), heading(_heading), tol_position(_tol_position), tol_heading(_tol_heading)
{

}

base::Waypoint::Waypoint(const Eigen::Vector3d& _position, double _heading, double _tol_position, double _tol_heading)
     : position(_position), heading(_heading), tol_position(_tol_position), tol_heading(_tol_heading)
{

}
