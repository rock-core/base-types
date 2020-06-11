// Copyright 2020 Rock-Core
#include "Waypoint.hpp"

namespace base {

Waypoint::Waypoint()
    : position(Position::Identity()),
      heading(0),
      tol_position(0),
      tol_heading(0) {
}

Waypoint::Waypoint(base::Vector3d const& position,
                   double heading)
    : position(position),
      heading(heading),
      tol_position(0),
      tol_heading(0) {
}

Waypoint::Waypoint(base::Vector3d const& position,
                   double heading,
                   double tol_position,
                   double tol_heading)
    : position(position),
      heading(heading),
      tol_position(tol_position),
      tol_heading(tol_heading) {
}

Waypoint::Waypoint(Eigen::Vector3d const& position,
                   double heading,
                   double tol_position,
                   double tol_heading)
    : position(position),
      heading(heading),
      tol_position(tol_position),
      tol_heading(tol_heading) {
}

bool Waypoint::hasValidPosition() const {
    return position.allFinite();
}

}  // namespace base
