// Copyright 2020 Rock-Core
#ifndef BASE_WAYPOINT_HH_
#define BASE_WAYPOINT_HH_

#include <base/Eigen.hpp>
#include <base/Pose.hpp>

namespace base {
/**
 * Representation of a position associated with a heading in radians.
 */
struct Waypoint {
    /**
     * Position is initialized with identity vector (1, 0, 0).
     * All the other values are initialized with zeros.
     */
    Waypoint();

    /* The default values are initialized with zeros. */
    explicit Waypoint(base::Vector3d const &position,
                      double heading = 0);

    /* use Vector3d */
    Waypoint(base::Vector3d const &position,
             double heading,
             double tol_position,
             double tol_heading);

    /* convenience: same for Eigen::Vector3d */
    Waypoint(Eigen::Vector3d const &position,
             double heading,
             double tol_position,
             double tol_heading);

    /* Three-dimensional position (x, y, z) */
    base::Vector3d position;

    /* heading in radians */
    double heading;

    /* tollerance of the position in meters */
    double tol_position;

    /* tollerance of the heading in radians */
    double tol_heading;

    bool hasValidPosition() const;
};

/**
 * Returns an waypoint object initialized with unknown values.
 */
template <>
inline Waypoint unknown() {
    return Waypoint(
        base::Vector3d(base::Vector3d::Ones() * unknown<double>()),
        unknown<double>(),
        unknown<double>(),
        unknown<double>());
}

}  // namespace base

#endif
