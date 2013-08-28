#ifndef __BASE_ODOMETRY_HPP__
#define __BASE_ODOMETRY_HPP__

#warning "the headers base/odometry.h and base/Odometry.hpp are deprecated. Include the corresponding headers from the slam/odometry package (e.g. odometry/State.hpp for the odometry::State<> template)"
#include <odometry/State.hpp>
#include <odometry/Gaussian2D.hpp>
#include <odometry/Gaussian3D.hpp>
#include <odometry/Sampling2D.hpp>
#include <odometry/Sampling3D.hpp>

namespace base
{
    namespace odometry = ::odometry;
}

#endif
