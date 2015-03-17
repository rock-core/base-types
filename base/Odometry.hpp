#ifndef __BASE_ODOMETRY_HPP__
#define __BASE_ODOMETRY_HPP__

#define ROCK_DEPRECATED_HEADER_SINCE 0
#warning "the headers base/odometry.h and base/Odometry.hpp are deprecated. Include the corresponding headers from the slam/odometry package, and update the namespaces:"
#warning "   base::odometry::State is now odometry::State in odometry/State.hpp"
#warning "   base::odometry::Gaussian2D is now odometry::Gaussian2D in odometry/Gaussian2D.hpp"
#warning "   base::odometry::Gaussian3D is now odometry::Gaussian3D in odometry/Gaussian3D.hpp"
#warning "   base::odometry::Sampling2D is now odometry::Sampling2D in odometry/Sampling2D.hpp"
#warning "   base::odometry::Sampling3D is now odometry::Sampling3D in odometry/Sampling3D.hpp"
#include <base/DeprecatedHeader.hpp>

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
