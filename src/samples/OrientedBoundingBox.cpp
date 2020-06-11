// Copyright 2020 Rock-Core
#include "OrientedBoundingBox.hpp"

namespace base {
namespace samples {

OrientedBoundingBox::OrientedBoundingBox(const Time& time,
                                         const Vector3d& position,
                                         const Vector3d& dimension,
                                         const Orientation& orientation) {
    initOrientedBoundingBox(time, position, dimension, orientation);
}

void OrientedBoundingBox::initOrientedBoundingBox(const Time& time,
                                                  const Vector3d& position,
                                                  const Vector3d& dimension,
                                                  const Orientation& orientation) {
    initBoundingBox(time, position, dimension);
    this->orientation = orientation;
    this->cov_orientation = Matrix3d::Ones() * unknown<double>();
}

bool OrientedBoundingBox::hasValidBoundingBox() const {
    return BoundingBox::hasValidBoundingBox() && hasValidOrientation();
}

bool OrientedBoundingBox::hasValidOrientation() const {
    return orientation.toRotationMatrix().allFinite();
}

bool OrientedBoundingBox::hasValidCovariance() const {
    return BoundingBox::hasValidCovariance() && hasValidCovOrientation();
}

bool OrientedBoundingBox::hasValidCovOrientation() const {
    return cov_orientation.allFinite();
}

}  // namespace samples
}  // namespace base
