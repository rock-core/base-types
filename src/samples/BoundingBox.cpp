// Copyright 2020 Rock-Core
#include "BoundingBox.hpp"

namespace base {
namespace samples {

BoundingBox::BoundingBox(const Time& time,
                         const Vector3d& position,
                         const Vector3d& dimension) {
    initBoundingBox(time, position, dimension);
}

void BoundingBox::initBoundingBox(const Time& time,
                                  const Vector3d& position,
                                  const Vector3d& dimension) {
    this->time = time;
    this->position = position;
    this->dimension = dimension;
    this->cov_position = Matrix3d::Ones() * unknown<double>();
    this->cov_dimension = Matrix3d::Ones() * unknown<double>();
}

bool BoundingBox::hasValidBoundingBox() const {
    return hasValidPosition() &&
           hasValidDimension();
}

bool BoundingBox::hasValidPosition() const {
    return position.allFinite();
}

bool BoundingBox::hasValidDimension() const {
    return dimension.allFinite();
}

bool BoundingBox::hasValidCovariance() const {
    return hasValidCovPosition() && hasValidCovDimension();
}

bool BoundingBox::hasValidCovPosition() const {
    return cov_position.allFinite();
}

bool BoundingBox::hasValidCovDimension() const {
    return cov_dimension.allFinite();
}

}  // namespace samples
}  // namespace base
