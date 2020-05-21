// Copyright 2020 Rock-Core
#ifndef BASE_SAMPLES_BOUNDINGBOX_HPP_
#define BASE_SAMPLES_BOUNDINGBOX_HPP_

#include <base/Eigen.hpp>
#include <base/Float.hpp>
#include <base/Time.hpp>

namespace base {
namespace samples {
/** Axis-aligned Bounding Box (ABB) representation
 *
 * The data structure is used to represent the three-dimensional Axis-aligned
 * Bounding Box (x, y, z, w, h, l) with the associated uncertainty, where x, y
 * and z are the center position of the bounding box and w, h and l are the
 * bounding box width, height and length.
 */
struct BoundingBox {
    explicit BoundingBox(
        const Time& time = Time(),
        const Vector3d& position = Vector3d::Ones() * unknown<double>(),
        const Vector3d& dimension = Vector3d::Ones() * unknown<double>());

    /**
     * Initialize the structure fields.
     */
    void initBoundingBox(const Time& time,
                         const Vector3d& position,
                         const Vector3d& dimension);

    /**
     * Returns true if the position and the dimension of the bounding box are
     * different of NaN.
     */
    bool hasValidBoundingBox() const;

    /**
     * Returns true if the position of the bounding box is different of NaN.
     */
    bool hasValidPosition() const;

    /**
     * Returns true if the dimension of the bounding box is different of NaN.
     */
    bool hasValidDimension() const;

    /**
     * Returns true if the covariance associated to the position and the
     * dimension of the bounding box are different of NaN.
     */
    bool hasValidCovariance() const;

    /**
     * Returns true if the covariance associated to the position is different
     * of NaN.
     */
    bool hasValidCovPosition() const;

    /**
     * Returns true if the covariance associated to the dimension is different
     * of NaN.
     */
    bool hasValidCovDimension() const;

    /* The sample timestamp. */
    Time time;

    /* 3D center (x, y, z) position of the bounding box . */
    Vector3d position;

    /* 3D dimension (width, height, length) of the bounding box. */
    Vector3d dimension;

    /* The position uncertainty represented as a 3x3 matrix. */
    Matrix3d cov_position;

    /* The dimension uncertainty represented as a 3x3 matrix. */
    Matrix3d cov_dimension;
};

}  // namespace samples
}  // namespace base

#endif  // BASE_SAMPLES_ORIENTEDBOUDINGBOX_HPP_
