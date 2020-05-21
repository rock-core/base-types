// Copyright 2020 Rock-Core
#ifndef BASE_SAMPLES_ORIENTEDBOUNDINGBOX_HPP_
#define BASE_SAMPLES_ORIENTEDBOUNDINGBOX_HPP_

#include <base/Pose.hpp>
#include <base/samples/BoundingBox.hpp>

namespace base {
namespace samples {
/** Oriented Bounding Box (OBB) representation.
 *
 * The data structure represents a three-dimensional Oriented Bounding Box
 * (OBB) with the associated uncertainty. It is derived from
 * base::samples::BoundingBox and includes the rotation information with
 * respect to the bounding box center position.
 *
 * @see base::samples::BoundingBox
 */
struct OrientedBoundingBox : public BoundingBox {
    explicit OrientedBoundingBox(
        const Time& = Time(),
        const Vector3d& position = Vector3d::Ones() * unknown<double>(),
        const Vector3d& dimension = Vector3d::Ones() * unknown<double>(),
        const Orientation& orientation = Orientation(
            Vector4d::Ones() * unknown<double>()));

    /**
     * Initialize the structure fields with Vector4d to represent the
     * orientation.
     */
    void initOrientedBoundingBox(const Time& time,
                                 const Vector3d& position,
                                 const Vector3d& dimension,
                                 const Orientation& orientation);

    /**
     * Returns true if the position, dimension and orientation of the bounding
     * box are different of NaN.
     */
    bool hasValidBoundingBox() const;
    /**
     * Returns true if the orientation of the bounding box is different of NaN.
     */
    bool hasValidOrientation() const;
    /**
     * Returns true if the covariance associated to the position, dimension and
     * orientation of the bounding box are different of NaN.
     */
    bool hasValidCovariance() const;

    /**
     * Returns true if the covariance associated to the orientation is different
     * of NaN.
     */
    bool hasValidCovOrientation() const;

    /* Bounding box rotation with respect its center position. */
    Orientation orientation;

    /* The orientation uncertainty represented as a 3x3 matrix. */
    Matrix3d cov_orientation;
};

}  // namespace samples
}  // namespace base

#endif  // BASE_SAMPLES_ORIENTEDBOUNDINGBOX_HPP_
