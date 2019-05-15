#ifndef BASE_SAMPLES_POSE_WITH_COVARIANCE_HPP
#define BASE_SAMPLES_POSE_WITH_COVARIANCE_HPP

#include <string>
#include <base/Time.hpp>
#include <base/TransformWithCovariance.hpp>
#include <base/samples/RigidBodyState.hpp>

namespace base { namespace samples {

/**
 * Represents a 3D pose with uncertainty of the frame 'object_frame_id' in the frame 'frame_id'.
 * A frame ID shall be globally unique and correspond to a coordinate system.
 * Following the [Rock's conventions](http://rock.opendfki.de/wiki/WikiStart/Standards)
 * coordinate systems are right handed with X forward, Y left and Z up.
 *
 * The pose \f$ ^{reference}_{object}T \f$ in this structure transforms a vector \f$ v_{object} \f$
 * from the object frame to the reference frame:
 * \f$ v_{reference} = ^{reference}_{object}T \cdot v_{object} \f$
 * Example of a transformation chain:
 * \f$ ^{world}_{sensor}T = ^{world}_{body}T \cdot ^{body}_{sensor}T \f$
 */
class PoseWithCovariance
{
public:
    /** Reference timestamp of the pose sample */
    base::Time time;

    /** ID of the reference frame where this pose is expressed in */
    std::string frame_id;

    /** ID of the object frame defined by this pose */
    std::string object_frame_id;

    /** Pose of the object frame in the reference frame */
    base::TransformWithCovariance transform;

public:
    /** Default constructor */
    PoseWithCovariance();

    /** Initializes type from a TransformWithCovariance */
    explicit PoseWithCovariance(const base::TransformWithCovariance& transform);

    /** Initializes type from a RigidBodyState */
    explicit PoseWithCovariance(const base::samples::RigidBodyState& rbs);

    /** Performs a composition of this pose with a given pose.
    * The result is another pose with result = this * pose.
    * The frame ID's will be set accordingly (e.g. a_in_b = c_in_b * a_in_c).
    * Note that this method doesn't check if the composition
    * is frame wise valid.
    */
    PoseWithCovariance operator*(const PoseWithCovariance& pose) const;

    /** Sets the transformation as TransformWithCovariance */
    void setTransform(const base::TransformWithCovariance& transform);

    /** Sets the transformation as Eigen::Affine3d */
    void setTransform(const Eigen::Affine3d& transform);

    /** Returns transformation as TransformWithCovariance */
    const base::TransformWithCovariance& getTransformWithCovariance() const;

    /** Returns transformation as Eigen::Affine3d */
    Eigen::Affine3d getTransform() const;

    /** Returns the 6x6 covariance matrix */
    const base::TransformWithCovariance::Covariance& getCovariance() const;

    /** Converts the type to a RigidBodyState.
     * Note that the cross-covariances will be lost,
     * since the RigidBodyState does not store them.
     */
    base::samples::RigidBodyState toRigidBodyState() const;
};

}}

#endif