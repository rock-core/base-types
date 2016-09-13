#ifndef BASE_JOINT_TRANSFORM_HPP
#define BASE_JOINT_TRANSFORM_HPP

#include <string>
#include <base/Eigen.hpp>
#include <base/NamedVector.hpp>
#include <base/samples/Joints.hpp>
#include <base/samples/RigidBodyState.hpp>

namespace base
{

/** 
 * defines the frame transformation for a joint,
 * and the rotation axis
 */
struct JointTransform
{
    std::string sourceFrame;
    std::string targetFrame;
    base::Vector3d rotationAxis;
};

/** 
 * Vector of JointTranformations with added functionality
 * to fill a vector of RigidBodyStates given a jointsState sample
 */
struct JointTransformVector : public base::NamedVector<JointTransform>
{
    /** 
     * will fill the rbs structure with the information from the joints and 
     * the JointTransform configuration.
     */
    void setRigidBodyStates( const base::samples::Joints& joints, std::vector<base::samples::RigidBodyState>& rbs ) const;
};

}

#endif
