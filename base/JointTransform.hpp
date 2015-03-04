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
    void setRigidBodyStates( const base::samples::Joints& joints, std::vector<base::samples::RigidBodyState>& rbs ) const
    {
        if (joints.names.empty()) {
            throw std::runtime_error("base::JointTransformVector::"
                                     "setRigidBodyStates(): the vector "
                                     "'joints.names()' is empty");
        }

	rbs.resize( joints.size() );
	for( size_t i=0; i<joints.size(); i++ )
	{
            if (!joints[i].hasPosition()) {
                std::stringstream ss;
                ss << "base::JointTransformVector::setRigidBodyStates(): "
                      "the joint 'joints[" << i << "]' has no position";
                throw std::runtime_error(ss.str());
            }

            const JointTransform &jt( getElementByName( joints.names[i] ) );

	    rbs[i].time = joints.time;
	    rbs[i].sourceFrame = jt.sourceFrame;
	    rbs[i].targetFrame = jt.targetFrame;
	    rbs[i].setTransform( Eigen::Isometry3d( Eigen::AngleAxisd( joints[i].position, jt.rotationAxis ) ) );
	}
    }
};

}

#endif
