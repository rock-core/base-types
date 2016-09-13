#include "JointTransform.hpp"

namespace base {

void JointTransformVector::setRigidBodyStates(const samples::Joints& joints, std::vector< samples::RigidBodyState >& rbs) const
{
    if (joints.names.empty()) {
        throw std::runtime_error("JointTransformVector::"
                                    "setRigidBodyStates(): the vector "
                                    "'joints.names()' is empty");
    }

    rbs.resize( joints.size() );
    for( size_t i=0; i<joints.size(); i++ )
    {
        if (!joints[i].hasPosition()) {
            std::stringstream ss;
            ss << "JointTransformVector::setRigidBodyStates(): "
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

} //end namespace base