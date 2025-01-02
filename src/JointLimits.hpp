#ifndef BASE_JOINT_LIMITS_HPP
#define BASE_JOINT_LIMITS_HPP

#include <base/samples/Joints.hpp>
#include <base/JointLimitRange.hpp>

namespace base
{
    struct JointLimits : public NamedVector<JointLimitRange>
    {
        bool isValid( const base::samples::Joints& joints ) const;

	/** 
	 * Makes sure that all jointstate are within their respective limits
	 *
	 * Will throw if this is not the case. Will also throw if there are no
	 * limits for a particular joint.
	 */
	void validate( const base::samples::Joints& joints ) const;

    /**
     * Saturates all jointstate when they are out of their respective limits
     *
     * @return a pair with the status (isSaturated) and the saturated_joints
     */
    std::pair<bool, samples::Joints> saturate(const samples::Joints& joints);
    };
}

#endif
