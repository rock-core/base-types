#ifndef BASE_JOINT_LIMITS_HPP
#define BASE_JOINT_LIMITS_HPP

#include <base/samples/Joints.hpp>
#include <base/JointLimitRange.hpp>
#include <base/NamedVector.hpp>

namespace base
{
    struct JointLimits : public NamedVector<JointLimitRange>
    {
        bool isValid( const base::samples::Joints& joints ) const
        {
            if (joints.hasNames())
            {
                for( size_t i=0; i<joints.size(); i++ )
                    if (! (*this)[joints.names[i]].isValid( joints[i] ))
                        return false;
            }
            else
            {
                for( size_t i=0; i<joints.size(); i++ )
                    if (! (*this)[i].isValid( joints[i] ))
                        return false;
            }
            return true;
        }

	/** 
	 * Makes sure that all jointstate are within their respective limits
	 *
	 * Will throw if this is not the case. Will also throw if there are no
	 * limits for a particular joint.
	 */
	void validate( const base::samples::Joints& joints ) const
	{
            if (joints.hasNames())
            {
                for( size_t i=0; i<joints.size(); i++ )
                    (*this)[joints.names[i]].validate( joints[i] );
            }
            else
            {
                for( size_t i=0; i<joints.size(); i++ )
                    (*this)[i].validate( joints[i] );
            }
	}
    };
}

#endif
