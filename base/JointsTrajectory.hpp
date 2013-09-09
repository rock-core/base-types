#ifndef BASE_JOINTSTRAJECTORY_HPP
#define BASE_JOINTSTRAJECTORY_HPP

#include <vector>
#include <base/JointState.hpp>
#include <base/NamedVector.hpp>
#include <base/Time.hpp>

namespace base {

/** 
 * @brief Holds a time-series of JointStates for multiple joints
 *
 * This structure holds a time-series in the form of std::vector of JointState,
 * for multiple optionally named joints. 
 *
 * The time series can have optional time information attached to each sample
 * in the series through the times member.
 *
 * For the data-type to be valid, the length of each time series needs to be
 * the same for all joints. The times vector needs to be either empty or also
 * of that size
 */
struct JointsTrajectory 
    : public NamedVector<std::vector<JointState> >
{
    /**
     * @brief optional array of time values corresponding to the samples of the JointState
     *
     * This vector needs to be either empty or the same size as each of the
     * std::vector<JointState> in the elements vector.
     */
    std::vector<base::Time> times;

    /** @return true if the structure is valid
     */
    bool isValid()
    {
	size_t samples = 0;
	if( !elements.empty() )
	    samples = elements[0].size();

	for(size_t i=0; i<elements.size(); ++i)
	{
	    if( elements[i].size() != size )
		return false;
	}

	if( !times.empty() && times.size() != samples )
	    return false;

	return true;
    }

    /**
     * @return true if the JointState series has timing information
     */
    bool isTimed()
    {
	return !times.empty();
    }
   
    /** 
     * @return the total duration of the time series if time information is available
     */
    base::Time duration()
    {
	base::Time summed;
	for(size_t i=0; i<times.size(); i++)
	{
	    summed = summed+samples[i];
	}
	return summed;
    }
}; 

}

#endif

