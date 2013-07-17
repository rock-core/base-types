#ifndef BASE_JOINT_LIMIT_RANGE_HPP
#define BASE_JOINT_LIMIT_RANGE_HPP

#include <base/JointState.hpp>
#include <sstream>

namespace base
{

struct JointLimitRange
{
    JointState min;
    JointState max;

    struct OutOfBounds : public std::runtime_error
    {
	static std::string errorString( std::string name, double min, double max, double value )
	{
	    std::ostringstream ss;
	    ss << "The " << name << " value " << value 
		<< " was outside the allowed bounds [" << min << ":" << max << "].";
	    std::string s(ss.str());
	}
	
	std::string name;
	double min, max, value;
	OutOfBounds( std::string name, double min, double max, double value )
	    : std::runtime_error( errorString( min, max, value ) ),
	    name( name ),
	    min( min ), max( max ), value( value )
	{
	}
	~OutOfBounds() throw() {}
    };

    /** 
     * Checks that the provided JointState is within limits for the values that are valid
     *
     * will throw OutOfBoundsException if limits are exceeded. 
     */
    void assert( const JointState &state ) const
    {
	if( state.hasPosition() )
	{
	    if( min.hasPosition() && min.position > state.position )
		throw OutOfBounds( "position", min.position, max.position, state.position );
	    if( max.hasPosition() && max.position < state.position )
		throw OutOfBounds( "position", min.position, max.position, state.position );
	}

	if( state.hasVelocity() )
	{
	    if( min.hasVelocity() && min.velocity > state.velocity )
		throw OutOfBounds( "velocity", min.velocity, max.velocity, state.velocity );
	    if( max.hasVelocity() && max.velocity < state.velocity )
		throw OutOfBounds( "velocity", min.velocity, max.velocity, state.velocity );
	}

	if( state.hasEffort() )
	{
	    if( min.hasEffort() && min.effort > state.effort )
		throw OutOfBounds( "effort", min.effort, max.effort, state.effort );
	    if( max.hasEffort() && max.effort < state.effort )
		throw OutOfBounds( "effort", min.effort, max.effort, state.effort );
	}

	if( state.hasRaw() )
	{
	    if( min.hasRaw() && min.raw > state.raw )
		throw OutOfBounds( "raw", min.raw, max.raw, state.raw );
	    if( max.hasRaw() && max.raw < state.raw )
		throw OutOfBounds( "raw", min.raw, max.raw, state.raw );
	}
    }
};

}

#endif
