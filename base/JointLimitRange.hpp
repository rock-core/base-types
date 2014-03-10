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
            return ss.str();
	}
	
	std::string name;
	double min, max, value;
        OutOfBounds()
            : std::runtime_error( std::string() ), min(0), max(0), value(0) {}
	OutOfBounds( std::string name, double min, double max, double value )
	    : std::runtime_error( errorString( name, min, max, value ) )
            , name( name )
            , min( min ), max( max ), value( value )
	{
	}
	~OutOfBounds() throw() {}
    };

    /** 
     * Checks that the provided JointState is within limits for the values that are valid
     *
     * @throw OutOfBoundsException if limits are exceeded. 
     */
    void validate( const JointState &state ) const
    {
        std::pair<bool, OutOfBounds> check = verifyValidity(state);
        if (check.first)
            throw check.second;
    }

    /** 
     * Checks that the provided JointState is within limits for the values that are valid
     *
     * @return true if the check passes, false otherwise
     */
    bool isValid( const JointState &state ) const
    {
        std::pair<bool, OutOfBounds> check = verifyValidity(state);
        return check.first;
    }

    /** Creates a JointLimitRange structure with the position range set to \c
     * min, \c max
     */
    static JointLimitRange Position(double min, double max)
    {
        JointLimitRange result;
        result.min.position = min;
        result.max.position = max;
        return result;
    }
        
    /** Creates a JointLimitRange structure with the speed range set to \c
     * min, \c max
     */
    static JointLimitRange Speed(double min, double max)
    {
        JointLimitRange result;
        result.min.speed = min;
        result.max.speed = max;
        return result;
    }

    /** Creates a JointLimitRange structure with the effort range set to \c
     * min, \c max
     */
    static JointLimitRange Effort(double min, double max)
    {
        JointLimitRange result;
        result.min.effort = min;
        result.max.effort = max;
        return result;
    }

    /** Creates a JointLimitRange structure with the raw range set to \c
     * min, \c max
     */
    static JointLimitRange Raw(double min, double max)
    {
        JointLimitRange result;
        result.min.raw = min;
        result.max.raw = max;
        return result;
    }

private:
    /** Internal helper method for validate and isValid */
    std::pair<bool, OutOfBounds> verifyValidity( const JointState& state ) const
    {
        using std::make_pair;
	if( state.hasPosition() )
	{
	    if( min.hasPosition() && min.position > state.position )
		return make_pair(false, OutOfBounds( "position", min.position, max.position, state.position ));
	    if( max.hasPosition() && max.position < state.position )
		return make_pair(false, OutOfBounds( "position", min.position, max.position, state.position ));
	}

	if( state.hasSpeed() )
	{
	    if( min.hasSpeed() && min.speed > state.speed )
		return make_pair(false, OutOfBounds( "speed", min.speed, max.speed, state.speed ));
	    if( max.hasSpeed() && max.speed < state.speed )
		return make_pair(false, OutOfBounds( "speed", min.speed, max.speed, state.speed ));
	}

	if( state.hasEffort() )
	{
	    if( min.hasEffort() && min.effort > state.effort )
		return make_pair(false, OutOfBounds( "effort", min.effort, max.effort, state.effort ));
	    if( max.hasEffort() && max.effort < state.effort )
		return make_pair(false, OutOfBounds( "effort", min.effort, max.effort, state.effort ));
	}

	if( state.hasRaw() )
	{
	    if( min.hasRaw() && min.raw > state.raw )
		return make_pair(false, OutOfBounds( "raw", min.raw, max.raw, state.raw ));
	    if( max.hasRaw() && max.raw < state.raw )
		return make_pair(false, OutOfBounds( "raw", min.raw, max.raw, state.raw ));
	}
        return make_pair(true, OutOfBounds());
    }
};

}

#endif
