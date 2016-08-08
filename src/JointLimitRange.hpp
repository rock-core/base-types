#ifndef BASE_JOINT_LIMIT_RANGE_HPP
#define BASE_JOINT_LIMIT_RANGE_HPP

#include <base/JointState.hpp>

namespace base
{

struct JointLimitRange
{
    JointState min;
    JointState max;

    struct OutOfBounds : public std::runtime_error
    {
	static std::string errorString( std::string name, double min, double max, double value );
	
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
    void validate( const JointState &state ) const;

    /** 
     * Checks that the provided JointState is within limits for the values that are valid
     *
     * @return true if the check passes, false otherwise
     */
    bool isValid( const JointState &state ) const;

    /** Creates a JointLimitRange structure with the position range set to \c
     * min, \c max
     */
    static JointLimitRange Position(double min, double max);
        
    /** Creates a JointLimitRange structure with the speed range set to \c
     * min, \c max
     */
    static JointLimitRange Speed(double min, double max);

    /** Creates a JointLimitRange structure with the effort range set to \c
     * min, \c max
     */
    static JointLimitRange Effort(double min, double max);

    /** Creates a JointLimitRange structure with the raw range set to \c
     * min, \c max
     */
    static JointLimitRange Raw(double min, double max);

    /** Creates a JointLimitRange structure with the acceleration range set to \c
     * min, \c max
     */
    static JointLimitRange Acceleration(double min, double max);
    

private:
    /** Internal helper method for validate and isValid */
    std::pair<bool, OutOfBounds> verifyValidity( const JointState& state ) const;
  
};

}

#endif
