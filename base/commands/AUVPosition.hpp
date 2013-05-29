#ifndef BASE_COMMANDS_AUVPOSITION
#define BASE_COMMANDS_AUVPOSITION

namespace base
{
    namespace commands
    {

    /** @deprecated */
    struct AUVPosition
    {
        double heading;    //! absolute heading, in radians (positive
                           //! counter-clockwise, has to be in -PI/PI)
        double z;          //! absolute altitude, in meters (goes positive upwards)
        double x;    	   //! X Position in Pool 0,0 means middle of the Pool
        double y;          //! Y Position

        AUVPosition()
            : heading(0), z(0), x(0), y(0) {}
    };
    }
}
#endif

