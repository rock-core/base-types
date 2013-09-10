#ifndef BASE_COMMANDS_MOTION2D
#define BASE_COMMANDS_MOTION2D

namespace base
{
    namespace commands
    {

    /** A unified motion control data structure for differential drive-based
     *  robots.
     */
    struct Motion2D
    {
        double translation; //! translation value in m/s
        double rotation;    //! rotation value in rad/s. Positive is counter-clockwise
        // That's actually enough information for a differential drive,
        // for an omnidirectional drive we would add a heading/angle
        // value.
    };

    }
}

#endif

