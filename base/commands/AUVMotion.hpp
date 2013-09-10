#ifndef BASE_COMMANDS_AUVMOTION
#define BASE_COMMANDS_AUVMOTION

namespace base
{
    namespace commands
    {

    /** @deprecated */
    struct AUVMotion
    {
        double heading;    //! absolute heading, in radians (positive
                           //! counter-clockwise, has to be in -PI/PI)
        double z;          //! absolute altitude, in meters (goes positive upwards)
        double x_speed;    //! desired forward speed, in m/s
        double y_speed;    //! desired left speed, in m/s

        AUVMotion()
            : heading(0), z(0), x_speed(0), y_speed(0) {}
    };

    }
}

#endif

