#ifndef __MOTION_COMMAND__
#define __MOTION_COMMAND__

namespace base {

	/** A unified motion control data structure for differential drive-based
     *  robots.
     */
    struct MotionCommand2D
    {
        double translation; //! translation value in m/s
        double rotation;    //! rotation value in rad/s. Positive is counter-clockwise
        // That's actually enough information for a differential drive,
        // for an omnidirectional drive we would add a heading/angle
        // value.
    };

    struct AUVMotionCommand
    {
        double heading;    //! absolute heading, in radians (positive
                           //! counter-clockwise, has to be in -PI/PI)
        double z;          //! absolute altitude, in meters (goes positive upwards)
        double x_speed;    //! desired forward speed, in m/s
        double y_speed;    //! desired left speed, in m/s

#ifndef __orogen
        AUVMotionCommand()
            : heading(0), z(0), x_speed(0), y_speed(0) {}
#endif
    };
}


#endif
