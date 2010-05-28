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


	
}


#endif