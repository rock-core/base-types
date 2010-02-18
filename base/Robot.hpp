#ifndef ROBOT_HPP
#define ROBOT_HPP

namespace base
{
    namespace robot
    {
	class FourWheelGeom
	{
	public:
	    static const double WIDTH		    = 0.300; // To be removed [m]
	    static const double ROTATION_RADIUS	    = 0.300; // To be removed [m]

	    static const double WHEEL_RADIUS_MAX    = 0.190; // radius at the location of the spike[m]
	    static const double WHEEL_RADIUS_EFF    = 0.170; //(to be calculated) effective wheel radius for one rotation[m]
	    static const double TRACK		    = 0.515; // distance between the centers of the left and right wheels.[m]  
	    static const double WHEEL_BASE	    = 0.510; // Distance between the centers of the front and rear wheels. [m]

	    enum WHEEL
	    {
		REAR_LEFT   = 0,
		REAR_RIGHT  = 1,
		FRONT_RIGHT = 2,
		FRONT_LEFT  = 3
	    };
	};
    }
}

#endif

