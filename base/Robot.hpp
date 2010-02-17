#ifndef ROBOT_HPP
#define ROBOT_HPP

namespace base
{
    namespace robot
    {
	class FourWheelGeom
	{
	public:
	    static const double WIDTH           = 0.300; // [m]
	    static const double ROTATION_RADIUS = 0.300; // [m]
	    static const double WHEEL_RADIUS    = 0.150; // [m]

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

