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
	    static const double WHEEL_RADIUS_EFF    = 0.178; //(to be calculated) effective wheel radius for one rotation. [m]
	    static const double TRACK		    = 0.515; // distance between the centers of the left and right wheels. [m]  
	    static const double WHEEL_BASE	    = 0.510; // Distance between the centers of the front and rear wheels. [m]

	    enum WHEEL
	    {
		REAR_LEFT   = 0,
		REAR_RIGHT  = 1,
		FRONT_RIGHT = 2,
		FRONT_LEFT  = 3
	    };
	};
	
	class MotorConstants 
	{
	public:
	    static const double  _2PI_5		=1.2566370614;  // Angle between wheel legs
	    static const double  _PI_5		=0.6283185307;  // Half of angle between wheel legs

	    static const double  POS_P		=3.80; 		// Position proportional gain
	    static const double  VEL_I		=0.65; 		// Velocity integral gain
	    static const double  VEL_P		=0.07; 		// Velocity proportional gain
	    static const double  VEL_SMOOTH	=0.6; 		// Velocity smoothing factor 0 to 1
	    
	    static const double  VEL_FF		=1.00; 		// Velocity feed forward
	    static const double  ACC_FF		=0.00; 		// Acceleration feed forward 

	    static const double  INT_WIND_UP    = 0.06; 	// Integrator windup coefficient
	};
    }
}

#endif

