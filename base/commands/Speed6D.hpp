#ifndef BASE_COMMANDS_SPEED6D_HPP
#define BASE_COMMANDS_SPEED6D_HPP

namespace base
{
    namespace commands
    {

    /** A unified speed control data structure for 6-dof vehicles (e.g. AUVs)
     */
    struct Speed6D
    {
        double surge; //! forward speed in m/s
        double sway;  //! rightward speed (aka strafing) in m/s
        double heave; //! downward speed in m/s

        double roll;  //! rotation about the surge (forward/x) axis, posit. makes vehicle roll clockwise (to the right)
        double pitch; //! rotation about the sway (rightward/y) axis, posit. makes vehicle's nose rise
        double yaw;   //! rotation about the heave (downward/z) axis, posit. makes vehicle turn right
    };
    }
}

#endif

