#ifndef BASE_ACTUATORS_COMMANDS_H
#define BASE_ACTUATORS_COMMANDS_H

namespace base {
    namespace actuators {
        /** The control mode as requested by a given output.
         */
        enum DRIVE_MODE
        {
            DM_PWM = 0, //! direct duty control
            DM_SPEED = 1, //! speed control
            DM_POSITION = 2, //! position control
            DM_UNINITIALIZED = 3
        };

        /** Synchronized set of commands for a set of actuators
         */
        template<int count>
        struct Command {
            base::Time time;

            DRIVE_MODE mode[count];   //! one of DM_PWM, DM_SPEED, DM_POSITION
            double     target[count]; //! speeds are in rad/s, positions in rad and PWM in [-1, 1]
        };

        //! Workaround for a bug in GCCXML where the typedef is not enough to
        // get the definition of Command<4>
        struct __gccxml_workaround_CommandInstanciator { Command<4> c4; };
        typedef Command<4> FourWheelCommand;
    }
}

#endif
