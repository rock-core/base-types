#ifndef BASE_ACTUATORS_COMMANDS_H
#define BASE_ACTUATORS_COMMANDS_H

#define ROCK_DEPRECATED_HEADER_SINCE 0
#warning "the actuators:: functionality has been replaced by base::samples::Joints and base::commands::Joints, in addition an equivalent of the PIDValues structure can now be found in the control/motor_controller package"
#include <base/DeprecatedHeader.hpp>

#include <vector>
#include <base/Time.hpp>

namespace base {
    namespace actuators {
        /** \deprecated in Rock, use the PIDSettings class from the control/motor_controller package
         *
         * Structure holding PID values
         *
         * The actual meaning of those values will obviously depend on the
         * actuator and control electronics.
         *
         */
        struct PIDValues{
            float kp;
            float ki;
            float kd;
            float maxPWM;

            PIDValues() : kp(0), ki(0), kd(0), maxPWM(0) {};
        };

        /** \deprecated See the documentation of Command
         * The control mode requested for a controller output
         *
         */
        enum DRIVE_MODE
        {
            DM_PWM = 0, //! direct duty control
            DM_SPEED = 1, //! speed control
            DM_POSITION = 2, //! position control
            DM_UNINITIALIZED = 3
        };

        /** \deprecated use base::commands::Joints
         *
         * Synchronized set of commands for a set of actuators
         *
         * Since this type contains std::vector, one must preallocate it
         * and resize the mode and target vectors accordingly before
         * updateHook().
         */
        struct Command {
            base::Time time;

            std::vector<DRIVE_MODE> mode;   //! one of DM_PWM, DM_SPEED, DM_POSITION
            std::vector<double>     target; //! speeds are in rad/s, positions in rad and PWM in [-1, 1]

            void resize(int size)
            {
                mode.resize(size);
                std::fill(mode.begin(), mode.end(), DM_UNINITIALIZED);
                target.resize(size);
                std::fill(target.begin(), target.end(), 0);
            }
            
            void invert(int pos)
	    {
		if (mode[pos] == DM_POSITION)
		    target[pos] = 2 * M_PI - target[pos];
		else
		    target[pos] *= -1;
	    }
        };

        /** \deprecated */
        enum ADAPTATIVE_MODE
        {
            PID_POSITION = 64,
            PID_SPEED    = 128
        };

        /** \deprecated */
        struct AdaptativeCommand {
            double     target;

            int mode; //! OR-ed set of DRIVE_MODE and ADAPTATIVE_MODE
            PIDValues  pid_position;
            PIDValues  pid_speed;

            AdaptativeCommand()
                : target(0), mode(DM_UNINITIALIZED) {}
        };

        /** \deprecated
         *
         * Synchronized set of adaptive commands for a set of actuators
         *
         * Since this type contains std::vector, one must preallocate it
         * and resize the mode and target vectors accordingly before
         * updateHook().
         */
        struct AdaptativeCommands {
            base::Time time;
            std::vector<AdaptativeCommand> commands;

            void resize(int size)
            {
                commands.resize(size);
            }
        };
    }
}

#endif
