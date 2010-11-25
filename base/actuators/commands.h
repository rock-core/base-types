#ifndef BASE_ACTUATORS_COMMANDS_H
#define BASE_ACTUATORS_COMMANDS_H

#include <vector>
#include <base/time.h>

namespace base {
    namespace actuators {
        /** Structure holding PID values
         *
         * The actual meaning of those values will obviously depend on the
         * actuator and control electronics.
         */
        struct PIDValues{
            float kp;
            float ki;
            float kd;
            float maxPWM;

            PIDValues() : kp(0), ki(0), kd(0), maxPWM(0) {};
        };

        /** The control mode requested for a controller output
         */
        enum DRIVE_MODE
        {
            DM_PWM = 0, //! direct duty control
            DM_SPEED = 1, //! speed control
            DM_POSITION = 2, //! position control
            DM_UNINITIALIZED = 3
        };

        /** Synchronized set of commands for a set of actuators
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
        };

        enum ADAPTATIVE_MODE
        {
            PID_POSITION = 64,
            PID_SPEED    = 128
        };

        struct AdaptativeCommand {
            double     target;

            int mode; //! OR-ed set of DRIVE_MODE and ADAPTATIVE_MODE
            PIDValues  pid_position;
            PIDValues  pid_speed;

            AdaptativeCommand()
                : target(0), mode(DM_UNINITIALIZED) {}
        };

        /** Synchronized set of adaptive commands for a set of actuators
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
