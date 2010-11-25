#ifndef BASE_ACTUATORS_STATUS_HH
#define BASE_ACTUATORS_STATUS_HH

#include <base/time.h>
#include <vector>

namespace base {
    namespace actuators {
        /** Represents the state of a single actuator
         *
         * +current+ is the current reading
         *
         * +position+ is the position of the actuated part (i.e. motor + gear),
         * in radians. Wether this position is absolute or relative depends on
         * the type of encoder on the particular actuator it represents.
         *
         * +positionExtern+ is used in cases where an elastic coupling exists
         * between the motor and the actuated part (for instance, the wheel). In
         * this case, \c position is before the elasticity and \c positionExtern
         * after
         *
         * +pwm+ is the current duty-cycle
         *
         * +error+ is a bitfield representing the actuator state.
         */
        struct MotorState {
            int      current;  //! current in mA
            double   position; //! position in radians
            double   positionExtern; //! position of external encoder in radians
            float    pwm;      //! duty cycle in [-1; 1]

            MotorState()
                : current(0), position(0), positionExtern(0), pwm(0) {}
        };

        /** Synchronized set of actuator states */
        struct Status {
            base::Time time;
            uint64_t index;    //! index of this packet, i.e. the position of this packet since the beginning
            std::vector<MotorState> states;

            void resize(int size)
            {
                states.resize(size);
            }
        };
    }
}

#endif


