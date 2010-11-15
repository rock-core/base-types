#ifndef BASE_ACTUATORS_STATUS_HH
#define BASE_ACTUATORS_STATUS_HH

namespace base {
    namespace actuators {
        struct ErrorState {
            bool motorOverheated;
            bool boardOverheated;
            bool overCurrent;
            bool timeout;
            bool badConfig;
            bool encodersNotInitalized;
            bool hardwareShutdown;
#ifndef __orogen

            ErrorState() :
                motorOverheated(false), boardOverheated(false), overCurrent(false), timeout(false), badConfig(false), encodersNotInitalized(false), hardwareShutdown(false)
            {}
            
            bool hasError() 
            {
                return (motorOverheated || boardOverheated || overCurrent || timeout || badConfig || encodersNotInitalized || hardwareShutdown);
            }
                    
            void printError()
            {
                std::cout << "MotorOverheated      :" << motorOverheated << std::endl;
                std::cout << "boardOverheated      :" << boardOverheated << std::endl;
                std::cout << "overCurrent          :" << overCurrent << std::endl;
                std::cout << "timeout              :" << timeout << std::endl;
                std::cout << "badConfig            :" << badConfig << std::endl;
                std::cout << "encodersNotInitalized:" << encodersNotInitalized << std::endl;
                std::cout << "hardwareShutdown     :" << hardwareShutdown << std::endl;
            }
            
            ErrorState merge(ErrorState const& es) const
            {
                ErrorState ret;
                ret.motorOverheated = this->motorOverheated || es.motorOverheated;
                ret.boardOverheated = this->boardOverheated || es.boardOverheated;
                ret.overCurrent     = this->overCurrent || es.overCurrent;
                ret.timeout         = this->timeout || es.timeout;
                ret.badConfig       = this->badConfig || es.badConfig;
                ret.encodersNotInitalized = this->encodersNotInitalized || es.encodersNotInitalized;
                ret.hardwareShutdown = this->hardwareShutdown || es.hardwareShutdown;
                return ret;
            }
#endif
        };
    
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
            struct ErrorState error; //! indicates the current status of the actuator
        };

        /** Synchronized set of actuator states */
        template<int count>
        struct Status {
            base::Time time;
            uint64_t index;    //! index of this packet, i.e. the position of this packet since the beginning
            MotorState states[count];
        };

        //! Workaround for a bug in GCCXML where the typedef is not enough to
        // get the definition of Status<4>
        struct internalStatusInstanciator { Status<4> s4; };
        typedef Status<4> FourWheeled;
    }
}

#endif


