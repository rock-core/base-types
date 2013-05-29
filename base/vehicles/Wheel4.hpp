#ifndef BASE_TYPES_VEHICLES_WHEEL4_HH
#define BASE_TYPES_VEHICLES_WHEEL4_HH

namespace base {
    namespace vehicles {
        /** Standard joint order for all 4-wheeled systems */
        enum WHEEL4_ACTUATORS {
            WHEEL4_REAR_LEFT  = 0,
            WHEEL4_REAR_RIGHT = 1,
            WHEEL4_FRONT_RIGHT = 2,
            WHEEL4_FRONT_LEFT = 3
        };
    }
}

#endif

