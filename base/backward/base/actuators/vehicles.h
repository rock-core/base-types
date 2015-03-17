#ifndef BASE_TYPES_ACTUATORS_VEHICLES_HH
#define BASE_TYPES_ACTUATORS_VEHICLES_HH

#define ROCK_DEPRECATED_HEADER_SINCE 0
#warning "the actuators:: functionality has been replaced by base::samples::Joints and base::commands::Joints"
#include <base/DeprecatedHeader.hpp>

namespace base {
    namespace actuators {
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

