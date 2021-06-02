#ifndef __BASE_SAMPLES_EVENT_ARRAY_HH
#define __BASE_SAMPLES_EVENT_ARRAY_HH

#include <base/samples/Event.hpp>
#include <vector>

namespace base {
namespace samples {

    struct EventArray
    {
        ::base::Time time;
        uint16_t height;
        uint16_t width;

        std::vector<Event> events;
    };
}  // end namespace samples
}  // end namespace base

#endif
