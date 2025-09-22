#ifndef __BASE_SAMPLES_EVENT_HH
#define __BASE_SAMPLES_EVENT_HH

#include <base/Time.hpp>

namespace base {
namespace samples {

    struct Event
    {
        Event():x(0),y(0),ts(::base::Time::fromSeconds(0)),polarity(false){}
        Event(uint16_t x_, uint16_t y_, ::base::Time ts_, uint8_t p_)
            :x(x_),y(y_),ts(ts_),polarity(p_){}

        uint16_t x;
        uint16_t y;
        /** @meta role logical_time */
        ::base::Time ts;
        uint8_t polarity;
    };
}  // end namespace samples
}  // end namespace base

#endif
