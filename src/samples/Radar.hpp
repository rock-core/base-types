#ifndef __BASE_SAMPLES_RADAR_HPP__
#define __BASE_SAMPLES_RADAR_HPP__

#include <base/Time.hpp>
#include <vector>

namespace base {
    namespace samples {

        struct Echo {
            base::Time timestamp;
            std::vector<uint8_t> returns;
            uint16_t sweep_length;
            uint16_t angle;
            int16_t range;
            int16_t scale;
        };

        struct Radar {
        public:
            int echo_count = 0;
            std::vector<Echo> echoes;
            int16_t range;   // range scale (0 to 21)
            int16_t heading; // heading correction value (relative angle of heading) (0 to
                             // 8191)
            base::Time time;

            Radar();

            Radar(int16_t v_heading, int16_t range);

            ~Radar();

            void clear();

            void addEcho(uint8_t* echo_data,
                uint16_t sweep_length,
                uint16_t angle,
                uint16_t range,
                uint16_t scale);

            void setRange(int16_t v_range);

            void setHeading(int16_t v_heading);
        };

    }
} // namespaces

#endif
