#include "Radar.hpp"

using namespace std;

namespace base {
    namespace samples {
        Radar::Radar()
        {
            time = base::Time::now();
        }

        Radar::Radar(int16_t v_heading, int16_t v_range)
            : heading(v_heading)
            , range(v_range)
        {
            time = base::Time::now();
        }

        Radar::~Radar()
        {
        }

        void Radar::addEcho(uint8_t* echo_data,
            uint16_t sweep_length,
            uint16_t angle,
            uint16_t range,
            uint16_t scale)
        {
            Echo echo;
            echo.timestamp = base::Time::now();
            echo.returns = std::vector<uint8_t>(&echo_data[0], &echo_data[sweep_length]);
            echo.sweep_length = sweep_length;
            echo.angle = angle;
            echo.range = range;
            echo.scale = scale;
            echoes.push_back(echo);
            echo_count++;
        }

        void Radar::clear()
        {
            echo_count = 0;
            echoes.clear();
            time = base::Time::now();
        }

        void Radar::setHeading(int16_t v_heading)
        {
            heading = v_heading;
        }

        void Radar::setRange(int16_t v_range)
        {
            range = v_range;
        }
    }
} // end namespace base::samples
