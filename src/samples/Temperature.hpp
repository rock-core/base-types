#ifndef __BASE_SAMPLES_TEMPERATURE_HH__
#define __BASE_SAMPLES_TEMPERATURE_HH__

#include <base/Temperature.hpp>
#include <base/Time.hpp>

namespace base
{
    namespace samples
    {
        struct Temperature : public base::Temperature
        {
            /**
             * The sample timestamp
             *
             * @meta role logical_time */
            base::Time time;

            Temperature() : base::Temperature() { }

            Temperature(base::Time const& time, base::Temperature temp)
                : base::Temperature(temp.getKelvin())
                , time(time) { }

            static Temperature fromKelvin(base::Time const& time, double kelvin);

            static Temperature fromCelsius(base::Time const& time, double celsius);

        };
    }
}

#endif
