#ifndef __BASE_SAMPLES_PRESSURE_HPP
#define __BASE_SAMPLES_PRESSURE_HPP

#include <base/Pressure.hpp>
#include <base/Time.hpp>
#include <base/Float.hpp>

namespace base
{
    namespace samples
    {
        /** Timestamped pressure */
        struct Pressure : public base::Pressure
        {
            /** The sample timestamp */
            base::Time time;

            Pressure()
                : base::Pressure(base::Pressure::fromPascal(base::unknown<float>())) {}

            Pressure(base::Time const& time, base::Pressure pressure)
                : base::Pressure(pressure)
                , time(time) {}

            static Pressure fromPascal(base::Time const& time, float pascal);

            static Pressure fromBar(base::Time const& time, float bar);

            static Pressure fromPSI(base::Time const& time, float psi);
        };
    }
}

#endif

