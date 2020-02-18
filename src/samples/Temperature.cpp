#include "Temperature.hpp"

namespace base { namespace samples{

    Temperature Temperature::fromKelvin(base::Time const& time, double kelvin)
    {
        return Temperature(time, base::Temperature::fromKelvin(kelvin));
    }

    Temperature Temperature::fromCelsius(base::Time const& time, double celsius)
    {
        return Temperature(time, base::Temperature::fromCelsius(celsius));
    }

} } //end namespace base::samples
