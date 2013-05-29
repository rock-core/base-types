#ifndef BASE_FLOAT_H
#define BASE_FLOAT_H

#include <boost/math/special_functions/fpclassify.hpp>

namespace base {
    template<typename T> T NaN() { return std::numeric_limits<T>::quiet_NaN(); }
    template<typename T> bool isNaN(T value) { return boost::math::isnan(value); }
    template<typename T> T unset() { return std::numeric_limits<T>::quiet_NaN(); }
    template<typename T> bool isUnset(T value) { return boost::math::isnan(value); }
    template<typename T> T unknown() { return std::numeric_limits<T>::quiet_NaN(); }
    template<typename T> bool isUnknown(T value) { return boost::math::isnan(value); }
    template<typename T> T infinity() { return std::numeric_limits<T>::infinity(); }
    template<typename T> bool isInfinity(T value) { return boost::math::isinf(value); }
}

#endif
