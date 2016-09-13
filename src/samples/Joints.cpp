#include "Joints.hpp"

namespace base { namespace samples {

Joints Joints::Positions(const std::vector< double >& positions)
{
    Joints result;
    result.elements.resize(positions.size());
    for (std::size_t i = 0; i != positions.size(); ++i)
        result[i].position = positions[i];
    return result;
}

Joints Joints::Positions(const std::vector< double >& positions, const std::vector< std::string >& names)
{
    Joints result = Positions(positions);
    if (result.elements.size() != names.size())
        throw std::runtime_error("the position and names vectors differ");
    result.names = names;
    return result;
}

Joints Joints::Speeds(const std::vector< float >& speeds)
{
    Joints result;
    result.elements.resize(speeds.size());
    for (std::size_t i = 0; i != speeds.size(); ++i)
        result[i].speed= speeds[i];
    return result;
}

Joints Joints::Speeds(const std::vector< float >& speeds, const std::vector< std::string >& names)
{
    Joints result = Speeds(speeds);
    if (result.elements.size() != names.size())
        throw std::runtime_error("the speeds and names vectors differ");
    result.names = names;
    return result;
}

Joints Joints::Efforts(const std::vector< float >& efforts)
{
    Joints result;
    result.elements.resize(efforts.size());
    for (std::size_t i = 0; i != efforts.size(); ++i)
        result[i].effort = efforts[i];
    return result;
}

Joints Joints::Efforts(const std::vector< float >& efforts, const std::vector< std::string >& names)
{
    Joints result = Efforts(efforts);
    if (result.elements.size() != names.size())
        throw std::runtime_error("the effort and names vectors differ");
    result.names = names;
    return result;
}

Joints Joints::Raw(const std::vector< float >& raw)
{
    Joints result;
    result.elements.resize(raw.size());
    for (std::size_t i = 0; i != raw.size(); ++i)
        result[i].raw = raw[i];
    return result;
}

Joints Joints::Raw(const std::vector< float >& raw, const std::vector< std::string >& names)
{
    Joints result = Raw(raw);
    if (result.elements.size() != names.size())
        throw std::runtime_error("the raw and names vectors differ");
    result.names = names;
    return result;
}

Joints Joints::Accelerations(const std::vector< float >& acceleration)
{
    Joints result;
    result.elements.resize(acceleration.size());
    for (std::size_t i = 0; i != acceleration.size(); ++i)
        result[i].acceleration = acceleration[i];
    return result;
}

Joints Joints::Accelerations(const std::vector< float >& acceleration, const std::vector< std::string >& names)
{
    Joints result = Accelerations(acceleration);
    if (result.elements.size() != names.size())
        throw std::runtime_error("the acceleration and names vectors differ");
    result.names = names;
    return result;
}

}} //end namespace base::samples


