#ifndef BASE_SAMPLES_JOINTS_HPP
#define BASE_SAMPLES_JOINTS_HPP

#include <stdexcept>
#include <vector>

#include <base/Time.hpp>
#include <base/JointState.hpp>
#include <base/NamedVector.hpp>

namespace base
{
    namespace samples
    {
        /** Data structure that gives out state readings for a set of joints
         */
        struct Joints : public base::NamedVector<JointState>
        {
            /** The sample timestamp */
            base::Time time;

            static Joints Positions(std::vector<double> const& positions)
            {
                Joints result;
                result.elements.resize(positions.size());
                for (std::size_t i = 0; i != positions.size(); ++i)
                    result[i].position = positions[i];
                return result;
            }

            static Joints Positions(std::vector<double> const& positions, std::vector<std::string> const& names)
            {
                Joints result = Positions(positions);
                if (result.elements.size() != names.size())
                    throw std::runtime_error("the position and names vectors differ");
                result.names = names;
                return result;
            }

            static Joints Speeds(std::vector<float> const& speeds)
            {
                Joints result;
                result.elements.resize(speeds.size());
                for (std::size_t i = 0; i != speeds.size(); ++i)
                    result[i].speed= speeds[i];
                return result;
            }

            static Joints Speeds(std::vector<float> const& speeds, std::vector<std::string> const& names)
            {
                Joints result = Speeds(speeds);
                if (result.elements.size() != names.size())
                    throw std::runtime_error("the speeds and names vectors differ");
                result.names = names;
                return result;
            }

            static Joints Efforts(std::vector<float> const& efforts)
            {
                Joints result;
                result.elements.resize(efforts.size());
                for (std::size_t i = 0; i != efforts.size(); ++i)
                    result[i].effort = efforts[i];
                return result;
            }

            static Joints Efforts(std::vector<float> const& efforts, std::vector<std::string> const& names)
            {
                Joints result = Efforts(efforts);
                if (result.elements.size() != names.size())
                    throw std::runtime_error("the effort and names vectors differ");
                result.names = names;
                return result;
            }

            static Joints Raw(std::vector<float> const& raw)
            {
                Joints result;
                result.elements.resize(raw.size());
                for (std::size_t i = 0; i != raw.size(); ++i)
                    result[i].raw = raw[i];
                return result;
            }

            static Joints Raw(std::vector<float> const& raw, std::vector<std::string> const& names)
            {
                Joints result = Raw(raw);
                if (result.elements.size() != names.size())
                    throw std::runtime_error("the raw and names vectors differ");
                result.names = names;
                return result;
            }

            static Joints Accelerations(std::vector<float> const& acceleration)
            {
                Joints result;
                result.elements.resize(acceleration.size());
                for (std::size_t i = 0; i != acceleration.size(); ++i)
                    result[i].acceleration = acceleration[i];
                return result;
            }

            static Joints Accelerations(std::vector<float> const& acceleration, std::vector<std::string> const& names)
            {
                Joints result = Accelerations(acceleration);
                if (result.elements.size() != names.size())
                    throw std::runtime_error("the acceleration and names vectors differ");
                result.names = names;
                return result;
            }
        };
    }
}

#endif

