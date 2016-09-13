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

            static Joints Positions(std::vector<double> const& positions);

            static Joints Positions(std::vector<double> const& positions, std::vector<std::string> const& names);

            static Joints Speeds(std::vector<float> const& speeds);
                

            static Joints Speeds(std::vector<float> const& speeds, std::vector<std::string> const& names);

            static Joints Efforts(std::vector<float> const& efforts);

            static Joints Efforts(std::vector<float> const& efforts, std::vector<std::string> const& names);

            static Joints Raw(std::vector<float> const& raw);

            static Joints Raw(std::vector<float> const& raw, std::vector<std::string> const& names);

            static Joints Accelerations(std::vector<float> const& acceleration);

            static Joints Accelerations(std::vector<float> const& acceleration, std::vector<std::string> const& names);
        };
    }
}

#endif

