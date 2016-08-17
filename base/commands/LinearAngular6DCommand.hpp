#ifndef BASE_COMMANDS_LINEAR_ANGULAR_6D_COMMAND_HPP_
#define BASE_COMMANDS_LINEAR_ANGULAR_6D_COMMAND_HPP_

#include <base/Time.hpp>
#include <base/Eigen.hpp>
#include <base/Float.hpp>

namespace base
{
    namespace commands
    {
        /** Common command structure for all controller types, in all control frames */
        struct LinearAngular6DCommand
        {
            /** The command timestamp */
            base::Time time;
            /** The linear part of the command, as (x,y,z) */
            base::Vector3d linear;
            /** The angular part of the command, as (r,p,y) */
            base::Vector3d angular;

            LinearAngular6DCommand(){
                for(int i = 0; i < 3; i++){
                    linear(i) = base::unset<double>();
                    angular(i) = base::unset<double>();
                }
            }

            double& x() { return linear(0); }
            double& y() { return linear(1); }
            double& z() { return linear(2); }
            double& roll() { return angular(0); }
            double& pitch() { return angular(1); }
            double& yaw() { return angular(2); }

            double x() const { return linear(0); }
            double y() const { return linear(1); }
            double z() const { return linear(2); }
            double roll() const { return angular(0); }
            double pitch() const { return angular(1); }
            double yaw() const { return angular(2); }
        };
    }

    // For barckwards compatibility only
    typedef base::commands::LinearAngular6DCommand LinearAngular6DCommand;
}

#endif
