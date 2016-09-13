#include "Trajectory.hpp"

namespace base {

Trajectory::Trajectory() : speed(0)
{

}

bool Trajectory::driveForward() const
{
    return speed >= 0;
}

} //end namespace base