#include "Trajectory.hpp"

base::Trajectory::Trajectory() : speed(0)
{

}

bool base::Trajectory::driveForward() const
{
    return speed >= 0;
}

