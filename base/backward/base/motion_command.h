#ifndef __MOTION_COMMAND__
#define __MOTION_COMMAND__

#include <base/commands/AUVMotion.hpp>
#include <base/commands/AUVPosition.hpp>
#include <base/commands/Motion2D.hpp>
#include <base/commands/Speed6D.hpp>

namespace base
{
    typedef commands::AUVMotion AUVMotionCommand;
    typedef commands::AUVPosition AUVPositionCommand;
    typedef commands::Motion2D MotionCommand2D;
    typedef commands::Speed6D SpeedCommand6D;
}

#endif
