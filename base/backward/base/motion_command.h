#ifndef __MOTION_COMMAND__
#define __MOTION_COMMAND__

#define ROCK_DEPRECATED_HEADER_SINCE 0
#warning "motion_command.h has been split into specific headers (one per type), and the types renamed:"
#warning "   AUVMotionCommand is now commands::AUVMotion in base/commands/AUVMotion.hpp"
#warning "   AUVPositionCommand is now commands::AUVPosition in base/commands/AUVPosition.hpp"
#warning "   MotionCommand2D is now commands::Motion2D in base/commands/Motion2D.hpp"
#warning "   SpeedCommand6D is now commands::Speed6D in base/commands/Speed6D.hpp"
#include <base/DeprecatedHeader.hpp>

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
