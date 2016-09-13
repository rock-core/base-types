#include "Motion2D.hpp"

namespace base { namespace commands {

bool operator==(const Motion2D& lhs, const Motion2D& rhs)
{
    return lhs.translation == rhs.translation && lhs.rotation == rhs.rotation && lhs.heading == rhs.heading;
}

bool operator!=(const Motion2D& lhs, const Motion2D& rhs)
{
    return !(lhs == rhs);
}

}} // end namespace base::commands

