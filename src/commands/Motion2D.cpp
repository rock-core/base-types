#include "Motion2D.hpp"

bool base::commands::operator==(const base::commands::Motion2D& lhs, const base::commands::Motion2D& rhs)
{
    return lhs.translation == rhs.translation && lhs.rotation == rhs.rotation && lhs.heading == rhs.heading;
}

bool base::commands::operator!=(const base::commands::Motion2D& lhs, const base::commands::Motion2D& rhs)
{
    return !(lhs == rhs);
}

