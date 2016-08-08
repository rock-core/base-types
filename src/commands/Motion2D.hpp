#ifndef BASE_COMMANDS_MOTION2D
#define BASE_COMMANDS_MOTION2D

#include "base/Angle.hpp"

namespace base
{
    namespace commands
    {

    /** A unified motion control data structure for differential drive-based
     *  robots.
     */
    struct Motion2D
    {
        double translation; //! translation value in m/s
        double rotation;    //! rotation value in rad/s. Positive is counter-clockwise
        base::Angle heading;     //! heading in rad. Positive is counter-clockwise
        
        Motion2D():translation(0), rotation(0), heading(base::Angle::fromRad(0)){};
        Motion2D(double translation, double rotation):translation(translation), rotation(rotation), heading(base::Angle::fromRad(0)){};
        Motion2D(double translation, double rotation, base::Angle heading):translation(translation), rotation(rotation), heading(heading){};
    };
    bool operator==(const Motion2D& lhs, const Motion2D& rhs);
    bool operator!=(const Motion2D& lhs, const Motion2D& rhs);

    }
}

#endif

