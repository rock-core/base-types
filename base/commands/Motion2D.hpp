#ifndef BASE_COMMANDS_MOTION2D
#define BASE_COMMANDS_MOTION2D

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
        double heading;     //! heading in rad. Positive is counter-clockwise
        Motion2D():translation(0), rotation(0), heading(0){};
        Motion2D(double translation, double rotation, double heading):translation(translation), rotation(rotation), heading(heading){};
    };
    inline bool operator==(const Motion2D& lhs, const Motion2D& rhs){ return lhs.translation == rhs.translation && lhs.rotation == rhs.rotation && lhs.heading == rhs.heading;}
    inline bool operator!=(const Motion2D& lhs, const Motion2D& rhs){ return !(lhs == rhs); }

    }
}

#endif

