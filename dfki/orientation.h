#ifndef ORIENTATION_H__
#define ORIENTATION_H__ 

#ifndef __orogen
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#endif

#include <dfki/base_types.h>

namespace DFKI {

    typedef struct {
        double x;
        double y;
        double z;
        double w;
    } Quaternion;

    struct Orientation {

        /** Timestamp of the orientation reading */
        Time stamp;
        
        /** The orientation quaternion */
        Eigen::Quaterniond q;

#ifndef __orogen
        Orientation()
            : q(Eigen::Quaterniond::Identity()) {}
#endif
    };
}

#endif
