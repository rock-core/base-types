#ifndef ORIENTATION_H__
#define ORIENTATION_H__ 

#ifndef __orogen
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#endif

#include <dfki/base_types.h>

namespace DFKI {
    
    /** opaque types for orogon */
    struct Quaternion {
        double x;
        double y;
        double z;
        double w;
    };

    struct Vector3 {
        double x;
        double y;
        double z;
    };

    struct OrientationReading {
         /** Timestamp of the orientation reading */
        Time stamp;
        
        /** The orientation quaternion */
        Eigen::Quaterniond value;
    };

    struct AccelerometerReading {
        /** Timestamp of the orientation reading */
        Time stamp;
  
        /** calibrated accelerometer readings */
        Eigen::Vector3d value;
    };

    struct AngularRateReading {
        /** Timestamp of the orientation reading */
        Time stamp;
  
        /** calibrated gyro reading*/
        Eigen::Vector3d value;
    };

    struct MagnetometerReading {
        /** Timestamp of the orientation reading */
        Time stamp;
  
        /** calibrated magnetometer reading*/
        Eigen::Vector3d value;
    };

}

#endif
