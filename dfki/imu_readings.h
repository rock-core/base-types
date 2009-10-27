#ifndef ORIENTATION_H__
#define ORIENTATION_H__ 

#include <dfki/time.h>
#include <dfki/linear_algebra.h>

namespace DFKI {    
    struct OrientationReading {
         /** Timestamp of the orientation reading */
        Time stamp;
        
        /** The orientation quaternion */
        DFKI::Quaternion value;
    };

    struct AccelerometerReading {
        /** Timestamp of the orientation reading */
        Time stamp;
  
        /** calibrated accelerometer readings */
        DFKI::Vector3 value;
    };

    struct AngularRateReading {
        /** Timestamp of the orientation reading */
        Time stamp;
  
        /** calibrated gyro reading*/
        DFKI::Vector3 value;
    };

    struct MagnetometerReading {
        /** Timestamp of the orientation reading */
        Time stamp;
  
        /** calibrated magnetometer reading*/
        DFKI::Vector3 value;
    };

}

#endif
