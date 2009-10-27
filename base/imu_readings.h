#ifndef ORIENTATION_H__
#define ORIENTATION_H__ 

#include <base/time.h>
#include <base/linear_algebra.h>

namespace base {    
    struct OrientationReading {
         /** Timestamp of the orientation reading */
        Time stamp;
        
        /** The orientation quaternion */
        base::Quaternion value;
    };

    struct AccelerometerReading {
        /** Timestamp of the orientation reading */
        Time stamp;
  
        /** calibrated accelerometer readings */
        base::Vector3 value;
    };

    struct AngularRateReading {
        /** Timestamp of the orientation reading */
        Time stamp;
  
        /** calibrated gyro reading*/
        base::Vector3 value;
    };

    struct MagnetometerReading {
        /** Timestamp of the orientation reading */
        Time stamp;
  
        /** calibrated magnetometer reading*/
        base::Vector3 value;
    };

}

#endif
