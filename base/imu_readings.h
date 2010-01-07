#ifndef IMU_READINGS_H__
#define IMU_READINGS_H__ 

#include <base/time.h>
#include <base/linear_algebra.h>

namespace base {    
    struct IMUReading {
         /** Timestamp of the orientation reading */
        Time stamp;
        
        /** The orientation quaternion */
        base::Quaternion orientation;

        /** calibrated accelerometer readings */
        base::Vector3 acc;

        /** calibrated gyro reading*/
        base::Vector3 gyro;

        /** calibrated magnetometer reading*/
        base::Vector3 mag;
    };

    struct IMURawReading {
         /** Timestamp of the orientation reading */
        Time stamp;
        
        /** raw accelerometer readings */
        DFKI::Vector3 acc;

        /** raw gyro reading*/
        DFKI::Vector3 gyro;

        /** raw magnetometer reading*/
        DFKI::Vector3 mag;
    };
}

#endif
