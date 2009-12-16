#ifndef IMU_READINGS_H__
#define IMU_READINGS_H__ 

#include <dfki/time.h>
#include <dfki/linear_algebra.h>

namespace DFKI {    
    struct IMUReading {
         /** Timestamp of the orientation reading */
        Time stamp;
        
        /** The orientation quaternion */
        DFKI::Quaternion orientation;

        /** calibrated accelerometer readings */
        DFKI::Vector3 acc;

        /** calibrated gyro reading*/
        DFKI::Vector3 gyro;

        /** calibrated magnetometer reading*/
        DFKI::Vector3 mag;
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
