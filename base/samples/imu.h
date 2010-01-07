#ifndef BASE_SAMPLES_IMU_H__
#define BASE_SAMPLES_IMU_H__ 

#ifdef __orogen
#error "this header cannot be used in orogen-parsed code. Use wrappers/samples/imu.h and wrappers::IMUSensors instead"
#endif

#include <base/time.h>
#include <Eigen/Geometry>

namespace base { namespace samples {
    struct IMUSensors
    {
         /** Timestamp of the orientation reading */
        Time time;
        
        /** raw accelerometer readings */
        Eigen::Vector3d acc;

        /** raw gyro reading*/
        Eigen::Vector3d gyro;

        /** raw magnetometer reading*/
        Eigen::Vector3d mag;
    };
}} // namespaces

#endif

