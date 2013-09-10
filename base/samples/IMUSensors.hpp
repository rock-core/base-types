#ifndef BASE_SAMPLES_IMU_SENSORS_H__
#define BASE_SAMPLES_IMU_SENSORS_H__ 

#include <base/Time.hpp>
#include <base/Eigen.hpp>

namespace base { namespace samples {
    struct IMUSensors
    {
         /** Timestamp of the orientation reading */
        Time time;
        
        /** raw accelerometer readings */
        base::Vector3d acc;

        /** raw gyro reading*/
        base::Vector3d gyro;

        /** raw magnetometer reading*/
        base::Vector3d mag;
    };
}} // namespaces

#endif

