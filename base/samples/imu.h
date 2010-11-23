#ifndef BASE_SAMPLES_IMU_H__
#define BASE_SAMPLES_IMU_H__ 

#ifdef __GCCXML__
#define EIGEN_DONT_VECTORIZE
#endif

#ifdef __orogen
#error "this header cannot be used in orogen-parsed code. Use wrappers/samples/imu.h and wrappers::IMUSensors instead"
#endif

#include <base/time.h>
#include <base/eigen.h>

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

