#ifndef ORIENTATION_H__
#define ORIENTATION_H__ 

#ifndef __orogen
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#endif

#include <dfki/base_types.h>

namespace DFKI {

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

    struct IMUReading {

        /** Timestamp of the orientation reading */
        Time stamp;
        
        /** The orientation quaternion */
        Eigen::Quaterniond orientation;

        /** calibrated gyro readings */
        Eigen::Vector3d gyro;

        /** calibrated accelerometer readings */
        Eigen::Vector3d accel;

        /** calibrated magnetometer readings */
        Eigen::Vector3d mag;

#ifndef __orogen
        IMUReading()
            : orientation(Eigen::Quaterniond::Identity()), 
            gyro(Eigen::Vector3d::Identity()), 
            accel(Eigen::Vector3d::Identity()), 
            mag(Eigen::Vector3d::Identity()) {}
#endif
    };
}

#endif
