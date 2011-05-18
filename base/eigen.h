#ifndef __BASE_EIGEN_HH__
#define __BASE_EIGEN_HH__

#include <Eigen/Core>
#include <Eigen/Geometry> 

#include <Eigen/SVD> 

namespace base
{
    // We define these typedefs to workaround alignment requirements for normal
    // Eigen types. This reduces the amount of knowledge people have to have to
    // manipulate these types -- as well as the structures that use them -- and
    // make them usable in Orocos dataflow.
    //
    // Eigen supports converting them to "standard" eigen types in a
    // straightforward way. Moreover, vectorization does not help for small
    // sizes
    typedef Eigen::Matrix<double, 2, 1, Eigen::DontAlign>     Vector2d;
    typedef Eigen::Matrix<double, 3, 1, Eigen::DontAlign>     Vector3d;

    typedef Eigen::Matrix<double, 2, 2, Eigen::DontAlign>     Matrix2d;
    typedef Eigen::Matrix<double, 3, 3, Eigen::DontAlign>     Matrix3d;
    typedef Eigen::Matrix<double, 4, 4, Eigen::DontAlign>     Matrix4d;

    typedef Eigen::Quaternion<double, Eigen::DontAlign>    Quaterniond;
    typedef Eigen::Transform<double, 3, Eigen::Affine, Eigen::DontAlign>	      
							      Affine3d;
    // alias for backward compatibility
    typedef Affine3d					   Transform3d;
}

#endif

