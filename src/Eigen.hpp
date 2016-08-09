#ifndef __BASE_EIGEN_HH__
#define __BASE_EIGEN_HH__

#include <Eigen/Core>
#include <Eigen/Geometry> 


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
    typedef Eigen::Matrix<double, 4, 1, Eigen::DontAlign>     Vector4d;
    typedef Eigen::Matrix<double, 6, 1, Eigen::DontAlign>     Vector6d;
    typedef Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::DontAlign> 
                                                              VectorXd;

    typedef Eigen::Matrix<double, 2, 2, Eigen::DontAlign>     Matrix2d;
    typedef Eigen::Matrix<double, 3, 3, Eigen::DontAlign>     Matrix3d;
    typedef Eigen::Matrix<double, 4, 4, Eigen::DontAlign>     Matrix4d;
    typedef Eigen::Matrix<double, 6, 6, Eigen::DontAlign>     Matrix6d;
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::DontAlign> 
                                                              MatrixXd;

    typedef Eigen::Quaternion<double, Eigen::DontAlign>   Quaterniond;
    typedef Eigen::AngleAxis<double>    AngleAxisd;
    typedef Eigen::Transform<double, 3, Eigen::Affine, Eigen::DontAlign> Affine3d;
    typedef Eigen::Transform<double, 3, Eigen::Isometry, Eigen::DontAlign> Isometry3d;

    // alias for backward compatibility
    typedef Affine3d					   Transform3d;

    /**
     * @brief Check if NaN values
     */
    template<typename _Derived>
    static inline bool isnotnan(const Eigen::MatrixBase<_Derived>& x)
    {
        return ((x.array() == x.array())).all();
    };

    template<typename _Derived>
    static inline bool isfinite(const Eigen::MatrixBase<_Derived>& x)
    {
        return isnotnan(x - x);
    };

}

#endif

