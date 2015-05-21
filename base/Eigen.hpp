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

    typedef Eigen::Quaternion<double, Eigen::DontAlign>    Quaterniond;
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

    // Guarantee Semi-Positive Definite (SPD) matrix.
    template <typename _MatrixType>
    static _MatrixType guaranteeSPD (const _MatrixType &A)
    {
        _MatrixType spdA;
        Eigen::VectorXd s;
        s.resize(A.rows(), 1);

        /**
        * Single Value Decomposition
        */
        Eigen::JacobiSVD <Eigen::MatrixXd > svdOfA (A, Eigen::ComputeThinU | Eigen::ComputeThinV);

        s = svdOfA.singularValues(); //!eigenvalues

        for (register int i=0; i<s.size(); ++i)
        {
            if (s(i) < 0.00)
                s(i) = 0.00;
        }
        spdA = svdOfA.matrixU() * s.matrix().asDiagonal() * svdOfA.matrixV().transpose();

        return spdA;
    };

}

#endif

