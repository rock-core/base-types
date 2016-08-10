#ifndef __BASE_EIGEN_HH__
#define __BASE_EIGEN_HH__

#include <Eigen/Core>
#include <Eigen/Geometry> 

#include <Eigen/Eigenvalues>

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

    /**
     * Guarantee Symmetric (semi-) Positive Definite (SPD) matrix.
     * The input matrix must be symmetric already (only the lower triangular part will be considered)
     */
    template <typename _MatrixType>
    static typename _MatrixType::PlainObject guaranteeSPD (const _MatrixType &A, const typename _MatrixType::RealScalar& minEig = 0.0)
    {
        typedef typename _MatrixType::PlainObject Matrix;
        typedef Eigen::SelfAdjointEigenSolver<Matrix> EigenDecomp;
        typedef typename EigenDecomp::RealVectorType EigenValues;

        // Eigenvalue decomposition
        Eigen::SelfAdjointEigenSolver<Matrix> eigOfA (A, Eigen::ComputeEigenvectors);

        // truncate Eigenvalues:
        EigenValues s = eigOfA.eigenvalues();
        s = s.cwiseMax(minEig);

        // re-compose matrix:
        Matrix spdA = eigOfA.eigenvectors() * s.asDiagonal() * eigOfA.eigenvectors().adjoint();

        return spdA;
    };
    
    /** Component wise fuzzy equality check.
     *  This method is similar to numpy.allclose() in python.
     *  @param rtol Relative tolerance.
     *  @param atol Absolut tolerance.
     *  
     *  @return True if @p a and @p b are nearly identical. I.e.
     *          if abs(a - b) <= (atol + rtol * abs(b))
     * 
     *  @note The equation is not symmetric in a and b,
     *        so that allclose(a, b) might be different
     *        from allclose(b, a) in some rare cases.
     *  @note The standard rules for comparision with nan and inf apply. I.e.:
     *        * Comparision with nan is never true. If @p a or @p b contain nan,
     *          the result will be false.
     *        * Everything is smaller than inf, thus the result is false if @p a
     *          or @p b contain inf.
     *        * Comparing inf with itself returns false.*/
    template<typename DerivedA, typename DerivedB>
    static bool allClose(const Eigen::DenseBase<DerivedA>& a,
                         const Eigen::DenseBase<DerivedB>& b,
                         const typename DerivedA::RealScalar& rtol = Eigen::NumTraits<typename DerivedA::RealScalar>::dummy_precision(),
                         const typename DerivedA::RealScalar& atol = Eigen::NumTraits<typename DerivedA::RealScalar>::epsilon())
    {
      return ((a.derived() - b.derived()).array().abs()
              <= (atol + rtol * b.derived().array().abs())).all();
    }

}

#endif

