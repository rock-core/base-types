#pragma once

#include <Eigen/Eigenvalues>

namespace base {
    
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
    
    
    
} // end namespace base