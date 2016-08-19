#include <boost/test/unit_test.hpp>
#include <base/Eigen.hpp>
#include <iostream>
#include "../src/TwistWithCovariance.cpp"

BOOST_AUTO_TEST_SUITE(Eigen)


template<class Matrix>
void test_SPD_helper(const Matrix& A, double precision)
{
    typedef typename Matrix::PlainObject Mat;
    Mat B = base::guaranteeSPD(A+A.transpose(), precision);
    Eigen::SelfAdjointEigenSolver<Mat> eig(B);
    BOOST_CHECK((eig.eigenvalues().array()>=0.0).all());

}

BOOST_AUTO_TEST_CASE(test_SPD)
{
    test_SPD_helper(Eigen::Matrix4f::Random(), 1e-6);
    test_SPD_helper(Eigen::Matrix3d::Random(), 1e-15);
    test_SPD_helper(Eigen::MatrixXf::Random(10,10), 1e-6);
    test_SPD_helper(Eigen::MatrixXd::Random(10,10), 1e-15);
}

BOOST_AUTO_TEST_SUITE_END()
