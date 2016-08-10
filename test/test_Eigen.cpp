#include <boost/test/unit_test.hpp>
#include <base/Eigen.hpp>
#include <iostream>

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

BOOST_AUTO_TEST_CASE(test_allClose_same)
{
    Eigen::Matrix3d a;
    a << 1, 2, 3,
         4, 5, 6,
         7, 8, 9;
    const Eigen::Matrix3d b = a;
    BOOST_CHECK(base::allClose(a,b));
}

BOOST_AUTO_TEST_CASE(test_allClose_different)
{
    Eigen::Matrix3d a;
    a << 1, 2, 3,
         4, 5, 6,
         7, 8, 9;
    Eigen::Matrix3d b;
    BOOST_CHECK(!base::allClose(a,b));
}

BOOST_AUTO_TEST_CASE(test_allClose_zero)
{
    Eigen::Matrix3d a;
    Eigen::Matrix3d b;
    a.setZero();
    b.setZero();
    BOOST_CHECK(base::allClose(a,b));
}

BOOST_AUTO_TEST_CASE(test_allClose_nan)
{
    //From the gnu c manual:
    //NaN is unordered: it is not equal to, greater than, or less than anything, including itself.
    //thus allClose should always return false as soon as one of the matrices contains NaN.
    Eigen::Matrix3d a;
    Eigen::Matrix3d nan;
    a.setZero();
    nan << 1,   2,   3,
           NAN, NAN, NAN,
           4,   5,   6;
    BOOST_CHECK(!base::allClose(a, nan));
    BOOST_CHECK(!base::allClose(nan, a));
    BOOST_CHECK(!base::allClose(nan, nan));
    
    Eigen::Matrix3d inf;
    inf.setConstant(INFINITY);
    BOOST_CHECK(!base::allClose(nan, inf));
    BOOST_CHECK(!base::allClose(inf, nan));
    
    inf *= -1;
    BOOST_CHECK(!base::allClose(nan, inf));
    BOOST_CHECK(!base::allClose(inf, nan));

}

BOOST_AUTO_TEST_CASE(test_allClose_inf)
{

    Eigen::Matrix3d a;   
    Eigen::Matrix3d inf;
    inf.setConstant(INFINITY);
    BOOST_CHECK(!base::allClose(a, inf));
    BOOST_CHECK(!base::allClose(inf, inf));
}


BOOST_AUTO_TEST_SUITE_END()
