#include <boost/test/unit_test.hpp>
#include <base/Eigen.hpp>
#include <iostream>
#include <base/geometry/Spline.hpp>


BOOST_AUTO_TEST_SUITE(Spline)

using base::geometry::Spline;

BOOST_AUTO_TEST_CASE(test_derive)
{
    std::vector<double> coordinates;
    coordinates.push_back(0);
    coordinates.push_back(1);
    coordinates.push_back(0);
    coordinates.push_back(-1);
    coordinates.push_back(0);
    coordinates.push_back(1);
    Spline<1> spline;
    spline.interpolate(coordinates);

    Spline<1> dSpline = spline.derive(1);

    double t0 = spline.getStartParam();
    double t1 = spline.getEndParam();
    double t_delta = (t1 - t0) / 100;
    for (double t = t0; t < t1; t += t_delta)
    {
        std::pair<
            Spline<1>::vector_t,
            Spline<1>::vector_t> point_and_tangent = spline.getPointAndTangent(t);
        BOOST_CHECK_CLOSE(point_and_tangent.second(0), dSpline.getPoint(t)(0), 1e-6);
    }
}

BOOST_AUTO_TEST_CASE(test_derive_singleton_raises)
{
    std::vector<double> coordinates;
    coordinates.push_back(0);
    Spline<1> spline;
    spline.interpolate(coordinates);
    BOOST_REQUIRE_THROW(spline.derive(1), std::invalid_argument);
}

BOOST_AUTO_TEST_SUITE_END()

