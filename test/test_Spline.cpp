#include <boost/test/unit_test.hpp>
#include <base/Eigen.hpp>
#include <iostream>
#include <base/geometry/Spline.hpp>


BOOST_AUTO_TEST_SUITE(SplineTests)

using namespace std;
using namespace base::geometry;

BOOST_AUTO_TEST_CASE(test_interpolate_with_nonempty_curve_updates_the_start_and_end_param_properly)
{
    Spline<1> spline;
    spline.interpolate({1.0, 2.0}, {3.0, 4.0});
    BOOST_CHECK_EQUAL(3, spline.getStartParam());
    BOOST_CHECK_EQUAL(4, spline.getEndParam());
}

BOOST_AUTO_TEST_CASE(test_interpolate_with_singleton_curve_updates_the_start_and_end_param_properly)
{
    Spline<1> spline;
    spline.interpolate(std::vector<double>{1.0}, std::vector<double>{3.0});
    BOOST_CHECK_EQUAL(3, spline.getStartParam());
    BOOST_CHECK_EQUAL(3, spline.getEndParam());
}

BOOST_AUTO_TEST_CASE(test_interpolate_with_no_points_sets_the_start_and_end_to_zero)
{
    Spline<1> spline;
    spline.interpolate(std::vector<double>(), std::vector<double>());
    BOOST_CHECK_EQUAL(0, spline.getStartParam());
    BOOST_CHECK_EQUAL(0, spline.getEndParam());
}

BOOST_AUTO_TEST_CASE(test_interpolate_accepts_points_and_parameters)
{
    auto points = vector<double>{1.0, 2.0, 3.0};
    auto parameters = vector<double>{1.0, 1.8, 2.0};
    auto types = {
        SplineBase::ORDINARY_POINT,
        SplineBase::ORDINARY_POINT,
        SplineBase::ORDINARY_POINT
    };

    Spline<1> spline;
    spline.interpolate(points, parameters, types);
    auto p0 = spline.getPoint(1.0);
    auto p1 = spline.getPoint(1.8);
    auto p2 = spline.getPoint(2.0);
    BOOST_CHECK_CLOSE(1.0, p0(0), 1e-3);
    BOOST_CHECK_CLOSE(2.0, p1(0), 1e-3);
    BOOST_CHECK_CLOSE(3.0, p2(0), 1e-3);
}

BOOST_AUTO_TEST_CASE(test_interpolate_accepts_derivatives_and_parameters)
{
    auto points = vector<double>{1.0, 0.0, 2.0, 0.0};
    auto parameters = vector<double>{1.0, 2.0};
    auto types = {
        SplineBase::ORDINARY_POINT,
        SplineBase::DERIVATIVE_TO_PRIOR,
        SplineBase::ORDINARY_POINT,
        SplineBase::DERIVATIVE_TO_PRIOR
    };

    Spline<1> spline;
    spline.interpolate(points, parameters, types);
    auto p0 = spline.getPointAndTangent(1.0);
    auto p1 = spline.getPointAndTangent(2.0);
    BOOST_CHECK_CLOSE(1.0, p0.first(0), 1e-3);
    BOOST_CHECK_CLOSE(0.0, p0.second(0), 1e-3);
    BOOST_CHECK_CLOSE(2.0, p1.first(0), 1e-3);
    BOOST_CHECK_CLOSE(0.0, p1.second(0), 1e-3);
}


BOOST_AUTO_TEST_CASE(test_interpolate_takes_dimension_into_account_to_validate_the_number_of_parameters)
{
    Spline<2> spline;
    spline.interpolate({1.0, 2.0}, std::vector<double>{1.0});
    BOOST_CHECK_EQUAL(1, spline.getStartParam());
    BOOST_CHECK_EQUAL(1, spline.getEndParam());
}

BOOST_AUTO_TEST_CASE(test_interpolate_accepts_not_being_given_any_parameter)
{
    Spline<2> spline;
    spline.interpolate({1.0, 2.0, 3.0, 4.0});
    BOOST_CHECK_EQUAL(0, spline.getStartParam());
    BOOST_CHECK_EQUAL(Eigen::Vector2d(1.0, 2.0), spline.getPoint(0));
    BOOST_CHECK_EQUAL(Eigen::Vector2d(3.0, 4.0), spline.getPoint(spline.getEndParam()));
}

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

