#include <boost/test/unit_test.hpp>
#include <base/Eigen.hpp>
#include <iostream>
#include <base/geometry/Spline.hpp>
#include <base/Trajectory.hpp>


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

BOOST_AUTO_TEST_CASE(test_derive_beyond_the_degree_returns_a_zero_curve)
{
    Spline<1> spline;
    spline.interpolate(vector<double>{0, 1, 0, 2});
    Spline<1> d = spline.derive(5);
    BOOST_CHECK_EQUAL(0, d.getPoint(spline.getStartParam())(0));
    BOOST_CHECK_EQUAL(0, d.getPoint(spline.getEndParam())(0));
}

// Reference values in the following tests have been computed with SISL,
// which was used to implement the splines before
BOOST_AUTO_TEST_CASE(test_interpolate_uses_the_same_knots_than_sisl)
{
    Spline<1> spline;
    spline.interpolate(vector<double>{0, 1, 2, 3, 4});
    vector<double> expected{0, 0, 0, 1.5, 2.5, 4, 4, 4};
    vector<double> knots = spline.getKnots();
    BOOST_CHECK_EQUAL_COLLECTIONS(expected.begin(), expected.end(), knots.begin(), knots.end());

    Spline<1> spline4(0.1, 4);
    spline4.interpolate(vector<double>{0, 1, 3, 6, 7, 9});
    expected = vector<double>{0, 0, 0, 0, 3, 6, 9, 9, 9, 9};
    knots = spline4.getKnots();
    BOOST_CHECK_EQUAL_COLLECTIONS(expected.begin(), expected.end(), knots.begin(), knots.end());
}

BOOST_AUTO_TEST_CASE(test_interpolate_with_tangent_points)
{
    Spline<3> spline;
    vector<Eigen::Vector3d> points{
        Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0),
        Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(1, 1, 0),
        Eigen::Vector3d(1, 1, 0) };
    vector<SplineBase::CoordinateType> types{
        SplineBase::ORDINARY_POINT,
        SplineBase::TANGENT_POINT_FOR_NEXT, SplineBase::TANGENT_POINT_FOR_PRIOR,
        SplineBase::ORDINARY_POINT,
        SplineBase::TANGENT_POINT_FOR_NEXT, SplineBase::TANGENT_POINT_FOR_PRIOR,
        SplineBase::ORDINARY_POINT };
    vector<double> parameters;
    spline.interpolate(points, parameters, vector<double>(), types);

    vector<double> expected_parameters{0, 1, 2};
    BOOST_CHECK_EQUAL_COLLECTIONS(expected_parameters.begin(), expected_parameters.end(),
            parameters.begin(), parameters.end());
    BOOST_CHECK(spline.getPoint(0.5).isApprox(Eigen::Vector3d(0.75, 0, 0), 1e-12));
    BOOST_CHECK(spline.getPoint(1.5).isApprox(Eigen::Vector3d(1, 0.375, 0), 1e-12));
}

BOOST_AUTO_TEST_CASE(test_interpolate_raises_the_order_if_there_are_not_enough_points)
{
    Spline<2> spline(0.1, 4);
    spline.interpolate(vector<double>{0, 0, 3, 4});
    BOOST_CHECK_EQUAL(4, spline.getCurveOrder());
    BOOST_CHECK_EQUAL(4, spline.getPointCount());
    vector<double> expected{0, 0, 0, 0, 5, 5, 5, 5};
    vector<double> knots = spline.getKnots();
    BOOST_CHECK_EQUAL_COLLECTIONS(expected.begin(), expected.end(), knots.begin(), knots.end());
    BOOST_CHECK(spline.getPoint(2.5).isApprox(Eigen::Vector2d(1.5, 2), 1e-12));
}

/** The parabola y = x^2 for x in [-1, 1], as a quadratic Bezier curve over
 * [0, 1]
 */
static Spline<2> parabola()
{
    Spline<2> spline;
    spline.reset({-1, 1, 0, -1, 1, 1}, {0, 0, 0, 1, 1, 1}, 1);
    return spline;
}

BOOST_AUTO_TEST_CASE(test_reset)
{
    Spline<2> spline = parabola();
    BOOST_CHECK_EQUAL(0, spline.getStartParam());
    BOOST_CHECK_EQUAL(1, spline.getEndParam());
    BOOST_CHECK_EQUAL(3, spline.getCurveOrder());
    BOOST_CHECK_EQUAL(1, spline.getSISLCurveType());
    BOOST_CHECK(spline.getPoint(0.25).isApprox(Eigen::Vector2d(-0.5, 0.25), 1e-12));

    // Rational curves that are actually polynomial (all weights equal)
    Spline<2> rational;
    rational.reset({-2, 2, 2, 0, -2, 2, 2, 2, 2}, {0, 0, 0, 1, 1, 1}, 2);
    BOOST_CHECK(rational.getPoint(0.25).isApprox(Eigen::Vector2d(-0.5, 0.25), 1e-12));
}

BOOST_AUTO_TEST_CASE(test_curvature_and_length)
{
    Spline<2> spline = parabola();
    BOOST_CHECK_CLOSE(2, spline.getCurvature(0.5), 1e-9);
    // curvature of y = x^2 is 2 / (1 + 4x^2)^1.5
    BOOST_CHECK_CLOSE(2 / pow(5, 1.5), spline.getCurvature(1), 1e-9);
    // getCurvatureMax() samples the curve
    BOOST_CHECK_CLOSE(2, spline.getCurvatureMax(), 0.5);
    BOOST_CHECK_CLOSE(sqrt(5) + asinh(2) / 2, spline.getCurveLength(1e-9), 1e-7);
    BOOST_CHECK_CLOSE(sqrt(5) / 2 + asinh(2) / 4, spline.getCurveLength(0.5, 1, 1e-9), 1e-7);

    Spline<1> line(0.1, 2);
    line.interpolate(vector<double>{0, 2, -1});
    BOOST_CHECK_CLOSE(5, line.getCurveLength(), 1e-9);
}

BOOST_AUTO_TEST_CASE(test_closest_points)
{
    Spline<2> spline = parabola();

    // equidistant to both ends
    vector<double> points;
    vector< pair<double, double> > curves;
    spline.findClosestPoints(Eigen::Vector2d(0, 2), points, curves, 1e-6);
    BOOST_REQUIRE_EQUAL(2, points.size());
    BOOST_CHECK_SMALL(points[0], 1e-9);
    BOOST_CHECK_CLOSE(1, points[1], 1e-9);

    // The tangent is orthogonal to the error at the closest point
    Eigen::Vector2d p(0.5, 0);
    double t = spline.findOneClosestPoint(p);
    auto pt = spline.getPointAndTangent(t);
    BOOST_CHECK_SMALL((pt.first - p).dot(pt.second), 1e-9);
    BOOST_CHECK_CLOSE(t, spline.localClosestPointSearch(p, 0.9, 0.5, 1), 1e-9);
}

BOOST_AUTO_TEST_CASE(test_line_and_sphere_intersections)
{
    Spline<2> spline = parabola();
    vector<double> points;
    vector< pair<double, double> > curves;
    spline.findLineIntersections(Eigen::Vector2d(0, 0.25), Eigen::Vector2d(0, 1),
            points, curves, 1e-6);
    BOOST_REQUIRE_EQUAL(2, points.size());
    BOOST_CHECK_CLOSE(0.25, points[0], 1e-9);
    BOOST_CHECK_CLOSE(0.75, points[1], 1e-9);

    // tangential intersection
    spline.findLineIntersections(Eigen::Vector2d(0, 0), Eigen::Vector2d(0, 1),
            points, curves, 1e-6);
    BOOST_REQUIRE_EQUAL(1, points.size());
    BOOST_CHECK_CLOSE(0.5, points[0], 1e-6);

    spline.findSphereIntersections(Eigen::Vector2d(0, 0), sqrt(2), points, curves);
    BOOST_REQUIRE_EQUAL(2, points.size());
    BOOST_CHECK_SMALL(points[0], 1e-9);
    BOOST_CHECK_CLOSE(1, points[1], 1e-9);
}

BOOST_AUTO_TEST_CASE(test_split_append_crop_and_reverse)
{
    Spline<2> spline;
    spline.interpolate(vector<double>{0, 0, 1, 2, 3, 1, 4, 4, 6, 3});
    double const start = spline.getStartParam(), end = spline.getEndParam();
    double const split = start + (end - start) * 0.3;

    Spline<2> first(spline), second;
    first.split(second, split);
    BOOST_CHECK_EQUAL(split, first.getEndParam());
    BOOST_CHECK_EQUAL(split, second.getStartParam());
    for (int i = 0; i <= 10; ++i)
    {
        double t = start + (split - start) * i / 10;
        BOOST_CHECK(first.getPoint(t).isApprox(spline.getPoint(t), 1e-12));
        t = split + (end - split) * i / 10;
        BOOST_CHECK(second.getPoint(t).isApprox(spline.getPoint(t), 1e-12));
    }

    first.append(second);
    BOOST_CHECK_CLOSE(end, first.getEndParam(), 1e-12);
    for (int i = 0; i <= 10; ++i)
    {
        double const t = start + (end - start) * i / 10;
        BOOST_CHECK(first.getPoint(t).isApprox(spline.getPoint(t), 1e-12));
    }

    Spline<2> cropped(spline);
    cropped.crop(split, end);
    BOOST_CHECK(cropped.getStartPoint().isApprox(spline.getPoint(split), 1e-12));
    BOOST_CHECK(cropped.getEndPoint().isApprox(spline.getEndPoint(), 1e-12));

    Spline<2> reversed(spline);
    reversed.reverse();
    BOOST_CHECK_EQUAL(start, reversed.getStartParam());
    BOOST_CHECK_EQUAL(end, reversed.getEndParam());
    BOOST_CHECK(reversed.getPoint(start + 1).isApprox(spline.getPoint(end - 1), 1e-12));
}

BOOST_AUTO_TEST_CASE(test_heading_and_distance_errors)
{
    // Straight line from the origin in the direction of 135 degrees
    Spline<3> spline;
    spline.interpolate(vector<Eigen::Vector3d>{
        Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(-2, 2, 0) });
    double const t = spline.getEndParam() / 2;
    BOOST_CHECK_CLOSE(3 * M_PI / 4, spline.getHeading(t), 1e-9);
    BOOST_CHECK_CLOSE(-M_PI / 2, spline.headingError(M_PI / 4, t), 1e-9);
    BOOST_CHECK_CLOSE(3 * M_PI / 4, spline.headingError(-M_PI / 2, t), 1e-9);

    // Points on the left of the curve have a positive distance
    Eigen::Vector3d const p = spline.getPoint(t);
    BOOST_CHECK_CLOSE(sqrt(2), spline.distanceError(p + Eigen::Vector3d(-1, -1, 5), t), 1e-9);
    BOOST_CHECK_CLOSE(-sqrt(2), spline.distanceError(p + Eigen::Vector3d(1, 1, 0), t), 1e-9);

    Eigen::Vector3d const error = spline.poseError(p + Eigen::Vector3d(1, 1, 0), M_PI, 0);
    BOOST_CHECK_CLOSE(-sqrt(2), error(0), 1e-9);
    BOOST_CHECK_CLOSE(M_PI / 4, error(1), 1e-9);
    BOOST_CHECK_CLOSE(t, error(2), 1e-6);
}

BOOST_AUTO_TEST_CASE(test_reset_to_a_singleton)
{
    Spline<2> spline = parabola();
    spline.reset({1, 2}, {});
    BOOST_CHECK(spline.isSingleton());
    BOOST_CHECK_EQUAL(1, spline.getPointCount());
    BOOST_CHECK(spline.getKnots().empty());
    BOOST_CHECK(spline.getPoint(0).isApprox(Eigen::Vector2d(1, 2)));
}

BOOST_AUTO_TEST_CASE(test_closest_point_of_a_singleton_is_its_parameter)
{
    Spline<2> spline;
    spline.interpolate({1.0, 2.0}, std::vector<double>{3.0});
    vector<double> points;
    vector< pair<double, double> > curves;
    spline.findClosestPoints(Eigen::Vector2d(0, 0), points, curves);
    BOOST_REQUIRE_EQUAL(1, points.size());
    BOOST_CHECK_EQUAL(3, points[0]);
}

BOOST_AUTO_TEST_CASE(test_split_at_the_ends_of_a_high_dimensional_curve)
{
    Spline<5> spline;
    spline.interpolate(vector<double>{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 3, 3, 3, 3, 3});
    Spline<5>::vector_t const start = spline.getStartPoint(), end = spline.getEndPoint();

    Spline<5> first(spline), second;
    first.split(second, first.getStartParam());
    BOOST_CHECK(first.isSingleton());
    BOOST_CHECK(first.getStartPoint().isApprox(start));
    BOOST_CHECK(second.getEndPoint().isApprox(end));

    first = spline;
    first.split(second, first.getEndParam());
    BOOST_CHECK(second.isSingleton());
    BOOST_CHECK(second.getStartPoint().isApprox(end));
    BOOST_CHECK(first.getStartPoint().isApprox(start));
}

BOOST_AUTO_TEST_CASE(test_simplify)
{
    // A densely sampled straight line can be represented by a single segment
    vector<double> coordinates;
    for (int i = 0; i < 20; ++i)
    {
        coordinates.push_back(i);
        coordinates.push_back(2 * i);
    }
    Spline<2> spline(0.1, 3);
    spline.interpolate(coordinates);
    vector<double> errors = spline.simplify(1e-6);
    BOOST_CHECK_LT(spline.getPointCount(), 20);
    BOOST_CHECK_SMALL(errors[0], 1e-6);
    BOOST_CHECK_SMALL(errors[1], 1e-6);
    BOOST_CHECK(spline.getEndPoint().isApprox(Eigen::Vector2d(19, 38), 1e-12));
}

BOOST_AUTO_TEST_CASE( spline_to_points )
{
    std::vector<base::Vector3d> pointsIn;

    for(int i = 0; i < 10; i++)
        pointsIn.push_back(base::Vector3d(i, i, 0));

    base::geometry::Spline3 spline;
    spline.interpolate(pointsIn);

    std::vector<base::Vector3d> pointsOut = spline.sample(0.1);
    for(std::vector<base::Vector3d>::iterator it = pointsOut.begin(); it != pointsOut.end(); it++)
    {
        BOOST_CHECK(fabs(it->x() - it->y()) < 0.001);
    }

    BOOST_CHECK(pointsOut.begin()->x() == 0);
    BOOST_CHECK(pointsOut.begin()->y() == 0);

    BOOST_CHECK(pointsOut.rbegin()->x() == 9);
    BOOST_CHECK(pointsOut.rbegin()->y() == 9);
}

BOOST_AUTO_TEST_CASE( trajectory )
{
    base::Trajectory tr;

    tr.speed = 5;
    BOOST_CHECK(tr.driveForward());

    tr.speed = -5;
    BOOST_CHECK(!tr.driveForward());
}

BOOST_AUTO_TEST_SUITE_END()

