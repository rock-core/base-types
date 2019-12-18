#include <boost/test/unit_test.hpp>
#include <base/Angle.hpp>
#include <sstream>

using namespace std;
using namespace base;
namespace utf = boost::unit_test;

BOOST_AUTO_TEST_SUITE(AngleTests)

static const Angle a90 = Angle::fromDeg( 90 );

BOOST_AUTO_TEST_CASE( angle_normalization_fromRad_fromDeg )
{
    BOOST_CHECK( a90.isApprox( Angle::fromRad( M_PI/2.0 )) );
}

BOOST_AUTO_TEST_CASE( angle_normalization_above_pi )
{
    BOOST_CHECK( a90.isApprox( Angle::fromDeg( 90 + 720 )) );
}

BOOST_AUTO_TEST_CASE( angle_normalization_below_minus_pi )
{
    BOOST_CHECK( a90.isApprox( Angle::fromDeg( 90 + 720 )) );
}

BOOST_AUTO_TEST_CASE( angle_sum )
{
    BOOST_CHECK_CLOSE( (Angle::fromDeg(45)+Angle::fromDeg(-45)).getRad(), Angle::fromRad(0).getRad(), 1e-3 );
}

BOOST_AUTO_TEST_CASE( angle_product )
{
    BOOST_CHECK( (2*a90).isApprox( Angle::fromDeg( 180 )) );
}

BOOST_AUTO_TEST_CASE( angle_sum_normalizes )
{
    BOOST_CHECK( a90.isApprox( Angle::fromDeg( 90 ) + Angle::fromDeg( 720 )) );
    BOOST_CHECK( a90.isApprox( Angle::fromDeg( 90 - 720 )) );
    BOOST_CHECK( a90.isApprox( Angle::fromDeg( 90 ) - Angle::fromDeg( 720 )) );
}

BOOST_AUTO_TEST_CASE( angle_is_initialized_to_unknown )
{
    BOOST_CHECK( isUnknown(Angle()) );
}

BOOST_AUTO_TEST_CASE( unknown_Angle_specialization )
{
    BOOST_CHECK( isUnknown(unknown<Angle>()) );
}

BOOST_AUTO_TEST_CASE(angle_Min_Max)
{
    Angle max = Angle::Max();
    BOOST_CHECK( max.isApprox( Angle::fromRad(  M_PI )) );
    BOOST_CHECK( max.isApprox( Angle::fromRad(  M_PI_2 * 2 )) );
    BOOST_CHECK( max.isApprox( Angle::fromRad( -M_PI )) );
    BOOST_CHECK( max.isApprox( Angle::fromRad( -M_PI_2 * 2 )) );

    Angle min = Angle::Min();
    BOOST_CHECK( !(min == max) );
    BOOST_CHECK( min == Angle::fromRad( nextafter( -M_PI, 0 )) );
    BOOST_CHECK( min == Angle::fromRad( nextafter( -M_PI_2 * 2, 0 )) );
    BOOST_CHECK( !(min == Angle::fromRad( -M_PI )) );
    BOOST_CHECK( !(min == Angle::fromRad( -M_PI_2 * 2 )) );
    BOOST_CHECK( min.getRad() > -M_PI );
    BOOST_CHECK( min.getRad() > -M_PI_2 * 2 );
}

BOOST_AUTO_TEST_CASE( vectorToVector )
{
    BOOST_CHECK_SMALL(Angle::vectorToVector(Vector3d(2, 0, 0), Vector3d(3, 0, 0)).getRad(), 1e-3);
    BOOST_CHECK_SMALL(Angle::vectorToVector(Vector3d(2, 0, 0), Vector3d(3, 0, 0), Vector3d::UnitZ()).getRad(), 1e-3);

    BOOST_CHECK_SMALL(M_PI/2 - Angle::vectorToVector(Vector3d(2, 0, 0), Vector3d(0, 3, 0)).getRad(), 1e-3);
    BOOST_CHECK_SMALL(M_PI/2 - Angle::vectorToVector(Vector3d(2, 0, 0), Vector3d(0, 3, 0), Vector3d::UnitZ()).getRad(), 1e-3);
    BOOST_CHECK_SMALL(-M_PI/2 - Angle::vectorToVector(Vector3d(2, 0, 0), Vector3d(0, 3, 0), -Vector3d::UnitZ()).getRad(), 1e-3);

    BOOST_CHECK_SMALL(M_PI/2 - Angle::vectorToVector(Vector3d(0, 2, 0), Vector3d(3, 0, 0)).getRad(), 1e-3);
    BOOST_CHECK_SMALL(-M_PI/2 - Angle::vectorToVector(Vector3d(0, 2, 0), Vector3d(3, 0, 0), Vector3d::UnitZ()).getRad(), 1e-3);
    BOOST_CHECK_SMALL(M_PI/2 - Angle::vectorToVector(Vector3d(0, 2, 0), Vector3d(3, 0, 0), -Vector3d::UnitZ()).getRad(), 1e-3);

    BOOST_CHECK_SMALL(M_PI - Angle::vectorToVector(Vector3d(2, 0, 0), Vector3d(-3, 0.001, 0)).getRad(), 1e-3);
    BOOST_CHECK_SMALL(M_PI - Angle::vectorToVector(Vector3d(2, 0, 0), Vector3d(-3, 0.001, 0), Vector3d::UnitZ()).getRad(), 1e-3);
    BOOST_CHECK_SMALL(-M_PI - Angle::vectorToVector(Vector3d(2, 0, 0), Vector3d(-3, 0.001, 0), -Vector3d::UnitZ()).getRad(), 1e-3);

    BOOST_CHECK_SMALL(M_PI - Angle::vectorToVector(Vector3d(2, 0, 0), Vector3d(-3, -0.001, 0)).getRad(), 1e-3);
    BOOST_CHECK_SMALL(-M_PI - Angle::vectorToVector(Vector3d(2, 0, 0), Vector3d(-3, -0.001, 0), Vector3d::UnitZ()).getRad(), 1e-3);
    BOOST_CHECK_SMALL(M_PI - Angle::vectorToVector(Vector3d(2, 0, 0), Vector3d(-3, -0.001, 0), -Vector3d::UnitZ()).getRad(), 1e-3);
}

BOOST_AUTO_TEST_CASE( vectorToVector_handles_cos_rounded_above_1 )
{
    double const magic = 0.70699999999999985;
    Eigen::Vector3d const v(magic, magic, 0);
    BOOST_CHECK_SMALL(Angle::vectorToVector(v, v).getRad(), 1e-3);
    BOOST_CHECK_SMALL(Angle::vectorToVector(v, v, Vector3d::UnitZ()).getRad(), 1e-3);
}

BOOST_AUTO_TEST_CASE( vectorToVector_handles_cos_rounded_below_1 )
{
    double const magic = 0.70699999999999985;
    Eigen::Vector3d const v(magic, magic, 0);
    BOOST_CHECK_CLOSE(Angle::vectorToVector(v, -v).getRad(), M_PI, 1e-3);
    BOOST_CHECK_CLOSE(Angle::vectorToVector(v, -v, Vector3d::UnitZ()).getRad(), M_PI, 1e-3);
}

BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE(AngleSegmentTests)

void angleIsInsideSegmentTest(Angle a) {
    Angle delta = base::Angle::fromRad(1e-3);
    Angle just_before = a - 2 * delta;
    Angle just_after = a + 2 * delta;

    AngleSegment s;

    // "Easy" non-pathological tests
    s = AngleSegment(just_before, (2 * delta).getRad());
    BOOST_TEST(s.isInside(a));
    s = AngleSegment(just_before, (0.9 * delta).getRad());
    BOOST_TEST(!s.isInside(a));

    // End of segment is exactly pi
    s = AngleSegment(just_before, M_PI - just_before.getRad());
    BOOST_TEST(s.isInside(a));
    // End of segment is -180 degrees
    s = AngleSegment(just_before, M_PI - just_before.getRad() + 1e-3);
    BOOST_TEST(s.isInside(a));
    // End of segment is "just before" the angle, segment being almost a full turn
    s = AngleSegment(just_after, 2 * M_PI - 3 * delta.getRad());
    BOOST_TEST(!s.isInside(a));
}

void segmentIsInsideTest(Angle min, double length) {
    AngleSegment s(min, length);
    Angle delta = base::Angle::fromRad(1e-3);
    double delta_rad = delta.getRad();
    double min_rad = min.getRad();

    // The delta value to use to go "in". Handles length < delta
    double delta_in_rad = std::min(length / 2, delta_rad);

    // "Easy" non-pathological tests
    BOOST_TEST(!s.isInside(Angle::fromRad(min_rad - delta_rad)));
    BOOST_TEST(!s.isInside(Angle::fromRad(min_rad + length + delta_rad)));
    BOOST_TEST(s.isInside(Angle::fromRad(min_rad + length / 2)));
    BOOST_TEST(s.isInside(Angle::fromRad(min_rad + delta_in_rad)));
    BOOST_TEST(s.isInside(Angle::fromRad(min_rad + length - delta_in_rad)));

    // Angle is at the diametrically opposed position
    BOOST_TEST(!s.isInside(Angle::fromRad(min_rad + length / 2 + M_PI)));
}

void REQUIRE_SEGMENTS_EQUAL(AngleSegment s1, AngleSegment s2) {
    BOOST_TEST(s1.startRad == s2.startRad);
    BOOST_TEST(s1.width == s2.width);
    BOOST_TEST(s1.endRad == s2.endRad);
}

void REQUIRE_INTERSECTIONS_EQUAL(vector<AngleSegment> const& v1,
                                 vector<AngleSegment> const& v2) {
    BOOST_TEST(v1.size() == v2.size());
    for (size_t i = 0; i < v1.size(); ++i) {
        REQUIRE_SEGMENTS_EQUAL(v1[i], v2[i]);
    }
}

BOOST_AUTO_TEST_CASE(angle_segment_test_isInside)
{
    angleIsInsideSegmentTest(Angle::fromDeg(90));
    angleIsInsideSegmentTest(Angle::fromDeg(190));
    segmentIsInsideTest(Angle::fromDeg(350), Angle::fromDeg(45).getRad());
    segmentIsInsideTest(Angle::fromDeg(160), Angle::fromDeg(45).getRad());
}

BOOST_AUTO_TEST_CASE(
    it_properly_intersects_with_itself_a_segment_that_does_not_cross_the_180_line
) {
    AngleSegment s(Angle::fromDeg(-20), Angle::deg2Rad(45));
    vector<AngleSegment> expected { s };
    REQUIRE_INTERSECTIONS_EQUAL(expected, s.getIntersections(s));
}

BOOST_AUTO_TEST_CASE(it_finds_intersection_with_segments_that_do_not_cross_the_180_line,
                     * utf::tolerance(0.0001))
{
    vector<AngleSegment> expected { AngleSegment(Angle::fromDeg(20), Angle::deg2Rad(5)) };
    AngleSegment s1(Angle::fromDeg(-20), Angle::deg2Rad(45));
    AngleSegment s2(Angle::fromDeg(20), Angle::deg2Rad(45));
    REQUIRE_INTERSECTIONS_EQUAL(expected, s1.getIntersections(s2));
    REQUIRE_INTERSECTIONS_EQUAL(expected, s2.getIntersections(s1));
}

BOOST_AUTO_TEST_CASE(
    it_finds_intersection_with_overlapping_segments_that_do_not_cross_the_180_line,
    * utf::tolerance(0.0001)
) {
    vector<AngleSegment> expected {
        AngleSegment(Angle::fromDeg(20), Angle::deg2Rad(45))
    };
    AngleSegment s1(Angle::fromDeg(0), Angle::deg2Rad(70));
    AngleSegment s2(Angle::fromDeg(20), Angle::deg2Rad(45));
    REQUIRE_INTERSECTIONS_EQUAL(expected, s1.getIntersections(s2));
    REQUIRE_INTERSECTIONS_EQUAL(expected, s2.getIntersections(s1));
}

BOOST_AUTO_TEST_CASE(it_handles_non_intersecting_segments_that_do_not_cross_the_180_line,
                     * utf::tolerance(0.0001))
{
    vector<AngleSegment> expected { AngleSegment(Angle::fromDeg(20), Angle::deg2Rad(5)) };
    AngleSegment s1(Angle::fromDeg(-30), Angle::fromDeg(45).getRad());
    AngleSegment s2(Angle::fromDeg(20), Angle::fromDeg(45).getRad());
    BOOST_TEST(s1.getIntersections(s2).empty());
    BOOST_TEST(s2.getIntersections(s1).empty());
}



BOOST_AUTO_TEST_CASE(
    it_properly_intersects_with_itself_a_segment_that_crosses_the_180_line
) {
    AngleSegment s(Angle::fromDeg(120), Angle::deg2Rad(90));
    vector<AngleSegment> expected { s };
    REQUIRE_INTERSECTIONS_EQUAL(expected, s.getIntersections(s));
}

BOOST_AUTO_TEST_CASE(it_finds_intersection_with_one_segment_that_crosses_the_180_line,
                     * utf::tolerance(0.0001))
{
    vector<AngleSegment> expected {
        AngleSegment(Angle::fromDeg(120), Angle::deg2Rad(10))
    };
    AngleSegment s1(Angle::fromDeg(120), Angle::deg2Rad(70));
    AngleSegment s2(Angle::fromDeg(20), Angle::deg2Rad(110));
    REQUIRE_INTERSECTIONS_EQUAL(expected, s1.getIntersections(s2));
    REQUIRE_INTERSECTIONS_EQUAL(expected, s2.getIntersections(s1));
}

BOOST_AUTO_TEST_CASE(
    it_finds_intersection_with_overlapping_segments_that_cross_the_180_line,
    * utf::tolerance(0.0001)
) {
    vector<AngleSegment> expected {
        AngleSegment(Angle::fromDeg(120), Angle::deg2Rad(70))
    };
    AngleSegment s1(Angle::fromDeg(120), Angle::deg2Rad(70));
    AngleSegment s2(Angle::fromDeg(110), Angle::deg2Rad(90));
    REQUIRE_INTERSECTIONS_EQUAL(expected, s1.getIntersections(s2));
    REQUIRE_INTERSECTIONS_EQUAL(expected, s2.getIntersections(s1));
}

BOOST_AUTO_TEST_CASE(
    it_handles_non_intersecting_segments_with_one_segment_that_crosses_the_180_line,
    * utf::tolerance(0.0001)
) {
    AngleSegment s1(Angle::fromDeg(140), Angle::deg2Rad(70));
    AngleSegment s2(Angle::fromDeg(20), Angle::deg2Rad(110));
    BOOST_TEST(s1.getIntersections(s2).empty());
    BOOST_TEST(s2.getIntersections(s1).empty());
}

BOOST_AUTO_TEST_CASE(
    it_handles_ranges_that_intersect_twice,
    * utf::tolerance(0.0001)
) {
    vector<AngleSegment> expected {
        AngleSegment(Angle::fromDeg(80), Angle::deg2Rad(10)),
        AngleSegment(Angle::fromDeg(-110), Angle::deg2Rad(10))
    };
    AngleSegment s1(Angle::fromDeg(-110), Angle::deg2Rad(200));
    AngleSegment s2(Angle::fromDeg(80), Angle::deg2Rad(180));
    REQUIRE_INTERSECTIONS_EQUAL(expected, s1.getIntersections(s2));
    REQUIRE_INTERSECTIONS_EQUAL(expected, s2.getIntersections(s1));
}

BOOST_AUTO_TEST_SUITE_END()
