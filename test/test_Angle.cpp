#include <boost/test/unit_test.hpp>
#include <base/Angle.hpp>
#include <sstream>

using namespace std;
using namespace base;

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

BOOST_AUTO_TEST_CASE( angle_segment_test )
{
    Angle a = Angle::fromDeg( 90 );
    {
        AngleSegment s(Angle::fromDeg(89), Angle::fromDeg(2).getRad());
        BOOST_CHECK( s.isInside(a) );
        s = AngleSegment(Angle::fromDeg(89), (180-89) / 180.0 * M_PI);
        BOOST_CHECK( s.isInside(a) );
        s = AngleSegment(Angle::fromDeg(89), (190-89) / 180.0 * M_PI);
        BOOST_CHECK( s.isInside(a) );
        s = AngleSegment(Angle::fromDeg(89), (360-89) / 180.0 * M_PI);
        BOOST_CHECK( s.isInside(a) );
        s = AngleSegment(Angle::fromDeg(91), (360-2) / 180.0 * M_PI);
        BOOST_CHECK(! s.isInside(a) );
    }
    a = Angle::fromDeg( 190 );
    {
        AngleSegment s(Angle::fromDeg(170), (30) / 180.0 * M_PI);
        BOOST_CHECK( s.isInside(a) );
        s = AngleSegment(Angle::fromDeg(185), (15) / 180.0 * M_PI);
        BOOST_CHECK( s.isInside(a) );
        s = AngleSegment(Angle::fromDeg(170), (350) / 180.0 * M_PI);
        BOOST_CHECK( s.isInside(a) );
        s = AngleSegment(Angle::fromDeg(170), (350-170) / 180.0 * M_PI);
        BOOST_CHECK( s.isInside(a) );
        s = AngleSegment(Angle::fromDeg(200), (380-200) / 180.0 * M_PI);
        BOOST_CHECK(! s.isInside(a) );
    }
    {
        AngleSegment s(Angle::fromDeg(20), Angle::fromDeg(45).getRad());
        BOOST_CHECK( s.isInside(Angle::fromDeg(20)));
        BOOST_CHECK( s.isInside(Angle::fromDeg(30)));
        BOOST_CHECK( s.isInside(Angle::fromDeg(65)));
        BOOST_CHECK(! s.isInside(Angle::fromDeg(19)));
        BOOST_CHECK(! s.isInside(Angle::fromDeg(66)));
    }

    {
        AngleSegment s(Angle::fromDeg(350), Angle::fromDeg(45).getRad());
        BOOST_CHECK( s.isInside(Angle::fromDeg(20)));
        BOOST_CHECK( s.isInside(Angle::fromDeg(35)));
        BOOST_CHECK( s.isInside(Angle::fromDeg(350)));
        BOOST_CHECK( s.isInside(Angle::fromDeg(355)));
        BOOST_CHECK(! s.isInside(Angle::fromDeg(36)));
        BOOST_CHECK(! s.isInside(Angle::fromDeg(349)));
    }

    {
        AngleSegment s(Angle::fromDeg(160), Angle::fromDeg(45).getRad());
        BOOST_CHECK( s.isInside(Angle::fromDeg(160)));
        BOOST_CHECK( s.isInside(Angle::fromDeg(180)));
        BOOST_CHECK( s.isInside(Angle::fromDeg(181)));
        BOOST_CHECK( s.isInside(Angle::fromDeg(205)));
        BOOST_CHECK(! s.isInside(Angle::fromDeg(159)));
        BOOST_CHECK(! s.isInside(Angle::fromDeg(206)));
    }

    {
        AngleSegment result(Angle::fromDeg(0), 0);
        AngleSegment s1(Angle::fromDeg(160), Angle::fromDeg(45).getRad());
        AngleSegment s2(Angle::fromDeg(100), Angle::fromDeg(45).getRad());
        vector<AngleSegment> ret = s1.getIntersections(s2);
        BOOST_CHECK(ret.size() == 0);
    }
    {
        AngleSegment s1(Angle::fromDeg(160), Angle::fromDeg(45).getRad());
        AngleSegment s2(Angle::fromDeg(180), Angle::fromDeg(45).getRad());
        vector<AngleSegment> ret = s1.getIntersections(s2);
        AngleSegment result = ret[0];
        BOOST_CHECK(ret.size() == 1);
        BOOST_CHECK(fabs(result.startRad - Angle::fromDeg(180).getRad()) < 0.0001);
        BOOST_CHECK(fabs(result.getWidth() - Angle::fromDeg(25).getRad()) < 0.0001);
    }
    {
        AngleSegment s1(Angle::fromDeg(180), Angle::fromDeg(45).getRad());
        AngleSegment s2(Angle::fromDeg(220), Angle::fromDeg(45).getRad());
        vector<AngleSegment> ret = s1.getIntersections(s2);
        AngleSegment result = ret[0];
        BOOST_CHECK(ret.size() == 1);
        BOOST_CHECK(fabs(result.getWidth() - Angle::fromDeg(5).getRad()) < 0.0001);
        BOOST_CHECK(fabs(result.startRad - Angle::fromDeg(220).getRad()) < 0.0001);
    }
    {
        AngleSegment s1(Angle::fromDeg(160), Angle::fromDeg(45).getRad());
        AngleSegment s2(Angle::fromDeg(165), Angle::fromDeg(20).getRad());
        vector<AngleSegment> ret = s1.getIntersections(s2);
        AngleSegment result = ret[0];
        BOOST_CHECK(ret.size() == 1);
        BOOST_CHECK(fabs(result.getWidth() - Angle::fromDeg(20).getRad()) < 0.0001);
        BOOST_CHECK(fabs(result.startRad - Angle::fromDeg(165).getRad()) < 0.0001);
    }
    {
        AngleSegment s1(Angle::fromDeg(160), 2*M_PI);
        AngleSegment s2(Angle::fromDeg(180), Angle::fromDeg(45).getRad());
        vector<AngleSegment> ret = s1.getIntersections(s2);
        AngleSegment result = ret[0];
        BOOST_CHECK(ret.size() == 1);
        BOOST_CHECK(fabs(result.getWidth() - Angle::fromDeg(45).getRad()) < 0.0001);
        BOOST_CHECK(fabs(result.startRad - Angle::fromDeg(180).getRad()) < 0.0001);
    }

    {
        AngleSegment s1(Angle::fromDeg(-20), 2*M_PI);
        AngleSegment s2(Angle::fromDeg(50), Angle::fromDeg(45).getRad());
        vector<AngleSegment> ret = s1.getIntersections(s2);
        AngleSegment result = ret[0];
        BOOST_CHECK(ret.size() == 1);
        BOOST_CHECK(fabs(result.getWidth() - Angle::fromDeg(45).getRad()) < 0.0001);
        BOOST_CHECK(fabs(result.startRad - Angle::fromDeg(50).getRad()) < 0.0001);
    }
    {
        AngleSegment s1(Angle::fromDeg(-20), 2*M_PI);
        AngleSegment s2(Angle::fromDeg(0), 2*M_PI);
        vector<AngleSegment> ret = s1.getIntersections(s2);
        AngleSegment result = ret[0];
        BOOST_CHECK(ret.size() == 1);
        BOOST_CHECK(fabs(result.getWidth() - 2*M_PI) < 0.0001);
        BOOST_CHECK(fabs(result.startRad - Angle::fromDeg(0).getRad()) < 0.0001);
    }
    {
        AngleSegment s1(Angle::fromDeg(-20), 2*M_PI - Angle::fromDeg(1).getRad());
        AngleSegment s2(Angle::fromDeg(0), 2*M_PI - Angle::fromDeg(1).getRad());
        vector<AngleSegment> ret = s1.getIntersections(s2);
        AngleSegment result = ret[0];
        BOOST_CHECK(ret.size() == 2);
        BOOST_CHECK(fabs(result.getWidth() - (2*M_PI - Angle::fromDeg(21).getRad())) < 0.0001);
        BOOST_CHECK(fabs(result.startRad - Angle::fromDeg(0).getRad()) < 0.0001);
    }

    {
        for(int x = 0; x < 360; x++)
        {
            for(int y = 0; y < 360; y++)
            {
                AngleSegment s1(Angle::fromDeg(x), 2*M_PI - Angle::fromDeg(1).getRad());
                AngleSegment s2(Angle::fromDeg(y), 2*M_PI - Angle::fromDeg(1).getRad());
                vector<AngleSegment> ret = s1.getIntersections(s2);
                double sum = 0;
                for(vector<AngleSegment>::const_iterator it = ret.begin(); it != ret.end(); it++)
                {
                    sum += it->width;
                }
                if(abs(x-y) < 2 || (x == 0 && y == 359 )|| (y == 0 && x == 359))
                {
                    if(ret.size() != 1)
                    {
                        ostringstream out;
                        for(vector<AngleSegment>::const_iterator it = ret.begin(); it != ret.end(); it++)
                        {
                            out << "A1 " << x << " A2 " << y << " Result 8" << *it << endl;
                        }
                        BOOST_TEST_MESSAGE(out.str());
                        BOOST_FAIL("one intersection was expected, got " + to_string(ret.size()));
                    }

                    double diff = fabs((2*M_PI - Angle::fromDeg(1).getRad()) - sum);
                    if (diff >= 0.000001) {
                        BOOST_REQUIRE(diff < 0.000001);
                    }
                }
                else
                {
                    if(ret.size() != 2)
                    {
                        ostringstream out;
                        for(vector<AngleSegment>::const_iterator it = ret.begin(); it != ret.end(); it++)
                        {
                            out << "A1 " << x << " A2 " << y << " Result 8" << *it << endl;
                        }
                        BOOST_TEST_MESSAGE(out.str());
                        BOOST_FAIL("two intersections were expected, got " + to_string(ret.size()));
                    }

                    double diff = fabs((2*M_PI - Angle::fromDeg(2).getRad()) - sum);
                    if (diff >= 0.000001) {
                        BOOST_REQUIRE(diff < 0.000001);
                    }
                }
            }
        }
    }
}

BOOST_AUTO_TEST_SUITE_END()
