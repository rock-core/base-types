#include <boost/test/unit_test.hpp>
#include <base/Angle.hpp>

BOOST_AUTO_TEST_SUITE(AngleTests)

using namespace base;

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
    using base::Angle;
    using base::Vector3d;
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

BOOST_AUTO_TEST_CASE( angle_segment_test )
{
    Angle a = Angle::fromDeg( 90 );
    {
        base::AngleSegment s(base::Angle::fromDeg(89), base::Angle::fromDeg(2).getRad());
        BOOST_CHECK( s.isInside(a) );
        s = base::AngleSegment(base::Angle::fromDeg(89), (180-89) / 180.0 * M_PI);
        BOOST_CHECK( s.isInside(a) );
        s = base::AngleSegment(base::Angle::fromDeg(89), (190-89) / 180.0 * M_PI);
        BOOST_CHECK( s.isInside(a) );
        s = base::AngleSegment(base::Angle::fromDeg(89), (360-89) / 180.0 * M_PI);
        BOOST_CHECK( s.isInside(a) );
        s = base::AngleSegment(base::Angle::fromDeg(91), (360-2) / 180.0 * M_PI);
        BOOST_CHECK(! s.isInside(a) );
    }
    a = Angle::fromDeg( 190 );
    {
        base::AngleSegment s(base::Angle::fromDeg(170), (30) / 180.0 * M_PI);
        BOOST_CHECK( s.isInside(a) );
        s = base::AngleSegment(base::Angle::fromDeg(185), (15) / 180.0 * M_PI);
        BOOST_CHECK( s.isInside(a) );
        s = base::AngleSegment(base::Angle::fromDeg(170), (350) / 180.0 * M_PI);
        BOOST_CHECK( s.isInside(a) );
        s = base::AngleSegment(base::Angle::fromDeg(170), (350-170) / 180.0 * M_PI);
        BOOST_CHECK( s.isInside(a) );
        s = base::AngleSegment(base::Angle::fromDeg(200), (380-200) / 180.0 * M_PI);
        BOOST_CHECK(! s.isInside(a) );
    }
    {
        base::AngleSegment s(base::Angle::fromDeg(20), base::Angle::fromDeg(45).getRad());
        BOOST_CHECK( s.isInside(Angle::fromDeg(20)));
        BOOST_CHECK( s.isInside(Angle::fromDeg(30)));
        BOOST_CHECK( s.isInside(Angle::fromDeg(65)));
        BOOST_CHECK(! s.isInside(Angle::fromDeg(19)));
        BOOST_CHECK(! s.isInside(Angle::fromDeg(66)));
    }

    {
        base::AngleSegment s(base::Angle::fromDeg(350), base::Angle::fromDeg(45).getRad());
        BOOST_CHECK( s.isInside(Angle::fromDeg(20)));
        BOOST_CHECK( s.isInside(Angle::fromDeg(35)));
        BOOST_CHECK( s.isInside(Angle::fromDeg(350)));
        BOOST_CHECK( s.isInside(Angle::fromDeg(355)));
        BOOST_CHECK(! s.isInside(Angle::fromDeg(36)));
        BOOST_CHECK(! s.isInside(Angle::fromDeg(349)));
    }

    {
        base::AngleSegment s(base::Angle::fromDeg(160), base::Angle::fromDeg(45).getRad());
        BOOST_CHECK( s.isInside(Angle::fromDeg(160)));
        BOOST_CHECK( s.isInside(Angle::fromDeg(180)));
        BOOST_CHECK( s.isInside(Angle::fromDeg(181)));
        BOOST_CHECK( s.isInside(Angle::fromDeg(205)));
        BOOST_CHECK(! s.isInside(Angle::fromDeg(159)));
        BOOST_CHECK(! s.isInside(Angle::fromDeg(206)));
    }

    {
        base::AngleSegment result(base::Angle::fromDeg(0), 0);
        base::AngleSegment s1(base::Angle::fromDeg(160), base::Angle::fromDeg(45).getRad());
        base::AngleSegment s2(base::Angle::fromDeg(100), base::Angle::fromDeg(45).getRad());
        std::vector<AngleSegment> ret = s1.getIntersections(s2);
        BOOST_CHECK(ret.size() == 0);
    }
    {
        base::AngleSegment s1(base::Angle::fromDeg(160), base::Angle::fromDeg(45).getRad());
        base::AngleSegment s2(base::Angle::fromDeg(180), base::Angle::fromDeg(45).getRad());
        std::vector<AngleSegment> ret = s1.getIntersections(s2);
        base::AngleSegment result = ret[0];
        BOOST_CHECK(ret.size() == 1);
        BOOST_CHECK(fabs(result.startRad - base::Angle::fromDeg(180).getRad()) < 0.0001);
        BOOST_CHECK(fabs(result.getWidth() - base::Angle::fromDeg(25).getRad()) < 0.0001);
    }
    {
        base::AngleSegment s1(base::Angle::fromDeg(180), base::Angle::fromDeg(45).getRad());
        base::AngleSegment s2(base::Angle::fromDeg(220), base::Angle::fromDeg(45).getRad());
        std::vector<AngleSegment> ret = s1.getIntersections(s2);
        base::AngleSegment result = ret[0];
        BOOST_CHECK(ret.size() == 1);
        BOOST_CHECK(fabs(result.getWidth() - base::Angle::fromDeg(5).getRad()) < 0.0001);
        BOOST_CHECK(fabs(result.startRad - base::Angle::fromDeg(220).getRad()) < 0.0001);
    }
    {
        base::AngleSegment s1(base::Angle::fromDeg(160), base::Angle::fromDeg(45).getRad());
        base::AngleSegment s2(base::Angle::fromDeg(165), base::Angle::fromDeg(20).getRad());
        std::vector<AngleSegment> ret = s1.getIntersections(s2);
        base::AngleSegment result = ret[0];
        BOOST_CHECK(ret.size() == 1);
        BOOST_CHECK(fabs(result.getWidth() - base::Angle::fromDeg(20).getRad()) < 0.0001);
        BOOST_CHECK(fabs(result.startRad - base::Angle::fromDeg(165).getRad()) < 0.0001);
    }
    {
        base::AngleSegment s1(base::Angle::fromDeg(160), 2*M_PI);
        base::AngleSegment s2(base::Angle::fromDeg(180), base::Angle::fromDeg(45).getRad());
        std::vector<AngleSegment> ret = s1.getIntersections(s2);
        base::AngleSegment result = ret[0];
        BOOST_CHECK(ret.size() == 1);
        BOOST_CHECK(fabs(result.getWidth() - base::Angle::fromDeg(45).getRad()) < 0.0001);
        BOOST_CHECK(fabs(result.startRad - base::Angle::fromDeg(180).getRad()) < 0.0001);
    }

    {
        base::AngleSegment s1(base::Angle::fromDeg(-20), 2*M_PI);
        base::AngleSegment s2(base::Angle::fromDeg(50), base::Angle::fromDeg(45).getRad());
        std::vector<AngleSegment> ret = s1.getIntersections(s2);
        base::AngleSegment result = ret[0];
        BOOST_CHECK(ret.size() == 1);
        BOOST_CHECK(fabs(result.getWidth() - base::Angle::fromDeg(45).getRad()) < 0.0001);
        BOOST_CHECK(fabs(result.startRad - base::Angle::fromDeg(50).getRad()) < 0.0001);
    }
    {
        base::AngleSegment s1(base::Angle::fromDeg(-20), 2*M_PI);
        base::AngleSegment s2(base::Angle::fromDeg(0), 2*M_PI);
        std::vector<AngleSegment> ret = s1.getIntersections(s2);
        base::AngleSegment result = ret[0];
        BOOST_CHECK(ret.size() == 1);
        BOOST_CHECK(fabs(result.getWidth() - 2*M_PI) < 0.0001);
        BOOST_CHECK(fabs(result.startRad - base::Angle::fromDeg(0).getRad()) < 0.0001);
    }
    {
        base::AngleSegment s1(base::Angle::fromDeg(-20), 2*M_PI - base::Angle::fromDeg(1).getRad());
        base::AngleSegment s2(base::Angle::fromDeg(0), 2*M_PI - base::Angle::fromDeg(1).getRad());
        std::vector<AngleSegment> ret = s1.getIntersections(s2);
        base::AngleSegment result = ret[0];
        BOOST_CHECK(ret.size() == 2);
        BOOST_CHECK(fabs(result.getWidth() - (2*M_PI - base::Angle::fromDeg(21).getRad())) < 0.0001);
        BOOST_CHECK(fabs(result.startRad - base::Angle::fromDeg(0).getRad()) < 0.0001);
    }

    {
        for(int x = 0; x < 360; x++)
        {
            for(int y = 0; y < 360; y++)
            {
                base::AngleSegment s1(base::Angle::fromDeg(x), 2*M_PI - base::Angle::fromDeg(1).getRad());
                base::AngleSegment s2(base::Angle::fromDeg(y), 2*M_PI - base::Angle::fromDeg(1).getRad());
                std::vector<AngleSegment> ret = s1.getIntersections(s2);
                double sum = 0;
                for(std::vector<AngleSegment>::const_iterator it = ret.begin(); it != ret.end(); it++)
                {
                    sum += it->width;
//                     std::cout << "A1 " << x << " A2 " << y << " Result 8" << *it << std::endl;
                }
                if(abs(x-y) < 2 || (x == 0 && y == 359 )|| (y == 0 && x == 359))
                {
                    if(ret.size() != 1)
                    {
                        for(std::vector<AngleSegment>::const_iterator it = ret.begin(); it != ret.end(); it++)
                        {
                            std::cout << "A1 " << x << " A2 " << y << " Result 8" << *it << std::endl;
                        }
                    }
                    BOOST_CHECK(ret.size() == 1);
                    assert(fabs((2*M_PI - base::Angle::fromDeg(1).getRad()) - sum) < 0.000001);
                }
                else
                {
                    if(ret.size() != 2)
                    {
                        for(std::vector<AngleSegment>::const_iterator it = ret.begin(); it != ret.end(); it++)
                        {
                            std::cout << "A1 " << x << " A2 " << y << " Result 8" << *it << std::endl;
                        }
                    }

                    BOOST_CHECK(ret.size() == 2);
                    assert(fabs((2*M_PI - base::Angle::fromDeg(2).getRad()) - sum) < 0.000001);
                }
            }
        }
    }
}

BOOST_AUTO_TEST_SUITE_END()
