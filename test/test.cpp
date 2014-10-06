#define BOOST_TEST_MODULE BaseTypes
#include <boost/test/unit_test.hpp>

#include <base/Angle.hpp>
#include <base/commands/AUVMotion.hpp>
#include <base/commands/AUVPosition.hpp>
#include <base/commands/Joints.hpp>
#include <base/commands/Motion2D.hpp>
#include <base/commands/Speed6D.hpp>
#include <base/Deprecated.hpp>
#include <base/Eigen.hpp>
#include <base/Float.hpp>
#include <base/JointState.hpp>
#include <base/NamedVector.hpp>
#include <base/JointLimitRange.hpp>
#include <base/JointLimits.hpp>
#include <base/JointsTrajectory.hpp>
#include <base/Point.hpp>
#include <base/Pose.hpp>
#include <base/Pressure.hpp>
//#include <base/samples/CompressedFrame.hpp>
#include <base/samples/DistanceImage.hpp>
#include <base/samples/Frame.hpp>
#include <base/samples/IMUSensors.hpp>
#include <base/samples/Joints.hpp>
#include <base/samples/LaserScan.hpp>
#include <base/samples/Pointcloud.hpp>
#include <base/samples/Pressure.hpp>
#include <base/samples/RigidBodyAcceleration.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <base/samples/SonarBeam.hpp>
#include <base/samples/SonarScan.hpp>
#include <base/Temperature.hpp>
#include <base/Time.hpp>
#include <base/TimeMark.hpp>
#include <base/Trajectory.hpp>
#include <base/Waypoint.hpp>

#ifdef SISL_FOUND
#include <base/Trajectory.hpp>
#endif

#define BASE_LOG_DEBUG
#include <base/Logging.hpp>

#include <Eigen/SVD>
#include <Eigen/LU>

using namespace std;

BOOST_AUTO_TEST_CASE(sonar_scan)
{
    base::samples::SonarScan sonar_scan;
    base::samples::SonarBeam sonar_beam;

    sonar_scan.init(50,100,base::Angle::fromDeg(20),base::Angle::fromDeg(1));
    BOOST_CHECK(sonar_scan.data.size() == 50*100);
    BOOST_CHECK(sonar_scan.getNumberOfBytes() == 50*100);
    BOOST_CHECK(sonar_scan.getBinCount() == 50*100);
    BOOST_CHECK(sonar_scan.number_of_beams == 50);
    BOOST_CHECK(sonar_scan.number_of_bins == 100);
    BOOST_CHECK(sonar_scan.speed_of_sound == 0);
    BOOST_CHECK(sonar_scan.sampling_interval == 0);
    BOOST_CHECK(sonar_scan.angular_resolution == base::Angle::fromDeg(1));
    BOOST_CHECK(sonar_scan.start_bearing == base::Angle::fromDeg(20));
    BOOST_CHECK(sonar_scan.beamwidth_vertical == base::Angle::fromRad(0));
    BOOST_CHECK(sonar_scan.beamwidth_horizontal == base::Angle::fromRad(0));
    BOOST_CHECK(sonar_scan.time_beams.empty() == true);
    BOOST_CHECK(sonar_scan.polar_coordinates == true);

    //all should be valid because no seperate time stamp for each beam was set
    for(int i=20;i>-30;--i)
        BOOST_CHECK(sonar_scan.hasSonarBeam(base::Angle::fromDeg(i)));

    //wrong memory layout
    BOOST_REQUIRE_THROW(sonar_scan.addSonarBeam(sonar_beam),std::runtime_error);
    BOOST_REQUIRE_THROW(sonar_scan.getSonarBeam(base::Angle::fromRad(0),sonar_beam),std::runtime_error);

    sonar_beam.beam.resize(101);
    sonar_scan.toggleMemoryLayout();
    //too many bins 
    BOOST_REQUIRE_THROW(sonar_scan.addSonarBeam(sonar_beam),std::runtime_error);

    sonar_beam.beam.resize(100);
    sonar_beam.bearing = base::Angle::fromDeg(25);
    //wrong bearing  
    BOOST_REQUIRE_THROW(sonar_scan.addSonarBeam(sonar_beam,false),std::runtime_error);

    //add sonar beam
    sonar_beam.bearing = base::Angle::fromDeg(20);
    sonar_beam.speed_of_sound = 1500;
    sonar_beam.beamwidth_horizontal = 0.1;
    sonar_beam.beamwidth_vertical = 0.2;
    sonar_beam.sampling_interval = 0.01;
    sonar_beam.time = base::Time::now();
    for(int i=0;i<100;++i)
        sonar_beam.beam[i]=i;
    sonar_scan.addSonarBeam(sonar_beam,false);

    sonar_beam.bearing = base::Angle::fromDeg(-29);
    for(int i=0;i<100;++i)
        sonar_beam.beam[i]=23+i;
    sonar_scan.addSonarBeam(sonar_beam,false);

    BOOST_CHECK(sonar_scan.hasSonarBeam(base::Angle::fromDeg(20)));
    for(int i=19;i>-29;--i)
        BOOST_CHECK(!sonar_scan.hasSonarBeam(base::Angle::fromDeg(i)));
    BOOST_CHECK(sonar_scan.hasSonarBeam(base::Angle::fromDeg(-29)));

    base::samples::SonarBeam temp_beam;
    BOOST_CHECK_THROW(sonar_scan.getSonarBeam(base::Angle::fromDeg(21),temp_beam),std::runtime_error);
    sonar_scan.getSonarBeam(base::Angle::fromDeg(-29),temp_beam);

    BOOST_CHECK_SMALL(temp_beam.bearing.rad-sonar_beam.bearing.rad,0.0001);
    BOOST_CHECK(temp_beam.speed_of_sound == sonar_beam.speed_of_sound);
    BOOST_CHECK(temp_beam.beamwidth_horizontal == sonar_beam.beamwidth_horizontal);
    BOOST_CHECK(temp_beam.beamwidth_vertical == sonar_beam.beamwidth_vertical);
    BOOST_CHECK(temp_beam.sampling_interval == sonar_beam.sampling_interval);
    BOOST_CHECK(temp_beam.time == sonar_beam.time);
    BOOST_CHECK(temp_beam.beam == sonar_beam.beam);

    //toggleMemoryLayout
    sonar_scan.toggleMemoryLayout();
    for(int i=0;i<sonar_scan.number_of_bins;++i)
        BOOST_CHECK(sonar_scan.data[i*sonar_scan.number_of_beams] == i);
    for(int i=0;i<sonar_scan.number_of_bins;++i)
        BOOST_CHECK(sonar_scan.data[sonar_scan.number_of_beams-1+i*sonar_scan.number_of_beams] == i+23);

    sonar_scan.toggleMemoryLayout();
    sonar_scan.getSonarBeam(base::Angle::fromDeg(-29),temp_beam);
    BOOST_CHECK(temp_beam.speed_of_sound == sonar_beam.speed_of_sound);
    BOOST_CHECK(temp_beam.beamwidth_horizontal == sonar_beam.beamwidth_horizontal);
    BOOST_CHECK(temp_beam.beamwidth_vertical == sonar_beam.beamwidth_vertical);
    BOOST_CHECK(temp_beam.sampling_interval == sonar_beam.sampling_interval);
    BOOST_CHECK(temp_beam.time == sonar_beam.time);
    BOOST_CHECK(temp_beam.beam == sonar_beam.beam);
}

BOOST_AUTO_TEST_CASE( time_test )
{
    std::cout << base::Time::fromSeconds( 35.553 ) << std::endl;
    std::cout << base::Time::fromSeconds( -5.553 ) << std::endl;
}

BOOST_AUTO_TEST_CASE( time_fromSeconds )
{
    base::Time seconds;

    seconds = base::Time::fromSeconds( 35.553 );
    BOOST_REQUIRE_EQUAL( 35553000, seconds.toMicroseconds() );
    seconds = base::Time::fromSeconds( -5.553 );
    BOOST_REQUIRE_EQUAL( -5553000, seconds.toMicroseconds() );
    seconds = base::Time::fromSeconds( 0.01 );
    BOOST_REQUIRE_EQUAL( 10000, seconds.toMicroseconds() );
}

BOOST_AUTO_TEST_CASE( time_multiply )
{
    base::Time t = base::Time::fromSeconds( 35 );
    BOOST_REQUIRE_EQUAL( 35 * 1e6 * 0.025, (t * 0.025).toMicroseconds() );
}

BOOST_AUTO_TEST_CASE(time_fromString)
{
    base::Time now = base::Time::now();
    std::string nowString = now.toString(base::Time::Microseconds);
    base::Time expectedNow = base::Time::fromString(nowString);

    BOOST_REQUIRE_EQUAL(nowString, expectedNow.toString());
    BOOST_REQUIRE_EQUAL(now.toMicroseconds(),expectedNow.toMicroseconds());

    // Timezone conversion check -- since it depends on the current local time, either summer or winter check would
    // fail in case of an error
    // Summer
    base::Time tzOrig = base::Time::fromString("20120601-10:00:00", base::Time::Seconds);
    base::Time tzConverted = base::Time::fromString(tzOrig.toString());
    BOOST_REQUIRE_MESSAGE(tzOrig == tzConverted, "summer time: orig: " << tzOrig.toString() << " vs. converted: " << tzConverted.toString());

    // Winter
    tzOrig = base::Time::fromString("20121201-10:00:00", base::Time::Seconds);
    tzConverted = base::Time::fromString(tzOrig.toString());
    BOOST_REQUIRE_MESSAGE(tzOrig == tzConverted, "winter time: " << tzOrig.toString() << " vs. converted: " << tzConverted.toString());
    // End time zone check

    base::Time formatNow = base::Time::fromString("2012-06-14--12.05.06Z:001001", base::Time::Microseconds,"%Y-%m-%d--%H.%M.%S%Z");
    BOOST_REQUIRE_EQUAL(formatNow.toMicroseconds(),1339668306001001);

    base::Time expectedSecondResolutionOnly = base::Time::fromString(formatNow.toString(), base::Time::Seconds);
    BOOST_REQUIRE_EQUAL(expectedSecondResolutionOnly.toMicroseconds(), 1339668306000000);

    base::Time expectedMillisecondResolutionOnly = base::Time::fromString(formatNow.toString(), base::Time::Milliseconds);
    BOOST_REQUIRE_EQUAL(expectedMillisecondResolutionOnly.toMicroseconds(), 1339668306001000);

    std::string secondResolutionFormat = formatNow.toString(base::Time::Seconds);
    BOOST_REQUIRE_EQUAL(secondResolutionFormat,"20120614-12:05:06");

    std::string millisecondResolutionFormat = formatNow.toString(base::Time::Milliseconds);
    BOOST_REQUIRE_EQUAL(millisecondResolutionFormat,"20120614-12:05:06:001");

    BOOST_REQUIRE_THROW(base::Time::fromString(millisecondResolutionFormat, base::Time::Microseconds), std::runtime_error);

    std::string microsecondResolutionFormat = formatNow.toString(base::Time::Microseconds);
    BOOST_REQUIRE_EQUAL(microsecondResolutionFormat,"20120614-12:05:06:001001");

    std::string customFormat = formatNow.toString(base::Time::Milliseconds, "Time: %Y%m%dT%H%M%S");
    BOOST_REQUIRE_EQUAL(customFormat,"Time: 20120614T120506:001");

    std::string defaultResolutionFormat = formatNow.toString();
    BOOST_REQUIRE_EQUAL(microsecondResolutionFormat,defaultResolutionFormat);

}

BOOST_AUTO_TEST_CASE( laser_scan_test )
{
    //configure laser scan
    base::samples::LaserScan laser_scan;
    laser_scan.start_angle = M_PI*0.25;
    laser_scan.angular_resolution = M_PI*0.01;
    laser_scan.speed = 330;
    laser_scan.minRange = 1000;
    laser_scan.maxRange = 20000;

    //add some points
    laser_scan.ranges.push_back(1000);
    laser_scan.ranges.push_back(1000);
    laser_scan.ranges.push_back(2000);
    laser_scan.ranges.push_back(999);
    laser_scan.ranges.push_back(2000);

    Eigen::Affine3d trans;
    trans.setIdentity();
    trans.translation() = Eigen::Vector3d(-1.0,0.0,0.0);
    std::vector<Eigen::Vector3d> points;
    laser_scan.convertScanToPointCloud(points,trans,false);

    //check translation
    BOOST_CHECK(points.size() == 5);
    BOOST_CHECK(abs(points[0].x()-( -1+cos(M_PI*0.25))) < 0.000001);
    BOOST_CHECK(abs(points[0].y()-( sin(M_PI*0.25))) < 0.000001);
    BOOST_CHECK(points[0].z() == 0);
    BOOST_CHECK(abs(points[1].x()-( -1+cos(M_PI*0.25+laser_scan.angular_resolution))) < 0.000001);
    BOOST_CHECK(abs(points[1].y()-( sin(M_PI*0.25+laser_scan.angular_resolution))) < 0.000001);
    BOOST_CHECK(abs(points[2].x()-( -1+2.0*cos(M_PI*0.25+laser_scan.angular_resolution*2))) < 0.000001);
    BOOST_CHECK(abs(points[2].y()-( 2.0*sin(M_PI*0.25+laser_scan.angular_resolution*2))) < 0.000001);
    BOOST_CHECK(isnan(points[3].x()));
    BOOST_CHECK(isnan(points[3].y()));
    BOOST_CHECK(isnan(points[3].z()));

    //check rotation and translation
    trans.setIdentity();
    trans.translation() = Eigen::Vector3d(-1.0,0.0,0.0);
    trans.rotate(Eigen::AngleAxisd(0.1*M_PI,Eigen::Vector3d::UnitZ()));
    laser_scan.convertScanToPointCloud(points,trans,false);
    BOOST_CHECK(points.size() == 5);
    double x = cos(M_PI*0.25);
    double y = sin(M_PI*0.25);
    BOOST_CHECK(abs(points[0].x()-(-1+x*cos(0.1*M_PI)-y*sin(0.1*M_PI))) < 0.0000001);
    BOOST_CHECK(abs(points[0].y()-(x*sin(0.1*M_PI)+y*cos(0.1*M_PI))) < 0.0000001);
    BOOST_CHECK(points[0].z() == 0);
    x = cos(M_PI*0.25+laser_scan.angular_resolution);
    y = sin(M_PI*0.25+laser_scan.angular_resolution);
    BOOST_CHECK(abs(points[1].x()-(-1+x*cos(0.1*M_PI)-y*sin(0.1*M_PI))) < 0.0000001);
    BOOST_CHECK(abs(points[1].y()-(x*sin(0.1*M_PI)+y*cos(0.1*M_PI))) < 0.0000001);
    BOOST_CHECK(isnan(points[3].x()));
    BOOST_CHECK(isnan(points[3].y()));
    BOOST_CHECK(isnan(points[3].z()));

    //check skipping of invalid scan points  
    laser_scan.convertScanToPointCloud(points,trans);
    BOOST_CHECK(points.size() == 4);
    x = cos(M_PI*0.25);
    y = sin(M_PI*0.25);
    BOOST_CHECK(abs(points[0].x()-(-1+x*cos(0.1*M_PI)-y*sin(0.1*M_PI))) < 0.0000001);
    BOOST_CHECK(abs(points[0].y()-(x*sin(0.1*M_PI)+y*cos(0.1*M_PI))) < 0.0000001);
    BOOST_CHECK(points[0].z() == 0);
    x = cos(M_PI*0.25+laser_scan.angular_resolution);
    y = sin(M_PI*0.25+laser_scan.angular_resolution);
    BOOST_CHECK(abs(points[1].x()-(-1+x*cos(0.1*M_PI)-y*sin(0.1*M_PI))) < 0.0000001);
    BOOST_CHECK(abs(points[1].y()-(x*sin(0.1*M_PI)+y*cos(0.1*M_PI))) < 0.0000001);
    BOOST_CHECK(!isnan(points[3].x()));
    BOOST_CHECK(!isnan(points[3].y()));
    BOOST_CHECK(!isnan(points[3].z()));
}

BOOST_AUTO_TEST_CASE( pose_test )
{
    Eigen::Vector3d pos( 10, -1, 20.5 );
    Eigen::Quaterniond orientation( Eigen::AngleAxisd( 0.2, Eigen::Vector3d(0.5, 1.4, 0.1) ) );

    base::Pose p( pos, orientation ); 
    Eigen::Affine3d t( p.toTransform() );

    BOOST_CHECK( pos.isApprox( t.translation() ) );
    BOOST_CHECK( orientation.isApprox( Eigen::Quaterniond(t.rotation()), 0.01 ) );

    cout << Eigen::Quaterniond(t.rotation()).coeffs().transpose() << endl;
    cout << orientation.coeffs().transpose() << endl;
}

BOOST_AUTO_TEST_CASE( rbs_to_transform )
{
    // test casting operator from rigid body state to eigen::Transform
    base::samples::RigidBodyState rbs;

    Eigen::Affine3d r1( rbs );
    base::Affine3d r2( rbs );
}

base::Angle rand_angle()
{
    return base::Angle::fromRad(((rand() / (RAND_MAX + 1.0))-0.5) * M_PI);
}

BOOST_AUTO_TEST_CASE( angle_test )
{
    using namespace base;

    Angle a = Angle::fromDeg( 90 );
    BOOST_CHECK( a.isApprox( Angle::fromRad( M_PI/2.0 )) );
    BOOST_CHECK( a.isApprox( Angle::fromDeg( 90 + 720 )) );
    BOOST_CHECK( a.isApprox( Angle::fromDeg( 90 ) + Angle::fromDeg( 720 )) );
    BOOST_CHECK( a.isApprox( Angle::fromDeg( 90 - 720 )) );
    BOOST_CHECK( a.isApprox( Angle::fromDeg( 90 ) - Angle::fromDeg( 720 )) );
    BOOST_CHECK( (2*a).isApprox( Angle::fromDeg( 180 )) );
    BOOST_CHECK_CLOSE( (Angle::fromDeg(45)+Angle::fromDeg(-45)).getRad(), Angle::fromRad(0).getRad(), 1e-3 );

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

BOOST_AUTO_TEST_CASE( yaw_test )
{
    using namespace base;

    for(int i=0;i<10;i++)
    {
	Angle roll = rand_angle();
	Angle pitch = rand_angle();
	Angle yaw = rand_angle();
	Eigen::Quaterniond pitchroll = 
	    Eigen::AngleAxisd( pitch.getRad(), Eigen::Vector3d::UnitY() ) *
	    Eigen::AngleAxisd( roll.getRad(), Eigen::Vector3d::UnitX() );

	Eigen::Quaterniond rot =
	    Eigen::AngleAxisd( yaw.getRad(), Eigen::Vector3d::UnitZ() ) *
	    pitchroll;

	BOOST_CHECK_CLOSE( yaw.getRad(), Angle::fromRad(getYaw( rot )).getRad(), 1e-3 );

	rot = base::removeYaw( rot );
	BOOST_CHECK( rot.isApprox( pitchroll ) );
    }
}

BOOST_AUTO_TEST_CASE( angle_segment )
{
    using base::Angle;
    using base::AngleSegment;
    
    {
        Angle start = Angle::fromRad(-M_PI);
        AngleSegment test(start, 2*M_PI);
        for(int i = 0; i < 20; i++)
        {
            Angle angle = Angle::fromRad(i*2*M_PI/20 - M_PI);
            BOOST_CHECK(test.isInside(angle));
        }
        
    }
    
    {
        Angle start = Angle::fromRad(-M_PI / 2.0);
        AngleSegment test(start, M_PI);
        {
        Angle angle = Angle::fromRad(0);
        BOOST_CHECK(test.isInside(angle));
        }
        {
        Angle angle = Angle::fromRad(-M_PI / 2.0);
        BOOST_CHECK(test.isInside(angle));
        }
        {
        Angle angle = Angle::fromRad(M_PI / 2.0);
        BOOST_CHECK(test.isInside(angle));
        }
        {
        Angle angle = Angle::fromRad(M_PI / 3 * 4);
        BOOST_CHECK(!test.isInside(angle));
        }
        {
        Angle angle = Angle::fromRad(-M_PI / 3 * 4);
        BOOST_CHECK(!test.isInside(angle));
        }
        
    }
    {
        Angle start = Angle::fromRad(M_PI / 2.0);
        AngleSegment test(start, M_PI);
        {
        Angle angle = Angle::fromRad(0);
        BOOST_CHECK(!test.isInside(angle));
        }
        {
        Angle angle = Angle::fromRad(-M_PI / 2.0);
        BOOST_CHECK(test.isInside(angle));
        }
        {
        Angle angle = Angle::fromRad(M_PI / 2.0);
        BOOST_CHECK(test.isInside(angle));
        }
        {
        Angle angle = Angle::fromRad(M_PI / 3 * 4);
        BOOST_CHECK(test.isInside(angle));
        }
        {
        Angle angle = Angle::fromRad(M_PI / 1 * 4);
        BOOST_CHECK(!test.isInside(angle));
        }
        {
        Angle angle = Angle::fromRad(-M_PI / 3 * 4);
        BOOST_CHECK(test.isInside(angle));
        }
        {
        Angle angle = Angle::fromRad(-M_PI / 1 * 4);
        BOOST_CHECK(!test.isInside(angle));
        }
        
    }

}

BOOST_AUTO_TEST_CASE( angle_between_vectors )
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

BOOST_AUTO_TEST_CASE( logging_test )
{
#ifdef BASE_LONG_NAMES
        FILE* s = fopen("test.out", "w");

#ifdef WIN32
        BASE_LOG_CONFIGURE(INFO_P, s);
#else
        BASE_LOG_CONFIGURE(INFO, s);
#endif
        BASE_LOG_INFO("info-message")
#else
        FILE* s = fopen("test.out", "w");
#ifdef WIN32
        LOG_CONFIGURE(INFO_P, s);
#else 
	LOG_CONFIGURE(INFO, s);
#endif

        LOG_INFO("info-message")
#endif

        std::string test("additional-argument");

        int number = 1000000;
        time_t start,stop;
        time(&start);
        for(int i = 0; i < number; i++)
        {
#ifdef BASE_LONG_NAMES
            BASE_LOG_FATAL("test fatal log %s", test.c_str())
#else
            LOG_FATAL("test fatal log %s", test.c_str())
#endif
        }
        time(&stop);
        double seconds = difftime(stop, start)/(number*1.0);
        printf("Estimated time per log msg %f seconds", seconds);
}

#include <base/Float.hpp>

BOOST_AUTO_TEST_CASE( test_inf_nan )
{
    {
        float inf = base::infinity<float>();
        BOOST_REQUIRE( base::isInfinity(inf) );
        BOOST_REQUIRE( base::isInfinity(inf * 10) );
        BOOST_REQUIRE(inf == inf);
    }

    {
        double inf = base::infinity<double>();
        BOOST_REQUIRE( base::isInfinity(inf) );
        BOOST_REQUIRE( base::isInfinity(inf * 10) );
        BOOST_REQUIRE(inf == inf);
    }

    {
        float nan = base::unset<float>();
        BOOST_REQUIRE( base::isUnset(nan) );
        BOOST_REQUIRE( base::isUnset(nan * 10) );
        BOOST_REQUIRE(nan != nan);
    }

    {
        double nan = base::unset<double>();
        BOOST_REQUIRE( base::isUnset(nan) );
        BOOST_REQUIRE( base::isUnset(nan * 10) );
        BOOST_REQUIRE(nan != nan);
    }

    {
        float nan = base::unknown<float>();
        BOOST_REQUIRE( base::isUnknown(nan) );
        BOOST_REQUIRE( base::isUnknown(nan * 10) );
        BOOST_REQUIRE(nan != nan);
    }

    {
        double nan = base::unknown<double>();
        BOOST_REQUIRE( base::isUnknown(nan) );
        BOOST_REQUIRE( base::isUnknown(nan * 10) );
        BOOST_REQUIRE(nan != nan);
    }
}


BOOST_AUTO_TEST_CASE( frame_test )
{
    using namespace base::samples::frame;

    Frame frame;
    frame.init(200,300,8,MODE_GRAYSCALE);
    BOOST_CHECK(frame.getNumberOfBytes() == 200*300*1);
    BOOST_CHECK(frame.getPixelSize() == 1);
    BOOST_CHECK(frame.getPixelCount() == 200*300);
    BOOST_CHECK(frame.getChannelCount() == 1);
    BOOST_CHECK(frame.isCompressed() == false);
    BOOST_CHECK(frame.getHeight() == 300);
    BOOST_CHECK(frame.getWidth() == 200);

    frame.init(200,300,9,MODE_GRAYSCALE);
    BOOST_CHECK(frame.getNumberOfBytes() == 200*300*2);
    BOOST_CHECK(frame.getPixelSize() == 2);
    BOOST_CHECK(frame.getPixelCount() == 200*300);
    BOOST_CHECK(frame.getChannelCount() == 1);
    BOOST_CHECK(frame.isCompressed() == false);
    BOOST_CHECK(frame.getHeight() == 300);
    BOOST_CHECK(frame.getWidth() == 200);

    frame.init(200,300,8,MODE_RGB);
    BOOST_CHECK(frame.getNumberOfBytes() == 200*300*3);
    BOOST_CHECK(frame.getPixelSize() == 3);
    BOOST_CHECK(frame.getPixelCount() == 200*300);
    BOOST_CHECK(frame.getChannelCount() == 3);
    BOOST_CHECK(frame.isCompressed() == false);
    BOOST_CHECK(frame.getHeight() == 300);
    BOOST_CHECK(frame.getWidth() == 200);

    frame.init(200,300,8,MODE_GRAYSCALE,-1,200*300*1);
    BOOST_CHECK(frame.getNumberOfBytes() == 200*300*1);
    BOOST_CHECK(frame.getPixelSize() == 1);
    BOOST_CHECK(frame.getPixelCount() == 200*300);
    BOOST_CHECK(frame.getChannelCount() == 1);
    BOOST_CHECK(frame.isCompressed() == false);
    BOOST_CHECK(frame.getHeight() == 300);
    BOOST_CHECK(frame.getWidth() == 200);

    frame.init(200,300,8,MODE_PJPG,-1,0.5*200*300);
    BOOST_CHECK(frame.getNumberOfBytes() == 0.5*200*300);
    BOOST_CHECK(frame.getPixelSize() == 1);
    BOOST_CHECK(frame.getPixelCount() == 200*300);
    BOOST_CHECK(frame.getChannelCount() == 1);
    BOOST_CHECK(frame.isCompressed() == true);
    BOOST_CHECK(frame.getHeight() == 300);
    BOOST_CHECK(frame.getWidth() == 200);

    BOOST_CHECK_THROW(frame.init(200,300,8,MODE_RGB,-1,0.5*200*300),std::runtime_error);

    frame.init(200,300,8,MODE_GRAYSCALE);
    Frame frame2(frame);
    BOOST_CHECK(frame2.getNumberOfBytes() == 200*300);
    BOOST_CHECK(frame2.getPixelSize() == 1);
    BOOST_CHECK(frame2.getPixelCount() == 200*300);
    BOOST_CHECK(frame2.getChannelCount() == 1);
    BOOST_CHECK(frame2.isCompressed() == false);
    BOOST_CHECK(frame2.getHeight() == 300);
    BOOST_CHECK(frame2.getWidth() == 200);
}

BOOST_AUTO_TEST_CASE( rbs_validity )
{
    base::samples::RigidBodyState rbs;
    rbs.initUnknown();
    // check if values are unknown
    BOOST_CHECK(!rbs.isKnownValue(rbs.cov_position));
    BOOST_CHECK(!rbs.isKnownValue(rbs.cov_velocity));
    BOOST_CHECK(!rbs.isKnownValue(rbs.cov_orientation));
    BOOST_CHECK(!rbs.isKnownValue(rbs.cov_angular_velocity));
    BOOST_CHECK(rbs.position == Eigen::Vector3d::Zero());
    BOOST_CHECK(rbs.velocity == Eigen::Vector3d::Zero());
    BOOST_CHECK(rbs.angular_velocity == Eigen::Vector3d::Zero());
    BOOST_CHECK(rbs.orientation.x() == 0 && rbs.orientation.y() == 0 && 
                rbs.orientation.z() == 0 && rbs.orientation.w() == 1);
    
    // check if values are valid
    BOOST_CHECK(rbs.hasValidPosition());
    BOOST_CHECK(rbs.hasValidPositionCovariance());
    BOOST_CHECK(rbs.hasValidOrientation());
    BOOST_CHECK(rbs.hasValidOrientationCovariance());
    BOOST_CHECK(rbs.hasValidVelocity());
    BOOST_CHECK(rbs.hasValidVelocityCovariance());
    BOOST_CHECK(rbs.hasValidAngularVelocity());
    BOOST_CHECK(rbs.hasValidAngularVelocityCovariance());
    
    rbs.invalidate();
    // check if values are invalid
    BOOST_CHECK(!rbs.hasValidPosition());
    BOOST_CHECK(!rbs.hasValidPositionCovariance());
    BOOST_CHECK(!rbs.hasValidOrientation());
    BOOST_CHECK(!rbs.hasValidOrientationCovariance());
    BOOST_CHECK(!rbs.hasValidVelocity());
    BOOST_CHECK(!rbs.hasValidVelocityCovariance());
    BOOST_CHECK(!rbs.hasValidAngularVelocity());
    BOOST_CHECK(!rbs.hasValidAngularVelocityCovariance());
}

#ifdef SISL_FOUND
#include <base/geometry/spline.h>
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

BOOST_AUTO_TEST_CASE( pressure )
{
    base::Pressure pressure;
    base::samples::Pressure pressureSample;
}

#endif 
