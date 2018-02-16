#define BOOST_TEST_MODULE BaseTypes
#include <boost/test/unit_test.hpp>

#include <base/Angle.hpp>
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
#include <base/samples/BodyState.hpp>
#include <base/samples/SonarBeam.hpp>
#include <base/samples/SonarScan.hpp>
#include <base/samples/DepthMap.hpp>
#include <base/TransformWithCovariance.hpp>
#include <base/samples/Sonar.hpp>
#include <base/samples/PoseWithCovariance.hpp>
#include <base/Temperature.hpp>
#include <base/Time.hpp>
#include <base/TimeMark.hpp>
#include <base/Trajectory.hpp>
#include <base/Waypoint.hpp>
#include <base/TwistWithCovariance.hpp>

#ifdef SISL_FOUND
#include <base/Trajectory.hpp>
#endif

#include <Eigen/SVD>
#include <Eigen/LU>
#include <Eigen/Geometry>

using namespace std;

BOOST_AUTO_TEST_CASE(twist_with_covariance_validity)
{
    base::TwistWithCovariance velocity;
    BOOST_CHECK(velocity.translation() == base::Vector3d::Zero());
    BOOST_CHECK(velocity.rotation() == base::Vector3d::Zero());
    BOOST_CHECK(velocity.hasValidVelocity() == true);
    BOOST_CHECK(velocity.hasValidCovariance() == false);
    std::cout<<"TwistWithCovariance\n";
    std::cout<<velocity<<std::endl;
    velocity.setCovariance(base::Matrix6d::Identity());
    std::cout<<"TwistWithCovariance\n";
    std::cout<<velocity<<std::endl;
    BOOST_CHECK(velocity.hasValidCovariance() == true);
    velocity.invalidateVelocity();
    BOOST_CHECK(velocity.hasValidVelocity() == false);
    BOOST_CHECK(velocity.hasValidCovariance() == true);
    velocity.invalidateCovariance();
    BOOST_CHECK(velocity.hasValidVelocity() == false);
    BOOST_CHECK(velocity.hasValidCovariance() == false);
}

BOOST_AUTO_TEST_CASE(twist_with_covariance_operations)
{
    base::TwistWithCovariance vel1, vel2, vel3;
    base::Vector6d vec;
    vec<< 0.3,0.3,0.3,1.0,1.0,1.0;
    vel1.setVelocity(vec);
    vel2.setVelocity(vec);
    vel3 = vel1 * vel2;
    BOOST_CHECK(vel3.translation() == Eigen::Vector3d::Zero());
    BOOST_CHECK(vel3.rotation() == Eigen::Vector3d::Zero());
    vel3 = vel1 - vel2;
    BOOST_CHECK(vel3.translation() == Eigen::Vector3d::Zero());
    BOOST_CHECK(vel3.rotation() == Eigen::Vector3d::Zero());
    vel3 = -vel1 + vel2;
    BOOST_CHECK(vel3.translation() == Eigen::Vector3d::Zero());
    BOOST_CHECK(vel3.rotation() == Eigen::Vector3d::Zero());
    vel3 = (-vel1 / 0.5) + 2.0 * vel1;
    BOOST_CHECK(vel3.translation() == Eigen::Vector3d::Zero());
    BOOST_CHECK(vel3.rotation() == Eigen::Vector3d::Zero());

    /** Add uncertainty to the covariance **/
    vel1.cov = 0.1 * base::Matrix6d::Identity();
    vel2.cov = 0.2 * base::Matrix6d::Identity();

    vel3.invalidateCovariance();
    vel3 = -vel1 + vel2;
    BOOST_CHECK(vel3.hasValidCovariance());
    BOOST_CHECK(vel3.translation() == Eigen::Vector3d::Zero());
    BOOST_CHECK(vel3.rotation() == Eigen::Vector3d::Zero());
    std::cout<<"TwistWithCovariance add operator\n";
    std::cout<<vel3<<std::endl;

    vel3.invalidateCovariance();
    vel3 = vel1 - vel2;
    BOOST_CHECK(vel3.hasValidCovariance());
    BOOST_CHECK(vel3.translation() == Eigen::Vector3d::Zero());
    BOOST_CHECK(vel3.rotation() == Eigen::Vector3d::Zero());
    std::cout<<"TwistWithCovariance subtract operator\n";
    std::cout<<vel3<<std::endl;

    vel3.invalidateCovariance();
    vel3 = (-vel1 / 0.5) + 2.0 * vel1;
    BOOST_CHECK(vel3.hasValidCovariance());
    BOOST_CHECK(vel3.translation() == Eigen::Vector3d::Zero());
    BOOST_CHECK(vel3.rotation() == Eigen::Vector3d::Zero());

    vel3.invalidateCovariance();
    vel3 = vel1 * vel2;
    BOOST_CHECK(vel3.hasValidCovariance());
    std::cout<<"TwistWithCovariance cross product\n";
    std::cout<<vel3<<std::endl;
}

BOOST_AUTO_TEST_CASE(body_state_validity)
{
    base::samples::BodyState bs;
    bs.initUnknown();
    // check if values are valid
    BOOST_CHECK(bs.hasValidPose());
    BOOST_CHECK(!bs.hasValidPoseCovariance());
    BOOST_CHECK(bs.hasValidVelocity());
    BOOST_CHECK(!bs.hasValidVelocityCovariance());

    bs.pose.setCovariance(base::Matrix6d::Identity());
    bs.velocity.setCovariance(base::Matrix6d::Identity());
    BOOST_CHECK(bs.hasValidPoseCovariance());
    BOOST_CHECK(bs.hasValidVelocityCovariance());
    // check std::cout operator when valid uncertainty
    std::cout<<"Body State\n"<<bs<<std::endl;

    bs.invalidate();
    // check if values are not valid
    BOOST_CHECK(!bs.hasValidPose());
    BOOST_CHECK(!bs.hasValidPoseCovariance());
    BOOST_CHECK(!bs.hasValidVelocity());
    BOOST_CHECK(!bs.hasValidVelocityCovariance());

    // check std::cout operator when not valid uncertainty
    std::cout<<"Body State\n"<<bs<<std::endl;
}

BOOST_AUTO_TEST_CASE(body_state_operations)
{
    base::samples::BodyState bs1, bs2, bs3;
    bs1.initUnknown();
    bs2.initUnknown();
    bs1.pose = base::TransformWithCovariance(base::Affine3d::Identity(), 0.1 * base::Matrix6d::Identity());
    bs1.velocity = base::TwistWithCovariance(base::Vector6d::Zero(), static_cast<base::TwistWithCovariance::Covariance>(0.1 * base::TwistWithCovariance::Covariance::Identity()));
    bs2.pose = base::TransformWithCovariance(base::Affine3d(Eigen::AngleAxisd(90.0 *
                    M_PI/180.0, Eigen::Vector3d::UnitZ())), 0.2 * base::Matrix6d::Identity());
    bs2.velocity = base::TwistWithCovariance(base::Vector6d::Ones(), static_cast<base::TwistWithCovariance::Covariance>(0.2 * base::TwistWithCovariance::Covariance::Identity()));
    bs3 = bs1 * bs2;
    std::cout<<"Body State Composition\n"<<bs3<<std::endl;
    BOOST_CHECK(bs3.hasValidPose());
    BOOST_CHECK(bs3.hasValidPoseCovariance());
    BOOST_CHECK(bs3.hasValidVelocity());
    BOOST_CHECK(bs3.hasValidVelocityCovariance());

    bs3.position().setOnes();
    std::cout<<"Body State Composition\n"<<bs3<<std::endl;
    bs3.position() = bs1.position() + bs2.position();
    std::cout<<"Body State Composition\n"<<bs3<<std::endl;
    bs3.orientation() = base::Orientation::Identity();
    bs3.orientation() = bs1.orientation() * bs2.orientation();
    std::cout<<"Body State Composition\n"<<bs3<<std::endl;
}

BOOST_AUTO_TEST_CASE(joint_state)
{
    base::JointState state;
    BOOST_CHECK(state.getMode() == base::JointState::UNSET);

    // Test position field
    state.setField(base::JointState::POSITION, 0.3);
    BOOST_CHECK(state.hasPosition() == true);
    BOOST_CHECK(state.hasSpeed() == false);
    BOOST_CHECK(state.hasEffort() == false);
    BOOST_CHECK(state.hasRaw() == false);
    BOOST_CHECK(state.hasAcceleration() == false);
    BOOST_CHECK(state.isPosition() == true);
    BOOST_CHECK(state.isSpeed() == false);
    BOOST_CHECK(state.isEffort() == false);
    BOOST_CHECK(state.isRaw() == false);
    BOOST_CHECK(state.isAcceleration() == false);

    BOOST_CHECK(state.getField(base::JointState::POSITION) == 0.3);
    BOOST_CHECK(state.getMode() == base::JointState::POSITION);

    state.setField(base::JointState::POSITION, base::NaN<double>()); 

    // Test speed field
    state.setField(base::JointState::SPEED, -.1f);

    BOOST_CHECK(state.hasPosition() == false);
    BOOST_CHECK(state.hasSpeed() == true);
    BOOST_CHECK(state.hasEffort() == false);
    BOOST_CHECK(state.hasRaw() == false);
    BOOST_CHECK(state.hasAcceleration() == false);
    BOOST_CHECK(state.isPosition() == false);
    BOOST_CHECK(state.isSpeed() == true);
    BOOST_CHECK(state.isEffort() == false);
    BOOST_CHECK(state.isRaw() == false);
    BOOST_CHECK(state.isAcceleration() == false);

    BOOST_CHECK(state.getField(base::JointState::SPEED) == -.1f);
    BOOST_CHECK(state.getMode() == base::JointState::SPEED);

    state.setField(base::JointState::SPEED, base::NaN<float>());

    // Test effort field
    state.setField(base::JointState::EFFORT, -.5f);

    BOOST_CHECK(state.hasPosition() == false);
    BOOST_CHECK(state.hasSpeed() == false);
    BOOST_CHECK(state.hasEffort() == true);
    BOOST_CHECK(state.hasRaw() == false);
    BOOST_CHECK(state.hasAcceleration() == false);
    BOOST_CHECK(state.isPosition() == false);
    BOOST_CHECK(state.isSpeed() == false);
    BOOST_CHECK(state.isEffort() == true);
    BOOST_CHECK(state.isRaw() == false);
    BOOST_CHECK(state.isAcceleration() == false);

    BOOST_CHECK(state.getField(base::JointState::EFFORT) == -.5f);
    BOOST_CHECK(state.getMode() == base::JointState::EFFORT);

    state.setField(base::JointState::EFFORT, base::NaN<float>());

    // Test raw field
    state.setField(base::JointState::RAW, 1.5f);

    BOOST_CHECK(state.hasPosition() == false);
    BOOST_CHECK(state.hasSpeed() == false);
    BOOST_CHECK(state.hasEffort() == false);
    BOOST_CHECK(state.hasRaw() == true);
    BOOST_CHECK(state.hasAcceleration() == false);
    BOOST_CHECK(state.isPosition() == false);
    BOOST_CHECK(state.isSpeed() == false);
    BOOST_CHECK(state.isEffort() == false);
    BOOST_CHECK(state.isRaw() == true);
    BOOST_CHECK(state.isAcceleration() == false);

    BOOST_CHECK(state.getField(base::JointState::RAW) == 1.5f);
    BOOST_CHECK(state.getMode() == base::JointState::RAW);

    state.setField(base::JointState::RAW, base::NaN<float>());

    // Test acceleration field
    state.setField(base::JointState::ACCELERATION, -0.7f);

    BOOST_CHECK(state.hasPosition() == false);
    BOOST_CHECK(state.hasSpeed() == false);
    BOOST_CHECK(state.hasEffort() == false);
    BOOST_CHECK(state.hasRaw() == false);
    BOOST_CHECK(state.hasAcceleration() == true);
    BOOST_CHECK(state.isPosition() == false);
    BOOST_CHECK(state.isSpeed() == false);
    BOOST_CHECK(state.isEffort() == false);
    BOOST_CHECK(state.isRaw() == false);
    BOOST_CHECK(state.isAcceleration() == true);

    BOOST_CHECK(state.getField(base::JointState::ACCELERATION) == -0.7f);
    BOOST_CHECK(state.getMode() == base::JointState::ACCELERATION);

    // Test invalid field
    BOOST_REQUIRE_THROW(state.getField(99), std::runtime_error);
    BOOST_REQUIRE_THROW(state.setField(99, 0.5), std::runtime_error);

    //Test with multiple fields 
    state.setField(base::JointState::RAW, 0.1);
    BOOST_REQUIRE_THROW(state.getMode(), std::runtime_error);
    BOOST_CHECK(state.isPosition() == false);
    BOOST_CHECK(state.isSpeed() == false);
    BOOST_CHECK(state.isEffort() == false);
    BOOST_CHECK(state.isRaw() == false);
    BOOST_CHECK(state.isAcceleration() == false);
}

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

    base::Time null_time;
    null_time.fromMicroseconds(0);
    BOOST_CHECK(null_time.isNull());
    null_time.fromMilliseconds(0);
    BOOST_CHECK(null_time.isNull());


    base::Time max_time;
    max_time = base::Time::fromMicroseconds(std::numeric_limits<int64_t>::max());
    BOOST_REQUIRE_EQUAL(max_time.toMicroseconds(), std::numeric_limits<int64_t>::max());
    BOOST_REQUIRE_EQUAL(max_time, base::Time::max());
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

BOOST_AUTO_TEST_CASE( time_fromMicroseconds )
{
    base::Time microseconds;

    microseconds = base::Time::fromMicroseconds( -1 );
    BOOST_REQUIRE_EQUAL( -1, microseconds.toMicroseconds() );
    microseconds = base::Time::fromMicroseconds( 1 );
    BOOST_REQUIRE_EQUAL( 1, microseconds.toMicroseconds() );
}

BOOST_AUTO_TEST_CASE( time_fromMilliseconds )
{
    base::Time milliseconds;

    milliseconds = base::Time::fromMilliseconds( -1 );
    BOOST_REQUIRE_EQUAL( -1000, milliseconds.toMicroseconds() );
    milliseconds = base::Time::fromMilliseconds( 1 );
    BOOST_REQUIRE_EQUAL( 1000, milliseconds.toMicroseconds() );
}

BOOST_AUTO_TEST_CASE(time_operators)
{
    base::Time time_10 = base::Time::fromMicroseconds(10);
    base::Time time_1= base::Time::fromMicroseconds(1);
    base::Time time_1_neg= base::Time::fromMicroseconds(-1);
    base::Time time_10_neg= base::Time::fromMicroseconds(-10);
    
    BOOST_CHECK(time_1!=time_10);

    BOOST_CHECK(time_10>time_1);
    BOOST_CHECK(time_1<time_10);
    BOOST_CHECK(time_1>time_1_neg);
    BOOST_CHECK(time_1_neg>time_10_neg);

    //test multiplication and equality
    BOOST_CHECK(time_1*10==time_10);
    BOOST_REQUIRE_EQUAL(time_1*10,time_10);
    
    BOOST_CHECK(time_10/10==time_1);
    BOOST_REQUIRE_EQUAL(time_10/10,time_1);
    
    BOOST_CHECK(time_1_neg*(-1)==time_1);
    BOOST_REQUIRE_EQUAL(time_1_neg*(-1),time_1);
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
    BOOST_CHECK(std::isnan(points[3].x()));
    BOOST_CHECK(std::isnan(points[3].y()));
    BOOST_CHECK(std::isnan(points[3].z()));

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
    BOOST_CHECK(std::isnan(points[3].x()));
    BOOST_CHECK(std::isnan(points[3].y()));
    BOOST_CHECK(std::isnan(points[3].z()));

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
    BOOST_CHECK(!std::isnan(points[3].x()));
    BOOST_CHECK(!std::isnan(points[3].y()));
    BOOST_CHECK(!std::isnan(points[3].z()));
}

BOOST_AUTO_TEST_CASE(distance_image_test)
{
    /** Distance image **/
    base::samples::DistanceImage dimage(2,2);
    dimage.setIntrinsic(1, 1, 1, 1);
    dimage.data.push_back(1.0); // 1st value
    dimage.data.push_back(2.0); // 2nd value
    dimage.data.push_back(2.0); // 3rd value
    dimage.data.push_back(1.0); //4th value

    /** Point cloud **/
    base::samples::Pointcloud point_cloud = dimage.getPointCloud();
    BOOST_CHECK(point_cloud.points.size() == dimage.data.size());

    BOOST_CHECK(point_cloud.points[0] == Eigen::Vector3d(-1.0, -1.00, 1.00));
    BOOST_CHECK(point_cloud.points[1] == Eigen::Vector3d(0.0, -2.00, 2.00));
    BOOST_CHECK(point_cloud.points[2] == Eigen::Vector3d(-2.0, 0.00, 2.00));
    BOOST_CHECK(point_cloud.points[3] == Eigen::Vector3d(0.0, 0.00, 1.00));

    /** Scene and image point projections **/
    dimage.setSize(3,2);
    dimage.clear();
    dimage.data[0] = 1.0;
    dimage.data[1] = 2.0;
    dimage.data[2] = 0.0;
    dimage.data[3] = std::numeric_limits<base::samples::DistanceImage::scalar>::denorm_min();
    dimage.data[4] = std::numeric_limits<base::samples::DistanceImage::scalar>::infinity();
    dimage.data[5] = std::numeric_limits<base::samples::DistanceImage::scalar>::quiet_NaN();

    Eigen::Vector3d scene_point = Eigen::Vector3d::Zero();
    size_t x = std::numeric_limits<size_t>::max(), y = std::numeric_limits<size_t>::max();
    // test valid projections
    BOOST_CHECK(dimage.getScenePoint(0,0,scene_point));
    BOOST_CHECK(dimage.getImagePoint(scene_point,x,y));
    BOOST_CHECK(x == 0);
    BOOST_CHECK(y == 0);

    BOOST_CHECK(dimage.getScenePoint(1,0,scene_point));
    BOOST_CHECK(dimage.getImagePoint(scene_point,x,y));
    BOOST_CHECK(x == 1);
    BOOST_CHECK(y == 0);

    // test invalid distacnes
    scene_point.setZero();
    x = std::numeric_limits<size_t>::max();
    y = std::numeric_limits<size_t>::max();
    BOOST_CHECK(dimage.getScenePoint(2,0,scene_point) == false);
    BOOST_CHECK(scene_point == Eigen::Vector3d::Zero());
    BOOST_CHECK(dimage.getScenePoint(0,1,scene_point) == false);
    BOOST_CHECK(scene_point == Eigen::Vector3d::Zero());
    BOOST_CHECK(dimage.getScenePoint(1,1,scene_point) == false);
    BOOST_CHECK(scene_point == Eigen::Vector3d::Zero());
    BOOST_CHECK(dimage.getScenePoint(2,1,scene_point) == false);
    BOOST_CHECK(scene_point == Eigen::Vector3d::Zero());
    BOOST_CHECK(dimage.getImagePoint(scene_point,x,y) == false);
    BOOST_CHECK(x == std::numeric_limits<size_t>::max());
    BOOST_CHECK(y == std::numeric_limits<size_t>::max());

    // test bounds
    BOOST_CHECK(dimage.getScenePoint(3,2,scene_point) == false);
    BOOST_CHECK(scene_point == Eigen::Vector3d::Zero());
    BOOST_CHECK(dimage.getScenePoint(0,2,scene_point) == false);
    BOOST_CHECK(scene_point == Eigen::Vector3d::Zero());
    BOOST_CHECK(dimage.getScenePoint(3,0,scene_point) == false);
    BOOST_CHECK(scene_point == Eigen::Vector3d::Zero());
    scene_point = Eigen::Vector3d(1.0, 1.0, 1.0);
    BOOST_CHECK(dimage.getImagePoint(scene_point,x,y) == false);
    BOOST_CHECK(x == std::numeric_limits<size_t>::max());
    BOOST_CHECK(y == std::numeric_limits<size_t>::max());
}

BOOST_AUTO_TEST_CASE(depth_map_test)
{
    // setup scan
    base::samples::DepthMap scan;
    scan.vertical_size = 5;
    scan.horizontal_size = 2;
    scan.vertical_interval.push_back(0.0);
    scan.vertical_interval.push_back(0.0);
    // add points
    for(unsigned j = 0; j < scan.horizontal_size; j++)
    {
	scan.timestamps.push_back(base::Time::now());
	scan.horizontal_interval.push_back(base::Angle::fromDeg(-90.0 * j).getRad());
    }
    for(unsigned i = 0; i < scan.vertical_size-1; i++)
    {
	for(unsigned j = 0; j < scan.horizontal_size; j++)
	    scan.distances.push_back((j+1) * 2.0);
    }
    scan.distances.push_back(0.0);
    scan.distances.push_back(0.0);
    
    
    // create reference points
    std::vector<Eigen::Vector3d> ref_points;
    ref_points.push_back(Eigen::Vector3d(2.0,0.0,0.0));
    ref_points.push_back(Eigen::Vector3d(0.0,-4.0,0.0));
    ref_points.push_back(Eigen::Vector3d(0.0,0.0,-2.0));
    ref_points.push_back(Eigen::Vector3d(0.0,0.0,-4.0));
    ref_points.push_back(Eigen::Vector3d(-2.0,0.0,0.0));
    ref_points.push_back(Eigen::Vector3d(0.0,4.0,0.0));
    ref_points.push_back(Eigen::Vector3d(0.0,0.0,2.0));
    ref_points.push_back(Eigen::Vector3d(0.0,0.0,4.0));
    
    
    // Test polar projection
    // check transformation using the identity
    std::vector<Eigen::Vector3d> scan_points;
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    scan.convertDepthMapToPointCloud(scan_points, transform);
    
    BOOST_CHECK(scan_points.size() == 8);
    for(unsigned i = 0; i < scan_points.size(); i++)
	BOOST_CHECK(scan_points[i].isApprox(ref_points[i], 1e-12));
    
    
    // check transformation with translation
    transform.translation() = Eigen::Vector3d(5.0,0.0,0.0);
    scan_points.clear();
    scan.convertDepthMapToPointCloud(scan_points, transform);
    
    BOOST_CHECK(scan_points.size() == 8);
    for(unsigned i = 0; i < scan_points.size(); i++)
	BOOST_CHECK(scan_points[i].isApprox(ref_points[i] + transform.translation(), 1e-12));
    
    
    // check transformation with translation and roations
    transform.setIdentity();
    transform.translation() = Eigen::Vector3d(-7.0,3.5,1.0);
    transform.rotate(Eigen::AngleAxisd(0.1*M_PI,Eigen::Vector3d::UnitZ()) * 
		    Eigen::AngleAxisd(0.2*M_PI,Eigen::Vector3d::UnitY()) * 
		    Eigen::AngleAxisd(-0.3*M_PI,Eigen::Vector3d::UnitX()));
    scan_points.clear();
    scan.convertDepthMapToPointCloud(scan_points, transform);
    
    BOOST_CHECK(scan_points.size() == 8);
    for(unsigned i = 0; i < scan_points.size(); i++)
	BOOST_CHECK(scan_points[i].isApprox(transform * ref_points[i], 1e-12));
    
    
    // use multiple transformations
    std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > transformations;
    transformations.push_back(transform);
    transformations.push_back(transform * transform);
    scan_points.clear();
    scan.convertDepthMapToPointCloud(scan_points, transformations, true, true, false);
    BOOST_CHECK(scan_points.size() == 8);
    for(unsigned i = 0; i < scan_points.size(); i++)
	BOOST_CHECK(scan_points[i].isApprox(transformations[(i%2==0)?0:1] * ref_points[i], 1e-12));
    
    scan_points.clear();
    scan.convertDepthMapToPointCloud(scan_points, transformations[0], transformations[1], true, true, false);
    BOOST_CHECK(scan_points.size() == 8);
    for(unsigned i = 0; i < scan_points.size(); i++)
	BOOST_CHECK(scan_points[i].isApprox(transformations[(i%2==0)?0:1] * ref_points[i], 1e-12));

    // check transformation with float types
    std::vector<Eigen::Vector3f> scan_points_f;
    Eigen::Affine3f transform_f = Eigen::Affine3f::Identity();
    scan.convertDepthMapToPointCloud(scan_points_f, transform_f);
    BOOST_CHECK(scan_points_f.size() == 8);
    for(unsigned i = 0; i < scan_points_f.size(); i++)
        BOOST_CHECK(scan_points_f[i].isApprox(ref_points[i].cast<float>(), 1e-6));

    scan_points_f.clear();
    scan.convertDepthMapToPointCloud(scan_points_f, transform_f, transform_f);
    BOOST_CHECK(scan_points_f.size() == 8);
    for(unsigned i = 0; i < scan_points_f.size(); i++)
        BOOST_CHECK(scan_points_f[i].isApprox(ref_points[i].cast<float>(), 1e-6));

    scan_points_f.clear();
    std::vector<Eigen::Affine3f, Eigen::aligned_allocator<Eigen::Affine3f> > transformations_f;
    transformations_f.push_back(transform_f);
    transformations_f.push_back(transform_f);
    scan.convertDepthMapToPointCloud(scan_points_f, transformations_f, true, true, false);
    BOOST_CHECK(scan_points_f.size() == 8);
    for(unsigned i = 0; i < scan_points_f.size(); i++)
        BOOST_CHECK(scan_points_f[i].isApprox(ref_points[i].cast<float>(), 1e-6));
    
    
    // Test vertical irregular transformation
    // add irregular scan angles
    scan.vertical_interval.clear();
    scan.timestamps.clear();
    for(unsigned j = 0; j < scan.vertical_size; j++)
    {
	scan.timestamps.push_back(base::Time::now());
	scan.vertical_interval.push_back(base::Angle::fromDeg(90.0 * j).getRad());
    }
    scan_points.clear();
    scan.convertDepthMapToPointCloud(scan_points, Eigen::Affine3d::Identity());
    BOOST_CHECK(scan_points.size() == 8);
    for(unsigned i = 0; i < scan_points.size(); i++)
 	BOOST_CHECK(scan_points[i].isApprox(ref_points[i], 1e-12));
    
    
    // check transformation with translation and roations
    scan_points.clear();
    scan.convertDepthMapToPointCloud(scan_points, transform);
    BOOST_CHECK(scan_points.size() == 8);
    for(unsigned i = 0; i < scan_points.size(); i++)
 	BOOST_CHECK(scan_points[i].isApprox(transform * ref_points[i], 1e-12));
    
    
    // use multiple transformations
    Eigen::Affine3d delta = Eigen::Affine3d::Identity();
    delta.translation() = Eigen::Vector3d(0.1,-0.2,-0.02);
    delta.rotate(Eigen::AngleAxisd(-0.05*M_PI,Eigen::Vector3d::UnitZ()) * 
		    Eigen::AngleAxisd(-0.02*M_PI,Eigen::Vector3d::UnitY()) * 
		    Eigen::AngleAxisd(0.07*M_PI,Eigen::Vector3d::UnitX()));
    transformations.clear();
    for(unsigned i = 1; i <= scan.vertical_size; i++)
	transformations.push_back(transform * Eigen::Affine3d(delta.matrix() * (double)i));
    scan_points.clear();
    scan.convertDepthMapToPointCloud(scan_points, transformations);
    BOOST_CHECK(scan_points.size() == 8);
    for(unsigned i = 0; i < scan_points.size(); i++)
 	BOOST_CHECK(scan_points[i].isApprox(transformations[i/2] * ref_points[i], 1e-12));
    
    
    // don't skip invalid points
    scan_points.clear();
    scan.convertDepthMapToPointCloud(scan_points, transform, false, false);

    BOOST_CHECK(scan_points.size() == 10);
    for(unsigned i = 0; i < scan_points.size(); i++)
	BOOST_CHECK((i < 8) ? base::isnotnan(scan_points[i]) : !base::isnotnan(scan_points[i]));



    // Test planar projection
    // setup scan and reference points
    ref_points.clear();
    scan.reset();
    scan.horizontal_projection = base::samples::DepthMap::PLANAR;
    scan.vertical_projection = base::samples::DepthMap::PLANAR;
    scan.horizontal_interval.push_back(-1.0);
    scan.horizontal_interval.push_back(1.0);
    scan.vertical_interval.push_back(-0.5);
    scan.vertical_interval.push_back(0.5);
    scan.horizontal_size = 8;
    scan.vertical_size = 5;
    double h_step_width = (scan.horizontal_interval.back() - scan.horizontal_interval.front()) / (double)(scan.horizontal_size-1);
    double v_step_width = (scan.vertical_interval.back() - scan.vertical_interval.front()) / (double)(scan.vertical_size-1);

    for(unsigned j = 0; j < scan.vertical_size; j++)
    {
	for(unsigned i = 0; i < scan.horizontal_size; i++)
	{
	    // fill distances
	    double distance = 0.1 * (((double)i) * scan.vertical_size + (double)j + 0.1);
	    scan.distances.push_back(distance);
	    
	    // compute reference point
	    Eigen::Vector3d ref_point(distance, 0.0, 0.0);
	    ref_points.push_back(Eigen::AngleAxisd(atan(-1.0 * (scan.horizontal_interval.front() + (double)i * h_step_width)), Eigen::Vector3d::UnitZ()) *
				Eigen::AngleAxisd(atan((scan.vertical_interval.front() + (double)j * v_step_width)), Eigen::Vector3d::UnitY()) *
				ref_point);
	}
    }

    // check identity transformation
    scan_points.clear();
    scan.convertDepthMapToPointCloud(scan_points);
    BOOST_CHECK(scan_points.size() == scan.vertical_size * scan.horizontal_size);
    for(unsigned i = 0; i < scan_points.size(); i++)
    {
	BOOST_CHECK(scan_points[i].isApprox(ref_points[i], 1e-6));
    }

    // check with transformation
    scan_points.clear();
    scan.convertDepthMapToPointCloud(scan_points, transform);
    BOOST_CHECK(scan_points.size() == scan.vertical_size * scan.horizontal_size);
    for(unsigned i = 0; i < scan_points.size(); i++)
    {
	BOOST_CHECK(scan_points[i].isApprox(transform * ref_points[i], 1e-6));
    }


    // check measurement states
    scan.reset();
    scan.distances.push_back(1.0);
    scan.distances.push_back(0.0);
    scan.distances.push_back(base::infinity<float>());
    scan.distances.push_back(base::NaN<float>());
    scan.horizontal_size = 1;
    scan.vertical_size = 4;
    
    BOOST_CHECK(scan.getIndexState(0) == base::samples::DepthMap::VALID_MEASUREMENT);
    BOOST_CHECK(scan.getIndexState(1) == base::samples::DepthMap::TOO_NEAR);
    BOOST_CHECK(scan.getIndexState(2) == base::samples::DepthMap::TOO_FAR);
    BOOST_CHECK(scan.getIndexState(3) == base::samples::DepthMap::MEASUREMENT_ERROR);
    BOOST_CHECK(scan.getMeasurementState(3,0) == base::samples::DepthMap::MEASUREMENT_ERROR);
    BOOST_CHECK(scan.isIndexValid(0));
    BOOST_CHECK(!scan.isIndexValid(1));
    BOOST_CHECK(!scan.isIndexValid(2));
    BOOST_CHECK(!scan.isIndexValid(3));
    
    
    // check exceptions
    scan.vertical_size = 5;
    BOOST_CHECK_THROW(scan.isIndexValid(4), std::out_of_range);
    BOOST_CHECK_THROW(scan.getIndexState(4), std::out_of_range);
    BOOST_CHECK_THROW(scan.getMeasurementState(0,0), std::out_of_range);
    BOOST_CHECK_THROW(scan.convertDepthMapToPointCloud(scan_points, transform), std::out_of_range);
    scan.vertical_size = 2;
    BOOST_CHECK_THROW(scan.convertDepthMapToPointCloud(scan_points, transform), std::out_of_range);
    scan.vertical_size = 4;
    BOOST_CHECK_THROW(scan.getMeasurementState(5,0), std::out_of_range);
    BOOST_CHECK_THROW(scan.getMeasurementState(0,1), std::out_of_range);
    scan.convertDepthMapToPointCloud(scan_points, transform);
    BOOST_CHECK(scan_points.size() == 1);

    // check horizontal angular interpolation
    scan.reset();
    scan.horizontal_size = 3;
    scan.vertical_size = 1;
    scan.vertical_projection = base::samples::DepthMap::POLAR;
    scan.horizontal_projection = base::samples::DepthMap::POLAR;
    scan.distances.push_back(sqrt(2.0));
    scan.distances.push_back(1.0);
    scan.distances.push_back(sqrt(2.0));
    scan.vertical_interval.push_back(0.0);
    scan.horizontal_interval.push_back(M_PI - M_PI_4);
    scan.horizontal_interval.push_back(-M_PI + M_PI_4);
    scan_points.clear();
    scan.convertDepthMapToPointCloud(scan_points);
    BOOST_CHECK(scan_points.size() == scan.vertical_size * scan.horizontal_size);
    BOOST_CHECK(scan_points[0].isApprox(Eigen::Vector3d(-1.0,1.0,0.0), 1e-6));
    BOOST_CHECK(scan_points[1].isApprox(Eigen::Vector3d(1.0,0.0,0.0), 1e-6));
    BOOST_CHECK(scan_points[2].isApprox(Eigen::Vector3d(-1.0,-1.0,0.0), 1e-6));

    scan.horizontal_interval.clear();
    scan.horizontal_interval.push_back(-M_PI_4);
    scan.horizontal_interval.push_back(-2.0*M_PI+M_PI_4);
    scan_points.clear();
    scan.convertDepthMapToPointCloud(scan_points);
    BOOST_CHECK(scan_points.size() == scan.vertical_size * scan.horizontal_size);
    BOOST_CHECK(scan_points[0].isApprox(Eigen::Vector3d(1.0,-1.0,0.0), 1e-6));
    BOOST_CHECK(scan_points[1].isApprox(Eigen::Vector3d(-1.0,0.0,0.0), 1e-6));
    BOOST_CHECK(scan_points[2].isApprox(Eigen::Vector3d(1.0,1.0,0.0), 1e-6));

    scan.horizontal_interval.clear();
    scan.horizontal_interval.push_back(M_PI_4);
    scan.horizontal_interval.push_back(-M_PI_4);
    scan_points.clear();
    scan.convertDepthMapToPointCloud(scan_points);
    BOOST_CHECK(scan_points.size() == scan.vertical_size * scan.horizontal_size);
    BOOST_CHECK(scan_points[0].isApprox(Eigen::Vector3d(1.0,1.0,0.0), 1e-6));
    BOOST_CHECK(scan_points[1].isApprox(Eigen::Vector3d(1.0,0.0,0.0), 1e-6));
    BOOST_CHECK(scan_points[2].isApprox(Eigen::Vector3d(1.0,-1.0,0.0), 1e-6));

    // check vertical angular interpolation
    scan.reset();
    scan.horizontal_size = 1;
    scan.vertical_size = 3;
    scan.vertical_projection = base::samples::DepthMap::POLAR;
    scan.horizontal_projection = base::samples::DepthMap::POLAR;
    scan.distances.push_back(sqrt(2.0));
    scan.distances.push_back(1.0);
    scan.distances.push_back(sqrt(2.0));
    scan.horizontal_interval.push_back(0.0);
    scan.vertical_interval.push_back(-M_PI + M_PI_4);
    scan.vertical_interval.push_back(M_PI - M_PI_4);
    scan_points.clear();
    scan.convertDepthMapToPointCloud(scan_points);
    BOOST_CHECK(scan_points.size() == scan.vertical_size * scan.horizontal_size);
    BOOST_CHECK(scan_points[0].isApprox(Eigen::Vector3d(-1.0,0.0,1.0), 1e-6));
    BOOST_CHECK(scan_points[1].isApprox(Eigen::Vector3d(1.0,0.0,0.0), 1e-6));
    BOOST_CHECK(scan_points[2].isApprox(Eigen::Vector3d(-1.0,0.0,-1.0), 1e-6));

    scan.vertical_interval.clear();
    scan.vertical_interval.push_back(M_PI_4);
    scan.vertical_interval.push_back(2.0*M_PI-M_PI_4);
    scan_points.clear();
    scan.convertDepthMapToPointCloud(scan_points);
    BOOST_CHECK(scan_points.size() == scan.vertical_size * scan.horizontal_size);
    BOOST_CHECK(scan_points[0].isApprox(Eigen::Vector3d(1.0,0.0,-1.0), 1e-6));
    BOOST_CHECK(scan_points[1].isApprox(Eigen::Vector3d(-1.0,0.0,0.0), 1e-6));
    BOOST_CHECK(scan_points[2].isApprox(Eigen::Vector3d(1.0,0.0,1.0), 1e-6));

    scan.vertical_interval.clear();
    scan.vertical_interval.push_back(-M_PI_4);
    scan.vertical_interval.push_back(M_PI_4);
    scan_points.clear();
    scan.convertDepthMapToPointCloud(scan_points);
    BOOST_CHECK(scan_points.size() == scan.vertical_size * scan.horizontal_size);
    BOOST_CHECK(scan_points[0].isApprox(Eigen::Vector3d(1.0,0.0,1.0), 1e-6));
    BOOST_CHECK(scan_points[1].isApprox(Eigen::Vector3d(1.0,0.0,0.0), 1e-6));
    BOOST_CHECK(scan_points[2].isApprox(Eigen::Vector3d(1.0,0.0,-1.0), 1e-6));
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

BOOST_AUTO_TEST_CASE( min_max_angle_test )
{
    using namespace base;

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

BOOST_AUTO_TEST_CASE( transform_with_covariance )
{
    // test if the relative transform also 
    // takes the uncertainty into account
    base::Matrix6d lt1; 
    lt1 <<
	0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 
	0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
	0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
	0.0, 0.0, 0.0, -3.0, 0.0, 0.0, 
	0.0, 0.0, 0.0, 0.0, -2.0, 0.0, 
	0.0, 0.0, 0.0, 0.0, 0.0, -1.0;

    base::TransformWithCovariance t1(
	    Eigen::Affine3d(Eigen::Translation3d(Eigen::Vector3d(1,0,0)) * Eigen::AngleAxisd( M_PI/2.0, Eigen::Vector3d::UnitX()) ),
	    lt1 );

    base::Matrix6d lt2; 
    lt2 <<
	0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
	0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
	0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 
	0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 
	0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 
	0.0, 0.0, 0.0, 0.0, 0.0, 3.0;

    base::TransformWithCovariance t2(
	    Eigen::Affine3d(Eigen::Translation3d(Eigen::Vector3d(0,1,2)) * Eigen::AngleAxisd( M_PI/2.0, Eigen::Vector3d::UnitY()) ),
	    lt2 );

    // chain a transform with uncertainty
    base::TransformWithCovariance tr = t2 * t1;

    // and recover the second transform
    base::TransformWithCovariance t2r = tr.compositionInv( t1 );
    base::TransformWithCovariance t1r = tr.preCompositionInv( t2 );

    const double sigma = 1e-12;

    BOOST_CHECK( t2.getTransform().matrix().isApprox( t2r.getTransform().matrix(), sigma ) );
    BOOST_CHECK( t2.getCovariance().isApprox( t2r.getCovariance(), sigma ) );

    BOOST_CHECK( t1.getTransform().matrix().isApprox( t1r.getTransform().matrix(), sigma ) );
    BOOST_CHECK( t1.getCovariance().isApprox( t1r.getCovariance(), sigma ) );
}

BOOST_AUTO_TEST_CASE( pose_with_covariance )
{
    base::samples::RigidBodyState rbs;
    rbs.time = base::Time::fromSeconds(1000);
    rbs.sourceFrame = "laser";
    rbs.targetFrame = "body";
    rbs.position = base::Vector3d(1,2,3);
    rbs.cov_position = 0.1 * base::Matrix3d::Identity();
    rbs.orientation = Eigen::AngleAxisd(0.5*M_PI,Eigen::Vector3d::UnitZ()) *
                        Eigen::AngleAxisd(-0.2*M_PI,Eigen::Vector3d::UnitY()) *
                        Eigen::AngleAxisd(0.1*M_PI,Eigen::Vector3d::UnitX());
    rbs.cov_orientation = 0.02 * base::Matrix3d::Identity();

    base::samples::PoseWithCovariance t(rbs);
    BOOST_CHECK(rbs.time == t.time);
    BOOST_CHECK(rbs.sourceFrame == t.object_frame_id);
    BOOST_CHECK(rbs.targetFrame == t.frame_id);
    BOOST_CHECK(rbs.position.isApprox(t.transform.translation, 1e-12));
    BOOST_CHECK(rbs.cov_position.isApprox(t.transform.getTranslationCov(), 1e-12));
    BOOST_CHECK(rbs.orientation.isApprox(t.transform.orientation, 1e-12));
    BOOST_CHECK(rbs.cov_orientation.isApprox(t.transform.getOrientationCov(), 1e-12));

    base::samples::RigidBodyState rbs2 = t.toRigidBodyState();
    BOOST_CHECK(rbs.time == rbs2.time);
    BOOST_CHECK(rbs.sourceFrame == rbs2.sourceFrame);
    BOOST_CHECK(rbs.targetFrame == rbs2.targetFrame);
    BOOST_CHECK(rbs.position.isApprox(rbs2.position, 1e-12));
    BOOST_CHECK(rbs.cov_position.isApprox(rbs2.cov_position, 1e-12));
    BOOST_CHECK(rbs.orientation.isApprox(rbs2.orientation, 1e-12));
    BOOST_CHECK(rbs.cov_orientation.isApprox(rbs2.cov_orientation, 1e-12));

    // test operator*
    base::Matrix6d lt1;
    lt1 <<
        0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, -3.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, -2.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, -1.0;
    base::samples::PoseWithCovariance body_in_world(base::TransformWithCovariance(
            Eigen::Affine3d(Eigen::Translation3d(Eigen::Vector3d(1,0,0)) * Eigen::AngleAxisd( M_PI/2.0, Eigen::Vector3d::UnitX()) ),
            lt1 ));
    body_in_world.time = base::Time();

    base::Matrix6d lt2;
    lt2 <<
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.2, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 2.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 3.0;
    base::samples::PoseWithCovariance sensor_in_body(base::TransformWithCovariance(
            Eigen::Affine3d(Eigen::Translation3d(Eigen::Vector3d(0,1,2)) * Eigen::AngleAxisd( M_PI/2.0, Eigen::Vector3d::UnitY()) ),
            lt2 ));
    sensor_in_body.time = base::Time::now();

    body_in_world.frame_id = "world";
    body_in_world.object_frame_id = "body";
    sensor_in_body.object_frame_id = "sensor";
    sensor_in_body.frame_id = "body";
    base::samples::PoseWithCovariance sensor_in_world = body_in_world * sensor_in_body;

    // check composition
    BOOST_CHECK(sensor_in_world.object_frame_id == sensor_in_body.object_frame_id);
    BOOST_CHECK(sensor_in_world.frame_id == body_in_world.frame_id);
    BOOST_CHECK(sensor_in_world.time == sensor_in_body.time);

    base::samples::PoseWithCovariance body_in_world_r(sensor_in_world.transform.compositionInv(sensor_in_body.transform));
    base::samples::PoseWithCovariance sensor_in_body_r(sensor_in_world.transform.preCompositionInv(body_in_world.transform));

    BOOST_CHECK( body_in_world.getTransform().matrix().isApprox( body_in_world_r.getTransform().matrix(), 1e-12 ) );
    BOOST_CHECK( body_in_world.getCovariance().isApprox( body_in_world_r.getCovariance(), 1e-12 ) );

    BOOST_CHECK( sensor_in_body.getTransform().matrix().isApprox( sensor_in_body_r.getTransform().matrix(), 1e-12 ) );
    BOOST_CHECK( sensor_in_body.getCovariance().isApprox( sensor_in_body_r.getCovariance(), 1e-12 ) );
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
