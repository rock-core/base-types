#define BOOST_TEST_MODULE BaseTypes
#include <boost/test/included/unit_test.hpp>

#include "base/angle.h"
#include "base/time.h"
#include "base/timemark.h"
#include "base/pose.h"
#include "base/samples/imu.h"
#include "base/samples/laser_scan.h"
#include "base/samples/sonar_scan.h"
#include "base/samples/rigid_body_state.h"

#define BASE_LOG_DEBUG
#include "base/logging.h"

#include <Eigen/SVD>
#include <Eigen/LU>

using namespace std;

BOOST_AUTO_TEST_CASE( pose_test )
{
    Eigen::Vector3d pos( 10, -1, 20.5 );
    Eigen::Quaterniond orientation( Eigen::AngleAxisd( 0.2, Eigen::Vector3d(0.5, 1.4, 0.1) ) );

    base::Pose p( pos, orientation ); 
    Eigen::Transform3d t( p.toTransform() );

    BOOST_CHECK( pos.isApprox( t.translation() ) );
    BOOST_CHECK( orientation.isApprox( Eigen::Quaterniond(t.rotation()), 0.01 ) );

    cout << Eigen::Quaterniond(t.rotation()).coeffs().transpose() << endl;
    cout << orientation.coeffs().transpose() << endl;
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
    std::cout << a << std::endl;
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

BOOST_AUTO_TEST_CASE( logging_test )
{
        FILE* s = fopen("test.out", "w");
        BASE_LOG_CONFIGURE(INFO, s);
        BASE_LOG_INFO("info-message")
        std::string test("additional-argument");

        int number = 1000000;
        time_t start,stop;
        time(&start);
        for(int i = 0; i < number; i++)
        {
            BASE_LOG_FATAL("test fatal log %s", test.c_str())
        }
        time(&stop);
        double seconds = difftime(stop, start)/(number*1.0);
        printf("Estimated time per log msg %f seconds", seconds);
}

