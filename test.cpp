#define BOOST_TEST_MODULE BaseTypes
#include <boost/test/included/unit_test.hpp>

#include "base/time.h"
#include "base/pose.h"
#include "base/samples/imu.h"
#include "base/samples/laser_scan.h"
#include "base/samples/rigid_body_state.h"

#include <Eigen/SVD>
#include <Eigen/LU>

using namespace std;

BOOST_AUTO_TEST_CASE( time_test )
{
    double f = 0.5;
    base::Time t1 = base::Time::fromSeconds( f );
    BOOST_CHECK_EQUAL( t1.seconds, 0 );
    BOOST_CHECK_EQUAL( t1.microseconds, 500000 );
    
    f = 2.0;
    t1 = base::Time::fromSeconds( f );
    BOOST_CHECK_EQUAL( t1.seconds, 2 );
    BOOST_CHECK_EQUAL( t1.microseconds, 0 );

    f = -2.5;
    t1 = base::Time::fromSeconds( f );
    BOOST_CHECK_EQUAL( t1.seconds, -3 );
    BOOST_CHECK_EQUAL( t1.microseconds, 500000 );

    cout << t1 << endl;
    cout << base::Time(1,10) << endl;
}

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

