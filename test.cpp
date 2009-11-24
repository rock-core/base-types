#define BOOST_TEST_MODULE BaseTypes
#include <boost/test/included/unit_test.hpp>

#include "base/time.h"

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

}
