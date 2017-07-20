#include <boost/test/unit_test.hpp>
#include <base/Time.hpp>
#include <base/Timeout.hpp>
#include <limits>

using namespace base;

BOOST_AUTO_TEST_SUITE(Timeout)

BOOST_AUTO_TEST_CASE(constructor_test)
{
	base::Timeout timeout(base::Time::fromMilliseconds(0));
	BOOST_CHECK(!timeout.elapsed());
	base::Timeout timeout_neg(base::Time::fromMilliseconds(-1));
	BOOST_CHECK(timeout_neg.elapsed());
}


BOOST_AUTO_TEST_CASE(elapsed_timeout_test)
{
	base::Timeout timeout(base::Time::fromMilliseconds(1));
	BOOST_CHECK(!timeout.elapsed());
	sleep(1);
	BOOST_CHECK(timeout.elapsed());
	BOOST_CHECK(timeout.elapsed(base::Time::fromMilliseconds(1)));
}

BOOST_AUTO_TEST_CASE(restart_timeout_test)
{
	base::Timeout timeout(base::Time::fromMilliseconds(1));
	sleep(1);
	timeout.restart();
	BOOST_CHECK(!timeout.elapsed());
}
BOOST_AUTO_TEST_CASE(timeleft_timeout_test)
{
	base::Timeout timeout(base::Time::fromMilliseconds(1));
	BOOST_CHECK(timeout.timeLeft().toMicroseconds()>0);
	sleep(1);
	BOOST_CHECK(timeout.timeLeft().toMicroseconds()<=0);
	base::Timeout max_timeout(base::Time::fromMilliseconds(0));
	BOOST_REQUIRE_EQUAL(max_timeout.timeLeft().toMicroseconds(),
			std::numeric_limits<int64_t>::max());
}

BOOST_AUTO_TEST_SUITE_END()
