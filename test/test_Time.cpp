#include <boost/test/unit_test.hpp>
#include <base/Time.hpp>
#include <iostream>

BOOST_AUTO_TEST_SUITE(Time)

BOOST_AUTO_TEST_CASE(fromSeconds)
{
    base::Time seconds;

    seconds = base::Time::fromSeconds(35.553);
    BOOST_REQUIRE_EQUAL(35553000, seconds.toMicroseconds());
    seconds = base::Time::fromSeconds(-5.553);
    BOOST_REQUIRE_EQUAL(-5553000, seconds.toMicroseconds());
    seconds = base::Time::fromSeconds(0.01);
    BOOST_REQUIRE_EQUAL(10000, seconds.toMicroseconds());
}

BOOST_AUTO_TEST_CASE(fromMicroseconds)
{
    base::Time microseconds;

    microseconds = base::Time::fromMicroseconds(-1);
    BOOST_REQUIRE_EQUAL(-1, microseconds.toMicroseconds());
    microseconds = base::Time::fromMicroseconds(1);
    BOOST_REQUIRE_EQUAL(1, microseconds.toMicroseconds());
}

BOOST_AUTO_TEST_CASE(fromMilliseconds)
{
    base::Time milliseconds;

    milliseconds = base::Time::fromMilliseconds(-1);
    BOOST_REQUIRE_EQUAL(-1000, milliseconds.toMicroseconds());
    milliseconds = base::Time::fromMilliseconds(1);
    BOOST_REQUIRE_EQUAL(1000, milliseconds.toMicroseconds());
}

BOOST_AUTO_TEST_CASE(operators)
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
    BOOST_REQUIRE_EQUAL(time_1*10, time_10);

    BOOST_CHECK(time_10/10==time_1);
    BOOST_REQUIRE_EQUAL(time_10/10, time_1);

    BOOST_CHECK(time_1_neg*(-1)==time_1);
    BOOST_REQUIRE_EQUAL(time_1_neg*(-1), time_1);
}

BOOST_AUTO_TEST_CASE(multiply)
{
    base::Time t = base::Time::fromSeconds(35);
    BOOST_REQUIRE_EQUAL(35 * 1e6 * 0.025, (t * 0.025).toMicroseconds());
}

BOOST_AUTO_TEST_CASE(fromString)
{
    base::Time now = base::Time::now();
    std::string nowString = now.toString(base::Time::Microseconds);
    base::Time expectedNow = base::Time::fromString(nowString);

    BOOST_REQUIRE_EQUAL(nowString, expectedNow.toString());
    BOOST_REQUIRE_EQUAL(now.toMicroseconds(), expectedNow.toMicroseconds());

    // Timezone conversion check -- since it depends on the current local time,
    // either summer or winter check would fail in case of an error
    // Summer
    base::Time tzOrig = base::Time::fromString("20120601-10:00:00", base::Time::Seconds);
    base::Time tzConverted = base::Time::fromString(tzOrig.toString());
    BOOST_REQUIRE_MESSAGE(
        tzOrig == tzConverted,
        "summer time: orig: " << tzOrig.toString() <<
        " vs. converted: " << tzConverted.toString()
    );

    // Winter
    tzOrig = base::Time::fromString("20121201-10:00:00", base::Time::Seconds);
    tzConverted = base::Time::fromString(tzOrig.toString());
    BOOST_REQUIRE_MESSAGE(
        tzOrig == tzConverted,
        "winter time: " << tzOrig.toString() <<
        " vs. converted: " << tzConverted.toString());
    // End time zone check

    tzset();
    // 1339675506 epoch at 2012-06-13 12:05:06 UTC
    // conversion done with https://www.epochconverter.com/
    std::cout << timezone << std::endl;
    uint64_t expected_utc_us = (1339675506 + timezone) * 1000000;

    base::Time formatNow = base::Time::fromString(
        "2012-06-14--12.05.06Z:001001",
        base::Time::Microseconds,
        "%Y-%m-%d--%H.%M.%S%Z"
    );
    BOOST_REQUIRE_EQUAL(formatNow.toMicroseconds(), expected_utc_us + 1001);

    base::Time expectedSecondResolutionOnly = base::Time::fromString(
        formatNow.toString(), base::Time::Seconds
    );
    BOOST_REQUIRE_EQUAL(expectedSecondResolutionOnly.toMicroseconds(),
                        expected_utc_us);

    base::Time expectedMillisecondResolutionOnly = base::Time::fromString(
        formatNow.toString(), base::Time::Milliseconds
    );
    BOOST_REQUIRE_EQUAL(expectedMillisecondResolutionOnly.toMicroseconds(),
                        expected_utc_us + 1000);

    std::string secondResolutionFormat = formatNow.toString(base::Time::Seconds);
    BOOST_REQUIRE_EQUAL(secondResolutionFormat, "20120614-12:05:06");

    std::string millisecondResolutionFormat =
        formatNow.toString(base::Time::Milliseconds);
    BOOST_REQUIRE_EQUAL(millisecondResolutionFormat, "20120614-12:05:06:001");

    BOOST_REQUIRE_THROW(base::Time::fromString(millisecondResolutionFormat,
                        base::Time::Microseconds), std::runtime_error);

    std::string microsecondResolutionFormat =
        formatNow.toString(base::Time::Microseconds);
    BOOST_REQUIRE_EQUAL(microsecondResolutionFormat, "20120614-12:05:06:001001");

    std::string customFormat =
        formatNow.toString(base::Time::Milliseconds, "Time: %Y%m%dT%H%M%S");
    BOOST_REQUIRE_EQUAL(customFormat, "Time: 20120614T120506:001");

    std::string defaultResolutionFormat = formatNow.toString();
    BOOST_REQUIRE_EQUAL(microsecondResolutionFormat, defaultResolutionFormat);

}

BOOST_AUTO_TEST_CASE(time_toTimeValues)
{
    base::Time time = base::Time::fromMicroseconds(93784005006);
    std::vector<int> timeValues = time.toTimeValues();

    for (size_t i = 0; i < timeValues.size(); ++i)
    {
        BOOST_CHECK_EQUAL(timeValues.at(i), timeValues.size() - i);
    }

    // Test maximum time.
    base::Time maxTime = base::Time::max();
    std::vector<int> maxTimeValues = maxTime.toTimeValues();
    int64_t maxTimeMicroseconds = maxTimeValues.at(0) +         // µs
                           1000ll * (maxTimeValues.at(1) +      // ms
                           1000ll * (maxTimeValues.at(2) +      // s
                           60ll * (maxTimeValues.at(3) +        // m
                           60ll * (maxTimeValues.at(4) +        // h
                           24ll * maxTimeValues.at(5)))));      // d
    BOOST_CHECK_EQUAL(maxTimeMicroseconds, maxTime.microseconds);

    // Test minimum time.
    base::Time minTime =
        base::Time::fromMicroseconds(std::numeric_limits<int64_t>::min());
    std::vector<int> minTimeValues = minTime.toTimeValues();
    int64_t minTimeMicroseconds = minTimeValues.at(0) +         // µs
                           1000ll * (minTimeValues.at(1) +      // ms
                           1000ll * (minTimeValues.at(2) +      // s
                           60ll * (minTimeValues.at(3) +        // m
                           60ll * (minTimeValues.at(4) +        // h
                           24ll * minTimeValues.at(5)))));      // d
    BOOST_CHECK_EQUAL(minTimeMicroseconds, minTime.microseconds);

    // Test maximum values per field.
    // Not really capped since there are no months, but covered by max time above.
    int64_t days = 31;
    int64_t hours = 23;
    int64_t minutes = 59;
    int64_t seconds = 59;
    int64_t milliseconds = 999;
    int64_t microseconds = 999;
    uint64_t usecs = ((((days
                     * 24 + hours)
                     * 60 + minutes)
                     * 60 + seconds)
                     * 1000 + milliseconds)
                     * 1000 + microseconds;
    base::Time maxValuesTime = base::Time::fromMicroseconds(usecs);
    std::vector<int> maxValuesTimeValues = maxValuesTime.toTimeValues();
    BOOST_CHECK_EQUAL(maxValuesTimeValues.at(0), microseconds);
    BOOST_CHECK_EQUAL(maxValuesTimeValues.at(1), milliseconds);
    BOOST_CHECK_EQUAL(maxValuesTimeValues.at(2), seconds);
    BOOST_CHECK_EQUAL(maxValuesTimeValues.at(3), minutes);
    BOOST_CHECK_EQUAL(maxValuesTimeValues.at(4), hours);
    BOOST_CHECK_EQUAL(maxValuesTimeValues.at(5), days);
}

BOOST_AUTO_TEST_SUITE_END()