#include <boost/test/unit_test.hpp>
#include <base/samples/Sonar.hpp>

using namespace base;
using namespace base::samples;

BOOST_AUTO_TEST_SUITE(samples_Sonar)

BOOST_AUTO_TEST_CASE(resize_sets_the_beam_and_bin_count_fields)
{
    Sonar sonar;
    sonar.resize(10, 20, false);
    BOOST_REQUIRE_EQUAL(10, sonar.bin_count);
    BOOST_REQUIRE_EQUAL(20, sonar.beam_count);
}

BOOST_AUTO_TEST_CASE(resize_resizes_the_bearings_based_on_beam_count)
{
    Sonar sonar;
    sonar.resize(10, 20, false);
    BOOST_REQUIRE_EQUAL(20, sonar.bearings.size());
}

BOOST_AUTO_TEST_CASE(resize_resizes_the_bins_according_to_bin_and_beam_count)
{
    Sonar sonar;
    sonar.resize(10, 20, false);
    BOOST_REQUIRE_EQUAL(200, sonar.bins.size());
}

BOOST_AUTO_TEST_CASE(resize_resizes_the_timestamps_if_requested)
{
    { Sonar sonar;
        sonar.resize(10, 20, true);
        BOOST_REQUIRE_EQUAL(20, sonar.timestamps.size()); }

    { Sonar sonar;
        sonar.resize(10, 20, false);
        BOOST_REQUIRE(sonar.timestamps.empty()); }
}

BOOST_AUTO_TEST_CASE(clears_the_timestamps_if_per_beam_timestamp_is_not_requested)
{
    Sonar sonar;
    sonar.resize(10, 20, true);
    sonar.resize(10, 20, false);
    BOOST_REQUIRE(sonar.timestamps.empty());
}


BOOST_AUTO_TEST_CASE(getBeamRelativeStartTime_returns_bin_duration_scaled_to_index)
{
    Sonar sonar;
    sonar.bin_duration = Time::fromMilliseconds(1);
    BOOST_REQUIRE_EQUAL(Time::fromMilliseconds(1) * 5, sonar.getBinRelativeStartTime(5));
}

BOOST_AUTO_TEST_CASE(getBeamAcquisitionStartTime_returns_the_per_beam_timestamp_if_available)
{
    Sonar sonar;
    sonar.timestamps.push_back(Time::fromMilliseconds(1));
    BOOST_REQUIRE_EQUAL(Time::fromMilliseconds(1), sonar.getBeamAcquisitionStartTime(0));
}

BOOST_AUTO_TEST_CASE(getBeamAcquisitionStartTime_returns_the_global_timestamp_if_no_per_beam_timestamps_are_available)
{
    Sonar sonar;
    sonar.time = Time::fromMilliseconds(1);
    BOOST_REQUIRE_EQUAL(Time::fromMilliseconds(1), sonar.getBeamAcquisitionStartTime(0));
}

BOOST_AUTO_TEST_CASE(getBinTime_returns_the_absolute_bin_time)
{
    { Sonar sonar;
        sonar.timestamps.push_back(Time::fromMilliseconds(0));
        sonar.timestamps.push_back(Time::fromMilliseconds(1));
        sonar.bin_duration = Time::fromMilliseconds(2);
        BOOST_REQUIRE_EQUAL(Time::fromMilliseconds(1), sonar.getBinTime(0, 1));
        BOOST_REQUIRE_EQUAL(Time::fromMilliseconds(3), sonar.getBinTime(1, 1));
    }

    { Sonar sonar;
        sonar.time = Time::fromMilliseconds(1);
        sonar.bin_duration = Time::fromMilliseconds(2);
        BOOST_REQUIRE_EQUAL(Time::fromMilliseconds(1), sonar.getBinTime(0, 1));
        BOOST_REQUIRE_EQUAL(Time::fromMilliseconds(3), sonar.getBinTime(1, 1));
    }
}

BOOST_AUTO_TEST_CASE(getBinStartDistance_returns_the_start_time_scaled_by_the_speed_of_sound)
{
    Sonar sonar;
    sonar.speed_of_sound = 12.34;
    sonar.bin_duration = Time::fromMilliseconds(2);
    BOOST_REQUIRE_SMALL(12.34 * 0.01 - sonar.getBinStartDistance(5), 1e-6);
}

BOOST_AUTO_TEST_CASE(setRegularBeamBearings_sets_regular_beam_bearings_for_the_existing_beam_count)
{
    Sonar sonar;
    sonar.resize(5, 3, false);
    sonar.setRegularBeamBearings(Angle::fromRad(-0.24), Angle::fromRad(0.54));
    BOOST_REQUIRE_EQUAL(3, sonar.bearings.size());
    BOOST_REQUIRE_SMALL(-0.24 - sonar.bearings[0].getRad(), 1e-6);
    BOOST_REQUIRE_SMALL(-0.24 + 0.54 - sonar.bearings[1].getRad(), 1e-6);
    BOOST_REQUIRE_SMALL(-0.24 + 2 * 0.54 - sonar.bearings[2].getRad(), 1e-6);
}

BOOST_AUTO_TEST_CASE(pushBeam_overloads_without_time_throw_if_called_on_a_structure_with_per_timestamp)
{
    Sonar sonar;
    sonar.resize(0, 5, true);
    BOOST_REQUIRE_THROW(sonar.pushBeam(std::vector<float>()), std::invalid_argument);
    BOOST_REQUIRE_THROW(sonar.pushBeam(std::vector<float>(), Angle()), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(pushBeam_all_overloads_add_the_beam)
{
    Sonar sonar;
    sonar.resize(2, 0, false);
    { std::vector<float> bins; bins.push_back(1); bins.push_back(2);
        sonar.pushBeam(bins); }
    { std::vector<float> bins; bins.push_back(3); bins.push_back(4);
        sonar.pushBeam(bins, Angle()); }
    { std::vector<float> bins; bins.push_back(5); bins.push_back(6);
        sonar.pushBeam(Time(), bins); }
    { std::vector<float> bins; bins.push_back(7); bins.push_back(8);
        sonar.pushBeam(Time(), bins, Angle()); }
    BOOST_REQUIRE_EQUAL(8, sonar.bins.size());
    for (int i = 0; i < 8; ++i)
        BOOST_REQUIRE_EQUAL(float(i + 1), sonar.bins[i]);
}

BOOST_AUTO_TEST_CASE(pushBeam_overloads_with_time_add_the_time)
{
    Sonar sonar;
    sonar.resize(0, 0, false);
    sonar.pushBeam(Time::fromMilliseconds(1), std::vector<float>());
    BOOST_REQUIRE_EQUAL(Time::fromMilliseconds(1), sonar.getBeamAcquisitionStartTime(0));
    sonar.pushBeam(Time::fromMilliseconds(2), std::vector<float>(), Angle());
    BOOST_REQUIRE_EQUAL(Time::fromMilliseconds(2), sonar.getBeamAcquisitionStartTime(1));
}

BOOST_AUTO_TEST_CASE(pushBeam_overloads_with_angle_add_the_angle)
{
    Sonar sonar;
    sonar.resize(0, 0, false);
    sonar.pushBeam(std::vector<float>(), Angle::fromRad(1));
    BOOST_REQUIRE_SMALL(1 - sonar.getBeamBearing(0).getRad(), 1e-6);
    sonar.pushBeam(Time(), std::vector<float>(), Angle::fromRad(2));
    BOOST_REQUIRE_SMALL(2 - sonar.getBeamBearing(1).getRad(), 1e-6);
}

BOOST_AUTO_TEST_CASE(getBeamBins_returns_the_bins_of_a_beam)
{
    Sonar sonar;
    sonar.resize(2, 0, false);
    float bin0_values[] = { 1, 2 };
    std::vector<float> bin0(bin0_values, bin0_values + 2);
    float bin1_values[] = { 3, 4 };
    std::vector<float> bin1(bin1_values, bin1_values + 2);
    std::vector<float> getter;

    sonar.pushBeam(bin0);
    BOOST_REQUIRE(bin0 == sonar.getBeamBins(0));
    sonar.getBeamBins(0, getter);
    BOOST_REQUIRE(bin0 == getter);
    sonar.pushBeam(bin1);
    BOOST_REQUIRE(bin1 == sonar.getBeamBins(1));
    sonar.getBeamBins(1, getter);
    BOOST_REQUIRE(bin1 == getter);
}

BOOST_AUTO_TEST_CASE(getBeam_returns_a_Sonar_structure_valid_for_a_single_beam)
{
    Sonar sonar(Time::fromMilliseconds(1), Time::fromMilliseconds(2),
            2, Angle::fromRad(1), Angle::fromRad(2));
    sonar.speed_of_sound = 42;
    float bin0_values[] = { 1, 2 };
    std::vector<float> bin0(bin0_values, bin0_values + 2);
    float bin1_values[] = { 3, 4 };
    std::vector<float> bin1(bin1_values, bin1_values + 2);
    sonar.pushBeam(Time::fromMilliseconds(0), bin0, Angle::fromDeg(10));
    sonar.pushBeam(Time::fromMilliseconds(5), bin1, Angle::fromDeg(20));

    Sonar beam = sonar.getBeam(1);
    BOOST_CHECK_EQUAL(Time::fromMilliseconds(5), beam.time);
    BOOST_CHECK_EQUAL(Time::fromMilliseconds(2), beam.bin_duration);
    BOOST_CHECK_EQUAL(Angle::fromRad(1), beam.beam_width);
    BOOST_CHECK_EQUAL(Angle::fromRad(2), beam.beam_height);
    BOOST_CHECK_EQUAL(1, beam.bearings.size());
    BOOST_CHECK_EQUAL(Angle::fromDeg(20), beam.bearings[0]);
    BOOST_CHECK_EQUAL(42, beam.speed_of_sound);
    BOOST_CHECK_EQUAL(2, beam.bin_count);
    BOOST_CHECK_EQUAL(1, beam.beam_count);
    BOOST_CHECK(bin1 == beam.bins);
}

BOOST_AUTO_TEST_CASE(pushBeam_throws_if_the_number_of_bins_do_not_match_the_expected_count)
{
    Sonar sonar;
    sonar.resize(1, 0, false);
    BOOST_REQUIRE_THROW(sonar.pushBeam(std::vector<float>()), std::invalid_argument);
    BOOST_REQUIRE_THROW(sonar.pushBeam(std::vector<float>(), Angle()), std::invalid_argument);
    BOOST_REQUIRE_THROW(sonar.pushBeam(Time(), std::vector<float>()), std::invalid_argument);
    BOOST_REQUIRE_THROW(sonar.pushBeam(Time(), std::vector<float>(), Angle()), std::invalid_argument);
}

BOOST_AUTO_TEST_SUITE_END()
