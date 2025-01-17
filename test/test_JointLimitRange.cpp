#include <boost/test/unit_test.hpp>
#include <base/JointLimitRange.hpp>

using namespace std;
using namespace base;

namespace utf = boost::unit_test;

BOOST_AUTO_TEST_SUITE(JointLimitRangeTests)

BOOST_AUTO_TEST_CASE(it_saturates_a_joint_state_of_position_when_it_is_out_of_its_limit)
{
    base::JointLimitRange limit = base::JointLimitRange::Position(-100, 100);
    base::JointState state = base::JointState::Position(200);

    auto [saturated, new_state] = limit.saturate(state);
    BOOST_CHECK(saturated == true);
    BOOST_CHECK(new_state.position == 100);
}

BOOST_AUTO_TEST_CASE(it_does_not_saturate_a_joint_state_of_position_when_it_is_within_its_limit)
{
    base::JointLimitRange limit = base::JointLimitRange::Position(-100, 100);
    base::JointState state = base::JointState::Position(80);

    auto [saturated, new_state] = limit.saturate(state);
    BOOST_CHECK(saturated == false);
    BOOST_CHECK(new_state.position == 80);
}

BOOST_AUTO_TEST_CASE(it_saturates_a_joint_state_of_speed_when_it_is_out_of_its_limit)
{
    base::JointLimitRange limit = base::JointLimitRange::Speed(-100, 100);
    base::JointState state = base::JointState::Speed(200);

    auto [saturated, new_state] = limit.saturate(state);
    BOOST_CHECK(saturated == true);
    BOOST_CHECK(new_state.speed == 100);
}

BOOST_AUTO_TEST_CASE(it_does_not_saturate_a_joint_state_of_speed_when_it_is_within_its_limit)
{
    base::JointLimitRange limit = base::JointLimitRange::Speed(-100, 100);
    base::JointState state = base::JointState::Speed(80);

    auto [saturated, new_state] = limit.saturate(state);
    BOOST_CHECK(saturated == false);
    BOOST_CHECK(new_state.speed == 80);
}

BOOST_AUTO_TEST_CASE(it_saturates_a_joint_state_of_effort_when_it_is_out_of_its_limit)
{
    base::JointLimitRange limit = base::JointLimitRange::Effort(-100, 100);
    base::JointState state = base::JointState::Effort(200);

    auto [saturated, new_state] = limit.saturate(state);
    BOOST_CHECK(saturated == true);
    BOOST_CHECK(new_state.effort == 100);
}

BOOST_AUTO_TEST_CASE(it_does_not_saturate_a_joint_state_of_effort_when_it_is_within_its_limit)
{
    base::JointLimitRange limit = base::JointLimitRange::Effort(-100, 100);
    base::JointState state = base::JointState::Effort(80);

    auto [saturated, new_state] = limit.saturate(state);
    BOOST_CHECK(saturated == false);
    BOOST_CHECK(new_state.effort == 80);
}

BOOST_AUTO_TEST_CASE(it_saturates_a_joint_state_of_raw_when_it_is_out_of_its_limit)
{
    base::JointLimitRange limit = base::JointLimitRange::Raw(-100, 100);
    base::JointState state = base::JointState::Raw(200);

    auto [saturated, new_state] = limit.saturate(state);
    BOOST_CHECK(saturated == true);
    BOOST_CHECK(new_state.raw == 100);
}

BOOST_AUTO_TEST_CASE(it_does_not_saturate_a_joint_state_of_raw_when_it_is_within_its_limit)
{
    base::JointLimitRange limit = base::JointLimitRange::Raw(-100, 100);
    base::JointState state = base::JointState::Raw(80);

    auto [saturated, new_state] = limit.saturate(state);
    BOOST_CHECK(saturated == false);
    BOOST_CHECK(new_state.raw == 80);
}

BOOST_AUTO_TEST_CASE(it_saturates_a_joint_state_of_acceleration_when_it_is_out_of_its_limit)
{
    base::JointLimitRange limit = base::JointLimitRange::Acceleration(-100, 100);
    base::JointState state = base::JointState::Acceleration(200);

    auto [saturated, new_state] = limit.saturate(state);
    BOOST_CHECK(saturated == true);
    BOOST_CHECK(new_state.acceleration == 100);
}

BOOST_AUTO_TEST_CASE(it_does_not_saturate_a_joint_state_of_acceleration_when_it_is_within_its_limit)
{
    base::JointLimitRange limit = base::JointLimitRange::Acceleration(-100, 100);
    base::JointState state = base::JointState::Acceleration(80);

    auto [saturated, new_state] = limit.saturate(state);
    BOOST_CHECK(saturated == false);
    BOOST_CHECK(new_state.acceleration == 80);
}

BOOST_AUTO_TEST_SUITE_END()
