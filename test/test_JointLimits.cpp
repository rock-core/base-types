#include <boost/test/unit_test.hpp>
#include <base/JointLimits.hpp>

using namespace std;
using namespace base;

namespace utf = boost::unit_test;

BOOST_AUTO_TEST_SUITE(JointLimitsTests)

BOOST_AUTO_TEST_CASE(it_saturates_joints_when_they_are_out_of_their_limits)
{
    base::JointLimits limits;
    limits.elements.resize(2);
    limits.names.resize(2);
    limits.elements[0].min.speed = -100;
    limits.elements[0].max.speed = 100;
    limits.elements[1].min.speed = -50;
    limits.elements[1].max.speed = 50;
    base::samples::Joints joints;
    joints.elements.push_back(base::JointState::Speed(200));
    joints.elements.push_back(base::JointState::Speed(-120));

    auto [saturated, new_joints] = limits.saturate(joints);
    BOOST_CHECK(saturated == true);
    BOOST_CHECK(new_joints.elements[0].speed == 100);
    BOOST_CHECK(new_joints.elements[1].speed == -50);
}

BOOST_AUTO_TEST_CASE(it_does_not_saturate_joints_when_they_are_within_their_limits)
{
    base::JointLimits limits;
    limits.elements.resize(2);
    limits.names.resize(2);
    limits.elements[0].min.speed = -100;
    limits.elements[0].max.speed = 100;
    limits.elements[1].min.speed = -50;
    limits.elements[1].max.speed = 50;
    base::samples::Joints joints;
    joints.elements.push_back(base::JointState::Speed(80));
    joints.elements.push_back(base::JointState::Speed(-20));

    auto [saturated, new_joints] = limits.saturate(joints);
    BOOST_CHECK(saturated == false);
    BOOST_CHECK(new_joints.elements[0].speed == 80);
    BOOST_CHECK(new_joints.elements[1].speed == -20);
}

BOOST_AUTO_TEST_SUITE_END()
