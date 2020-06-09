#include <boost/test/unit_test.hpp>
#include <base/samples/RigidBodyStateSE3.hpp>

using namespace std;
using namespace base;

BOOST_AUTO_TEST_SUITE(RigidBodyStateSE3Tests)

BOOST_AUTO_TEST_CASE(rbs_se3_initialization)
{
    RigidBodyStateSE3 rbs;
    for(int i = 0; i < 3; i++)
    {
        BOOST_CHECK(std::isnan(rbs.pose.position[i]));
        BOOST_CHECK(std::isnan(rbs.twist.linear[i]));
        BOOST_CHECK(std::isnan(rbs.twist.angular[i]));
        BOOST_CHECK(std::isnan(rbs.acceleration.linear[i]));
        BOOST_CHECK(std::isnan(rbs.acceleration.angular[i]));
        BOOST_CHECK(std::isnan(rbs.wrench.force[i]));
        BOOST_CHECK(std::isnan(rbs.wrench.torque[i]));
    }
    for(int i = 0; i < 4; i++)
        BOOST_CHECK(std::isnan(rbs.pose.orientation.coeffs()[i]));

    BOOST_CHECK(!rbs.hasValidPose());
    BOOST_CHECK(!rbs.hasValidTwist());
    BOOST_CHECK(!rbs.hasValidAcceleration());
    BOOST_CHECK(!rbs.hasValidWrench());
}

BOOST_AUTO_TEST_CASE(rbs_adjoint_operator_twist)
{
    Pose transform;
    transform.position = Vector3d::Random();
    transform.orientation = Quaterniond::UnitRandom();

    // Check adjoint transform for twist basis vectors
    for(int i = 0; i < 3; i++)
    {
        Twist tw(Vector3d::Zero(), Vector3d::Zero());
        tw.linear(i) = 1;
        BOOST_CHECK((transform * tw).linear.isApprox(transform.toTransform().matrix().col(i).segment(0,3)));
        BOOST_CHECK((transform * tw).angular.isApprox(base::Vector3d::Zero()));

        tw.linear.setZero();
        tw.angular(i) = 1;
        base::Vector3d ang_part = transform.toTransform().matrix().col(i).segment(0,3);
        BOOST_CHECK((transform * tw).linear.isApprox(-transform.position.cross(ang_part)));
        BOOST_CHECK((transform * tw).angular.isApprox(ang_part));
    }

    // Check inverse
    Twist tw_a(base::Vector3d::Random(), base::Vector3d::Random());
    Twist tw_b = transform * tw_a;
    Pose inv_transform;
    inv_transform.fromTransform(transform.toTransform().inverse());
    Twist tw_c = inv_transform * tw_b;
    BOOST_CHECK(tw_c.linear.isApprox(tw_a.linear));
    BOOST_CHECK(tw_c.angular.isApprox(tw_a.angular));
}

BOOST_AUTO_TEST_CASE(rbs_adjoint_operator_spatial_acc)
{
    Pose transform;
    transform.position = Vector3d::Random();
    transform.orientation = Quaterniond::UnitRandom();

    // Check adjoint transform for spatial acceleration basis vectors
    for(int i = 0; i < 3; i++)
    {
        Acceleration acc(Vector3d::Zero(), Vector3d::Zero());
        acc.linear(i) = 1;
        BOOST_CHECK((transform * acc).linear.isApprox(transform.toTransform().matrix().col(i).segment(0,3)));
        BOOST_CHECK((transform * acc).angular.isApprox(base::Vector3d::Zero()));

        acc.linear.setZero();
        acc.angular(i) = 1;
        base::Vector3d ang_part = transform.toTransform().matrix().col(i).segment(0,3);
        BOOST_CHECK((transform * acc).linear.isApprox(-transform.position.cross(ang_part)));
        BOOST_CHECK((transform * acc).angular.isApprox(ang_part));
    }

    // Check inverse of adjoint transform
    Acceleration acc_a(base::Vector3d::Random(), base::Vector3d::Random());
    Acceleration acc_b = transform * acc_a;
    Pose inv_transform;
    inv_transform.fromTransform(transform.toTransform().inverse());
    Acceleration acc_c = inv_transform * acc_b;
    BOOST_CHECK(acc_c.linear.isApprox(acc_a.linear));
    BOOST_CHECK(acc_c.angular.isApprox(acc_a.angular));
}

BOOST_AUTO_TEST_CASE(rbs_adjoint_operator_wrench)
{
    Pose transform;
    transform.position = Vector3d::Random();
    transform.orientation = Quaterniond::UnitRandom();

    // Check adjoint transform for wrench basis vectors
    Wrench wr;
    for(int i = 0; i < 3; i++)
    {
        wr.force.setZero();
        wr.torque.setZero();
        wr.force(i) = 1;
        base::Vector3d force_transformed = transform.toTransform().matrix().col(i).segment(0,3);
        BOOST_CHECK((transform * wr).force.isApprox(force_transformed));
        BOOST_CHECK((transform * wr).torque.isApprox(-transform.position.cross(force_transformed)));

        wr.force.setZero();
        wr.torque(i) = 1;
        base::Vector3d torque_transformed = transform.toTransform().matrix().col(i).segment(0,3);
        BOOST_CHECK((transform * wr).force.isApprox(base::Vector3d::Zero()));
        BOOST_CHECK((transform * wr).torque.isApprox(torque_transformed));
    }

    // Check inverse of adjoint transform
    Wrench wr_a;
    wr_a.force = base::Vector3d::Random();
    wr_a.torque = base::Vector3d::Random();
    Wrench wr_b = transform * wr_a;
    Pose inv_transform;
    inv_transform.fromTransform(transform.toTransform().inverse());
    Wrench wr_c = inv_transform * wr_b;
    BOOST_CHECK(wr_c.force.isApprox(wr_a.force));
    BOOST_CHECK(wr_c.torque.isApprox(wr_a.torque));
}

BOOST_AUTO_TEST_SUITE_END()
