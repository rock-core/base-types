import basetypes
import numpy as np
from nose.tools import assert_equal, assert_raises_regexp, assert_almost_equal, assert_false, assert_true
from numpy.testing import assert_array_equal, assert_array_almost_equal


def test_get_set_microseconds():
    t = basetypes.Time()
    m = 1000023
    t.microseconds = m
    assert_equal(t.microseconds, m)


def test_vector2d_ctor():
    v = basetypes.Vector2d(1.0, 2.0)
    assert_equal(str(v), "[1.00, 2.00]")


def test_vector3d_ctor():
    v = basetypes.Vector3d(1.0, 2.0, 3.0)
    assert_equal(str(v), "[1.00, 2.00, 3.00]")


def test_vector4d_ctor():
    v = basetypes.Vector4d(1.0, 2.0, 3.0, 4.0)
    assert_equal(str(v), "[1.00, 2.00, 3.00, 4.00]")


def test_vector3d_get_set_data():
    v = basetypes.Vector3d(1.0, 2.0, 3.0)
    v.x = 5.0
    v.y = 6.0
    v.z = 7.0
    assert_equal(v.x, 5.0)
    assert_equal(v.y, 6.0)
    assert_equal(v.z, 7.0)


def test_vector3d_array_access():
    v = basetypes.Vector3d(1.0, 2.0, 3.0)
    assert_equal(v[0], 1.0)
    v[1] = 4.0
    assert_equal(v[1], 4.0)
    assert_raises_regexp(KeyError, "index must be", lambda i: v[i], -1)
    def assign(i):
        v[i] = 5.0
    assert_raises_regexp(KeyError, "index must be", assign, 3)


def test_matrix3d_get_set_data():
    m = basetypes.Matrix3d()
    m[0, 1] = 1.0
    assert_equal(m[0, 1], 1.0)
    random_state = np.random.RandomState(13)
    r = random_state.randn(3, 3)
    m = basetypes.Matrix3d(r)
    assert_array_almost_equal(m.toarray(), r)


def test_norms():
    v = basetypes.Vector3d(1.0, 2.0, 3.0)
    assert_almost_equal(v.norm(), 3.741657387)
    assert_equal(v.squared_norm(), 14.0)


def test_quaterniond_ctor():
    q = basetypes.Quaterniond(1.0, 0.0, 0.0, 0.0)
    assert_equal(str(q), "[im=1.00, real=(0.00, 0.00, 0.00)]")


def test_transform_with_cov_ctor():
    basetypes.TransformWithCovariance()


def test_transform_set_get_translation():
    p = basetypes.TransformWithCovariance()
    t = basetypes.Vector3d(1.0, 2.0, 3.0)
    p.translation = t
    assert_equal(str(p.translation), "[1.00, 2.00, 3.00]")


def test_transform_set_get_orientation():
    p = basetypes.TransformWithCovariance()
    o = basetypes.Quaterniond(1.0, 0.0, 0.0, 0.0)
    p.orientation = o
    assert_equal(str(p.orientation), "[im=1.00, real=(0.00, 0.00, 0.00)]")


def test_joint_state_get_set_position():
    js = basetypes.JointState()
    js.position = 5.0
    assert_equal(js.position, 5.0)


def test_joint_state_get_set_speed():
    js = basetypes.JointState()
    js.speed = 5.0
    assert_equal(js.speed, 5.0)


def test_joint_state_get_set_effort():
    js = basetypes.JointState()
    js.effort = 5.0
    assert_equal(js.effort, 5.0)


def test_joint_state_get_set_raw():
    js = basetypes.JointState()
    js.raw = 5.0
    assert_equal(js.raw, 5.0)


def test_joint_state_get_set_acceleration():
    js = basetypes.JointState()
    js.acceleration = 5.0
    assert_equal(js.acceleration, 5.0)


def test_joint_state_factories():
    js = basetypes.JointState.Position(5.0)
    assert_equal(js.position, 5.0)
    js = basetypes.JointState.Speed(5.0)
    assert_equal(js.speed, 5.0)
    js = basetypes.JointState.Effort(5.0)
    assert_equal(js.effort, 5.0)
    js = basetypes.JointState.Raw(5.0)
    assert_equal(js.raw, 5.0)
    js = basetypes.JointState.Acceleration(5.0)
    assert_equal(js.acceleration, 5.0)


def test_joints_resize():
    j = basetypes.Joints()
    assert_equal(j.size(), 0)
    assert_false(j.has_names())
    j.resize(5)
    assert_equal(j.size(), 5)
    assert_false(j.has_names())
    j.clear()
    assert_equal(j.size(), 0)


def test_joints_access():
    j = basetypes.Joints()
    j.resize(1)

    assert_equal(j.names[0], "")
    assert_false(j.elements[0].has_position())
    assert_false(j.elements[0].has_speed())
    assert_false(j.elements[0].has_effort())
    assert_false(j.elements[0].has_raw())
    assert_false(j.elements[0].has_acceleration())

    j.names[0] = "test_name"
    j.elements[0].position = 1.0
    assert_equal(j.names[0], "test_name")
    assert_true(j.elements[0].has_position())

    assert_equal(j["test_name"].position, 1.0)


def test_rigid_body_state_get_set_time():
    rbs = basetypes.RigidBodyState()
    assert_equal(rbs.time.microseconds, 0)
    rbs.time.microseconds = 500
    assert_equal(rbs.time.microseconds, 500)
    time = basetypes.Time()
    time.microseconds = 1000
    rbs.time = time
    assert_equal(rbs.time.microseconds, 1000)


def test_rigid_body_state_get_set_source_frame():
    rbs = basetypes.RigidBodyState()
    assert_equal(rbs.source_frame, "")
    rbs.source_frame = "source_frame"
    assert_equal(rbs.source_frame, "source_frame")


def test_rigid_body_state_get_set_target_frame():
    rbs = basetypes.RigidBodyState()
    assert_equal(rbs.target_frame, "")
    rbs.target_frame = "target_frame"
    assert_equal(rbs.target_frame, "target_frame")


def test_rigid_body_state_get_set_position():
    rbs = basetypes.RigidBodyState()
    assert_array_almost_equal(
        rbs.position.toarray(), np.array([np.nan, np.nan, np.nan]))
    rbs.position.x = 1.0
    rbs.position.y = 2.0
    rbs.position.z = 3.0
    assert_array_almost_equal(rbs.position.toarray(), np.array([1, 2, 3]))


def test_rigid_body_state_get_set_cov_position():
    rbs = basetypes.RigidBodyState()
    assert_array_almost_equal(
        rbs.cov_position.toarray(), np.ones((3, 3)) * np.nan)
    rbs.cov_position.fromarray(np.eye(3))
    assert_array_almost_equal(rbs.cov_position.toarray(), np.eye(3))


def test_rigid_body_state_get_set_orientation():
    rbs = basetypes.RigidBodyState()
    assert_array_almost_equal(
        rbs.orientation.toarray(), np.array([np.nan, np.nan, np.nan, np.nan]))
    rbs.orientation.fromarray(np.array([1.0, 2.0, 3.0, 4.0]))
    assert_array_almost_equal(
        rbs.orientation.toarray(), np.array([1.0, 2.0, 3.0, 4.0]))


def test_rigid_body_state_get_set_cov_orientation():
    rbs = basetypes.RigidBodyState()
    assert_array_almost_equal(
        rbs.cov_orientation.toarray(), np.ones((3, 3)) * np.nan)
    rbs.cov_orientation.fromarray(np.eye(3))
    assert_array_almost_equal(rbs.cov_orientation.toarray(), np.eye(3))


def test_rigid_body_state_get_set_velocity():
    rbs = basetypes.RigidBodyState()
    assert_array_almost_equal(
        rbs.velocity.toarray(), np.array([np.nan, np.nan, np.nan]))
    rbs.velocity.x = 1.0
    rbs.velocity.y = 2.0
    rbs.velocity.z = 3.0
    assert_array_almost_equal(rbs.velocity.toarray(), np.array([1, 2, 3]))


def test_rigid_body_state_get_set_cov_velocity():
    rbs = basetypes.RigidBodyState()
    assert_array_almost_equal(
        rbs.cov_velocity.toarray(), np.ones((3, 3)) * np.nan)
    rbs.cov_velocity.fromarray(np.eye(3))
    assert_array_almost_equal(rbs.cov_velocity.toarray(), np.eye(3))


def test_rigid_body_state_get_set_angular_velocity():
    rbs = basetypes.RigidBodyState()
    assert_array_almost_equal(
        rbs.angular_velocity.toarray(), np.array([np.nan, np.nan, np.nan]))
    rbs.angular_velocity.x = 1.0
    rbs.angular_velocity.y = 2.0
    rbs.angular_velocity.z = 3.0
    assert_array_almost_equal(rbs.angular_velocity.toarray(), np.array([1, 2, 3]))


def test_rigid_body_state_get_set_cov_angular_velocity():
    rbs = basetypes.RigidBodyState()
    assert_array_almost_equal(
        rbs.cov_angular_velocity.toarray(), np.ones((3, 3)) * np.nan)
    rbs.cov_angular_velocity.fromarray(np.eye(3))
    assert_array_almost_equal(rbs.cov_angular_velocity.toarray(), np.eye(3))


def test_create_frame_rgb():
    RGB = 2
    VALID = 1
    frame = basetypes.Frame(800, 600, 1, RGB, VALID, 800 * 600 * 3)
    assert_equal(frame.get_width(), 800)
    assert_equal(frame.get_height(), 600)
    assert_equal(frame.get_channel_count(), 3)
    assert_equal(frame.get_data_depth(), 1)
    image = frame.image
    assert_array_equal(image.shape, (800, 600, 3))
    assert_equal(image.dtype, np.uint8)
    frame.image = np.ones(image.shape, dtype=np.uint8)
    assert_true(np.all(frame.image == 1))
