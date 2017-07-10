import basetypes
from nose.tools import assert_equal, assert_raises_regexp, assert_almost_equal


def test_get_set_microseconds():
    t = basetypes.PyTime()
    m = 1000023
    t.microseconds = m
    assert_equal(t.microseconds, m)


def test_vector2d_ctor():
    v = basetypes.PyVector2d(1.0, 2.0)
    assert_equal(str(v), "[1.00, 2.00]")


def test_vector3d_ctor():
    v = basetypes.PyVector3d(1.0, 2.0, 3.0)
    assert_equal(str(v), "[1.00, 2.00, 3.00]")


def test_vector4d_ctor():
    v = basetypes.PyVector4d(1.0, 2.0, 3.0, 4.0)
    assert_equal(str(v), "[1.00, 2.00, 3.00, 4.00]")


def test_vector3d_get_set_data():
    v = basetypes.PyVector3d(1.0, 2.0, 3.0)
    v.x = 5.0
    v.y = 6.0
    v.z = 7.0
    assert_equal(v.x, 5.0)
    assert_equal(v.y, 6.0)
    assert_equal(v.z, 7.0)


def test_vector3d_array_access():
    v = basetypes.PyVector3d(1.0, 2.0, 3.0)
    assert_equal(v[0], 1.0)
    v[1] = 4.0
    assert_equal(v[1], 4.0)
    assert_raises_regexp(KeyError, "index must be", lambda i: v[i], -1)
    def assign(i):
        v[i] = 5.0
    assert_raises_regexp(KeyError, "index must be", assign, 3)


def test_norms():
    v = basetypes.PyVector3d(1.0, 2.0, 3.0)
    assert_almost_equal(v.norm(), 3.741657387)
    assert_equal(v.squared_norm(), 14.0)


def test_quaterniond_ctor():
    q = basetypes.PyQuaterniond(1.0, 0.0, 0.0, 0.0)
    assert_equal(str(q), "[im=1.00, real=(0.00, 0.00, 0.00)]")


def test_transform_with_cov_ctor():
    basetypes.PyTransformWithCovariance()


def test_transform_set_get_translation():
    p = basetypes.PyTransformWithCovariance()
    t = basetypes.PyVector3d(1.0, 2.0, 3.0)
    p.translation = t
    assert_equal(str(p.translation), "[1.00, 2.00, 3.00]")


def test_transform_set_get_orientation():
    p = basetypes.PyTransformWithCovariance()
    o = basetypes.PyQuaterniond(1.0, 0.0, 0.0, 0.0)
    p.orientation = o
    assert_equal(str(p.orientation), "[im=1.00, real=(0.00, 0.00, 0.00)]")


def test_joint_state_get_set_position():
    js = basetypes.PyJointState()
    js.position = 5.0
    assert_equal(js.position, 5.0)


def test_joint_state_get_set_speed():
    js = basetypes.PyJointState()
    js.speed = 5.0
    assert_equal(js.speed, 5.0)


def test_joint_state_get_set_effort():
    js = basetypes.PyJointState()
    js.effort = 5.0
    assert_equal(js.effort, 5.0)


def test_joint_state_get_set_raw():
    js = basetypes.PyJointState()
    js.raw = 5.0
    assert_equal(js.raw, 5.0)


def test_joint_state_get_set_acceleration():
    js = basetypes.PyJointState()
    js.acceleration = 5.0
    assert_equal(js.acceleration, 5.0)


def test_joint_state_factories():
    js = basetypes.PyJointState.Position(5.0)
    assert_equal(js.position, 5.0)
    js = basetypes.PyJointState.Speed(5.0)
    assert_equal(js.speed, 5.0)
    js = basetypes.PyJointState.Effort(5.0)
    assert_equal(js.effort, 5.0)
    js = basetypes.PyJointState.Raw(5.0)
    assert_equal(js.raw, 5.0)
    js = basetypes.PyJointState.Acceleration(5.0)
    assert_equal(js.acceleration, 5.0)


def test_rigid_body_state_get_set_time():
    rbs = basetypes.PyRigidBodyState()
    assert_equal(rbs.time.microseconds, 0)
    rbs.time.microseconds = 500
    assert_equal(rbs.time.microseconds, 500)
    time = basetypes.PyTime()
    time.microseconds = 1000
    rbs.time = time
    assert_equal(rbs.time.microseconds, 1000)


def test_rigid_body_state_get_set_source_frame():
    rbs = basetypes.PyRigidBodyState()
    assert_equal(rbs.source_frame, "")
    rbs.source_frame = "source_frame"
    assert_equal(rbs.source_frame, "source_frame")


def test_rigid_body_state_get_set_target_frame():
    rbs = basetypes.PyRigidBodyState()
    assert_equal(rbs.target_frame, "")
    rbs.target_frame = "target_frame"
    assert_equal(rbs.target_frame, "target_frame")
