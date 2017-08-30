import basetypes
import numpy as np
from nose.tools import assert_equal, assert_not_equal, assert_raises_regexp, \
    assert_almost_equal, assert_false, assert_true, assert_greater, \
    assert_regexp_matches
from numpy.testing import assert_array_equal, assert_array_almost_equal


def test_get_set_microseconds():
    t = basetypes.Time()
    m = 1000023
    t.microseconds = m
    assert_equal(t.microseconds, m)


def test_time_str():
    t = basetypes.Time.now()
    assert_regexp_matches(str(t), "<time=\d{8}-\d{2}:\d{2}:\d{2}:\d{6}>")


def test_no_overflow():
    t = basetypes.Time.now()
    mus = t.microseconds
    assert_greater(mus, 0)
    t.microseconds = mus


def test_time_operators():
    t1 = basetypes.Time()
    t1.microseconds = 0
    t2 = basetypes.Time()
    t2.microseconds = 1
    t3 = basetypes.Time()
    t3.microseconds = 1
    t4 = basetypes.Time()
    t4.microseconds = 2

    assert_true(t1 < t2 < t4)
    assert_true(t4 > t2 > t1)
    assert_true(t1 != t2)
    assert_false(t2 != t3)
    assert_true(t2 == t3)
    assert_false(t1 == t2)
    assert_true(t2 >= t3)
    assert_false(t2 > t3)
    assert_true(t2 <= t3)
    assert_false(t2 < t3)

    assert_true(t2 + t2 == t4)
    assert_true(t4 - t2 == t2)
    assert_true(t4 / 2 == t2)
    assert_true(t2 * 2 == t4)

    t5 = basetypes.Time()
    t5.microseconds = 10
    t5 /= 2
    assert_equal(t5.microseconds, 5)
    t5 -= t2 * 4
    assert_equal(t5, t2)
    t5 *= 2
    assert_equal(t5, t4)
    t5 += t1
    assert_equal(t5, t4)


def test_time_assign():
    t1 = basetypes.Time()
    t2 = basetypes.Time.now()
    assert_not_equal(t1, t2)
    t1.assign(t2)
    assert_equal(t1, t2)


def test_vector2d_ctor():
    v = basetypes.Vector2d(1.0, 2.0)
    assert_equal(str(v), "[1.00, 2.00]")


def test_vector2d_assign():
    obj1 = basetypes.Vector2d()
    obj2 = basetypes.Vector2d(1.0, 2.0)
    assert_not_equal(obj1, obj2)
    obj1.assign(obj2)
    assert_equal(obj1, obj2)


def test_vector2d_as_ndarray():
    random_state = np.random.RandomState(843)
    r = random_state.randn(2, 2)
    v = basetypes.Vector2d(3.23, 2.24)
    rv = r.dot(np.asarray(v))
    rv2 = r.dot(v.toarray())
    assert_array_almost_equal(rv, rv2)


def test_vector3d_ctor():
    v = basetypes.Vector3d(1.0, 2.0, 3.0)
    assert_equal(str(v), "[1.00, 2.00, 3.00]")


def test_vector3d_fromarray():
    v = basetypes.Vector3d()
    a = np.array([-2.32, 2.42, 54.23])
    v.fromarray(a)
    assert_equal(v[0], a[0])
    assert_equal(v[1], a[1])
    assert_equal(v[2], a[2])


def test_vector3d_as_ndarray():
    random_state = np.random.RandomState(843)
    r = random_state.randn(3, 3)
    v = basetypes.Vector3d(3.23, 2.24, 3.63)
    rv = r.dot(np.asarray(v))
    rv2 = r.dot(v.toarray())
    assert_array_almost_equal(rv, rv2)


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


def test_vector3d_assign():
    obj1 = basetypes.Vector3d()
    obj2 = basetypes.Vector3d(1.0, 2.0, 3.0)
    assert_not_equal(obj1, obj2)
    obj1.assign(obj2)
    assert_equal(obj1, obj2)


def test_norms():
    v = basetypes.Vector3d(1.0, 2.0, 3.0)
    assert_almost_equal(v.norm(), 3.741657387)
    assert_equal(v.squared_norm(), 14.0)


def test_vector4d_ctor():
    v = basetypes.Vector4d(1.0, 2.0, 3.0, 4.0)
    assert_equal(str(v), "[1.00, 2.00, 3.00, 4.00]")


def test_vector4d_as_ndarray():
    random_state = np.random.RandomState(843)
    r = random_state.randn(4, 4)
    v = basetypes.Vector4d(3.23, 2.24, 3.63, 2.05)
    rv = r.dot(np.asarray(v))
    rv2 = r.dot(v.toarray())
    assert_array_almost_equal(rv, rv2)


def test_vector4d_assign():
    obj1 = basetypes.Vector4d()
    obj2 = basetypes.Vector4d(1.0, 2.0, 3.0)
    assert_not_equal(obj1, obj2)
    obj1.assign(obj2)
    assert_equal(obj1, obj2)


def test_matrix3d_get_set_data():
    m = basetypes.Matrix3d()
    m[0, 1] = 1.0
    assert_equal(m[0, 1], 1.0)
    random_state = np.random.RandomState(13)
    r = random_state.randn(3, 3)
    m = basetypes.Matrix3d(r)
    assert_array_almost_equal(m.toarray(), r)


def test_matrix3d_array_access():
    random_state = np.random.RandomState(843)
    m = basetypes.Matrix3d()
    r = random_state.randn(3, 3)
    m.fromarray(r)
    assert_array_equal(np.asarray(m), r)


def test_matrix3d_assign():
    obj1 = basetypes.Matrix3d()
    obj2 = basetypes.Matrix3d()
    obj2.fromarray(np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]]))
    assert_not_equal(obj1, obj2)
    obj1.assign(obj2)
    assert_equal(obj1, obj2)


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
    assert_equal(str(js), "JointState [speed=5]")


def test_joint_state_get_set_effort():
    js = basetypes.JointState()
    js.effort = 5.0
    assert_equal(js.effort, 5.0)
    assert_equal(str(js), "JointState [effort=5]")


def test_joint_state_get_set_raw():
    js = basetypes.JointState()
    js.raw = 5.0
    assert_equal(js.raw, 5.0)
    assert_equal(str(js), "JointState [raw=5]")


def test_joint_state_get_set_acceleration():
    js = basetypes.JointState()
    js.acceleration = 5.0
    assert_equal(js.acceleration, 5.0)
    assert_equal(str(js), "JointState [acceleration=5]")


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


def test_joints_has_time():
    j = basetypes.Joints()
    assert_true(hasattr(j, "time"))


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


def test_joints_str():
    j = basetypes.Joints()
    j.names.resize(2)
    j.names[0] = "j1"
    j.names[1] = "j2"
    j.elements.resize(2)
    j.elements[0].position = 1.0
    j.elements[1].position = 2.0
    assert_equal(
        str(j), "Joints <time=19700101-01:00:00:000000> "
        "{j1: JointState [position=1], j2: JointState [position=2]}")


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


def test_rigid_body_state_str():
    rbs = basetypes.RigidBodyState()
    rbs.source_frame = "source"
    rbs.target_frame = "target"
    assert_equal(
        str(rbs), "RigidBodyState {<time=19700101-01:00:00:000000>, "
        "source_frame=source, target_frame=target, ...}")


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
    frame = basetypes.Frame(
        800, 600, 1, basetypes.frame_mode_t.MODE_RGB,
        basetypes.frame_status_t.STATUS_VALID, 800 * 600 * 3)
    assert_equal(frame.get_width(), 800)
    assert_equal(frame.get_height(), 600)
    assert_equal(frame.get_channel_count(), 3)
    assert_equal(frame.get_data_depth(), 1)
    image = frame.image
    assert_array_equal(image.shape, (800, 600, 3))
    assert_equal(image.dtype, np.uint8)
    frame.image = np.ones(image.shape, dtype=np.uint8)
    assert_true(np.all(frame.image == 1))


def test_create_frame_gray():
    frame = basetypes.Frame(
        800, 600, 1, basetypes.frame_mode_t.MODE_GRAYSCALE,
        basetypes.frame_status_t.STATUS_VALID, 800 * 600 * 1)
    assert_equal(frame.get_width(), 800)
    assert_equal(frame.get_height(), 600)
    assert_equal(frame.get_channel_count(), 1)
    assert_equal(frame.get_data_depth(), 1)
    image = frame.image
    assert_array_equal(image.shape, (800, 600))
    assert_equal(image.dtype, np.uint8)
    frame.image = np.ones(image.shape, dtype=np.uint8)
    assert_true(np.all(frame.image == 1))


def test_create_pointcloud():
    pcl = basetypes.Pointcloud()

    assert_equal(pcl.time.microseconds, 0)

    pcl.points.resize(100)
    point = pcl.points[0]
    point.x = 1.0
    point.y = 2.0
    point.z = 3.0
    assert_equal(pcl.points.size(), 100)
    assert_array_equal(pcl.points[0].toarray(), (1.0, 2.0, 3.0))

    pcl.colors.resize(100)
    color = pcl.colors[0]
    color[0] = 255.0
    color[1] = 255.0
    color[2] = 255.0
    color[3] = 255.0
    assert_equal(pcl.colors.size(), 100)
    assert_array_equal(pcl.colors[0].toarray(), (255.0, 255.0, 255.0, 255.0))


def test_laser_scan():
    ls = basetypes.LaserScan()

    time = basetypes.Time.now()
    ls.time = time
    assert_equal(ls.time.microseconds, time.microseconds)

    ls.min_range = 20
    ls.max_range = 30

    ls.ranges.resize(10)
    ls.remission.resize(10)
    for i in range(10):
        ls.ranges[i] = 25
        ls.remission[i] = 0.0
    ls.ranges[5] = 10
    assert_false(ls.is_valid_beam(5))
    assert_true(ls.is_valid_beam(0))
    assert_false(ls.is_range_valid(500))
    assert_true(ls.is_range_valid(25))


def test_laser_scan_str():
    ls = basetypes.LaserScan()
    ls.min_range = 20
    ls.max_range = 30
    ls.ranges.resize(6)
    ls.remission.resize(6)
    for i in range(6):
        ls.ranges[i] = 25
        ls.remission[i] = 0.0
    ls.ranges[1] = 10
    assert_equal(
        str(ls),
        "LaserScan {<time=19700101-01:00:00:000000>, min_range=20, max_range=30, ranges=[25, 10, 25, 25, 25, ...], "
        "remission=[0.0, 0.0, 0.0, 0.0, 0.0, ...]}"
    )


def test_imu_sensors():
    imu = basetypes.IMUSensors()

    imu.time = basetypes.Time.now()
    assert_false(imu.time.is_null())

    imu.acc.x = 1.0
    assert_equal(imu.acc.x, 1.0)

    imu.gyro.x = 2.0
    assert_equal(imu.gyro.x, 2.0)

    imu.mag.x = 3.0
    assert_equal(imu.mag.x, 3.0)


def test_angle():
    assert_equal(basetypes.Angle.rad2Deg(1),57.29577951308232)
    assert_equal(basetypes.Angle.deg2Rad(57.29577951308232), 1)
    assert_equal(basetypes.Angle.normalizeRad(7),0.7168146928204138)

    angle = basetypes.Angle.fromRad(1)
    assert_equal(angle.getDeg(), 57.29577951308232)

    angle = basetypes.Angle.fromDeg(1)
    assert_equal(angle.getDeg(), 1)

    angle = basetypes.Angle.Min()
    assert_true(angle.getDeg()< -179)

    angle = basetypes.Angle.Max()
    assert_true(angle.getDeg()> 179)

    angle1 = basetypes.Angle.fromRad(1)
    angle2 = basetypes.Angle.fromRad(0.9)

    assert_true(angle1.isApprox(angle2,0.2))
    assert_false(angle1.isApprox(angle2))
    assert_true(angle1.isApprox(angle1))

    assert_true(angle1>angle2)
    assert_false(angle2>angle1)
    assert_false(angle1 > angle1)
    assert_true(angle1 >= angle2)
    assert_true(angle1 >= angle1)

    assert_false(angle1 < angle2)
    assert_true(angle2 < angle1)
    assert_false(angle1 < angle1)
    assert_false(angle1 <= angle2)
    assert_true(angle1 <= angle1)

    assert_true(angle1 == angle1)
    assert_false(angle1 == angle2)
    assert_false(angle1 != angle1)
    assert_true(angle1 != angle2)
    assert_true(basetypes.Angle.fromRad(1) == basetypes.Angle.fromRad(1))

    angle = basetypes.Angle.fromDeg(1)
    sum = angle + angle
    assert_equal(sum.getDeg(), 2)
    sum = angle - angle
    assert_equal(sum.getDeg(), 0)
    angle += angle
    assert_equal(angle.getDeg(), 2)
    angle -= angle
    assert_equal(angle.getDeg(), 0)
    angle = basetypes.Angle.fromDeg(1)
    mul = angle * angle
    assert_equal(mul.getDeg(), 0.017453292519943295)
    mul = angle * 2.0
    assert_equal(mul.getDeg(), 2.0)

    angle1 = basetypes.Angle.fromDeg(1)
    angle2 = angle1
    angle2+= basetypes.Angle.fromDeg(1)
    assert_true(angle1 != angle2)

    angle = basetypes.Angle()