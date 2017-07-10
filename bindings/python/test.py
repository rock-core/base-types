import basetypes
from nose.tools import assert_equal, assert_raises_regexp


def test_get_set_microseconds():
    t = basetypes.PyTime()
    m = 1000023
    t.microseconds = m
    assert_equal(t.microseconds, m)


def test_vector3d_ctor():
    v = basetypes.PyVector3d(1.0, 2.0, 3.0)
    assert_equal(str(v), "[1.00, 2.00, 3.00]")


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
