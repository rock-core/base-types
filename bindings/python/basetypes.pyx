# distutils: language = c++
cimport basetypes
from cython.operator cimport dereference as deref
from libc.stdint cimport uint8_t, uint16_t, uint32_t, uint64_t, int64_t
cimport numpy as np
import numpy as np


np.import_array()  # must be here because we use the NumPy C API


cdef class Time:
    def __cinit__(self):
        self.thisptr = NULL
        self.delete_thisptr = False

    def __dealloc__(self):
        if self.thisptr != NULL and self.delete_thisptr:
            del self.thisptr

    def __init__(self):
        self.thisptr = new _basetypes.Time()
        self.delete_thisptr = True

    def __str__(self):
        return self.thisptr.toString(_basetypes.Resolution.Microseconds, "%Y%m%d-%H:%M:%S")

    def _get_microseconds(self):
        return self.thisptr.microseconds

    def _set_microseconds(self, int64_t microseconds):
        self.thisptr.microseconds = microseconds

    microseconds = property(_get_microseconds, _set_microseconds)

    def is_null(self):
        return self.thisptr.isNull()

    @staticmethod
    def now():
        cdef Time time = Time()
        time.thisptr[0] = _basetypes.now()
        return time

    # TODO more factory methods

    def __richcmp__(Time self, Time other, int op):
        if op == 0:
            return deref(self.thisptr) < deref(other.thisptr)
        elif op == 1:
            return deref(self.thisptr) <= deref(other.thisptr)
        elif op == 2:
            return deref(self.thisptr) == deref(other.thisptr)
        elif op == 3:
            return deref(self.thisptr) != deref(other.thisptr)
        elif op == 4:
            return deref(self.thisptr) > deref(other.thisptr)
        elif op == 5:
            return deref(self.thisptr) >= deref(other.thisptr)
        else:
            raise ValueError("Unknown comparison operation %d" % op)

    def __add__(Time self, Time other):
        cdef Time time = Time()
        time.thisptr[0] = deref(self.thisptr) + deref(other.thisptr)
        return time

    def __iadd__(Time self, Time other):
        self.thisptr[0] = deref(self.thisptr) + deref(other.thisptr)
        return self

    def __sub__(Time self, Time other):
        cdef Time time = Time()
        time.thisptr[0] = deref(self.thisptr) - deref(other.thisptr)
        return time

    def __isub__(Time self, Time other):
        self.thisptr[0] = deref(self.thisptr) - deref(other.thisptr)
        return self

    def __div__(Time self, int divider):
        cdef Time time = Time()
        time.thisptr[0] = deref(self.thisptr) / divider
        return time

    def __idiv__(Time self, int divider):
        self.thisptr[0] = deref(self.thisptr) / divider
        return self

    def __mul__(Time self, double factor):
        cdef Time time = Time()
        time.thisptr[0] = deref(self.thisptr) * factor
        return time

    def __imul__(Time self, double factor):
        self.thisptr[0] = deref(self.thisptr) * factor
        return self


cdef class Vector2d:
    def __cinit__(self):
        self.thisptr = NULL
        self.delete_thisptr = False

    def __dealloc__(self):
        if self.thisptr != NULL and self.delete_thisptr:
            del self.thisptr

    def __init__(self, double x=0.0, double y=0.0):
        self.thisptr = new _basetypes.Vector2d(x, y)
        self.delete_thisptr = True

    def __str__(self):
        return "[%.2f, %.2f]" % (self.thisptr.get(0), self.thisptr.get(1))

    def __array__(self, dtype=None):
        cdef np.npy_intp shape[1]
        shape[0] = <np.npy_intp> 2
        return np.PyArray_SimpleNewFromData(
            1, shape, np.NPY_DOUBLE, <void*> self.thisptr.data())

    def __getitem__(self, int i):
        if i < 0 or i > 1:
            raise KeyError("index must be 0 or 1 but was %d" % i)
        return self.thisptr.data()[i]

    def __setitem__(self, int i, double v):
        if i < 0 or i > 1:
            raise KeyError("index must be 0 or 1 but was %d" % i)
        self.thisptr.data()[i] = v

    def _get_x(self):
        return self.thisptr.data()[0]

    def _set_x(self, double x):
        self.thisptr.data()[0] = x

    x = property(_get_x, _set_x)

    def _get_y(self):
        return self.thisptr.data()[1]

    def _set_y(self, double y):
        self.thisptr.data()[1] = y

    y = property(_get_y, _set_y)

    def norm(self):
        return self.thisptr.norm()

    def squared_norm(self):
        return self.thisptr.squaredNorm()

    def toarray(self):
        cdef np.ndarray[double, ndim=1] array = np.empty(2)
        cdef int i
        for i in range(2):
            array[i] = self.thisptr.data()[i]
        return array

    # TODO add operators, fromarray


cdef class Vector3d:
    def __cinit__(self):
        self.thisptr = NULL
        self.delete_thisptr = False

    def __dealloc__(self):
        if self.thisptr != NULL and self.delete_thisptr:
            del self.thisptr

    def __init__(self, double x=0.0, double y=0.0, double z=0.0):
        self.thisptr = new _basetypes.Vector3d(x, y, z)
        self.delete_thisptr = True

    def __str__(self):
        return "[%.2f, %.2f, %.2f]" % (self.thisptr.get(0),
                                       self.thisptr.get(1),
                                       self.thisptr.get(2))

    def __array__(self, dtype=None):
        cdef np.npy_intp shape[1]
        shape[0] = <np.npy_intp> 3
        return np.PyArray_SimpleNewFromData(
            1, shape, np.NPY_DOUBLE, <void*> self.thisptr.data())

    def __getitem__(self, int i):
        if i < 0 or i > 2:
            raise KeyError("index must be 0, 1 or 2 but was %d" % i)
        return self.thisptr.data()[i]

    def __setitem__(self, int i, double v):
        if i < 0 or i > 2:
            raise KeyError("index must be 0, 1 or 2 but was %d" % i)
        self.thisptr.data()[i] = v

    def _get_x(self):
        return self.thisptr.data()[0]

    def _set_x(self, double x):
        self.thisptr.data()[0] = x

    x = property(_get_x, _set_x)

    def _get_y(self):
        return self.thisptr.data()[1]

    def _set_y(self, double y):
        self.thisptr.data()[1] = y

    y = property(_get_y, _set_y)

    def _get_z(self):
        return self.thisptr.data()[2]

    def _set_z(self, double z):
        self.thisptr.data()[2] = z

    z = property(_get_z, _set_z)

    def norm(self):
        return self.thisptr.norm()

    def squared_norm(self):
        return self.thisptr.squaredNorm()

    def toarray(self):
        cdef np.ndarray[double, ndim=1] array = np.empty(3)
        cdef int i
        for i in range(3):
            array[i] = self.thisptr.data()[i]
        return array

    def fromarray(self, np.ndarray[double, ndim=1] array):
        cdef int i
        for i in range(3):
            self.thisptr.data()[i] = array[i]

    # TODO add operators


cdef class Vector4d:
    def __cinit__(self):
        self.thisptr = NULL
        self.delete_thisptr = False

    def __dealloc__(self):
        if self.thisptr != NULL and self.delete_thisptr:
            del self.thisptr

    def __init__(self, v0=0.0, v1=0.0, v2=0.0, v3=0.0):
        self.thisptr = new _basetypes.Vector4d(v0, v1, v2, v3)
        self.delete_thisptr = True

    def __str__(self):
        return "[%.2f, %.2f, %.2f, %.2f]" % (
            self.thisptr.get(0), self.thisptr.get(1),
            self.thisptr.get(2), self.thisptr.get(3)
        )

    def __array__(self, dtype=None):
        cdef np.npy_intp shape[1]
        shape[0] = <np.npy_intp> 4
        return np.PyArray_SimpleNewFromData(
            1, shape, np.NPY_DOUBLE, <void*> self.thisptr.data())

    def __getitem__(self, int i):
        if i < 0 or i > 3:
            raise KeyError("index must be in [0, 3] but was %d" % i)
        return self.thisptr.data()[i]

    def __setitem__(self, int i, double v):
        if i < 0 or i > 3:
            raise KeyError("index must be in [0, 3] but was %d" % i)
        self.thisptr.data()[i] = v

    def norm(self):
        return self.thisptr.norm()

    def squared_norm(self):
        return self.thisptr.squaredNorm()

    def toarray(self):
        cdef np.ndarray[double, ndim=1] array = np.empty(4)
        cdef int i
        for i in range(4):
            array[i] = self.thisptr.data()[i]
        return array

    # TODO add operators, fromarray


cdef class Matrix3d:
    def __cinit__(self):
        self.thisptr = NULL
        self.delete_thisptr = False

    def __dealloc__(self):
        if self.thisptr != NULL and self.delete_thisptr:
            del self.thisptr

    def __init__(self, np.ndarray[double, ndim=2] array=None):
        self.thisptr = new _basetypes.Matrix3d()
        self.delete_thisptr = True
        cdef int i
        cdef int j
        if array is not None:
            for i in range(3):
                for j in range(3):
                    self.thisptr.data()[3 * j + i] = array[i, j]

    def __str__(self):
        return ("[%.2f, %.2f, %.2f],"
                "[%.2f, %.2f, %.2f],"
                "[%.2f, %.2f, %.2f]") % (
            self.thisptr.get(0, 0), self.thisptr.get(0, 1), self.thisptr.get(0, 2),
            self.thisptr.get(1, 0), self.thisptr.get(1, 1), self.thisptr.get(1, 2),
            self.thisptr.get(2, 0), self.thisptr.get(2, 1), self.thisptr.get(2, 2),
        )

    def __array__(self, dtype=None):
        cdef np.npy_intp shape[2]
        shape[0] = <np.npy_intp> 3
        shape[1] = <np.npy_intp> 3
        return np.PyArray_SimpleNewFromData(
            2, shape, np.NPY_DOUBLE, <void*> self.thisptr.data()).T

    def __getitem__(self, tuple indices):
        cdef int i
        cdef int j
        i, j = indices
        if i < 0 or i > 3 or j < 0 or j > 3:
            raise KeyError("indices must be in [0, 3] but were (%d, %d)"
                           % (i, j))
        return self.thisptr.get(i, j)

    def __setitem__(self, tuple indices, double v):
        cdef int i
        cdef int j
        i, j = indices
        if i < 0 or i > 3 or j < 0 or j > 3:
            raise KeyError("indices must be in [0, 3] but were (%d, %d)"
                           % (i, j))
        # Assumes column-major order!
        self.thisptr.data()[3 * j + i] = v

    def toarray(self):
        cdef np.ndarray[double, ndim=2] array = np.empty((3, 3))
        cdef int i
        cdef int j
        for i in range(3):
            for j in range(3):
                array[i, j] = self.thisptr.data()[3 * j + i]
        return array

    def fromarray(self, np.ndarray[double, ndim=2] array):
        cdef int i
        cdef int j
        for i in range(3):
            for j in range(3):
                self.thisptr.data()[3 * j + i] = array[i, j]

    # TODO operators


# TODO Vector6d, VectorXd, Pose


cdef class Quaterniond:
    def __cinit__(self):
        self.thisptr = NULL
        self.delete_thisptr = False

    def __dealloc__(self):
        if self.thisptr != NULL and self.delete_thisptr:
            del self.thisptr

    def __init__(self, double w=1.0, double x=0.0, double y=0.0, double z=0.0):
        self.thisptr = new _basetypes.Quaterniond(w, x, y, z)
        self.delete_thisptr = True

    def __str__(self):
        return "[im=%.2f, real=(%.2f, %.2f, %.2f)]" % (
            self.thisptr.w(), self.thisptr.x(), self.thisptr.y(),
            self.thisptr.z())

    def toarray(self):
        cdef np.ndarray[double, ndim=1] array = np.array([
            self.thisptr.w(), self.thisptr.x(), self.thisptr.y(),
            self.thisptr.z()])
        return array

    def fromarray(self, np.ndarray[double, ndim=1] array):
        self.thisptr[0] = _basetypes.Quaterniond(array[0], array[1], array[2], array[3])

    # TODO how can we modify a quaternion?


cdef class TransformWithCovariance:
    def __cinit__(self):
        self.thisptr = NULL
        self.delete_thisptr = False

    def __dealloc__(self):
        if self.thisptr != NULL and self.delete_thisptr:
            del self.thisptr

    def __init__(self):
        self.thisptr = new _basetypes.TransformWithCovariance()
        self.delete_thisptr = True

    def __str__(self):
        return "(translation=%s, orientation=%s)" % (self.translation,
                                                     self.orientation)

    def _get_translation(self):
        cdef Vector3d translation = Vector3d()
        del translation.thisptr
        translation.delete_thisptr = False
        translation.thisptr = &self.thisptr.translation
        return translation

    def _set_translation(self, Vector3d translation):
        self.thisptr.translation = deref(translation.thisptr)

    translation = property(_get_translation, _set_translation)

    def _get_orientation(self):
        cdef Quaterniond orientation = Quaterniond()
        del orientation.thisptr
        orientation.delete_thisptr = False
        orientation.thisptr = &self.thisptr.orientation
        return orientation

    def _set_orientation(self, Quaterniond orientation):
        self.thisptr.orientation = deref(orientation.thisptr)

    orientation = property(_get_orientation, _set_orientation)

    # TODO covariance


cdef class JointState:
    def __cinit__(self):
        self.thisptr = NULL
        self.delete_thisptr = False

    def __dealloc__(self):
        if self.thisptr != NULL and self.delete_thisptr:
            del self.thisptr

    def __init__(self):
        self.thisptr = new _basetypes.JointState()
        self.delete_thisptr = True

    def _get_position(self):
        return self.thisptr.position

    def _set_position(self, double value):
        self.thisptr.position = value

    position = property(_get_position, _set_position)

    def _get_speed(self):
        return self.thisptr.speed

    def _set_speed(self, double value):
        self.thisptr.speed = value

    speed = property(_get_speed, _set_speed)

    def _get_effort(self):
        return self.thisptr.effort

    def _set_effort(self, double value):
        self.thisptr.effort = value

    effort = property(_get_effort, _set_effort)

    def _get_raw(self):
        return self.thisptr.raw

    def _set_raw(self, double value):
        self.thisptr.raw = value

    raw = property(_get_raw, _set_raw)

    def _get_acceleration(self):
        return self.thisptr.acceleration

    def _set_acceleration(self, double value):
        self.thisptr.acceleration = value

    acceleration = property(_get_acceleration, _set_acceleration)

    def has_position(self):
        return self.thisptr.hasPosition()

    def has_speed(self):
        return self.thisptr.hasSpeed()

    def has_effort(self):
        return self.thisptr.hasEffort()

    def has_raw(self):
        return self.thisptr.hasRaw()

    def has_acceleration(self):
        return self.thisptr.hasAcceleration()

    def has_position(self):
        return self.thisptr.hasPosition()

    def is_speed(self):
        return self.thisptr.isSpeed()

    def is_effort(self):
        return self.thisptr.isEffort()

    def is_raw(self):
        return self.thisptr.isRaw()

    def is_acceleration(self):
        return self.thisptr.isAcceleration()

    @staticmethod
    def Position(double value):
        cdef JointState self = JointState()
        self.thisptr[0] = _basetypes.Position(value)
        return self

    @staticmethod
    def Speed(double value):
        cdef JointState self = JointState()
        self.thisptr[0] = _basetypes.Speed(value)
        return self

    @staticmethod
    def Effort(double value):
        cdef JointState self = JointState()
        self.thisptr[0] = _basetypes.Effort(value)
        return self

    @staticmethod
    def Raw(double value):
        cdef JointState self = JointState()
        self.thisptr[0] = _basetypes.Raw(value)
        return self

    @staticmethod
    def Acceleration(double value):
        cdef JointState self = JointState()
        self.thisptr[0] = _basetypes.Acceleration(value)
        return self


cdef class Joints:
    def __cinit__(self):
        self.thisptr = NULL
        self.delete_thisptr = False

    def __dealloc__(self):
        if self.thisptr != NULL and self.delete_thisptr:
            del self.thisptr

    def __init__(self):
        self.thisptr = new _basetypes.Joints()
        self.delete_thisptr = True

    def size(self):
        return self.thisptr.size()

    def resize(self, int size):
        self.thisptr.resize(size)

    def has_names(self):
        return self.thisptr.hasNames()

    def clear(self):
        self.thisptr.clear()

    def _get_time(self):
        cdef Time time = Time()
        del time.thisptr
        time.thisptr = &self.thisptr.time
        time.delete_thisptr = False
        return time

    def _set_time(self, Time time):
        self.thisptr.time = deref(time.thisptr)

    time = property(_get_time, _set_time)

    @property
    def names(self):
        cdef StringVectorReference names = StringVectorReference()
        names.thisptr = &self.thisptr.names
        return names

    @property
    def elements(self):
        cdef JointStateVectorReference elements = JointStateVectorReference()
        elements.thisptr = &self.thisptr.elements
        return elements

    def __getitem__(self, string name):
        cdef JointState joint_state = JointState()
        joint_state.thisptr[0] = self.thisptr.getElementByName(name)
        return joint_state

    def __setitem__(self, string name, JointState joint_state):
        cdef int i = self.thisptr.mapNameToIndex(name)
        self.thisptr.elements[i] = deref(joint_state.thisptr)

    # TODO factory methods


cdef class StringVectorReference:
    def __cinit__(self):
        self.thisptr = NULL

    def __dealloc__(self):
        pass

    def __getitem__(self, int i):
        return deref(self.thisptr)[i]

    def __setitem__(self, int i, string s):
        deref(self.thisptr)[i] = s

    def resize(self, int i):
        self.thisptr.resize(i)

    def size(self):
        return self.thisptr.size()


cdef class JointStateVectorReference:
    def __cinit__(self):
        self.thisptr = NULL

    def __dealloc__(self):
        pass

    def __getitem__(self, int i):
        cdef JointState joint_state = JointState()
        del joint_state.thisptr
        joint_state.delete_thisptr = False
        joint_state.thisptr = &deref(self.thisptr)[i]
        return joint_state

    def resize(self, int i):
        self.thisptr.resize(i)

    def size(self):
        return self.thisptr.size()


cdef class RigidBodyState:
    def __cinit__(self):
        self.thisptr = NULL
        self.delete_thisptr = False

    def __dealloc__(self):
        if self.thisptr != NULL and self.delete_thisptr:
            del self.thisptr

    def __init__(self, bool do_invalidation=True):
        self.thisptr = new _basetypes.RigidBodyState(do_invalidation)
        self.delete_thisptr = True

    def _get_time(self):
        cdef Time time = Time()
        del time.thisptr
        time.thisptr = &self.thisptr.time
        time.delete_thisptr = False
        return time

    def _set_time(self, Time time):
        self.thisptr.time = deref(time.thisptr)

    time = property(_get_time, _set_time)

    def _get_source_frame(self):
        return self.thisptr.sourceFrame

    def _set_source_frame(self, string value):
        self.thisptr.sourceFrame = value

    source_frame = property(_get_source_frame, _set_source_frame)

    def _get_target_frame(self):
        return self.thisptr.targetFrame

    def _set_target_frame(self, string value):
        self.thisptr.targetFrame = value

    target_frame = property(_get_target_frame, _set_target_frame)

    def _get_position(self):
        cdef Vector3d position = Vector3d()
        del position.thisptr
        position.delete_thisptr = False
        position.thisptr = &self.thisptr.position
        return position

    def _set_position(self, Vector3d value):
        self.thisptr.position = deref(value.thisptr)

    position = property(_get_position, _set_position)

    def _get_cov_position(self):
        cdef Matrix3d cov_position = Matrix3d()
        del cov_position.thisptr
        cov_position.delete_thisptr = False
        cov_position.thisptr = &self.thisptr.cov_position
        return cov_position

    def _set_cov_position(self, Matrix3d value):
        self.thisptr.cov_position = deref(value.thisptr)

    cov_position = property(_get_cov_position, _set_cov_position)

    def _get_orientation(self):
        cdef Quaterniond orientation = Quaterniond()
        del orientation.thisptr
        orientation.delete_thisptr = False
        orientation.thisptr = &self.thisptr.orientation
        return orientation

    def _set_orientation(self, Quaterniond value):
        self.thisptr.orientation = deref(value.thisptr)

    orientation = property(_get_orientation, _set_orientation)

    def _get_cov_orientation(self):
        cdef Matrix3d cov_orientation = Matrix3d()
        del cov_orientation.thisptr
        cov_orientation.delete_thisptr = False
        cov_orientation.thisptr = &self.thisptr.cov_orientation
        return cov_orientation

    def _set_cov_orientation(self, Matrix3d value):
        self.thisptr.cov_orientation = deref(value.thisptr)

    cov_orientation = property(_get_cov_orientation, _set_cov_orientation)

    def _get_velocity(self):
        cdef Vector3d velocity = Vector3d()
        del velocity.thisptr
        velocity.delete_thisptr = False
        velocity.thisptr = &self.thisptr.velocity
        return velocity

    def _set_velocity(self, Vector3d value):
        self.thisptr.velocity = deref(value.thisptr)

    velocity = property(_get_velocity, _set_velocity)

    def _get_cov_velocity(self):
        cdef Matrix3d cov_velocity = Matrix3d()
        del cov_velocity.thisptr
        cov_velocity.delete_thisptr = False
        cov_velocity.thisptr = &self.thisptr.cov_velocity
        return cov_velocity

    def _set_cov_velocity(self, Matrix3d value):
        self.thisptr.cov_velocity = deref(value.thisptr)

    cov_velocity = property(_get_cov_velocity, _set_cov_velocity)

    def _get_angular_velocity(self):
        cdef Vector3d angular_velocity = Vector3d()
        del angular_velocity.thisptr
        angular_velocity.delete_thisptr = False
        angular_velocity.thisptr = &self.thisptr.angular_velocity
        return angular_velocity

    def _set_angular_velocity(self, Vector3d value):
        self.thisptr.angular_velocity = deref(value.thisptr)

    angular_velocity = property(_get_angular_velocity, _set_angular_velocity)

    def _get_cov_angular_velocity(self):
        cdef Matrix3d cov_angular_velocity = Matrix3d()
        del cov_angular_velocity.thisptr
        cov_angular_velocity.delete_thisptr = False
        cov_angular_velocity.thisptr = &self.thisptr.cov_angular_velocity
        return cov_angular_velocity

    def _set_cov_angular_velocity(self, Matrix3d value):
        self.thisptr.cov_angular_velocity = deref(value.thisptr)

    cov_angular_velocity = property(
        _get_cov_angular_velocity, _set_cov_angular_velocity)


cdef class frame_mode_t:
    MODE_UNDEFINED = _basetypes.frame_mode_t.MODE_UNDEFINED
    MODE_GRAYSCALE = _basetypes.frame_mode_t.MODE_GRAYSCALE
    MODE_RGB = _basetypes.frame_mode_t.MODE_RGB
    MODE_UYVY = _basetypes.frame_mode_t.MODE_UYVY
    MODE_BGR = _basetypes.frame_mode_t.MODE_BGR
    #MODE_RGB32 = _basetypes.frame_mode_t.MODE_RGB32 # TODO I don't know why but this value is "not known"
    RAW_MODES = _basetypes.frame_mode_t.RAW_MODES
    MODE_BAYER = _basetypes.frame_mode_t.MODE_BAYER
    MODE_BAYER_RGGB = _basetypes.frame_mode_t.MODE_BAYER_RGGB
    MODE_BAYER_GRBG = _basetypes.frame_mode_t.MODE_BAYER_GRBG
    MODE_BAYER_BGGR = _basetypes.frame_mode_t.MODE_BAYER_BGGR
    MODE_BAYER_GBRG = _basetypes.frame_mode_t.MODE_BAYER_GBRG
    COMPRESSED_MODES = _basetypes.frame_mode_t.COMPRESSED_MODES
    MODE_PJPG = _basetypes.frame_mode_t.MODE_PJPG
    MODE_JPEG = _basetypes.frame_mode_t.MODE_JPEG
    MODE_PNG = _basetypes.frame_mode_t.MODE_PNG


cdef class frame_status_t:
    STATUS_EMPTY = _basetypes.frame_status_t.STATUS_EMPTY
    STATUS_VALID = _basetypes.frame_status_t.STATUS_VALID
    STATUS_INVALID = _basetypes.frame_status_t.STATUS_INVALID


cdef class Frame:
    # TODO frame attributes
    def __cinit__(self):
        self.thisptr = NULL
        self.delete_thisptr = False

    def __dealloc__(self):
        if self.thisptr != NULL and self.delete_thisptr:
            del self.thisptr

    def __init__(self, width=None, height=None, depth=None, mode=None, val=None,
                 size_in_bytes=None):
        if width is None:
            self.thisptr = new _basetypes.Frame()
        else:
            self.thisptr = new _basetypes.Frame(width, height, depth, mode, val, size_in_bytes)
        self.delete_thisptr = True

    def _get_time(self):
        cdef Time time = Time()
        del time.thisptr
        time.thisptr = &self.thisptr.time
        time.delete_thisptr = False
        return time

    def _set_time(self, Time time):
        self.thisptr.time = deref(time.thisptr)

    time = property(_get_time, _set_time)

    def _get_received_time(self):
        cdef Time received_time = Time()
        del received_time.thisptr
        received_time.thisptr = &self.thisptr.received_time
        received_time.delete_thisptr = False
        return received_time

    def _set_received_time(self, Time received_time):
        self.thisptr.received_time = deref(received_time.thisptr)

    received_time = property(_get_received_time, _set_received_time)

    def _get_image(self):
        # TODO can we return a reference?
        # TODO what if it is compressed?
        cdef int n_channels = self.thisptr.getChannelCount()
        dimensions = (self.thisptr.size.width, self.thisptr.size.height)
        if n_channels > 1:
            dimensions += (n_channels,)

        cdef uint32_t depth = self.thisptr.getDataDepth()
        if depth == 1:
            dtype = np.uint8
        elif depth == 2:
            dtype = np.uint16
        elif depth == 4:
            dtype = np.uint32
        elif depth == 8:
            dtype = np.uint64
        else:
            raise ValueError("Cannot handle depth of %d" % depth)

        cdef np.ndarray image
        image = np.empty(dimensions, dtype=dtype)
        cdef uint32_t i
        for i in range(self.thisptr.getNumberOfBytes()):
            image.data[i] = self.thisptr.image[i]
        return image

    def _set_image(self, np.ndarray image):
        cdef uint32_t i
        for i in range(self.thisptr.getNumberOfBytes()):
            self.thisptr.image[i] = image.data[i]

    image = property(_get_image, _set_image)

    def get_width(self):
        return self.thisptr.getWidth()

    def get_height(self):
        return self.thisptr.getHeight()

    def get_channel_count(self):
        return self.thisptr.getChannelCount()

    def get_data_depth(self):
        return self.thisptr.getDataDepth()


# TODO FramePair


cdef class Pointcloud:
    def __cinit__(self):
        self.thisptr = NULL
        self.delete_thisptr = False

    def __dealloc__(self):
        if self.thisptr != NULL and self.delete_thisptr:
            del self.thisptr

    def __init__(self):
        self.thisptr = new _basetypes.Pointcloud()
        self.delete_thisptr = True

    @property
    def points(self):
        cdef Vector3dVectorReference points = Vector3dVectorReference()
        points.thisptr = &self.thisptr.points
        return points

    @property
    def colors(self):
        cdef Vector4dVectorReference colors = Vector4dVectorReference()
        colors.thisptr = &self.thisptr.colors
        return colors


cdef class Vector3dVectorReference:
    def __cinit__(self):
        self.thisptr = NULL

    def __dealloc__(self):
        pass

    def __getitem__(self, int i):
        cdef Vector3d v = Vector3d()
        del v.thisptr
        v.delete_thisptr = False
        v.thisptr = &deref(self.thisptr)[i]
        return v

    def resize(self, int i):
        self.thisptr.resize(i)

    def size(self):
        return self.thisptr.size()


cdef class Vector4dVectorReference:
    def __cinit__(self):
        self.thisptr = NULL

    def __dealloc__(self):
        pass

    def __getitem__(self, int i):
        cdef Vector4d v = Vector4d()
        del v.thisptr
        v.delete_thisptr = False
        v.thisptr = &deref(self.thisptr)[i]
        return v

    def resize(self, int i):
        self.thisptr.resize(i)

    def size(self):
        return self.thisptr.size()


cdef class LaserScan:
    def __cinit__(self):
        self.thisptr = NULL
        self.delete_thisptr = False

    def __dealloc__(self):
        if self.thisptr != NULL and self.delete_thisptr:
            del self.thisptr

    def __init__(self):
        self.thisptr = new _basetypes.LaserScan()
        self.delete_thisptr = True

    def _get_time(self):
        cdef Time time = Time()
        del time.thisptr
        time.thisptr = &self.thisptr.time
        time.delete_thisptr = False
        return time

    def _set_time(self, Time time):
        self.thisptr.time = deref(time.thisptr)

    time = property(_get_time, _set_time)

    def _get_start_angle(self):
        return self.thisptr.start_angle

    def _set_start_angle(self, double start_angle):
        self.thisptr.start_angle = start_angle

    start_angle = property(_get_start_angle, _set_start_angle)

    def _get_angular_resolution(self):
        return self.thisptr.angular_resolution

    def _set_angular_resolution(self, double angular_resolution):
        self.thisptr.angular_resolution = angular_resolution

    angular_resolution = property(_get_angular_resolution, _set_angular_resolution)

    def _get_speed(self):
        return self.thisptr.speed

    def _set_speed(self, double speed):
        self.thisptr.speed = speed

    speed = property(_get_speed, _set_speed)

    @property
    def ranges(self):
        cdef UInt32VectorReference ranges = UInt32VectorReference()
        ranges.thisptr = &self.thisptr.ranges
        return ranges

    def _get_min_range(self):
        return self.thisptr.minRange

    def _set_min_range(self, uint32_t min_range):
        self.thisptr.minRange = min_range

    min_range = property(_get_min_range, _set_min_range)

    def _get_max_range(self):
        return self.thisptr.maxRange

    def _set_max_range(self, uint32_t max_range):
        self.thisptr.maxRange = max_range

    max_range = property(_get_max_range, _set_max_range)

    @property
    def remission(self):
        cdef FloatVectorReference remission = FloatVectorReference()
        remission.thisptr = &self.thisptr.remission
        return remission

    def is_valid_beam(self, unsigned int i):
        return self.thisptr.isValidBeam(i)

    def is_range_valid(self, uint32_t r):
        return self.thisptr.isRangeValid(r)


cdef class UInt32VectorReference:
    def __cinit__(self):
        self.thisptr = NULL

    def __dealloc__(self):
        pass

    def __getitem__(self, int i):
        return deref(self.thisptr)[i]

    def __setitem__(self, int i, uint32_t v):
        deref(self.thisptr)[i] = v

    def resize(self, int i):
        self.thisptr.resize(i)

    def size(self):
        return self.thisptr.size()


cdef class FloatVectorReference:
    def __cinit__(self):
        self.thisptr = NULL

    def __dealloc__(self):
        pass

    def __getitem__(self, int i):
        return deref(self.thisptr)[i]

    def __setitem__(self, int i, float v):
        deref(self.thisptr)[i] = v

    def resize(self, int i):
        self.thisptr.resize(i)

    def size(self):
        return self.thisptr.size()


cdef class IMUSensors:
    def __cinit__(self):
        self.thisptr = NULL
        self.delete_thisptr = False

    def __dealloc__(self):
        if self.thisptr != NULL and self.delete_thisptr:
            del self.thisptr

    def __init__(self):
        self.thisptr = new _basetypes.IMUSensors()
        self.delete_thisptr = True

    def _get_time(self):
        cdef Time time = Time()
        del time.thisptr
        time.thisptr = &self.thisptr.time
        time.delete_thisptr = False
        return time

    def _set_time(self, Time time):
        self.thisptr.time = deref(time.thisptr)

    time = property(_get_time, _set_time)

    def _get_acc(self):
        cdef Vector3d acc = Vector3d()
        del acc.thisptr
        acc.delete_thisptr = False
        acc.thisptr = &self.thisptr.acc
        return acc

    def _set_acc(self, Vector3d value):
        self.thisptr.acc = deref(value.thisptr)

    acc = property(_get_acc, _set_acc)

    def _get_gyro(self):
        cdef Vector3d gyro = Vector3d()
        del gyro.thisptr
        gyro.delete_thisptr = False
        gyro.thisptr = &self.thisptr.gyro
        return gyro

    def _set_gyro(self, Vector3d value):
        self.thisptr.gyro = deref(value.thisptr)

    gyro = property(_get_gyro, _set_gyro)

    def _get_mag(self):
        cdef Vector3d mag = Vector3d()
        del mag.thisptr
        mag.delete_thisptr = False
        mag.thisptr = &self.thisptr.mag
        return mag

    def _set_mag(self, Vector3d value):
        self.thisptr.mag = deref(value.thisptr)

    mag = property(_get_mag, _set_mag)


# TODO missing types:
# typedefs: Position, Point, Orientation
# Angle !!!
# (CircularBuffer)
# Eigen::Matrix3/4/6/Xd
# Eigen::Affine3d
# Eigen::Isometry3d
# JointLimitRange
# JointLimits
# JointsTrajectory
# JointTransform
# Pose / Pose2D
# (Spline)
# Temperature
# TimeMark
# Timeout
# Trajectory
# TwistWithCovariance
# Waypoint
# Wrench
# ~~commands/Joints~~
# commands/LinearAngular6DCommand
# commands/Motion2D
# (commands/Speed6D)
# samples/BodyState - what is the difference to RigidBodyState???
# samples/CommandSamples
# samples/DepthMap !!!
# samples/DistanceImage
# samples/Pressure
# (samples/RigidBodyAcceleration)
# (samples/Sonar)
# (samples/SonarBeam)
# samples/Wrench
# samples/Wrenches
