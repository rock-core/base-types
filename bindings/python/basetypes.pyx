# distutils: language = c++
cimport _basetypes
from cython.operator cimport dereference as deref
from libcpp.string cimport string
from libcpp.vector cimport vector
from libcpp cimport bool
cimport numpy as np
import numpy as np


cdef class Time:
    cdef _basetypes.Time* thisptr
    cdef bool delete_thisptr

    def __cinit__(self):
        self.thisptr = NULL
        self.delete_thisptr = False

    def __dealloc__(self):
        if self.thisptr != NULL and self.delete_thisptr:
            del self.thisptr

    def __init__(self):
        self.thisptr = new _basetypes.Time()
        self.delete_thisptr = True

    def _get_microseconds(self):
        return self.thisptr.microseconds

    def _set_microseconds(self, int microseconds):
        self.thisptr.microseconds = microseconds

    microseconds = property(_get_microseconds, _set_microseconds)


cdef class Vector2d:
    cdef _basetypes.Vector2d* thisptr
    cdef bool delete_thisptr

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

    # TODO add toarray / fromarray


cdef class Vector3d:
    cdef _basetypes.Vector3d* thisptr
    cdef bool delete_thisptr

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

    # TODO add fromarray


cdef class Vector4d:
    cdef _basetypes.Vector4d* thisptr
    cdef bool delete_thisptr

    def __cinit__(self):
        self.thisptr = NULL
        self.delete_thisptr = False

    def __dealloc__(self):
        if self.thisptr != NULL and self.delete_thisptr:
            del self.thisptr

    def __init__(self, v0, v1, v2, v3):
        self.thisptr = new _basetypes.Vector4d(v0, v1, v2, v3)
        self.delete_thisptr = True

    def __str__(self):
        return "[%.2f, %.2f, %.2f, %.2f]" % (
            self.thisptr.get(0), self.thisptr.get(1),
            self.thisptr.get(2), self.thisptr.get(3)
        )

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

    # TODO add toarray / fromarray


cdef class Matrix3d:
    cdef _basetypes.Matrix3d* thisptr
    cdef bool delete_thisptr

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


# TODO Vector6d, VectorXd, Point, Pose


cdef class Quaterniond:
    cdef _basetypes.Quaterniond* thisptr
    cdef bool delete_thisptr

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


cdef class TransformWithCovariance:
    cdef _basetypes.TransformWithCovariance* thisptr
    cdef bool delete_thisptr

    def __cinit__(self):
        self.thisptr = NULL
        self.delete_thisptr = False

    def __dealloc__(self):
        if self.thisptr != NULL and self.delete_thisptr:
            del self.thisptr

    def __init__(self):
        self.thisptr = new _basetypes.TransformWithCovariance()
        self.delete_thisptr = True

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

    def __str__(self):
        return "(translation=%s, orientation=%s)" % (self.translation,
                                                     self.orientation)


cdef class JointState:
    cdef _basetypes.JointState* thisptr
    cdef bool delete_thisptr

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
    cdef _basetypes.Joints* thisptr
    cdef bool delete_thisptr

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

    # TODO expose some useful functions


cdef class StringVectorReference:
    cdef vector[string]* thisptr

    def __cinit__(self):
        self.thisptr = NULL

    def __dealloc__(self):
        pass

    def __getitem__(self, int i):
        return deref(self.thisptr)[i]

    def __setitem__(self, int i, string s):
        deref(self.thisptr)[i] = s


cdef class JointStateVectorReference:
    cdef vector[_basetypes.JointState]* thisptr

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


cdef class RigidBodyState:
    cdef _basetypes.RigidBodyState* thisptr
    cdef bool delete_thisptr

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

# TODO DistanceImage, Frame, LaserScan, IMUSensors, PointCloud
