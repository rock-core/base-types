# distutils: language = c++
cimport _basetypes
from cython.operator cimport dereference as deref
from libcpp.string cimport string
from libcpp cimport bool


cdef class PyTime:
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


cdef class PyVector2d:
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


cdef class PyVector3d:
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


cdef class PyVector4d:
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
        return "[%.2f, %.2f, %.2f, %.2f]" % (self.thisptr.get(0),
                                             self.thisptr.get(1),
                                             self.thisptr.get(2),
                                             self.thisptr.get(3))

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


# TODO Vector6d, VectorXd


cdef class PyQuaterniond:
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


cdef class PyTransformWithCovariance:
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
        cdef PyVector3d translation = PyVector3d()
        del translation.thisptr
        translation.delete_thisptr = False
        translation.thisptr = &self.thisptr.translation
        return translation

    def _set_translation(self, PyVector3d translation):
        self.thisptr.translation = deref(translation.thisptr)

    translation = property(_get_translation, _set_translation)

    def _get_orientation(self):
        cdef PyQuaterniond orientation = PyQuaterniond()
        del orientation.thisptr
        orientation.delete_thisptr = False
        orientation.thisptr = &self.thisptr.orientation
        return orientation

    def _set_orientation(self, PyQuaterniond orientation):
        self.thisptr.orientation = deref(orientation.thisptr)

    orientation = property(_get_orientation, _set_orientation)

    def __str__(self):
        return "(translation=%s, orientation=%s)" % (self.translation,
                                                     self.orientation)


cdef class PyJointState:
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

    @staticmethod
    def Position(double value):
        cdef PyJointState self = PyJointState()
        self.thisptr[0] = _basetypes.Position(value)
        return self

    @staticmethod
    def Speed(double value):
        cdef PyJointState self = PyJointState()
        self.thisptr[0] = _basetypes.Speed(value)
        return self

    @staticmethod
    def Effort(double value):
        cdef PyJointState self = PyJointState()
        self.thisptr[0] = _basetypes.Effort(value)
        return self

    @staticmethod
    def Raw(double value):
        cdef PyJointState self = PyJointState()
        self.thisptr[0] = _basetypes.Raw(value)
        return self

    @staticmethod
    def Acceleration(double value):
        cdef PyJointState self = PyJointState()
        self.thisptr[0] = _basetypes.Acceleration(value)
        return self
