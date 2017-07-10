# distutils: language = c++
cimport _basetypes
from cython.operator cimport dereference as deref
from libcpp.string cimport string


cdef class PyTime:
    cdef _basetypes.Time* thisptr

    def __cinit__(self):
        self.thisptr = NULL

    def __dealloc__(self):
        if self.thisptr != NULL:
            del self.thisptr

    def __init__(self):
        self.thisptr = new _basetypes.Time()

    def _get_microseconds(self):
        return self.thisptr.microseconds

    def _set_microseconds(self, int microseconds):
        self.thisptr.microseconds = microseconds

    microseconds = property(_get_microseconds, _set_microseconds)


cdef class PyVector3d:
    cdef _basetypes.Vector3d* thisptr

    def __cinit__(self):
        self.thisptr = NULL

    def __dealloc__(self):
        if self.thisptr != NULL:
            del self.thisptr

    def __init__(self, double x=0.0, double y=0.0, double z=0.0):
        self.thisptr = new _basetypes.Vector3d(x, y, z)

    def __str__(self):
        return "[%.2f, %.2f, %.2f]" % (self.thisptr.get(0),
                                       self.thisptr.get(1),
                                       self.thisptr.get(2))

    def x(self):
        return self.thisptr.x()

    def y(self):
        return self.thisptr.y()

    def z(self):
        return self.thisptr.z()


cdef class PyQuaterniond:
    cdef _basetypes.Quaterniond* thisptr

    def __cinit__(self):
        self.thisptr = NULL

    def __dealloc__(self):
        if self.thisptr != NULL:
            del self.thisptr

    def __init__(self, double w=1.0, double x=0.0, double y=0.0, double z=0.0):
        self.thisptr = new _basetypes.Quaterniond(w, x, y, z)

    def __str__(self):
        return "[im=%.2f, real=(%.2f, %.2f, %.2f)]" % (
            self.thisptr.w(), self.thisptr.x(), self.thisptr.y(),
            self.thisptr.z())


cdef class PyTransformWithCovariance:
    cdef _basetypes.TransformWithCovariance* thisptr

    def __cinit__(self):
        self.thisptr = NULL

    def __dealloc__(self):
        if self.thisptr != NULL:
            del self.thisptr

    def __init__(self):
        self.thisptr = new _basetypes.TransformWithCovariance()

    def _get_translation(self):
        translation = PyVector3d()
        translation.thisptr[0] = self.thisptr.translation
        return translation

    def _set_translation(self, PyVector3d translation):
        self.thisptr.translation = deref(translation.thisptr)

    translation = property(_get_translation, _set_translation)

    def _get_orientation(self):
        orientation = PyQuaterniond()
        orientation.thisptr[0] = self.thisptr.orientation
        return orientation

    def _set_orientation(self, PyQuaterniond orientation):
        self.thisptr.orientation = deref(orientation.thisptr)

    orientation = property(_get_orientation, _set_orientation)

    def __str__(self):
        return "(translation=%s, orientation=%s)" % (self.translation,
                                                     self.orientation)

