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

    def x(self):
        return self.thisptr.x()

    def y(self):
        return self.thisptr.y()

    def z(self):
        return self.thisptr.z()


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

