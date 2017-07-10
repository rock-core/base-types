from libcpp.string cimport string


cdef extern from "base/Time.hpp" namespace "base":
    cdef cppclass Time:
        Time()
        int microseconds


cdef extern from "base/Eigen.hpp" namespace "base":
    cdef cppclass Vector3d:
        Vector3d()
        Vector3d(double, double, double)
        Vector3d(Vector3d&)
        double* data()
        int rows()
        double& get "operator()"(int rows)
        double x()
        double y()
        double z()
    cdef cppclass Quaterniond:
        Quaterniond()
        Quaterniond(double, double, double, double)
        double w()
        double x()
        double y()
        double z()


cdef extern from "base/TransformWithCovariance.hpp" namespace "base":
    cdef cppclass TransformWithCovariance:
        TransformWithCovariance()
        Vector3d translation
        Quaterniond orientation

