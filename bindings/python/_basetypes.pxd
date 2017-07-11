from libcpp.string cimport string


cdef extern from "base/Time.hpp" namespace "base":
    cdef cppclass Time:
        Time()
        int microseconds


cdef extern from "base/Eigen.hpp" namespace "base":
    cdef cppclass Vector2d:
        Vector2d()
        Vector2d(double, double)
        Vector2d(Vector2d&)
        double* data()
        int rows()
        double& get "operator()"(int row)
        double x()
        double y()
        double norm()
        double squaredNorm()

    cdef cppclass Vector3d:
        Vector3d()
        Vector3d(double, double, double)
        Vector3d(Vector3d&)
        double* data()
        int rows()
        double& get "operator()"(int row)
        double x()
        double y()
        double z()
        double norm()
        double squaredNorm()

    cdef cppclass Vector4d:
        Vector4d()
        Vector4d(double, double, double, double)
        Vector4d(Vector4d&)
        double* data()
        int rows()
        double& get "operator()"(int row)
        double x()
        double y()
        double z()
        double norm()
        double squaredNorm()

    cdef cppclass Matrix3d:
        Matrix3d()
        double* data()
        double& get "operator()"(int row, int col)
        int rows()
        int cols()

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


cdef extern from "base/JointState.hpp" namespace "base":
    cdef cppclass JointState:
        JointState()
        double position
        double speed
        double effort
        double raw
        double acceleration


cdef extern from "base/JointState.hpp" namespace "base::JointState":
    JointState Position(double)
    JointState Speed(double)
    JointState Effort(double)
    JointState Raw(double)
    JointState Acceleration(double)


cdef extern from "base/samples/RigidBodyState.hpp" namespace "base::samples":
    cdef cppclass RigidBodyState:
        RigidBodyState(bool)
        Time time
        string sourceFrame
        string targetFrame
        Vector3d position
        Matrix3d cov_position
        Quaterniond orientation
        #Matrix3d cov_orientation TODO
        Vector3d velocity
        #Matrix3d cov_velocity TODO
        Vector3d angular_velocity
        #Matrix3d cov_angular_velocity TODO
