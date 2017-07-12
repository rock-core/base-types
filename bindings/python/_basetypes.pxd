from libcpp.string cimport string
from libcpp.vector cimport vector
from libcpp cimport bool
from libc.stdint cimport uint8_t, uint16_t, uint32_t


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
        bool hasPosition()
        bool hasSpeed()
        bool hasEffort()
        bool hasRaw()
        bool hasAcceleration()
        bool isPosition()
        bool isSpeed()
        bool isEffort()
        bool isRaw()
        bool isAcceleration()


cdef extern from "base/JointState.hpp" namespace "base::JointState":
    JointState Position(double)
    JointState Speed(double)
    JointState Effort(double)
    JointState Raw(double)
    JointState Acceleration(double)


cdef extern from "base/NamedVector.hpp" namespace "base":
    cdef cppclass NamedVector[T]:
        vector[string] names
        vector[T] elements

        void resize(int)
        int size()
        bool empty()
        void clear()
        bool hasNames()
        T getElementByName(string)
        int mapNameToIndex(string name)


cdef extern from "base/samples/Joints.hpp" namespace "base::samples":
    cdef cppclass Joints(NamedVector[JointState]):
        Joints()
        Time time


cdef extern from "base/samples/RigidBodyState.hpp" namespace "base::samples":
    cdef cppclass RigidBodyState:
        RigidBodyState(bool)
        Time time
        string sourceFrame
        string targetFrame
        Vector3d position
        Matrix3d cov_position
        Quaterniond orientation
        Matrix3d cov_orientation
        Vector3d velocity
        Matrix3d cov_velocity
        Vector3d angular_velocity
        Matrix3d cov_angular_velocity


cdef extern from "base/samples/Frame.hpp" namespace "base::samples::frame":
    cdef enum frame_mode_t:
        MODE_UNDEFINED,
        MODE_GRAYSCALE,
        MODE_RGB,
        MODE_UYVY,
        MODE_BGR,
        MODE_BGR32,
        RAW_MODES,
        MODE_BAYER,
        MODE_BAYER_RGGB,
        MODE_BAYER_GRBG,
        MODE_BAYER_BGGR,
        MODE_BAYER_GBRG,
        COMPRESSED_MODES,
        MODE_PJPG,
        MODE_JPEG,
        MODE_PNG

    cdef enum frame_status_t:
        STATUS_EMPTY,
        STATUS_VALID,
        STATUS_INVALID

    cdef cppclass frame_size_t:
        int width
        int height

    cdef cppclass Frame:
        Frame()
        Frame(int width, int height, int depth, frame_mode_t mode, int val, int sizeInBytes)
        Frame(Frame other, bool bcopy)

        Time time
        Time received_time
        vector[uint8_t] image

        frame_size_t size

        uint32_t getChannelCount()
        uint32_t getPixelSize()
        uint32_t getRowSize()
        uint32_t getNumberOfBytes()
        uint32_t getPixelCount()
        uint32_t getDataDepth()
        uint16_t getWidth()
        uint16_t getHeight()
