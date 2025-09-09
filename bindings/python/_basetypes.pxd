from libcpp.string cimport string
from libcpp.vector cimport vector
from libcpp cimport bool
from libc.stdint cimport uint8_t, uint16_t, uint32_t, int64_t

cdef extern from "base/Angle.hpp" namespace "base":
    cdef cppclass Angle:
        Angle()

        double getDeg()
        double getRad()
        bool isApprox(Angle&, double)
        bool isApprox(Angle&)

        bool operator==(Angle&)
        bool operator<(Angle&)
        bool operator>(Angle&)
        bool operator<=(Angle&)
        bool operator>=(Angle&)

        Angle operator+(Angle&)
        Angle operator-(Angle&)
        Angle operator*(Angle&)
        Angle operator*(double)
        Angle operator*(double, Angle)

        Angle assign "operator="(Angle&)

        Angle flipped()
        Angle &flip()

        #std::ostream& operator << (std::ostream& os, Angle angle);  reimplemented
        #Angle operator*(double, Angle) used Angle operator*(double, Angle) instead
        #AngleSegment

cdef extern from "base/Angle.hpp" namespace "base::Angle":
    double rad2Deg(double)
    double deg2Rad(double)
    double normalizeRad(double)
    Angle fromRad(double)
    Angle fromDeg(double)
    Angle unknown()
    Angle Min()
    Angle Max()
    Angle vectorToVector(Vector3d&, Vector3d&)
    Angle vectorToVector(Vector3d&, Vector3d&, Vector3d&)


cdef extern from "base/Time.hpp" namespace "base":
    cdef cppclass Time:
        Time()

        int64_t microseconds

        Time& assign "operator="(Time&)

        bool operator<(Time)
        bool operator>(Time)
        bool operator==(Time)
        bool operator!=(Time)
        bool operator>=(Time)
        bool operator<=(Time)
        Time operator-(Time)
        Time operator+(Time)
        Time operator/(int)
        Time operator*(double)

        bool isNull()
        string toString(Resolution, string)


cdef extern from "base/Time.hpp" namespace "base::Time":
    cdef enum Resolution:
        Seconds
        Milliseconds
        Microseconds


cdef extern from "base/Time.hpp" namespace "base::Time":
    Time now()


cdef extern from "base/Eigen.hpp" namespace "base":
    cdef cppclass Vector2d:
        Vector2d()
        Vector2d(double, double)
        Vector2d(Vector2d&)
        double* data()
        int rows()
        Vector2d& assign "operator="(Vector2d&)
        bool operator==(Vector2d&)
        bool operator!=(Vector2d&)
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
        Vector3d& assign "operator="(Vector3d&)
        bool operator==(Vector3d&)
        bool operator!=(Vector3d&)
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
        Vector4d& assign "operator="(Vector4d&)
        bool operator==(Vector4d&)
        bool operator!=(Vector4d&)
        double& get "operator()"(int row)
        double x()
        double y()
        double z()
        double norm()
        double squaredNorm()

    cdef cppclass Matrix3d:
        Matrix3d()
        double* data()
        Matrix3d& assign "operator="(Matrix3d&)
        bool operator==(Matrix3d&)
        bool operator!=(Matrix3d&)
        double& get "operator()"(int row, int col)
        int rows()
        int cols()

    cdef cppclass Quaterniond:
        Quaterniond()
        Quaterniond(double, double, double, double)
        Quaterniond& assign "operator="(Quaterniond&)
        double w()
        double x()
        double y()
        double z()


cdef extern from "base/TransformWithCovariance.hpp" namespace "base":
    cdef cppclass TransformWithCovariance:
        TransformWithCovariance()
        TransformWithCovariance& assign "operator="(TransformWithCovariance&)
        Vector3d translation
        Quaterniond orientation


cdef extern from "base/JointState.hpp" namespace "base":
    cdef cppclass JointState:
        JointState()
        JointState& assign "operator="(JointState&)
        bool operator==(JointState&)
        bool operator!=(JointState&)
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

        NamedVector& assign "operator="(NamedVector&)
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
        RigidBodyState& assign "operator="(RigidBodyState&)
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

        Frame& assign "operator="(Frame&)

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


cdef extern from "base/samples/Pointcloud.hpp" namespace "base::samples":
    cdef cppclass Pointcloud:
        Pointcloud()

        Pointcloud& assign "operator="(Pointcloud&)

        Time time
        vector[Vector3d] points
        vector[Vector4d] colors


cdef extern from "base/samples/LaserScan.hpp" namespace "base::samples":
    cdef enum LASER_RANGE_ERRORS:  # TODO pyx
        TOO_FAR
        TOO_NEAR
        MEASUREMENT_ERROR
        OTHER_RANGE_ERRORS
        MAX_RANGE_ERROR

    cdef cppclass LaserScan:
        LaserScan()
        LaserScan& assign "operator="(LaserScan&)

        Time time
        double start_angle
        double angular_resolution
        double speed
        vector[uint32_t] ranges
        uint32_t minRange
        uint32_t maxRange
        vector[float] remission

        bool isValidBeam(unsigned int)
        void reset()
        bool isRangeValid(uint32_t)
        #convertScanToPointCloud(vector[T], Affine3d, bool) TODO wrap Affine3d
        #bool getPointFromScanBeamXForward(unsigned int i, Vector3d point) TODO
        #bool getPointFromScanBeam(unsigned int i, Vector3d point) TODO


cdef extern from "base/samples/IMUSensors.hpp" namespace "base::samples":
    cdef cppclass IMUSensors:
        IMUSensors()
        IMUSensors& assign "operator="(IMUSensors&)

        Time time
        Vector3d acc
        Vector3d gyro
        Vector3d mag

