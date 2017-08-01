from libcpp cimport bool
from libcpp.vector cimport vector
from libcpp.string cimport string
from libc.stdint cimport uint32_t
cimport _basetypes

cdef class Time:
    cdef _basetypes.Time* thisptr
    cdef bool delete_thisptr


cdef class Vector2d:
    cdef _basetypes.Vector2d* thisptr
    cdef bool delete_thisptr


cdef class Vector3d:
    cdef _basetypes.Vector3d* thisptr
    cdef bool delete_thisptr


cdef class Vector4d:
    cdef _basetypes.Vector4d* thisptr
    cdef bool delete_thisptr


cdef class Matrix3d:
    cdef _basetypes.Matrix3d* thisptr
    cdef bool delete_thisptr


cdef class Quaterniond:
    cdef _basetypes.Quaterniond* thisptr
    cdef bool delete_thisptr


cdef class TransformWithCovariance:
    cdef _basetypes.TransformWithCovariance* thisptr
    cdef bool delete_thisptr


cdef class JointState:
    cdef _basetypes.JointState* thisptr
    cdef bool delete_thisptr


cdef class Joints:
    cdef _basetypes.Joints* thisptr
    cdef bool delete_thisptr


cdef class StringVectorReference:
    cdef vector[string]* thisptr


cdef class JointStateVectorReference:
    cdef vector[_basetypes.JointState]* thisptr


cdef class RigidBodyState:
    cdef _basetypes.RigidBodyState* thisptr
    cdef bool delete_thisptr


cdef class Frame:
    cdef _basetypes.Frame* thisptr
    cdef bool delete_thisptr


cdef class Pointcloud:
    cdef _basetypes.Pointcloud* thisptr
    cdef bool delete_thisptr


cdef class Vector3dVectorReference:
    cdef vector[_basetypes.Vector3d]* thisptr


cdef class Vector4dVectorReference:
    cdef vector[_basetypes.Vector4d]* thisptr


cdef class LaserScan:
    cdef _basetypes.LaserScan* thisptr
    cdef bool delete_thisptr


cdef class UInt32VectorReference:
    cdef vector[uint32_t]* thisptr


cdef class FloatVectorReference:
    cdef vector[float]* thisptr


cdef class IMUSensors:
    cdef _basetypes.IMUSensors* thisptr
    cdef bool delete_thisptr
