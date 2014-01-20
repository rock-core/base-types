#include "Vizkit3DHelper.hpp"

namespace vizkit3d {

osg::Quat eigenQuatToOsgQuat(const Eigen::Quaterniond &quat) {
    return osg::Quat(quat.x(), quat.y(), quat.z(), quat.w());
}

osg::Vec3d eigenVectorToOsgVec3(const Eigen::Vector3d &vec) {
    return osg::Vec3d(vec.x(), vec.y(), vec.z());
}

}
