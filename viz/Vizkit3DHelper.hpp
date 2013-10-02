#ifndef VIZKIT3DHELPER_HPP
#define VIZKIT3DHELPER_HPP

#include <osg/Vec3d>
#include <osg/Quat>
#include <Eigen/Geometry>

namespace vizkit3d {

osg::Quat eigenQuatToOsgQuat(const Eigen::Quaterniond &quat);

osg::Vec3d eigenVectorToOsgVec3(const Eigen::Vector3d &vec);
}
#endif
