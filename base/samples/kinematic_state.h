#ifndef BASE_SAMPLES_KINEMATICSTATE__
#define BASE_SAMPLES_KINEMATICSTATE__

#include <base/time.h>

namespace base
{
namespace samples
{

/** 
 * @brief represents the current value for a named joint
 *
 * Currently only supports joints with single degree of freedom,
 * but can be extended to support different types of joints.
 */
struct JointValue
{
    /** @brief label of the joint
     */
    std::string label;

    /* @brief current joint angle in rad
     */
    double angle;
};

/** 
 * @brief Represents the kinematic state of a robot
 *
 * This class represents the kinematic state of a robot, 
 * which can be represented as values for joints, linear
 * elements, potentially positions a.s.o.
 * In the current state it handles only joints, which 
 * should be sufficient for a lot of systems.
 */
struct KinematicState
{
    /**
     * @brief timestamp for which the datum is valid
     */
    base::Time time;

    /**
     * @brief list of joint values which represent the kinematic state of the 
     * robot
     */
    std::vector<JointValue> joints;
};

}
}

#endif
