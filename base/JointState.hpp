#ifndef BASE_JOINT_STATE_HPP
#define BASE_JOINT_STATE_HPP

#include <base/Float.hpp>
#include <stdexcept>

namespace base
{
    /** Representation of the state of a given joint
     *
     * The joint does not have to necessarily be actuated. This type is also
     * used as inputs to controller, in which case the setpoints should be
     * tested with the has* and is* predicates (e.g. hasPosition(), ...)
     *
     * The values given in such structures can only be interpreted when
     * associated with kinematics data. In Rock, see the control/robot_model
     * package for more information.
     */
    struct JointState
    {
        enum MODE
        { POSITION, SPEED, EFFORT, RAW, UNSET };

        /** Current position of the actuator, in radians for angular
         * joints, in m for linear ones
         *
         * For angular joints that can move more than 360 degrees, this
         * accumulates the movement since initialization
         *
         * If the joint is an angular joint whose motion is constrained to less
         * than one full turn, the value should be in [-PI, PI]. base::Angle
         * could be used to manipulate it before setting it in this structure
         *
         * If the joint is an unconstrained angular joint (e.g. a wheel joint),
         * the range is [-inf, inf]
         */
        double position;

        /** Speed in radians per second for angular actuators, in m/s
         * for linear ones
         * 
         * This is an instantaneous speed. It means that, considering two
         * consecutive JointState samples,
         *    (position1 - position0)/(time1 - * time0).toSeconds()
         * is not necessarily equal to 'speed'
         */
        float speed;

        /** Torque in N.m for angular joints and N for linear ones
         */
        float effort;

        /** Raw command to/from the actuator, if this is an actuated joint. It
         * is commonly a PWM value in [0,1]
         */
        float raw;

        JointState()
            : position(base::unset<float>())
            , speed(base::unset<float>())
            , effort(base::unset<float>())
            , raw(base::unset<float>()) {}

        /** Returns a JointState object with the position field set to the given
         * value
         */
        static JointState Position(float value)
        {
            JointState ret;
            ret.position = value;
            return ret;
        }

        /** Returns a JointState object with the speed field set to the given
         * value
         */
        static JointState Speed(float value)
        {
            JointState ret;
            ret.speed = value;
            return ret;
        }

        /** Returns a JointState object with the effort field set to the given
         * value
         */
        static JointState Effort(float value)
        {
            JointState ret;
            ret.effort = value;
            return ret;
        }

        /** Returns a JointState object with the raw field set to the given
         * value
         */
        static JointState Raw(float value)
        {
            JointState ret;
            ret.raw = value;
            return ret;
        }

        /** Tests whether the position field is set */
        bool hasPosition() const { return !base::isUnset<float>(position); }
        /** Tests whether the speed field is set */
        bool hasSpeed() const { return !base::isUnset<float>(speed); }
        /** Tests whether the effort field is set */
        bool hasEffort() const { return !base::isUnset<float>(effort); }
        /** Tests whether the raw field is set */
        bool hasRaw() const { return !base::isUnset<float>(raw); }

        /** Tests whether the position field is the only field set
         *
         * This is commonly used in controllers to test whether this data
         * structure is a valid command for said controller (e.g. a position
         * controller would throw if isPosition() returns false)
         */
        bool isPosition() const { return hasPosition() && !hasSpeed() && !hasEffort() && !hasRaw(); }
        /** Tests whether the speed field is the only field set
         *
         * This is commonly used in controllers to test whether this data
         * structure is a valid command for said controller (e.g. a speed
         * controller would throw if isSpeed() returns false)
         */
        bool isSpeed() const { return !hasPosition() && hasSpeed() && !hasEffort() && !hasRaw(); }
        /** Tests whether the effort field is the only field set
         *
         * This is commonly used in controllers to test whether this data
         * structure is a valid command for said controller (e.g. a torque
         * controller would throw if isEffort() returns false)
         */
        bool isEffort() const { return !hasPosition() && !hasSpeed() && hasEffort() && !hasRaw(); }
        /** Tests whether the raw field is the only field set
         *
         * This is commonly used in controllers to test whether this data
         * structure is a valid command for said controller (e.g. a hardware
         * driver that is controlled in PWM would throw if isRaw() returns
         * false)
         */
        bool isRaw() const { return !hasPosition() && !hasSpeed() && !hasEffort() && hasRaw(); }

        /** Returns the field that matches the given mode
         *
         * @see MODE
         */
        float getField(int mode) const
        {
            switch(mode)
            {
                case POSITION: return position;
                case SPEED: return speed;
                case EFFORT: return effort;
                case RAW: return raw;
                default: throw std::runtime_error("invalid mode given to getField");
            }
        }

        /** Sets the given field to the given value
         *
         * @see MODE
         */
        void setField(int mode, float value)
        {
            switch(mode)
            {
                case POSITION:
                    position = value;
                    return;
                case SPEED:
                    speed = value;
                    return;
                case EFFORT:
                    effort = value;
                    return;
                case RAW:
                    raw = value;
                    return;
                default: throw std::runtime_error("invalid mode given to getField");
            }
        }

        /** Returns the mode of this JointState
         *
         * The mode is defined only if the structure has only one field set.
         * runtime_error is thrown if this is called with more than one field
         * set.
         *
         * @see MODE
         */
        MODE getMode() const
        {
            if (isPosition())    return POSITION;
            else if (isSpeed())  return SPEED;
            else if (isEffort()) return EFFORT;
            else if (isRaw())    return RAW;
            else if (hasPosition() || hasSpeed() || hasEffort() || hasRaw())
                throw std::runtime_error("getMode() called on a JointState that has more than one field set");
            else
                return UNSET;
        }
    };
}

#endif
