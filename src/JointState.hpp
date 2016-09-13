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
        { POSITION, SPEED, EFFORT, RAW, ACCELERATION, UNSET };

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

        /** Acceleration in radians per square second for angular actuators, in m/ss
         * for linear ones */
        float acceleration;

        JointState()
            : position(base::unset<double>())
            , speed(base::unset<float>())
            , effort(base::unset<float>())
            , raw(base::unset<float>())
            , acceleration(base::unset<float>()) {}

        /** Returns a JointState object with the position field set to the given
         * value
         */
        static JointState Position(double value);

        /** Returns a JointState object with the speed field set to the given
         * value
         */
        static JointState Speed(float value);

        /** Returns a JointState object with the effort field set to the given
         * value
         */
        static JointState Effort(float value);

        /** Returns a JointState object with the raw field set to the given
         * value
         */
        static JointState Raw(float value);

        /** Returns a JointState object with the acceleration field set to the given
         * value
         */
        static JointState Acceleration(float value);

        /** Tests whether the position field is set */
        bool hasPosition() const;
        /** Tests whether the speed field is set */
        bool hasSpeed() const;
        /** Tests whether the effort field is set */
        bool hasEffort() const;
        /** Tests whether the raw field is set */
        bool hasRaw() const;
        /** Tests whether the acceleration field is set */
        bool hasAcceleration() const;

        /** Tests whether the position field is the only field set
         *
         * This is commonly used in controllers to test whether this data
         * structure is a valid command for said controller (e.g. a position
         * controller would throw if isPosition() returns false)
         */
        bool isPosition() const;
        /** Tests whether the speed field is the only field set
         *
         * This is commonly used in controllers to test whether this data
         * structure is a valid command for said controller (e.g. a speed
         * controller would throw if isSpeed() returns false)
         */
        bool isSpeed() const;
        /** Tests whether the effort field is the only field set
         *
         * This is commonly used in controllers to test whether this data
         * structure is a valid command for said controller (e.g. a torque
         * controller would throw if isEffort() returns false)
         */
        bool isEffort() const;
        /** Tests whether the raw field is the only field set
         *
         * This is commonly used in controllers to test whether this data
         * structure is a valid command for said controller (e.g. a hardware
         * driver that is controlled in PWM would throw if isRaw() returns
         * false)
         */
        bool isRaw() const;
        /** Tests whether the acceleration field is the only field set
         *
         * This is commonly used in controllers to test whether this data
         * structure is a valid command for said controller (e.g. a hardware
         * driver that is controlled in acceleration would throw if isAcceleration() returns
         * false)
         */
        bool isAcceleration() const;

        /** Returns the field that matches the given mode
         *
         * @see MODE
         */
        double getField(int mode) const;

        /** Sets the given field to the given value
         *
         * @see MODE
         */
        void setField(int mode, double value);

        /** Returns the mode of this JointState
         *
         * The mode is defined only if the structure has only one field set.
         * runtime_error is thrown if this is called with more than one field
         * set.
         *
         * @see MODE
         */
        MODE getMode() const;
    };
}

#endif
