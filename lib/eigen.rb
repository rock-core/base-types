require 'base_types_ext'

module Eigen
    # 3-dimensional vector
    class Vector3
        # Returns the [x, y, z] tuple
        def to_a; [x, y, z] end

        # Returns the (1, 0, 0) unit vector
        def self.UnitX()
            return Vector3.new(1, 0, 0)
        end

        # Returns the (0, 1, 0) unit vector
        def self.UnitY()
            return Vector3.new(0, 1, 0)
        end

        # Returns the (0, 0, 1) unit vector
        def self.UnitZ()
            return Vector3.new(0, 0, 1)
        end

        # Returns the angle formed by +self+ and +v+, oriented from +self+ to
        # +v+
        def angle_to(v)
            Math.atan2(v.y, v.x) - Math.atan2(y, x)
        end

        # Support for Marshal
        def _dump(level) # :nodoc:
            Marshal.dump(to_a)
        end

        # Support for Marshal
        def self._load(coordinates) # :nodoc:
            new(*Marshal.load(coordinates))
        end

        def to_s # :nodoc:
            "Vector3(#{x}, #{y}, #{z})"
        end

        ##
        # :method: ==

        ##
        # :method: []
        #
        # Returns the i-th coordinate

        ##
        # :method: []=
        #
        # Sets the i-th coordinate

        ##
        # :method: x

        ##
        # :method: y

        ##
        # :method: z

        ##
        # :method: x=

        ##
        # :method: y=

        ##
        # :method: z=

        ##
        # :method: +

        ##
        # :method: -

        ##
        # :method: -@

        ##
        # :method: *
        # :call-seq:
        #   a * scalar => b
        #
        # Returns +a+ scaled with the given scalar

        ##
        # :method: cross
        # :call-seq:
        #   cross(b) => c
        #
        # Returns the cross product of +self+ with +b+

        ##
        # :method: norm
        #
        # Returns the norm of +self+

        ##
        # :method: normalize!
        # 
        # Makes this vector unit-length

        ##
        # :method: normalize
        # 
        # Returns a vector that has the same direction than +self+ but unit
        # length
    end

    # Representation and manipulation of a quaternion
    class Quaternion
        # Returns the quaternion as [w, x, y, z]
        def to_a; [w, x, y, z] end

        # Returns the unit quaternion (identity rotation)
        def self.Unit
            Quaternion.new(1, 0, 0, 0)
        end

        # Creates a quaternion from a set of euler angles.
        #
        # See Quaternion#from_euler for details
        def self.from_euler(*args)
            q = new(0, 0, 0, 0)
            q.from_euler(*args)
            q
        end

        # Extracts the yaw angle from this quaternion
        #
        # It decomposes the quaternion in euler angles using to_euler(2, 1, 0) 
        # and returns the first element. See #to_euler for details.
        def yaw
            to_euler(2, 1, 0)[0]
        end

        # The inverse of #yaw
        def self.from_yaw(yaw)
            from_euler(Eigen::Vector3.new(yaw, 0, 0), 2, 1, 0)
        end

        # Concatenates with another quaternion or transforms a vector
        def *(obj)
            if obj.kind_of?(Quaternion)
                concatenate(obj)
            else
                transform(obj)
            end
        end

        def _dump(level) # :nodoc:
            Marshal.dump(to_a)
        end

        def self._load(coordinates) # :nodoc:
            new(*Marshal.load(coordinates))
        end

        def to_s # :nodoc:
            "Quaternion(#{w}, (#{x}, #{y}, #{z}))"
        end

        ##
        # :method: w

        ##
        # :method: x

        ##
        # :method: y

        ##
        # :method: z

        ##
        # :method: w=

        ##
        # :method: x=

        ##
        # :method: y=

        ##
        # :method: z=

        ##
        # :method: concatenate
        # :call-seq:
        #   concatenate(q)
        #
        # Returns the rotation in which +q+ is applied first and +self+ second

        ##
        # :method: transform
        # :call-seq:
        #   transform(v)
        #
        # Transforms the given Eigen::Vector3 by the rotation represented with
        # this quaternion

        ##
        # :method: normalize!
        # 
        # Normalizes this quaternion

        ##
        # :method: normalize
        # 
        # Returns a quaternion that is a normalized version of +self+

        ##
        # :method: approx?
        # :call-seq:
        #   approx?(q, tolerance)
        # 
        # Returns true if +self+ and +q+ do not differ from more than
        # +tolerance+. The comparison is done on a coordinate basis.

        ##
        # :method: to_euler
        # :call-seq:
        #    to_euler(axis0, axis1, axis2) => Eigen::Vector3(a0, a1, a2)
        # 
        # Decomposes this quaternion in euler angles so that +self+ can be
        # obtained by applying the following rotations in order:
        #
        #   rotation of a2 around axis2
        #   rotation of a1 around axis1
        #   rotation of a0 around axis0
        #
        # note that 
        #
        #   self == Quaternion.from_euler(to_euler(axis0, axis1, axis2), axis0, axis1, axis2)

        ##
        # :method: from_euler
        # :call-seq:
        #    from_euler(Eigen::Vector3(a0, a1, a2), axis0, axis1, axis2)
        # 
        # Resets this quaternion so that it represents the rotation obtained by
        # applying the following rotations in order:
        #
        #   rotation of a2 around axis2
        #   rotation of a1 around axis1
        #   rotation of a0 around axis0
        #
        # note that 
        #
        #   self == Quaternion.from_euler(to_euler(axis0, axis1, axis2), axis0, axis1, axis2)
    end
end

