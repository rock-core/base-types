require 'base_types_ruby'

module Eigen
    # 3-dimensional vector
    class Vector3
        # Returns a vector with all values set to Base.unset
        def self.Unset
            return Vector3.new(Base.unset, Base.unset, Base.unset)
        end

        def dup
            Vector3.new(x, y, z)
        end

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

        # returns the (0, 0, 0) vector
        def self.Zero()
            return Vector3.new(0, 0, 0)
        end

        # Returns the angle formed by +self+ and +v+, oriented from +self+ to
        # +v+
        def angle_to(v)
            ret = Math.atan2(v.y, v.x) - Math.atan2(y, x)
            if ret > Math::PI
                ret -= 2*Math::PI
            end
            if ret < -Math::PI
                ret += 2*Math::PI
            end
            ret
        end

        # Tests for equality
        #
        # Since Vector3 stores the coordinates as floating-point values, this is
        # a bad test. Use
        #
        #   q.approx?(other_q, tolerance)
        #
        # instead
        def ==(v)
            v.kind_of?(self.class) &&
                __equal__(v)
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

        def data
            [x, y, z]
        end

        def data=(value)
            self.x,self.y,self.z = value
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

        ##
        # Computes the signed angle between two vectors, using the provided
        # vector as "positive" rotation direction
        #
        # The returned angle A is so that the rotation defined by A and axis
        # will transform +self+ into +v+
        def signed_angle_to(v, axis)
            dot_p   = self.dot(v)
            dir = self.cross(v).dot(axis)

            unsigned = Math.acos(dot_p / norm / v.norm)
            if dir > 0
                return unsigned
            else
                return -unsigned
            end
        end

        # @return [Qt::Quaternion] the Qt vector that is identical to this
        # one
        def to_qt
            Qt::Vector3D.new(x, y, z)
        end
    end

    # Representation and manipulation of a quaternion
    class Quaternion
        def dup
            Quaternion.new(w, x, y, z)
        end

        # Returns the quaternion as [w, x, y, z]
        def to_a; [w, x, y, z] end

        # Returns the identity unit quaternion (identity rotation)
        def self.Identity
                Quaternion.new(1, 0, 0, 0)
        end

        # DEPRECATED: please use identity instead. Returns the unit quaternion (identity rotation)
        def self.Unit
	    warn "[DEPRECATED] Quaternion.unit, please use Quaternion.identity."
	    self.Identity
        end

        # Creates a quaternion from an angle and axis description 
        def self.from_angle_axis(*args)
                q = new(1, 0, 0, 0)
            q.from_angle_axis(*args)
            q
        end

        # Returns an angle,axis representation equivalent to this quaternion
        #
        # If the angle turns out to be PI, there are two solutions and the one
        # with positive Z component is chosen.
        #
        # @param [Float] eps if the angle turns out to be closer to zero than eps, the
        #   rotation axis is undefined and chosen to be the Z axis.
        #
        # @return [(Float,Vector3)] the angle and axis. The angle is in [0, PI]
        def to_angle_axis(eps = 1e-12)
            w, x, y, z = to_a
            norm  = Math.sqrt(x*x+y*y+z*z);
            if norm < eps
                return 0, Eigen::Vector3.new(0,0,1);
            end

            angle = 2.0 * Math.atan2(norm, w);
            axis  = Eigen::Vector3.new(x, y, z) / norm
            return angle, axis
        end

        # Returns a scaled axis representation that is equivalent to this
        # quaternion
        #
        # @param [Float] eps see {#to_angle_axis}
        # @return [Vector3]
        def to_scaled_axis(eps = 1e-12)
            angle, axis = to_angle_axis(eps)
            return axis * angle
        end

        # Creates a quaternion from a set of euler angles.
        #
        # See Quaternion#from_euler for details
        def self.from_euler(*args)
            q = new(1, 0, 0, 0)
            q.from_euler(*args)
            q
        end

        # Creates a quaternion from a rotation matrix
        def self.from_matrix(m)
            q = new(1, 0, 0, 0)
            q.from_matrix(m)
            q
        end

        # Extracts the yaw angle from this quaternion
        #
        # It decomposes the quaternion in euler angles using to_euler
        # and returns the first element. See #to_euler for details.
        def yaw
            to_euler[0]
        end

        def pitch
            to_euler[1]
        end

        def roll
            to_euler[2]
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

        # Tests for equality
        #
        # Since Quaternion stores the coordinates as floating-point values, this is
        # a bad test. Use
        #
        #   (v - other_v).norm < threshold
        #
        # instead
        def ==(q)
            q.kind_of?(self.class) &&
                __equal__(q)
        end

        def re
            w
        end

        def re=(value)
            w = value
        end

        def im
            [x,y,z]
        end

        def im=(value)
            x,y,z = value
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
        #    to_euler => Eigen::Vector3(a0, a1, a2)
        # 
        # Decomposes this quaternion in euler angles so that +self+ can be
        # obtained by applying the following rotations in order:
        #
        #   rotation of a2 around x-axis
        #   rotation of a1 around y-axis
        #   rotation of a0 around z-axis
        #
        #   assuming angles in range of: a0:(-pi,pi), a1:(-pi/2,pi/2), a2:(-pi/2,pi/2)
        #
        # note that 
        #
        #   self == Quaternion.from_euler(to_euler, axis0, axis1, axis2)

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
        #   self == Quaternion.from_euler(to_euler, axis0, axis1, axis2)

        ## 
        # :method: inverse
        # :call-seq:
        #   inverse => quaternion
        #
        # Computes the quaternion that is inverse of this one


        # @return [Qt::Quaternion] the Qt quaternion that is identical to this
        # one
        def to_qt
            Qt::Quaternion.new(w, x, y, z)
        end
    end

    # Representation and manipulation of an angle axis
    class AngleAxis
        def dup
            AngleAxis.new(angle, axis)
        end

        # Returns the angle axis as [angle, x, y, z]
        def to_a; [angle, axis.to_a] end

        def from_a (array)
            aa = AngleAxis.new(array[0], Eigen::Vector3.new(array[1][0], array[1][1], array[1][2]))
            aa
        end

        # Returns the identity unit quaternion (identity rotation)
        def self.Identity
            AngleAxis.new(0, Eigen::Vector3.new(1, 0, 0))
        end

        # Creates a angle axis from a quaternion
        def self.from_quaternion(*args)
            aa = new(0, Eigen::Vector3.new(1, 0, 0))
            aa.from_quaternion(*args)
            aa
        end

        # Creates a quaternion from a set of euler angles.
        #
        # See Quaternion#from_euler for details
        def self.from_euler(*args)
            aa = new(0, Eigen::Vector3.new(1, 0, 0))
            aa.from_euler(*args)
            aa
        end

        # Creates a quaternion from a rotation matrix
        def self.from_matrix(m)
            aa = new(0, Eigen::Vector3.new(1, 0, 0))
            aa.from_matrix(m)
            aa
        end

        # Returns a scaled axis representation that is equivalent to this
        # quaternion
        #
        # @param [Float] eps see {#to_angle_axis}
        # @return [Vector3]
        def to_scaled_axis(eps = 1e-12)
            return axis * angle
        end

        # Concatenates with another angle axis or transforms a vector
        def *(obj)
            if obj.kind_of?(AngleAxis)
                concatenate(obj)
            else
                transform(obj)
            end
        end

        def _dump(level) # :nodoc:
            Marshal.dump(to_a)
        end

        def self._load(coordinates) # :nodoc:
            aa = new(0, Eigen::Vector3.new(1, 0, 0))
            aa.from_a(Marshal.load(coordinates))
            aa
        end

        def to_s # :nodoc:
            "AngleAxis( angle #{angle}, axis(#{axis.x}, #{axis.y}, #{axis.z}))"
        end

        # Tests for equality
        #
        # Since Quaternion stores the coordinates as floating-point values, this is
        # a bad test. Use
        #
        #   (v - other_v).norm < threshold
        #
        # instead
        def ==(q)
            q.kind_of?(self.class) &&
                __equal__(q)
        end
    end

    # Abritary size vector
    class VectorX
        def dup
            VectorX.from_a(to_a)
        end

        # Returns the array value in a vector
        def to_a()
            a = []
            for i in 0..size()-1
                    a << self[i]
            end
            a
        end

        def self.from_a(array)
            v = VectorX.new
            v.from_a(array)
            v
        end

        def from_a(array)
            resize(array.size())
            for i in 0..array.size()-1
                self[i] = array[i]
            end
        end

        def ==(v)
            v.kind_of?(self.class) &&
                __equal__(v)
        end

        def to_s # :nodoc:
            str = "VectorX("
            for i in 0..size()-1
                str += "#{self[i]} "
            end
            str[-1] = ")"
            str
        end

        def _dump(level) # :nodoc:
            Marshal.dump(to_a)
        end

        def self._load(coordinates) # :nodoc:
            m = new()
            m.from_a(Marshal.load(coordinates))
            m
        end
    end

    # Matrix 4x4
    class Matrix4
        def self.from_a(*args)
            m = new
            m.from_a(*args)
            m
        end

        # Returns the array value in a vector 
        def to_a(column_major=true)
            a = []
            if column_major
                for j in 0..3
                    for i in 0..3
                        a << self[i,j]
                    end
                end
            else
                for i in 0..3
                    for j in 0..3
                        a << self[i,j]
                    end
                end
            end
            a
        end

        # sets matrix from a 1d array
        def from_a(array, column_major=true)
            array.each_index do |i|
                v = array[i]
                if !v
                    v = 0.0
                end
                if column_major
                    self[i%4,i/4] = v
                else
                    self[i/4,i%4] = v
                end
            end
        end

        def ==(m)
            m.kind_of?(self.class) &&
                __equal__(m)
        end

        def to_s # :nodoc:
            str = "Matrix4(\n"
            for i in 0..3
                for j in 0..3
                    str += "#{self[i,j]} "
                end
                str[-1] = "\n"
            end
            str += ")"
            str
        end

        def _dump(level) # :nodoc:
            Marshal.dump({'rows' => rows, 'cols' => cols, 'data' => to_a})
        end

        def self._load(coordinates) # :nodoc:
            o = Marshal.load(coordinates)
            m = new()
            m.from_a(o['data'])
            m
        end
    end

    # Abritary size vector
    class MatrixX
        def dup
            MatrixX.from_a(to_a, rows, cols)
        end

        def self.from_a(*args)
            m = new
            m.from_a(*args)
            m
        end

        # Returns the array value in a vector 
        def to_a(column_major=true)
            a = []
            if column_major
                for j in 0..cols()-1
                    for i in 0..rows()-1
                        a << self[i,j]
                    end
                end
            else
                for i in 0..rows()-1
                    for j in 0..cols()-1
                        a << self[i,j]
                    end
                end
            end
            a
        end

        # sets matrix from a 1d array
        def from_a(array,nrows=-1,mcols=-1,column_major=true)
            if nrows == -1 && mcols == -1
                nrows = rows
                mcols = cols
            elsif nrows == -1
                nrows = array.size / mcols
            elsif mcols == -1
                ncols = array.size / nrows
            end
            resize(nrows,mcols)
            array.each_index do |i|
                v = array[i]
                if !v
                    v = 0.0
                end
                if column_major
                    self[i%nrows,i/nrows] = v
                else
                    self[i/mcols,i%mcols] = v
                end
            end
        end

        def pretty_print(pp)
            for i in 0..rows()-1
                for j in 0..cols()-1
                    pp.text " #{self[i,j]}"
                end
                pp.text "\n"
            end
        end

        def ==(m)
            m.kind_of?(self.class) &&
                __equal__(m)
        end

        def to_s # :nodoc:
            str = "MatrixX(\n"
            for i in 0..rows()-1
                for j in 0..cols()-1
                    str += "#{self[i,j]} "
                end
                str[-1] = "\n"
            end
            str += ")"
            str
        end

        def _dump(level) # :nodoc:
            Marshal.dump({'rows' => rows, 'cols' => cols, 'data' => to_a})
        end

        def self._load(coordinates) # :nodoc:
            o = Marshal.load(coordinates)
            m = new(o['rows'],o['cols'])
            m.from_a(o['data'],o['rows'],o['cols'])
            m
        end
    end

    class Isometry3
        def self.Identity
            Isometry3.new
        end

        def self.from_position_orientation( v, q )
            i = Isometry3.Identity
            i.prerotate( q )
            i.pretranslate( v )
            i
        end

        def dup
            raise NotImplementedError
        end

        def ==(q)
            q.kind_of?(self.class) &&
                __equal__(q)
        end

        def *(obj)
            if obj.kind_of?(Isometry3)
                concatenate(obj)
            else
                transform(obj)
            end
        end

        def to_s
            matrix.to_s
        end
    end

    class Affine3
        def self.Identity
            Affine3.new
        end

        def self.from_position_orientation( v, q )
            i = Affine3.Identity
            i.prerotate( q )
            i.pretranslate( v )
            i
        end

        def dup
            raise NotImplementedError
        end

        def ==(q)
            q.kind_of?(self.class) &&
                __equal__(q)
        end

        def *(obj)
            if obj.kind_of?(Affine3)
                concatenate(obj)
            else
                transform(obj)
            end
        end

        def to_s
            matrix.to_s
        end
    end
end

