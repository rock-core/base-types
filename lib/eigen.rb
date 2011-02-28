require 'base_types_ext'

module Eigen
    class Vector3
        def to_a; [x, y, z] end

        def self.UnitX()
            return Vector3.new(1, 0, 0)
        end

        def self.UnitY()
            return Vector3.new(0, 1, 0)
        end

        def self.UnitZ()
            return Vector3.new(0, 0, 1)
        end

        def angle_to(v)
            Math.atan2(v.y, v.x) - Math.atan2(y, x)
        end

        def _dump(level)
            Marshal.dump(to_a)
        end

        def self._load(coordinates)
            new(*Marshal.load(coordinates))
        end

        def to_s
            "Vector3(#{x}, #{y}, #{z})"
        end
    end

    class Quaternion
        # Returns the quaternion as [w, x, y, z]
        def to_a; [w, x, y, z] end

        def self.Unit
            Quaternion.new(1, 0, 0, 0)
        end

        def self.from_euler(*args)
            q = new(0, 0, 0, 0)
            q.from_euler(*args)
            q
        end

        def yaw
            to_euler(2, 1, 0)[0]
        end

        def self.from_yaw(yaw)
            from_euler(Eigen::Vector3.new(yaw, 0, 0), 2, 1, 0)
        end

        def *(obj)
            if obj.kind_of?(Quaternion)
                concatenate(obj)
            else
                transform(obj)
            end
        end

        def _dump(level)
            Marshal.dump(to_a)
        end

        def self._load(coordinates)
            new(*Marshal.load(coordinates))
        end

        def to_s
            "Quaternion(#{w}, (#{x}, #{y}, #{z}))"
        end
    end
end

