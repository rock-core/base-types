require 'eigen_ext'

module Eigen
    class Vector3
        def to_a; [x, y, z] end
    end

    class Quaternion
        # Returns the quaternion as [w, x, y, z]
        def to_a; [w, x, y, z] end

        def self.from_euler(*args)
            q = new(0, 0, 0, 0)
            q.from_euler(*args)
            q
        end

        def *(obj)
            if obj.kind_of?(Quaternion)
                concatenate(obj)
            else
                transform(obj)
            end
        end
    end
end

