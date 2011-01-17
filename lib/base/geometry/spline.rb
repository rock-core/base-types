require 'base_types_ext'

module Types
module Base
    module Geometry
        class Spline
            def self.interpolate(points, parameters = nil)
                if points.first.kind_of?(Numeric)
                    raise ArgumentError, "cannot guess the curve dimensions from a flat point array"
                end
                dimension = points.first.to_a.size

                spline = Spline.new(dimension)
                spline.interpolate(points, parameters)
                spline
            end

            def dup
                result = self.class.new(dimension, geometric_resolution, order)
                result.initialize_copy(self)
                result
            end

            def sample(delta_t)
                result = []
                start_param.step(end_param, delta_t) do |t|
                    result << get(t)
                end
                result
            end


            def interpolate(points, parameters = nil)
                if points.empty?
                    clear
                    return
                end

                if points.first.kind_of?(Numeric)
                    do_interpolate(points, parameters || [])
                else
                    coordinates = points.map(&:to_a).flatten!
                    if points.size * self.dimension != coordinates.size
                        raise ArgumentError, "point of wrong dimension given to #interpolate: got #{coordinates.size} coordinates overall, expected #{points.size * self.dimension}"
                    end

                    do_interpolate(coordinates, parameters || [])
                end
            end

            def start_point; get(start_param) end
            def end_point; get(end_param) end
        end

        # Specialization of Spline for 3D splines
        class Spline3 < Spline
            def self.interpolate(points, parameters = nil)
                spline = Spline3.new(3)
                spline.interpolate(points, coordinates)
                spline
            end

            def interpolate(points, parameters = nil)
                coordinates = []
                for p in points
                    coordinates << p.x << p.y << p.z
                end
                super(coordinates, parameters)
            end

            def get(param, with_tangent = false)
                result = super
                p = Eigen::Vector3.new(*result[0, 3])
                if with_tangent
                    t = Eigen::Vector3.new(*result[3, 3])
                    return p, t
                else
                    return p
                end
            end
        end
    end
end
end
