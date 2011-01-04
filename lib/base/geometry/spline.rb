require 'base_types_ext'

module BaseTypes
    module Geometry
        class Spline
            def self.interpolate(points, coordinates)
                if points.first.kind_of?(Numeric)
                    raise ArgumentError, "cannot guess the curve dimensions from a flat point array"
                end
                dimension = points.first.to_a.size

                spline = Spline.new(dimension)
                spline.interpolate(points, coordinates)
                spline
            end

            def dup
                result = Spline.new(dimension, geometric_resolution, order)
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
    end
end

