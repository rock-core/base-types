require 'base_types_ext'

module Types
module Base
    module Geometry
        # Representation and manipulation of N-dimensional B-splines
        class Spline
            # Returns a Spline object that interpolates the given points
            #
            # See Spline#interpolate for details. Unlike Spline#interpolate,
            # +points+ must be a nested array so that the curve dimension can be
            # guessed.
            def self.interpolate(points, parameters = nil)
                if points.first.kind_of?(Numeric)
                    raise ArgumentError, "cannot guess the curve dimensions from a flat point array"
                end
                dimension = points.first.to_a.size

                spline = Spline.new(dimension)
                spline.interpolate(points, parameters)
                spline
            end

            # Returns a copy of this curve
            def dup
                result = self.class.new(dimension, geometric_resolution, order)
                result.initialize_copy(self)
                result
            end

            # Generates a regular sampling of the curve
            #
            # Returns an array of points sampled at a regular interval in t
            def sample(delta_t)
                result = []
                start_param.step(end_param, delta_t) do |t|
                    result << get(t)
                end
                result
            end

            # Resets this curve so that it is an interpolation of the given set
            # of points
            #
            # The set of points can either be a list of DIM * SIZE numbers. It
            # can also be a nested array of the form
            #
            #   [[c1, c2, c3], [c1, c2, c3], ...]
            #
            # If given, +parameters+ is an array of the same size than the
            # number of points. This array represents the desired parameters of
            # each of the points in the resulting spline
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

            # Returns the point at the start of the curve
            def start_point; get(start_param) end
            # Returns the point at the end of the curve
            def end_point; get(end_param) end

            ##
            # :method: dimension
            #
            # Returns the dimension of the space in which the curve lies

            ##
            # :method: curve_length
            #
            # Returns the length of the curve

            ##
            # :method: curvature_max
            #
            # Returns the maximum curvature of this curve

            ##
            # :method: start_param
            #
            # Returns the parameter of the first point of the curve

            ##
            # :method: end_param
            #
            # Returns the parameter of the last point of the curve

            ##
            # :method: curvature_at
            # :call-seq:
            #   curvature_at(t) => value
            #
            # Returns the curvature at the given parameter

            ##
            # :method: variation_of_curvature_at
            # :call-seq:
            #   variation_of_curvature_at(t) => value
            #
            # Returns the variation of curvature at the given parameter

            ##
            # :method: simplify
            # :call-seq:
            #   simplify(tolerance)
            #
            # Removes the B-spline knots so that the resulting curve does not
            # differ from the original one for more than +tolerance+

            ##
            # :method: clear
            #
            # Removes all curve information (empty? returns true afterwards)

            ##
            # :method: join
            # :call-seq:
            #     join(other_curve, tolerance = 0, with_tangents = true)
            #
            # Joins +other_curve+ at the end of +self+.
            #
            # If the end of +self+
            # and the beginning of +other_curve+ are further away than
            # +tolerance+, an intermediate curve is created. It is a straight
            # line if +with_tangents+ is false, or takes into account the
            # tangents at the end of +self+ and the start of +other_curve+
            # otherwise
            #
            # If the end of +self+ and the beginning of +other_curve+ are closer
            # than +tolerance+, +other_curve+ is simply appended. See #append

            ##
            # :method: append
            # :call-seq:
            #   append(other_curve)
            #
            # Moves +other_curve+ to align its start point with the end point of
            # +self+ and appends it to +self+

            ##
            # :method: order
            #
            # Returns the curve order

            ##
            # :method: empty?
            #
            # True if this curve has no inforamtion (it is not the same than a
            # zero-length curve)

            ##
            # :method: singleton?
            #
            # True if this curve is zero-length

            ##
            # :method: reverse
            #
            # Reverses the direction of +self+

            ##
            # :method: get
            # :call-seq:
            #   get(t) => array_of_coordinates
            #   get(t, true) => array_of_coordinates, array_of_coordinates
            #
            # Returns the point and optionally the tangent at the given
            # parameter
            #
            # The point and tangent are represented as arrays of coordinates, of
            # size #dimension
        end

        # Specialization of Spline for 3D splines
        #
        # Unlike Spline, the points are represented using Eigen::Vector3 instead
        # of arrays of coordinates
        class Spline3 < Spline
            # Creates a Spline3 curve that interpolates the given points
            #
            # +points+ is an array of Eigen::Vector3 instances. See
            # Spline#interpolate for details on +parameters+
            def self.interpolate(points, parameters = nil)
                spline = Spline3.new
                spline.interpolate(points, coordinates)
                spline
            end

            def initialize(geometric_resolution = 0.1, order = 3)
                super(3, geometric_resolution, order)
            end

            ##
            # :method: find_closest_points
            # call-seq:
            #   find_closest_points(reference_point, geometric_resolution, guess = 0) => points, segments
            #
            # Finds the part of the curve that are the closest to
            # +reference_point+, with a tolerance of +geometric_resolution+
            #
            # The returned arrays are a set of single points (as parameters on
            # the curve) and a set of curve segments.
            def find_closest_points(reference_point, geometric_resolution)
                do_find_closest_points(reference_point.to_a, geometric_resolution)
            end

            # Return the length of the curve segment contained between +start_t+
            # and +end_t+. The length is computed by discretizing the curve with
            # a step distance of +geores+
            def length(start_t, end_t, geores)
                return do_length(start_t, end_t, geores)
            end

            # Resets this curve to a curve interpolating the given points
            #
            # +points+ is an array of Eigen::Vector3 instances. See
            # Spline#interpolate for details on +parameters+
            def interpolate(points, parameters = nil)
                coordinates = []
                for p in points
                    coordinates << p.x << p.y << p.z
                end
                super(coordinates, parameters)
            end

            # Returns the point at the given parameter.
            #
            # If +with_tangent+ is true, it returns a pair of Eigen::Vector3,
            # where the first one is the point and the second the tangent.
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
