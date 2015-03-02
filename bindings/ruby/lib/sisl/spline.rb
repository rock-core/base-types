require 'base_types_ruby'
require 'eigen'

# Bindings for the SISL spline library
module SISL
        # Representation and manipulation of N-dimensional B-splines
        class Spline
            # Returns a Spline object that interpolates the given points
            #
            # See Spline#interpolate for details. Unlike Spline#interpolate,
            # +points+ must be a nested array so that the curve dimension can be
            # guessed.
            def self.interpolate(points, parameters = nil, types = nil)
                if points.first.kind_of?(Numeric)
                    raise ArgumentError, "cannot guess the curve dimensions from a flat point array"
                end
                dimension = points.first.to_a.size

                spline = Spline.new(dimension)
                spline.interpolate(points, parameters, types)
                spline
            end

            def self.singleton(point)
                interpolate([point])
            end

            # Returns a copy of this curve
            def dup
                result = self.class.new(dimension, geometric_resolution, order)
                result.send(:initialize_copy, self)
                result
            end

            # Generates a regular sampling of the curve
            #
            # Returns an array of points sampled at a regular interval in t
            def sample(delta_t)
                result = []
                start_param.step(end_param, delta_t) do |t|
                    if t <= end_param
                        result << get(t)
                    else
                        result << get(end_param)
                        break
                    end
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
	    #
	    # types can be a Vector which has to bo of the same length than
	    # points, and provides the ability to parametrize the spline curve
	    # by giving values for tangents and/or derivatives
	    # The Vector needs to contain a list of the following symbols:
	    #
	    # :ORDINARY_POINT
	    # :KNUCKLE_POINT
	    # :DERIVATIVE_TO_NEXT
	    # :DERIVATIVE_TO_PRIOR
	    # :SECOND_DERIVATIVE_TO_NEXT
	    # :SECOND_DERIVATIVE_TO_PRIOR
	    # :TANGENT_POINT_FOR_NEXT
	    # :TANGENT_POINT_FOR_PRIOR
	    #
	    # By default types is nil, and all points are interpreted as
	    # :ORDINARY_POINT
	    #
            def interpolate(points, parameters = nil, types = nil)
                if points.empty?
                    clear
                    return
                end

		coordinates = points

		# flatten points if required
                if not points.first.kind_of?(Numeric)
                    coordinates = points.map(&:to_a).flatten!
                    if points.size * self.dimension != coordinates.size
                        raise ArgumentError, "point of wrong dimension given to #interpolate: got #{coordinates.size} coordinates overall, expected #{points.size * self.dimension}"
                    end
		end

		# check types argument size
		if types
                    if types.size != (coordinates.size / self.dimension)
                        raise ArgumentError, "if types are given, it needs to be of the same size as points. " +
                            "types.size = #{types.size}, points.size = #{coordinates.size / self.dimension}"
                    end
                    # convert the symbols to consts from the enum
                    types = types.map {|v| CoordinateType.const_get v} 
                else
                    types = [CoordinateType::ORDINARY_POINT] * (coordinates.size / self.dimension)
		end

		do_interpolate(coordinates, parameters || [], types || [])
            end

            def singleton(point)
                interpolate([point])
            end

            # Returns the point at the start of the curve
            def start_point; get(start_param) end
            # Returns the point at the end of the curve
            def end_point; get(end_param) end

            # Internal search method for +dichotomic_search+
            def do_dichotomic_search(start_t, start_p, end_t, end_p, resolution, block)
                if (start_p - end_p).norm < resolution
                    return (start_t + end_t) / 2
                end

                middle_t = (start_t + end_t) / 2
                middle_p = get(middle_t)

                block_result = block.call(middle_t, middle_p)
                if block_result.nil?
                    return middle_t
                elsif block_result
                    return do_dichotomic_search(middle_t, middle_p, end_t, end_p, resolution, block)
                else
                    return do_dichotomic_search(start_t, start_p, middle_t, middle_p, resolution, block)
                end
            end

            # Does a dochotomic search on the curve
            #
            # The given block is given a point and parameter on the curve and
            # must return
            # * true if the given point is before the point that is searched
            # * false if the given point is after the point that is searched
            # * nil if the point is an acceptable result for the search
            #
            # The resolution parameter gives the distance below which a segment
            # on the curve won't get refined further
            def dichotomic_search(resolution, start_t = nil, end_t = nil, &block)
                start_t ||= start_param
                end_t ||= end_param

                start_p = get(start_param)
                end_p   = get(end_param)
                return do_dichotomic_search(start_t, start_p, end_t, end_p, resolution, block)
            end

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
            
            def split(position)
                result = Spline.new(dimension, geometric_resolution, order)
                do_split(result, position)
                result
            end

            def _dump(level = -1)
                Marshal.dump([self.dimension,
                    self.geometric_resolution,
                    self.order,
                    self.coordinates,
                    self.knots,
                    self.sisl_curve_type])
            end

            def self._load(info, level = -1)
                dimension, resolution, order,
                    knots, coordinates, kind = Marshal.load(info)

                if dimension == 3
                    result = Spline3.new(resolution, order)
                else
                    result = Spline.new(dimension, resolution, order)
                end

                result.reset(coordinates, knots, kind)
                result
            end
	    
	    def pretty_print(pp)
		if(empty?)
		    pp.text "Curve is empty"
		    pp.breakable		
		else
		    pp.text "dimensions=[#{start_point}, #{end_point}]"
		    pp.breakable		
		    pp.text "length=[#{curve_length}]"
		    pp.breakable
		    pp.text "params=[#{start_param}, #{end_param}]"
		end
	    end
	    
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
            def self.interpolate(points, parameters = nil, types = nil)
                spline = Spline3.new
                spline.interpolate(points, parameters, types)
                spline
            end

            def self.singleton(point)
                interpolate([point])
            end

            def initialize(geometric_resolution = 0.1, order = 3)
                super(3, geometric_resolution, order)
            end

            # Returns a copy of this curve
            def dup
                result = self.class.new(geometric_resolution, order)
                result.initialize_copy(self)
                result
            end

            ##
            # call-seq:
            #   distance_to(reference_point, guess, geometric_resolution) => value
            #
            # Return sthe distance of the given point to the curve, i.e. the
            # distance between that point and the closest point of the curve
            def distance_to(reference_point, guess, geometric_resolution)
                closest = do_find_one_closest_point(reference_point.to_a, guess, geometric_resolution)
                (get(closest) - reference_point).norm
            end

            ##
            # call-seq:
            #   find_one_closest_point(reference_point, guess, geometric_resolution) => point
            #
            # Finds one point that is the closest to the curve, with a tolerance
            # of +geometric_resolution+
            #
            # The returned value is a parameter on the curve
            def find_one_closest_point(reference_point, guess, geometric_resolution)
                do_find_one_closest_point(reference_point.to_a, guess, geometric_resolution)
            end

            ##
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
            def interpolate(points, parameters = nil, types = nil)
                coordinates = []
                for p in points
                    coordinates << p.x << p.y << p.z
                end
                super(coordinates, parameters, types)
            end

            def singleton(point)
                interpolate([point])
            end

            # Returns the point at the given parameter.
            #
            # If +with_tangent+ is true, it returns a pair of Eigen::Vector3,
            # where the first one is the point and the second the tangent.
            def get(param, with_tangent = false)
		if(empty?)
		    return nil
		end
                result = super
                p = Eigen::Vector3.new(*result[0, 3])
                if with_tangent
                    t = Eigen::Vector3.new(*result[3, 3])
                    return p, t
                else
                    return p
                end
            end

            def split(position)
                result = Spline3.new(geometric_resolution, order)
                do_split(result, position)
                result
            end
        end
end

