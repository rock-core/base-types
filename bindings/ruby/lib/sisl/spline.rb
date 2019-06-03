require 'base_types_ruby'
require 'eigen'
require 'set'

# Bindings for the SISL spline library
module SISL
    # Representation and manipulation of N-dimensional B-splines
    class Spline
        # Returns a Spline object that interpolates the given points
        #
        # See {#interpolate} for more information. Note that the 'dimension' argument
        # must be provided in case 'points' is a flat array of coordinates or if
        # using the fluid interface (i.e. not providing any points at all)
        def self.interpolate(points = nil, parameters = nil, types = nil, dimension: nil)
            if !dimension && points.first.kind_of?(Numeric)
                raise ArgumentError, 'cannot guess the curve dimensions "\
                    "from a flat point array'
            end
            dimension ||= points.first.to_a.size

            spline = Spline.new(dimension)
            if points
                spline.interpolate(points, parameters, types)
                spline
            else
                Interpolator.new(spline)
            end
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

        # Implementation of the fluid interface to {#interpolate}
        #
        # @example provide parameters and ordinary points
        #   spline.interpolate
        #       .at(1).point(10)
        #       .at(2).point(20)
        #       .at(3).point(30)
        #       .to_spline
        #
        # @example provide derivative. Derivatives apply to the previous point.
        #   spline.interpolate
        #       .at(1).point(10).derivative(0)
        #       .at(2).point(20).derivative(1)
        #       .at(3).point(30).derivative(0)
        #       .to_spline
        #
        class Interpolator
            def initialize(spline)
                @spline = spline
                @dimension = spline.dimension
                @points = []
                @parameters = []
                @types = []

                @current_point = []
                @current_types = []
                @current_parameter = nil
            end

            # Registers a parameter as a relative offset from the previous parameter
            #
            # Once a parameter is given, it has to be given right before all
            # points
            #
            # For flexibility reasons, it may be used even for the very first point,
            # in which case it is equivalent to calling {#at}
            def at_relative(parameter)
                register_current
                if @parameters.empty?
                    at(parameter)
                else
                    at(parameter + @parameters.last)
                end
            end

            # Registers a parameter
            #
            # Once a parameter is given, it has to be given right before all
            # points
            def at(parameter)
                register_current

                if @current_parameter || (@parameters.empty? && !@points.empty?)
                    raise ArgumentError, 'when used, at() must be called exactly once '\
                        'before each point'
                end

                @current_parameter = parameter
                self
            end

            private def validate_parameter_given_if_needed
                return if @parameters.empty? || @current_parameter

                raise ArgumentError, 'when used, at() must be called exactly once '\
                    'before each point'
            end

            # Registers an ordinary point
            def point(*values)
                register_current
                validate_parameter_given_if_needed
                add_to_point(values, type: :ORDINARY_POINT)
                self
            end

            # Registers an ordinary point
            def knuckle_point(*values)
                register_current
                validate_parameter_given_if_needed
                add_to_point(values, type: :KNUCKLE_POINT)
                self
            end

            # @api private
            #
            # Register a points and its type
            def add_to_point(values, type:)
                if values.size != @dimension
                    raise ArgumentError, "expected a point of dimension #{values.size} "\
                        "but got #{values.size}"
                elsif @current_point.empty? && !POINT_TYPES.include?(type)
                    raise ArgumentError, 'must call #ordinary_point or #knuckle_point '\
                        "before defining #{type}"
                end

                type = type.to_sym
                if !VALID_TYPES.include?(type)
                    raise ArgumentError, "#{type} is not a known point type"
                elsif @current_types.include?(type)
                    raise ArgumentError, "#{type} already given for this point"
                end

                @current_point.concat(values)
                @current_types << type
            end

            private def register_current
                return if @current_types.empty?

                @points.concat(@current_point)
                @types.concat(@current_types)
                @parameters << @current_parameter if @current_parameter

                @current_point.clear
                @current_parameter = nil
                @current_types.clear
            end

            def derivative(*values)
                if @current_point.empty?
                    raise ArgumentError, 'must define a point before calling #derivative'
                end

                add_to_point(values, type: 'DERIVATIVE_TO_PRIOR')
                self
            end

            def second_derivative(*values)
                if @current_point.empty?
                    raise ArgumentError, 'must define a point before calling '\
                        '#second_derivative'
                end

                add_to_point(values, type: 'SECOND_DERIVATIVE_TO_PRIOR')
                self
            end

            def to_spline
                register_current
                @spline.interpolate(@points, @parameters, @types)
                @spline
            end
        end

        VALID_TYPES = %I[
            ORDINARY_POINT
            KNUCKLE_POINT
            DERIVATIVE_TO_NEXT
            DERIVATIVE_TO_PRIOR
            SECOND_DERIVATIVE_TO_NEXT
            SECOND_DERIVATIVE_TO_PRIOR
        ].to_set.freeze

        POINT_TYPES = %I[ORDINARY_POINT KNUCKLE_POINT].to_set.freeze

        # Resets this curve so that it is an interpolation of the given set
        # of points
        #
        # @overload interpolate
        #   @return Interpolator
        #
        #   A fluid interface to build a spline for interpolation. See the documentation
        #   of {Interpolator} for more information
        #
        # @overload interpolate(points, parameters = nil, types = nil)
        #   @param [Array] points either a list of numbers multiple of the spline
        #      dimension, or an array-of-arrays, with the nested arrays of size
        #      {#dimension}
        #   @param [Array,nil] parameters if non-nil, an array of the same size than the
        #      number of points. This array represents the desired parameters of
        #      each of the points in the resulting spline
        #   @param [Array,nil] types if non-nil, each entry in this array characterizes
        #      the entries in the 'points' array, allowing to specify first order and
        #      second order derivatives. Pass values in {VALID_TYPES}.
        #   @return [void]
        #
        def interpolate(points = nil, parameters = nil, types = nil)
            return Interpolator.new(self) unless points

            if points.empty?
                clear
                return
            end

            coordinates = points

            # flatten points if required
            unless points.first.kind_of?(Numeric)
                coordinates = points.map(&:to_a).flatten!
                if points.size * dimension != coordinates.size
                    raise ArgumentError, 'point of wrong dimension given to '\
                        "#interpolate: got #{coordinates.size} coordinates "\
                        "overall, expected #{points.size * dimension}"
                end
            end

            # check types argument size
            if types
                if types.size != (coordinates.size / dimension)
                    raise ArgumentError, 'if types are given, it needs to be '\
                        "of the same size as points. types.size = #{types.size}, "\
                        "points.size = #{coordinates.size / dimension}"
                end
                # convert the symbols to consts from the enum
                types = types.map { |v| CoordinateType.const_get(v) }
            else
                types = [CoordinateType::ORDINARY_POINT] *
                        (coordinates.size / dimension)
            end

            do_interpolate(coordinates, parameters || [], types || [])
        end

        def singleton(point)
            interpolate([point])
        end

        # Returns the point at the start of the curve
        def start_point
            get(start_param)
        end

        # Returns the point at the end of the curve
        def end_point
            get(end_param)
        end

        # Internal search method for +dichotomic_search+
        def do_dichotomic_search(start_t, start_p, end_t, end_p, resolution, block)
            return (start_t + end_t) / 2 if (start_p - end_p).norm < resolution

            middle_t = (start_t + end_t) / 2
            middle_p = get(middle_t)

            block_result = block.call(middle_t, middle_p)
            if block_result.nil?
                middle_t
            elsif block_result
                do_dichotomic_search(middle_t, middle_p, end_t, end_p,
                    resolution, block)
            else
                do_dichotomic_search(start_t, start_p, middle_t, middle_p,
                    resolution, block)
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
            do_dichotomic_search(start_t, start_p, end_t, end_p, resolution, block)
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

        def _dump(_level = -1)
            Marshal.dump([
                dimension,
                geometric_resolution,
                order,
                coordinates,
                knots,
                sisl_curve_type
            ])
        end

        def self._load(info, _level = -1)
            dimension, resolution, order,
                knots, coordinates, kind = Marshal.load(info)

            result =
                if dimension == 3
                    Spline3.new(resolution, order)
                else
                    Spline.new(dimension, resolution, order)
                end

            result.reset(coordinates, knots, kind)
            result
        end

        def pretty_print(pp)
            if empty?
                pp.text 'Curve is empty'
                pp.breakable
            else
                pp.text "start_point,end_point=[#{start_point}, #{end_point}]"
                pp.breakable
                pp.text "start_param,end_param=[#{start_param}, #{end_param}]"
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
            closest = do_find_one_closest_point(reference_point.to_a,
                guess, geometric_resolution)
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
        #   find_closest_points(reference_point, geometric_resolution, guess = 0)
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
            do_length(start_t, end_t, geores)
        end

        # Resets this curve to a curve interpolating the given points
        #
        # +points+ is an array of Eigen::Vector3 instances. See
        # Spline#interpolate for details on +parameters+
        def interpolate(points, parameters = nil, types = nil)
            coordinates = []
            points.each do |p|
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
            return if empty?

            result = super
            p = Eigen::Vector3.new(*result[0, 3])
            if with_tangent
                t = Eigen::Vector3.new(*result[3, 3])
                [p, t]
            else
                p
            end
        end

        def split(position)
            result = Spline3.new(geometric_resolution, order)
            do_split(result, position)
            result
        end
    end
end
