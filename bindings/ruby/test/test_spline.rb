require 'minitest/autorun'
require 'sisl/spline'

class TC_Geometry_Spline < Minitest::Test
    def test_base
        v = SISL::Spline.new(3)
        assert_equal 3, v.dimension
        assert_equal 0.1, v.geometric_resolution
        assert_equal 3, v.order

        v.interpolate([0, 0, 0, 0.5, 0.5, 0.5, 1, 1, 1])
        assert_equal [0, 0, 0], v.get(v.start_param)
        assert_equal [1, 1, 1], v.get(v.end_param)
    end

    def test_deriv
        v = SISL::Spline.new(3)

        v.interpolate(
	    [[0, 0, 0], 
	    [0, 0, 0],
	    [0, 0, 0], 
	    [1, 0, 0], 
	    [1, 0, 0], 
	    [1, 1, 0],
	    [1, 1, 0]], [],
	    [:ORDINARY_POINT, 
	    :TANGENT_POINT_FOR_NEXT,
	    :TANGENT_POINT_FOR_PRIOR,
	    :ORDINARY_POINT,
	    :TANGENT_POINT_FOR_NEXT,
	    :TANGENT_POINT_FOR_PRIOR,
	    :ORDINARY_POINT] )

        assert_equal [0, 0, 0], v.get(v.start_param)
        assert_equal [1, 1, 0], v.get(v.end_param)
        assert_equal [0.75, 0, 0], v.get(0.5).map {|p| p.round(6) }
        assert_equal [1, 0.375, 0], v.get(1.5)
    end

    def test_concat
        v1 = SISL::Spline.new(3)
        v1.interpolate([0, 0, 0, 1, 2, 3])

        v2 = SISL::Spline.new(3)
        v2.interpolate([2, 2, 2, 4, 6, 8])

        result = v1.dup
        result.append(v2)
        assert_equal(v1.start_param, result.start_param)
        assert_equal(v2.end_param - v2.start_param + v1.end_param, result.end_param)
        assert_equal([0, 0, 0], result.start_point)
        assert_equal([3, 6, 9], result.end_point)
    end

    def test_join_singletons
        v1 = SISL::Spline.new(3)
        v1.interpolate([0, 0, 0])
        v2 = SISL::Spline.new(3)
        v2.interpolate([4, 4, 4])

        expected = SISL::Spline.new(3)
        expected.interpolate([0, 0, 0, 4, 4, 4])

        result = v1.dup
        result.join(v2, -1)
        assert_equal(expected.sample(0.1), result.sample(0.1))
    end

    def test_join_curve_and_singleton
        v1 = SISL::Spline.new(3)
        v1.interpolate([0, 0, 0])
        v2 = SISL::Spline.new(3)
        v2.interpolate([4, 4, 4, 6, 6, 6])

        result = v1.dup
        result.join(v2, -1)
        # TODO: find something to actually test ...
    end

    def test_join_singleton_and_curve
        v1 = SISL::Spline.new(3)
        v1.interpolate([0, 0, 0, 4, 4, 4])
        v2 = SISL::Spline.new(3)
        v2.interpolate([6, 6, 6])

        result = v1.dup
        result.join(v2, -1)
        # TODO: find something to actually test ...
    end

    def test_join
        v1 = SISL::Spline.new(3)
        v1.interpolate([0, 0, 0, 1, 2, 3])

        v2 = SISL::Spline.new(3)
        v2.interpolate([4, 4, 4, 4, 6, 8])

        result = v1.dup
        result.join(v2, -1)
        assert_equal(v1.start_param, result.start_param)
        assert_equal([0, 0, 0], result.start_point)
        assert_equal([4, 6, 8], result.end_point)
    end

    def test_distance_to
        v1 = SISL::Spline3.new
        v1.interpolate([Eigen::Vector3.new(0, 0, 0), Eigen::Vector3.new(1, 1, 1)])

        assert(v1.distance_to(Eigen::Vector3.new(0.5, 0.5, 0.5), 0, 1e-9) < 1e-9)
    end

    def test_find_one_closest_point
        v1 = SISL::Spline3.new
        v1.interpolate([Eigen::Vector3.new(0, 0, 0), Eigen::Vector3.new(1, 1, 1)])

        test_p = Eigen::Vector3.new(0.5, 0.5, 0.5)
        t = v1.find_one_closest_point(test_p, 0, 1e-9)
        assert v1.get(t).approx?(test_p)
    end

    def test_it_is_aliased_to_base_geometry_spline_for_backward_compatibility
        require 'base/geometry/spline'
        assert_same SISL::Spline, Types::Base::Geometry::Spline
        assert_same SISL::Spline3, Types::Base::Geometry::Spline3
    end
end


