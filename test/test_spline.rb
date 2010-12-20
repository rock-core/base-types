$LOAD_PATH.unshift File.expand_path("../lib", File.dirname(__FILE__))

require 'test/unit'
require 'base/geometry/spline'

class TC_Geometry_Spline < Test::Unit::TestCase
    def test_base
        v = Base::Geometry::Spline.new(3)
        assert_equal 3, v.dimension
        assert_equal 0.1, v.geometric_resolution
        assert_equal 3, v.order

        v.interpolate([0, 0, 0, 0.5, 0.5, 0.5, 1, 1, 1])
        assert_equal [0, 0, 0], v.get(v.start_param)
        assert_equal [1, 1, 1], v.get(v.end_param)
    end

    def test_concat
        v1 = Base::Geometry::Spline.new(3)
        v1.interpolate([0, 0, 0, 1, 2, 3])

        v2 = Base::Geometry::Spline.new(3)
        v2.interpolate([2, 2, 2, 4, 6, 8])

        result = v1.dup
        result.append(v2)
        assert_equal(v1.start_param, result.start_param)
        assert_equal(v2.end_param - v2.start_param + v1.end_param, result.end_param)
        assert_equal([0, 0, 0], result.start_point)
        assert_equal([3, 6, 9], result.end_point)
    end

    def test_join
        v1 = Base::Geometry::Spline.new(3)
        v1.interpolate([0, 0, 0, 1, 2, 3])

        v2 = Base::Geometry::Spline.new(3)
        v2.interpolate([4, 4, 4, 4, 6, 8])

        result = v1.dup
        result.join(v2, -1)
        assert_equal(v1.start_param, result.start_param)
        assert_equal([0, 0, 0], result.start_point)
        assert_equal([4, 6, 8], result.end_point)
    end
end


