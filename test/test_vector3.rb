require 'test/unit'
require 'eigen'

class TC_Eigen_Vector3 < Test::Unit::TestCase
    def test_base
        v = Eigen::Vector3.new(1, 2, 3)
        assert_equal(1, v.x)
        assert_equal(2, v.y)
        assert_equal(3, v.z)
    end

    def test_add
        v0 = Eigen::Vector3.new(1, 2, 3)
        v1 = Eigen::Vector3.new(-1, -2, -3)
        assert_equal([0, 0, 0], (v0 + v1).to_a)
    end

    def test_sub
        v0 = Eigen::Vector3.new(1, 2, 3)
        v1 = Eigen::Vector3.new(-1, -2, -3)
        assert_equal([2, 4, 6], (v0 - v1).to_a)
    end

    def test_opposite
        v0 = Eigen::Vector3.new(1, 2, 3)
        assert_equal([-1, -2, -3], (-v0).to_a)
    end

    def test_scale
        v0 = Eigen::Vector3.new(1, 2, 3)
        assert_equal([2, 4, 6], (v0 * 2).to_a)
    end

    def test_dump_load
        v = Eigen::Vector3.new(0.2, 0.5, 0.1)
        dumped = Marshal.dump(v)
        loaded = Marshal.load(dumped)
        assert((v - loaded).norm < 0.0001)
    end
end

