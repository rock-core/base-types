require 'test/unit'
require 'eigen'

class TC_Eigen_Quaternion < Test::Unit::TestCase
    def test_base
        v = Eigen::Quaternion.new(0, 1, 2, 3)
        assert_equal(0, v.w)
        assert_equal(1, v.x)
        assert_equal(2, v.y)
        assert_equal(3, v.z)
    end

    def test_to_euler
        q = Eigen::Quaternion.new(1, 0, 0, 0)
        result = q.to_euler(2, 1, 0)
        assert_equal([0, 0, 0], result.to_a)
    end

    def test_to_euler_to_quaternion
        q = Eigen::Quaternion.new(0.2, 0.5, 0.1, 0.5)
        q.normalize!

        v = q.to_euler(2, 1, 0)
        result = Eigen::Quaternion.from_euler(v, 2, 1, 0)

        assert(q.approx?(result, 0.0001))
    end

    def test_dump_load
        q = Eigen::Quaternion.new(0.2, 0.5, 0.1, 0.5)
        dumped = Marshal.dump(q)
        loaded = Marshal.load(dumped)
        assert(q.approx?(loaded, 0.0001))
    end
end

