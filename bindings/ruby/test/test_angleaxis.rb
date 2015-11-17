require 'minitest/autorun'
require 'eigen'

class TC_Eigen_AngleAxis < Minitest::Test
    def test_base
        aa = Eigen::AngleAxis.new(0, Eigen::Vector3.new(1, 0, 0))
        assert_equal(0, aa.angle)
        assert_equal(1, aa.axis.x)
        assert_equal(0, aa.axis.y)
        assert_equal(0, aa.axis.z)
    end

    def test_to_euler
        aa = Eigen::AngleAxis.new(1, Eigen::Vector3.new(1, 0, 0))
        result = aa.to_euler
        assert_equal([0, 0, 1], result.to_a)
    end

    def test_to_euler_to_quaternion
        aa = Eigen::AngleAxis.new(0.5, Eigen::Vector3.new(1, 0, 0))

        v = aa.to_euler
        result = Eigen::AngleAxis.from_euler(v, 2, 1, 0)

        assert(aa.approx?(result, 0.0001))
    end

    def test_approx_returns_true_on_equality
        aa = Eigen::AngleAxis.new(0, Eigen::Vector3.new(1, 0, 0))
        assert aa.approx?(aa)
    end

    def test_approx_returns_true_on_angleaxis_that_are_different_by_epsilon
        aa1 = Eigen::AngleAxis.new(0, Eigen::Vector3.new(1, 0, 0))
        aa2 = Eigen::AngleAxis.new(0,  Eigen::Vector3.new(1 + Float::EPSILON, 0 + Float::EPSILON, 0 + Float::EPSILON))
        assert aa1.approx?(aa2)
    end

    def test_approx_returns_false_on_angleaxis_that_are_different
        q1 = Eigen::AngleAxis.new(1, Eigen::Vector3.new(1, 1, 1))
        q2 = Eigen::AngleAxis.new(2, Eigen::Vector3.new(2, 2, 2))
        refute q1.approx?(q2)
    end

    def test_approx_returns_true_on_angleaxis_that_are_less_different_than_the_proqided_accuracy
        q1 = Eigen::AngleAxis.new(1, Eigen::Vector3.new(1, 1, 1))
        q2 = Eigen::AngleAxis.new(1.5, Eigen::Vector3.new(1.5, 1.5, 1.5))
        assert q1.approx?(q2, 2)
    end

    def test_quaternion_
        aa = Eigen::AngleAxis.from_quaternion( Eigen::Quaternion.new(0, 1, 0, 0))
        v = Eigen::Vector3.new(0, 1, 0)

        assert( Eigen::Vector3.new(0, -1, 0).approx?( aa*v, 0.0001 ) )
    end

    def test_inverse
        aa1 = Eigen::AngleAxis.from_euler( Eigen::Vector3.new(1, 0, 0), 2, 1, 0)
        aa2 = Eigen::AngleAxis.from_euler( Eigen::Vector3.new(-1, 0, 0), 2, 1, 0)

        assert(aa1.to_euler.approx?( aa2.inverse.to_euler, 0.0001 ))
    end

    def test_dump_load
        aa = Eigen::AngleAxis.new(0, Eigen::Vector3.new(1, 0, 0))
        dumped = Marshal.dump(aa)
        loaded = Marshal.load(dumped)
        assert(aa.approx?(loaded, 0.0001))
    end

    def test_dup
        aa = Eigen::AngleAxis.new(0, Eigen::Vector3.new(0, 0, 0))
        assert(aa.dup.approx?(aa, 0.0001))
    end
end

