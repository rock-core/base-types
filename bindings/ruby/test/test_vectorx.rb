# frozen_string_literal: true

require 'minitest/autorun'
require 'eigen'

class TC_Eigen_VectorX < Minitest::Test
    def test_new
        v = Eigen::VectorX.new(10)
        assert_equal(v.size, 10)
    end

    def test_from_a
        v = Eigen::VectorX.new(4)
        v.from_a([-3.1,-4.2, 5.3, 6.4])
        assert_equal([-3.1,-4.2, 5.3, 6.4], v.to_a)
    end

    def test_new_zero
        v = Eigen::VectorX.Zero(4)
        assert_equal([0, 0, 0, 0], v.to_a)
    end

    def test_zero
        v = Eigen::VectorX.new(4)
        v.from_a([-3.1,-4.2, 5.3, 6.4])
        v.zero
        assert_equal([0, 0, 0, 0], v.to_a)
    end
end
