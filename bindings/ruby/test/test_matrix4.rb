require 'minitest/autorun'
require 'eigen'

class TC_Eigen_Matrix4 < Minitest::Test
    def test_new_zero
        m = Eigen::Matrix4.Zero
        4.times do |i|
            4.times do |j|
                assert_equal 0, m[i, j]
            end
        end
    end

    def test_zero
        m = Eigen::Matrix4.new
        m.zero
        4.times do |i|
            4.times do |j|
                assert_equal 0, m[i, j]
            end
        end
    end

    def test_new_identity
        m = Eigen::Matrix4.Identity
        4.times do |i|
            4.times do |j|
                assert_equal((i == j ? 1 : 0), m[i, j])
            end
        end
    end

    def test_identity
        m = Eigen::Matrix4.new
        m.identity
        4.times do |i|
            4.times do |j|
                assert_equal((i == j ? 1 : 0), m[i, j])
            end
        end
    end
end
