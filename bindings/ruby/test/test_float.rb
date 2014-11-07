require 'minitest/autorun'
require 'base/float'

class TC_Float < Minitest::Test
    def test_special_values
        assert Base.infinity?(Infinity)
        assert Base.nan?(NaN)

        assert Base.infinity?(Base.infinity)
        assert Base.nan?(Base.nan)
        assert Base.nan?(Base.unset)
        assert Base.nan?(Base.unknown)

        assert Base.unset?(Base.unset)
        assert Base.unknown?(Base.unknown)
    end
end
