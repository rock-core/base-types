require 'minitest/autorun'
require 'orocos'

describe 'typelib conversion for SISL splines' do
    attr_reader :ruby_spline, :spline_t, :typelib_spline
    before do
        if !Orocos.loaded?
            Orocos.load
        end
        Orocos.load_typekit 'base'

        @spline_t = Orocos.registry.get('/wrappers/geometry/Spline')
        @ruby_spline = SISL::Spline.interpolate([[0,1], [2,3]])
        @typelib_spline = Typelib.from_ruby(ruby_spline, spline_t)
    end

    it "converts from the typelib type to the Ruby type" do
        typelib_spline.geometric_resolution = 2309
        spline = Typelib.to_ruby(typelib_spline)
        assert_kind_of SISL::Spline, spline
        assert_equal typelib_spline.kind.to_s, spline_t.field_types['kind'].name_of(spline.sisl_curve_type)
        assert_equal typelib_spline.dimension, spline.dimension
        assert_equal typelib_spline.geometric_resolution, spline.geometric_resolution
        assert_equal typelib_spline.knots.to_a, spline.knots
        assert_equal typelib_spline.vertices.to_a, spline.coordinates
        
    end
    it "converts from the Ruby type to the typelib type" do
        assert_equal spline_t.field_types['kind'].name_of(ruby_spline.sisl_curve_type), typelib_spline.kind.to_s
        assert_equal ruby_spline.dimension, typelib_spline.dimension
        assert_equal ruby_spline.geometric_resolution, typelib_spline.geometric_resolution
        assert_equal ruby_spline.knots, typelib_spline.knots.to_a
        assert_equal ruby_spline.coordinates, typelib_spline.vertices.to_a
    end
    it "creates a Spline3 object when converting a 3-dimensional spline" do
        ruby_spline = SISL::Spline.interpolate([[0,1,1], [2,3,3]])
        typelib_spline = Typelib.from_ruby(ruby_spline, spline_t)
        ruby_spline = Typelib.to_ruby(typelib_spline)
        assert_kind_of SISL::Spline3, ruby_spline
    end
end
