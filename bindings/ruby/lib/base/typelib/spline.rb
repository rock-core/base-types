
##
# base/geometry/Spline to BaseTypes::Geometry::Spline convertions
require 'base/geometry/spline'
Typelib.convert_to_ruby '/wrappers/geometry/Spline', SISL::Spline do |value|
    if value.dimension == 3
        result = SISL::Spline3.new(value.geometric_resolution, value.curve_order)
    else
        result = SISL::Spline.new(value.dimension, value.geometric_resolution, value.curve_order)
    end

    kind_t = value.class['kind']
    arr1 = []
    arr2 = []
    value.vertices.each do |v|
	if v.kind_of?(Typelib::Type)
	    arr1 << v.to_ruby.to_f
	else
	    arr1 << v.to_f
        end
    end
    value.knots.each do |v|
	if v.kind_of?(Typelib::Type)
	    arr2 << v.to_ruby.to_f
	else
	    arr2 << v.to_f
        end
    end
    result.reset(arr1,arr2, kind_t.value_of(value.kind.to_s))
    result
end

module SISL
    def self.convert_spline_to_typelib(value, type) # :nodoc:
        result = type.new
        result.geometric_resolution = value.geometric_resolution
        result.curve_order = value.order
        result.dimension   = value.dimension
        result.kind  = value.sisl_curve_type
        result.knots = value.knots
        result.vertices = value.coordinates
        result
    end
end

Typelib.convert_from_ruby SISL::Spline3, '/wrappers/geometry/Spline', &SISL.method(:convert_spline_to_typelib)
Typelib.convert_from_ruby SISL::Spline, '/wrappers/geometry/Spline', &SISL.method(:convert_spline_to_typelib)

