
##
# base/geometry/Spline to BaseTypes::Geometry::Spline convertions
require 'base/geometry/spline'
Typelib.convert_to_ruby '/wrappers/geometry/Spline', Types::Base::Geometry::Spline do |value|
    if value.dimension == 3
        result = Types::Base::Geometry::Spline3.new(value.geometric_resolution, value.curve_order)
    else
        begin
            result = Types::Base::Geometry::Spline.new(value.dimension, value.geometric_resolution, value.curve_order)  
        rescue Exception => e
            STDERR.puts "Eror during spline generation: #{e}, returning empty one instead"
            result = Types::Base::Geometry::Spline.new(1,1,1)
        end
    end

    kind_t = value.class['kind']
    result.reset(value.vertices.to_a, value.knots.to_a, kind_t.value_of(value.kind.to_s))
    result
end

def convert_spline_to_typelib(value, type) # :nodoc:
    result = type.new
    result.geometric_resolution = value.geometric_resolution
    result.curve_order = value.order
    result.dimension   = value.dimension
    result.kind  = value.sisl_curve_type
    result.knots = value.knots
    result.vertices = value.coordinates
    result
end

Typelib.convert_from_ruby Types::Base::Geometry::Spline3, '/wrappers/geometry/Spline', &method(:convert_spline_to_typelib)
Typelib.convert_from_ruby Types::Base::Geometry::Spline, '/wrappers/geometry/Spline', &method(:convert_spline_to_typelib)

