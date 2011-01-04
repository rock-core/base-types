# If we get a /base/Time, convert it to Ruby's Time class
Typelib.convert_to_ruby '/base/Time', :if => lambda { |t| t.has_field?('seconds') } do |value|
    Time.at(value.seconds, value.microseconds)
end
Typelib.convert_to_ruby '/base/Time', :if => lambda { |t| !t.has_field?('seconds') } do |value|
    microseconds = value.microseconds
    seconds = microseconds / 1_000_000
    Time.at(seconds, microseconds % 1_000_000)
end
# Tell Typelib that Time instances can be converted into /base/Time values
Typelib.convert_from_ruby Time, '/base/Time', :if => lambda { |t| t.has_field?('seconds') } do |value, typelib_type|
    result = typelib_type.new
    result.seconds      = value.tv_sec
    result.microseconds = value.tv_usec
    result
end
Typelib.convert_from_ruby Time, '/base/Time', :if => lambda { |t| !t.has_field?('seconds') } do |value, typelib_type|
    result = typelib_type.new
    result.microseconds = value.tv_sec * 1_000_000 + value.tv_usec
    result
end

##
# base/geometry/Spline to BaseTypes::Geometry::Spline convertions
require 'base/geometry/spline'
Typelib.convert_to_ruby '/wrappers/geometry/Spline', :dynamic_wrappers => false do |value|
    result = BaseTypes::Geometry::Spline.new(value.dimension, value.geometric_resolution, value.curve_order)

    kind_t = value.class.kind
    result.reset(value.vertices.to_a, value.knots.to_a, kind_t.value_of(value.kind.to_s))
    result
end
Typelib.convert_from_ruby BaseTypes::Geometry::Spline, '/wrappers/geometry/Spline' do |value, type|
    result = type.new
    result.geometric_resolution = value.geometric_resolution
    result.curve_order = value.order
    result.dimension   = value.dimension
    result.kind  = value.sisl_curve_type
    result.knots = value.knots
    result.vertices = value.coordinates
    result
end

##
# Eigen convertions
begin
    require 'eigen'
    Typelib.convert_to_ruby '/wrappers/Vector3' do |value|
        Eigen::Vector3.new(*value.data.to_a)
    end
    Typelib.convert_from_ruby Eigen::Vector3, '/wrappers/Vector3' do |value, type|
        type.new(:data => value.to_a)
    end
    Typelib.convert_to_ruby '/wrappers/Quaternion' do |value|
        Eigen::Quaternion.new(value.re, *value.im.to_a)
    end
    Typelib.convert_from_ruby Eigen::Quaternion, '/wrappers/Quaternion' do |value, type|
        data = value.to_a
        type.new(:re => data[0], :im => data[1, 3])
    end
rescue LoadError
    STDERR.puts "The Ruby Eigen library is not present, I am not providing extensions for the base geometry types"
end


