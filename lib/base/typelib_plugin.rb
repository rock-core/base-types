require 'base/geometry/spline'

# If we get a /base/Time, convert it to Ruby's Time class
Typelib.convert_to_ruby '/base/Time', Time, :if => lambda { |t| t.has_field?('seconds') } do |value|
    Time.at(value.seconds, value.microseconds)
end
Typelib.convert_to_ruby '/base/Time', Time, :if => lambda { |t| !t.has_field?('seconds') } do |value|
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

Typelib.specialize_model '/base/samples/RigidBodyState_m' do
    def invalid
        v3 = { :data => [NaN] * 3 }
        m3 = { :data => [NaN] * 9 }
        new(
            :time => Time.at(0),
            :position => v3,
            :cov_position => m3,
            :orientation => { :im => [NaN, NaN, NaN], :re => NaN },
            :cov_orientation => m3,
            :velocity => v3,
            :cov_velocity => m3,
            :angular_velocity => v3,
            :cov_angular_velocity => m3)
    end
end

##
# base/geometry/Spline to BaseTypes::Geometry::Spline convertions
require 'base/geometry/spline'
Typelib.convert_to_ruby '/wrappers/geometry/Spline', Types::Base::Geometry::Spline do |value|
    if value.dimension == 3
        result = Types::Base::Geometry::Spline3.new(value.geometric_resolution, value.curve_order)
    else
        result = Types::Base::Geometry::Spline.new(value.dimension, value.geometric_resolution, value.curve_order)
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

##
# Eigen convertions
begin
    require 'eigen'
    Typelib.convert_to_ruby '/wrappers/Matrix</double,3,1>', Eigen::Vector3 do |value|
        Eigen::Vector3.new(*value.data.to_a)
    end
    Typelib.convert_from_ruby Eigen::Vector3, '/wrappers/Matrix</double,3,1>' do |value, type|
        t = type.new
        t.data = value.to_a
        t
    end
    Typelib.convert_to_ruby '/wrappers/Quaternion</double>', Eigen::Quaternion do |value|
        Eigen::Quaternion.new(value.re, *value.im.to_a)
    end
    Typelib.convert_from_ruby Eigen::Quaternion, '/wrappers/Quaternion</double>' do |value, type|
        data = value.to_a
        t = type.new
        t.re = data[0]
        t.im = data[1, 3]
        t
    end

    Typelib.convert_to_ruby '/wrappers/MatrixX</double>', Eigen::MatrixX do |value|
        m = Eigen::MatrixX.new(value.rows,value.cols)
        m.from_a(value.data.to_a,value.rows,value.cols)
        m
    end
    Typelib.convert_from_ruby Eigen::MatrixX, '/wrappers/MatrixX</double>' do |value, type|
        t = type.new
        t.rows = value.rows
        t.cols = value.cols
        t.data = value.to_a
        t
    end
    
    Typelib.convert_to_ruby '/wrappers/VectorX</double>', Eigen::VectorX do |value|
        m = Eigen::VectorX.new(value.data.size)
        m.from_a(value.data.to_a)
        m
    end
    Typelib.convert_from_ruby Eigen::VectorX, '/wrappers/VectorX</double>' do |value, type|
        t = type.new
        value.to_a.each {|v| t.data.push(v) }
        t
    end

rescue LoadError
    STDERR.puts "The Ruby Eigen library is not present, I am not providing extensions for the base geometry types"
end


