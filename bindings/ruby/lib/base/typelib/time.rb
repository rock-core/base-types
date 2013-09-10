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

