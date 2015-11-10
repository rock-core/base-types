puts "you cannot require 'base/geometry/spline' anymore"
puts "It has been replaced by require 'sisl/spline' and"
puts "  Base::Geometry::Spline is now SISL::Spline"
puts "  Base::Geometry::Spline3 is now SISL::Spline3"
raise LoadError, "use sisl/spline"

