require 'sisl/spline'
require 'base/typelib/joint_state'
require 'base/typelib/time'
require 'base/typelib/rigid_body_state'
require 'base/typelib/samples/body_state'
require 'base/typelib/spline'
require 'base/typelib/named_vector'
require 'base/typelib/samples/joints'
require 'base/typelib/transform_with_covariance'

##
# Eigen convertions
begin
    require 'eigen'
    require 'base/typelib/eigen'

rescue LoadError
    STDERR.puts "The Ruby Eigen library is not present, I am not providing extensions for the base geometry types"
end

