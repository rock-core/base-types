# Loads mkmf which is used to make makefiles for Ruby extensions
require 'rubygems'
#workaround for require 'mkmf-rice'
gem 'rice', '>= 1.3.2'

require 'mkmf-rice'

# If you need some pkg-config dependencies, add them here
if !pkg_config('eigen3')
   STDERR.puts "could not find eigen3 pkg-config file, assuming eigen headers is in /usr/include/eigen3"
   $CXXFLAGS += " -I/usr/include/eigen3"
end

if !pkg_config('base-lib')
   STDERR.puts "could not find Rock's base-lib pkg-config file"
   exit(1)
end

create_makefile("base_types/base_types")#, "base")


