# Loads mkmf which is used to make makefiles for Ruby extensions
require 'rubygems'
#workaround for require 'mkmf-rice'
gem 'rice', '>= 1.3.2'

require 'mkmf-rice'

# Give it a name
extension_name = 'eigen_ext'

# If you need some pkg-config dependencies, add them here
if !pkg_config('eigen2')
   STDERR.puts "could not find eigen2 pkg-config file, assuming eigen headers is in /usr/include/eigen2"
   $CXXFLAGS += " -I/usr/include/eigen2"
end

create_makefile(extension_name)


