# Loads mkmf which is used to make makefiles for Ruby extensions
require 'rubygems'
#workaround for require 'mkmf-rice'
gem 'rice', '~> 1.3.2'

require 'mkmf-rice'

# Give it a name
extension_name = 'eigen_ext'

# If you need some pkg-config dependencies, add them here
pkg_config('eigen2')

create_makefile(extension_name)


