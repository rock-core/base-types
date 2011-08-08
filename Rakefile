require 'rake'

RUBY = RbConfig::CONFIG['RUBY_INSTALL_NAME']
desc "build ruby Eigen extension"
task :setup do
     Dir.chdir("ext") do   
        if !system("#{RUBY} extconf.rb")
	    raise "cannot configure the C extension"
	end
        system("make", "clean")
        if !system("make")
	    raise "cannot build the C extension"
	end
    end
    FileUtils.ln_sf "../ext/base_types_ext.so", "lib"
end
task :default => :setup

task :clean do
    FileUtils.rm_f File.join('ext', 'base_types_ext.so')
    FileUtils.rm_f File.join('lib', 'base_types_ext.so')
    FileUtils.rm_f File.join('ext', 'Eigen.o')
end

require 'rdoc/task'
RDoc::Task.new("docs") do |rdoc|
    rdoc.rdoc_dir = 'doc'
    rdoc.title    = "REigen"
    rdoc.options << '--show-hash'
    rdoc.rdoc_files.include('lib/**/*.rb', 'ext/**/*.cc')
end
