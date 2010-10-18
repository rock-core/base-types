require 'rake'

RUBY = RbConfig::CONFIG['RUBY_INSTALL_NAME']
desc "build ruby Eigen extension"
task :setup do
     Dir.chdir("ext") do   
        if !system("#{RUBY} extconf.rb") || !system("make")
	    raise "cannot build the C extension"
	end
    end
    FileUtils.ln_sf "../ext/eigen_ext.so", "lib"
end
task :default => :setup

task :clean do
    FileUtils.rm_f File.join('ext', 'eigen_ext.so')
    FileUtils.rm_f File.join('lib', 'eigen_ext.so')
    FileUtils.rm_f File.join('ext', 'Eigen.o')
end
