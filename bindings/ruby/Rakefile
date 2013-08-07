task :default

package_name = 'base_types'
begin
    require 'hoe'
    Hoe::plugin :compiler
    Hoe::plugin :yard

    hoe_spec = Hoe.spec package_name do
        self.version = '0.1'
        self.developer "Sylvain Joyeux", "sylvain.joyeux@m4x.org"
        self.extra_deps <<
            ['rake', '>= 0.8.0'] <<
            ["hoe",     ">= 3.0.0"] <<
            ["hoe-yard",     ">= 0.1.2"] <<
            ["rake-compiler",     ">= 0.8.0"]

        self.summary = 'Ruby-side code to handle some common complex types'
        self.readme_file = FileList['README*'].first
        self.description = paragraphs_of(readme_file, 3..5).join("\n\n")

        self.spec_extras = {
            :required_ruby_version => '>= 1.8.7'
        }
    end

    hoe_spec.spec.extensions = FileList["ext/**/extconf.rb"]

    Rake.clear_tasks(/^default$/)
    task :default => :compile
    task :doc => :yard

    begin
        require 'rdoc/task'
        RDoc::Task.new("docs") do |rdoc|
            rdoc.rdoc_dir = 'doc'
            rdoc.title    = "REigen"
            rdoc.options << '--show-hash'
            rdoc.rdoc_files.include('lib/**/*.rb', 'ext/**/*.cc')
        end
    rescue LoadError
        STDERR.puts "INFO: documentation targets are disabled as the rdoc gem is not installed"
    end

rescue LoadError => e
    puts "Extension for '#{package_name}' cannot be build -- loading gem failed: #{e}"
end
