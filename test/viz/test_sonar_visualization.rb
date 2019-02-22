require_relative 'helpers'
require 'orocos'
Orocos.load_typekit 'base'

module Vizkit
    describe 'SonarVisualization' do
        include TestHelpers

        before do
            register_widget(@vizkit3d_widget = 
                Vizkit.default_loader.create_plugin("vizkit3d::Vizkit3DWidget"))
            @sonar_viz = @vizkit3d_widget.createPlugin('base', 'SonarVisualization')
            @vizkit3d_widget.show
        end

        after do 
        end

        def init_sonar_sample(beam_count, bin_count, step, initial_bearing = 0)
            sonar = Types.base.samples.Sonar.new
            sonar.speed_of_sound = 1000
            sonar.beam_count = beam_count
            sonar.bin_count = beam_count*bin_count
            sonar.beam_width = Types.base.Angle.new
            sonar.beam_width.rad = 0.05
            sonar.bin_duration = Time.at(10.0/(sonar.speed_of_sound*sonar.bin_count/beam_count))
            normalizer = 1.0/beam_count
            beam_count.times do |i|
                sonar.bins.concat(Array.new(bin_count, (i+1)*normalizer))
                bearing =  Types.base.Angle.new
                bearing.rad = initial_bearing + i*step
                sonar.bearings << bearing
            end
            sonar
        end

        it "it shows a single fan with zero bearing" do
            step = 0.03
            sonar = init_sonar_sample(1, 100, step)
            @sonar_viz.updateSonar(sonar)
            confirm 'A sonar reading should appear, point in x (red) direction'
        end
        
        it "it shows a single fan with 90 degrees bearing" do
            step = 0.03
            sonar = init_sonar_sample(1, 100, step)
            sonar.bearings[0].rad = 1.57
            @sonar_viz.updateSonar(sonar)
            confirm 'A sonar reading should appear, point in y (green) direction'
        end
        
        it "it shows a single fan being updated" do
            step = 0.03
            sonar = init_sonar_sample(1, 100, step)
            @sonar_viz.updateSonar(sonar)
            confirm 'A sonar reading should appear, point in x (red) direction'
            sonar.bearings[0].rad = 1.57/2
            @sonar_viz.updateSonar(sonar)
            confirm 'A sonar reading should appear, pointing 45 degrees anticlockwise, "\
                "that is between the red and green directions'
            sonar.bearings[0].rad = 1.57
            @sonar_viz.updateSonar(sonar)
            confirm 'A sonar reading should appear, pointing to the y (green) direction'
        end
        
        it "it shows a multibeam fan with zero bearing" do
            step = 0.03
            sonar = init_sonar_sample(30, 100, step, -30*step/2)
            @sonar_viz.updateSonar(sonar)
            confirm "A multibeam sonar reading should appear, centered in x (red) direction." \
            "The beam should have a gradient from black to red in the anticlockwise direction"
        end
    end
end

