require_relative 'helpers'
require 'orocos'
Orocos.load_typekit 'base'

module Vizkit
    describe 'TrajectoryVisualization' do
        include TestHelpers

        before do
            register_widget(@vizkit3d_widget =
                Vizkit.default_loader.create_plugin("vizkit3d::Vizkit3DWidget"))
            @viz = @vizkit3d_widget.createPlugin('base', 'TrajectoryVisualization')

            @spline = SISL::Spline3.interpolate(
                (0..10).map { |i| Eigen::Vector3.new(i, 0, 0.1) },
                (0..10).step(0.05).to_a)
            @viz.setMaxNumberOfPoints(10000)

            @vizkit3d_widget.setCameraEye(5, 0, 20)
            @vizkit3d_widget.setCameraLookAt(5, 0, 0)
            @vizkit3d_widget.show
        end

        describe "updateSpline(Spline)" do
            it "enforces the max points setting" do
                @viz.setMaxNumberOfPoints(100);
                @viz.updateSpline(@spline)
                confirm 'The spline should be between 5 and 10'
            end

            it "disables max points if parameter set to zero" do
                @viz.setMaxNumberOfPoints(0);
                @viz.updateSpline(@spline)
                confirm 'The spline should be between 0 and 10'
            end

            it "allows to change the color" do
                @viz.setColor(Eigen::Vector3.new(0.2, 1, 0.2));
                @viz.updateSpline(@spline)
                confirm 'The spline should be green'
            end
        end

        describe "#clear" do
            it "clears the trajectory" do
                @viz.addSpline(@spline)
                confirm 'There should be a spline'
                @viz.clear
                confirm 'The spline has disappeared'
            end
        end

        describe "addSpline" do
            it "does not clear the previous spline" do
                @viz.addSpline(@spline)
                spline = SISL::Spline3.interpolate(
                    (0..10).map { |i| Eigen::Vector3.new(10, i * 0.2, 0.1) },
                    (0..10).step(0.05).to_a)
                @viz.addSpline(spline)
                confirm "A L shape should be displayed with the corner \n"\
                    "at (10, 0) and the ends at (0, 0) and (10, 2)"
            end

            it "does not enforce the max points setting by default" do
                @viz.setMaxNumberOfPoints(100);
                @viz.addSpline(@spline)
                confirm 'The spline should be between 0 and 10'
            end

            it "does enforce the max points setting if showAll is false" do
                @viz.setMaxNumberOfPoints(100);
                @viz.addSpline(@spline, 0, false);
                confirm 'The spline should be between 5 and 10'
            end

            it "customizes the stepSize" do
                @viz.setMaxNumberOfPoints(5);
                @viz.addSpline(@spline, 0.1);
                confirm 'The spline should be between 0 and 10'
            end
        end
    end
end

