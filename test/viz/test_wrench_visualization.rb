require_relative 'helpers'
require 'orocos'
Orocos.load_typekit 'base'

module Vizkit
    describe 'WrenchVisualization' do
        include TestHelpers

        before do
            register_widget(@vizkit3d_widget =
                Vizkit.default_loader.create_plugin("vizkit3d::Vizkit3DWidget"))
            @viz = @vizkit3d_widget.createPlugin('base', 'WrenchVisualization')

            @wrench = Types.base.samples.Wrench.new
            @wrench.frame_id = "test_frame"
            @wrench.force = Types.base.Vector3d.new(1,1,1)
            @wrench.torque = Types.base.Vector3d.new(-1,-1,1)
            @viz.updateData(@wrench)

            @viz.setResolution(1.0)

            @vizkit3d_widget.setCameraEye(5, 4, 5)
            @vizkit3d_widget.setCameraLookAt(0, 0, 0)
            @vizkit3d_widget.show
        end

        # describe "updateData(Wrench)" do
        #     @wrench = Types.base.samples.Wrench.new
        #     @wrench.frame_id = "test_frame"
        #     @wrench.force = Types.base.Vector3d.new(1,1,1)
        #     @wrench.torque = Types.base.Vector3d.new(-1,-1,1)

        #     it "updates wrench" do
        #         @viz.updateData(@wrench)
        #         confirm 'Force should be displayed as a red arrow from (0,0,0) to (1,1,1) and torque as a green arrow from (0,0,0) to (-1,-1,1) with a circular arrow rotating counter-clockwise around axis'
        #     end

        #     it "updates wrench" do
        #         @wrench.force = Types.base.Vector3d.new(1,0,0)
        #         @wrench.torque = Types.base.Vector3d.new(0,1,0)
        #         @viz.updateData(@wrench)
        #         confirm 'Force should be displayed as a red arrow from (0,0,0) to (1,0,0) and torque as a green arrow from (0,0,0) to (0,1,0) with a circular arrow rotating counter-clockwise around axis'
        #     end

        #     it "updates wrench" do
        #         @wrench.force = Types.base.Vector3d.new(0,1,0)
        #         @wrench.torque = Types.base.Vector3d.new(0,0,1)
        #         @viz.updateData(@wrench)
        #         confirm 'Force should be displayed as a red arrow from (0,0,0) to (0,1,0) and torque as a green arrow from (0,0,0) to (0,0,1) with a circular arrow rotating counter-clockwise around axis'
        #     end

        #     it "updates wrench" do
        #         @wrench.force = Types.base.Vector3d.new(0,0,1)
        #         @wrench.torque = Types.base.Vector3d.new(1,0,0)
        #         @viz.updateData(@wrench)
        #         confirm 'Force should be displayed as a red arrow from (0,0,0) to (0,0,1) and torque as a green arrow from (0,0,0) to (1,0,0) with a circular arrow rotating counter-clockwise around axis'
        #     end

        #     it "updates wrench" do
        #         @wrench.force = Types.base.Vector3d.new(0,0,0)
        #         @wrench.torque = Types.base.Vector3d.new(0,0,0)
        #         @viz.updateData(@wrench)
        #         confirm 'Nothing should be displayed'
        #     end

        #     it "updates wrench" do
        #         @wrench.force = Types.base.Vector3d.new(0.1,0,0)
        #         @wrench.torque = Types.base.Vector3d.new(0.3,0,0)
        #         @viz.updateData(@wrench)
        #         confirm 'Force should be displayed as a red arrow from (0,0,0) to (0,0,0.1) and torque as a green arrow from (0,0,0) to (0.3,0,0) with a circular arrow rotating counter-clockwise around axis. Both arrows should be displayed without the arrow head'
        #     end
        # end

        describe "setSeperateAxesTorque" do
            it "displays torque axis component" do
                torque = Types.base.Vector3d.new(1,1,1)
                @viz.setTorque(torque)
                @viz.setSeperateAxesTorque(true)
                confirm 'Torque should be displayed as three arrows along axes each with length 1 and a circular arrow rotating counter-clockwise'
                
            end

            it "displays torque" do
                torque = Types.base.Vector3d.new(1,1,1)
                @viz.setTorque(torque)
                @viz.setSeperateAxesTorque(false)
                confirm 'Torque should be displayed as one arrows starting from (0,0,0) to (1,1,1) with circular arrow rotating counter-clockwise'
            end
        end

        describe "setSeperateAxesForce" do
            it "displays force axis component" do
                force = Types.base.Vector3d.new(1,1,1)
                @viz.setForce(force)
                @viz.setSeperateAxesForce(true)
                confirm 'Force should be displayed as three arrows along axes each with length 1'
            end

            it "displays force" do
                force = Types.base.Vector3d.new(1,1,1)
                @viz.setForce(force)
                @viz.setSeperateAxesForce(false)
                confirm 'Force should be displayed as one arrows starting from (0,0,0) to (1,1,1)'
            end
        end

        # describe "setResolution" do
        #     wrench = Types.base.samples.Wrench.new
        #     wrench.frame_id = "test_frame"
        #     wrench.force = Types.base.Vector3d.new(1,0,0)
        #     wrench.torque = Types.base.Vector3d.new(0,1,0)

        #     it "sets resolution to 1" do
        #         @viz.updateData(wrench)
        #         @viz.setResolution(1.0)
        #         confirm "Force and torque will be displayed as arrows from (0,0,0) to (1,0,0) / (0,1,0)"
        #     end
        #     it "it sets resolution to 0.1" do
        #         @viz.updateData(wrench)
        #         @viz.setResolution(0.1)
        #         confirm "Force and torque will be displayed as arrows from (0,0,0) to (0.1,0,0) / (0,0.1,0) without arrow head"
        #     end
        # end
    end
end

