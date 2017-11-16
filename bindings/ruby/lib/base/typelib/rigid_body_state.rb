Typelib.specialize_model '/base/samples/RigidBodyState_m' do
    def invalid
        if !@invalid_sample
            v3 = { data: [NaN] * 3 }
            m3 = { data: [NaN] * 9 }
            @invalid_sample = new(
                time: Time.at(0),
                position: v3,
                cov_position: m3,
                orientation: { im: [NaN, NaN, NaN], re: NaN },
                cov_orientation: m3,
                velocity: v3,
                cov_velocity: m3,
                angular_velocity: v3,
                cov_angular_velocity: m3)
        end
        @invalid_sample.dup
    end

    def Invalid; invalid end

    def from_pose(pose)
        rbs = invalid
        rbs.position = pose.position
        rbs.orientation = pose.orientation
        rbs
    end
end

Typelib.specialize '/base/samples/RigidBodyState_m' do
    def transform
	Eigen::Isometry3.from_position_orientation( position, orientation )
    end

    def transform= t
	self.position = t.translation
	self.orientation = t.rotation
    end
end

