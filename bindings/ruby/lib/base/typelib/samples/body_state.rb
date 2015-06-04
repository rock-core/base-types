Typelib.specialize_model '/base/samples/BodyState_m' do
    def invalid
        v3 = { :data => [NaN] * 3 }
        m6 = { :data => [NaN] * 36 }
        new(
            :time => Time.at(0),
            :pose => { :orientation => AngleAxis.new(NaN, v3), :translation => v3, :cov => m6},
            :velocity => { :rot => v3, :vel => v3, :cov => m6})
    end

    def Invalid; invalid end
end

Typelib.specialize '/base/samples/BodyState_m' do
    def transform
	Eigen::Isometry3.from_position_orientation( position, orientation )
    end

    def transform= t
	self.pose.position = t.translation
	self.pose.orientation = AngleAxis.from_matrix(t.rotation)
    end
end

