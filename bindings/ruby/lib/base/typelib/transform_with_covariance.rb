Typelib.specialize_model '/base/TransformWithCovariance_m' do
    def invalid
        translation = { :data => [NaN] * 3 }
        cov = { :data => [NaN] * 36 }
        new(
            :rotation => { :angle => NaN, :axis => [NaN, NaN, NaN]},
            :translation => translation,
            :cov => cov)
    end

    def Invalid; invalid end

end

Typelib.specialize '/base/TransformWithCovariance_m' do
    def transform
	    Eigen::Isometry3.from_position_orientation( position, orientation )
    end

    def transform= t
	    self.trans = t
    end
end

