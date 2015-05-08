Typelib.specialize_model '/base/TransformWithCovariance_m' do
    def invalid
        trans = { :data => [NaN] * 16 }
        cov = { :data => [NaN] * 36 }
        new(
            :trans => trans,
            :cov => cov)
    end

    def Invalid; invalid end

end

Typelib.specialize '/base/TransformWithCovariance_m' do
    def transform
	    Eigen::Affine3.from_position_orientation( position, orientation )
    end

    def transform= t
	    self.trans = t
    end
end

