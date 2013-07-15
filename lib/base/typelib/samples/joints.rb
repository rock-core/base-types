require 'base/float'

Typelib.specialize_model '/base/samples/Joints' do
    def from_positions(*positions)
        states = positions.map do |p|
            j = self[:elements].deference.new
            j.position = p
            j
        end
        new(:elements => states)
    end
end

Typelib.specialize '/base/samples/Joints' do
    include Base::NamedVector
end
