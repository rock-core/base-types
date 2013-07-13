require 'base/float'

Typelib.specialize_model '/base/samples/Joints' do
    def from_positions(*positions)
        states = positions.map do |p|
            j = self[:states].deference.new
            j.position = p
            j
        end
        new(:states => states)
    end
end

