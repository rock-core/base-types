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

    def from_speeds(*speeds)
        states = speeds.map do |s|
            j = self[:elements].deference.new
            j.speed = s
            j
        end
        new(:elements => states)
    end

    def from_efforts(*efforts)
        states = efforts.map do |e|
            j = self[:elements].deference.new
            j.effort = e
            j
        end
        new(:elements => states)
    end

    def from_raws(*raws)
        states = raws.map do |r|
            j = self[:elements].deference.new
            j.raw = r
            j
        end
        new(:elements => states)
    end

    def from_accelerations(*accelerations)
        states = accelerations.map do |r|
            j = self[:elements].deference.new
            j.acceleration = r
            j
        end
        new(:elements => states)
    end
end

Typelib.specialize '/base/samples/Joints' do
    include Base::NamedVector
end
