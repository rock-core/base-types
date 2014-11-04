require 'base/float'

Typelib.specialize_model '/base/JointState' do
    def Position(value)
        result = new
        result.position = value
        result
    end
    def Speed(value)
        result = new
        result.speed = value
        result
    end
    def Effort(value)
        result = new
        result.effort = value
        result
    end
    def Raw(value)
        result = new
        result.raw = value
        result
    end
    def Acceleration(value)
        result = new
        result.acceleration = value
        result
    end
end

Typelib.specialize '/base/JointState' do
    def initialize
        self.position = Base::unset
        self.speed = Base::unset
        self.effort = Base::unset
        self.raw = Base::unset
        self.acceleration = Base::unset
    end

    def [](mode)
        case mode
        when :POSITION
            position
        when :SPEED
            speed
        when :EFFORT
            effort
        when :RAW
            raw
        when :ACCELERATION
            acceleration
        else raise ArgumentError, "#{mode} is not a valid mode, was expecting one of :POSITION, :SPEED, :EFFORT or :RAW"
        end
    end

    def []=(mode, value)
        case mode
        when :POSITION
            self.position = value
        when :SPEED
            self.speed = value
        when :EFFORT
            self.effort = value
        when :RAW
            self.raw = value
        when :ACCELERATION
            self.acceleration = value
        else raise ArgumentError, "#{mode} is not a valid mode, was expecting one of :POSITION, :SPEED, :ACCELERATION, :EFFORT or :RAW"
        end
    end
end

