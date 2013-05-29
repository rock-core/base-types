require 'base/float'

Typelib.specialize_model '/base/JointState' do
    def new(*args)
        result = super
        if args.empty?
            result.position = Base::unset
            result.speed = Base::unset
            result.effort = Base::unset
            result.raw = Base::unset
        end
        result
    end

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
end

Typelib.specialize '/base/JointState' do
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
        else raise ArgumentError, "#{mode} is not a valid mode, was expecting one of :POSITION, :SPEED, :EFFORT or :RAW"
        end
    end
end

