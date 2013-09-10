module Base
    module NamedVector
        def each(&block)
            elements.each(&block)
        end

        def each_with_name
            elements.each_with_index do |j, i|
                n = names[i]
                if n.empty? then n = nil end
                yield(j, n)
            end
        end
    end
end
