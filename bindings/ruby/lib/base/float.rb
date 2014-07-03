if !defined?(Infinity)
    Infinity = Float::INFINITY
end
if !defined?(NaN)
    NaN = Float::NAN
end

module Base
    def self.infinity?(v); v.to_f.infinite? end
    def self.infinity; Float::INFINITY end
    def self.nan?(v); v.to_f.nan? end
    def self.nan; Float::NAN end
    def self.unknown?(v); nan?(v) end
    def self.unknown; nan end
    def self.unset?(v); nan?(v) end
    def self.unset; nan end
end
