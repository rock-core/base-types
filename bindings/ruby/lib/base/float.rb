if !defined?(Infinity)
    Infinity = 1e200 ** 200
end
if !defined?(NaN)
    NaN = Infinity / Infinity
end

module Base
    def self.infinity?(v); v == Infinity end
    def self.infinity; Infinity end
    def self.nan?(v); v != v end
    def self.nan; NaN end
    def self.unknown?(v); nan?(v) end
    def self.unknown; nan end
    def self.unset?(v); nan?(v) end
    def self.unset; nan end
end
