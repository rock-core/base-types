Typelib.convert_to_ruby '/wrappers/Matrix</double,3,1>', Eigen::Vector3 do |value|
    Eigen::Vector3.new(*value.data.to_a)
end
Typelib.convert_from_ruby Eigen::Vector3, '/wrappers/Matrix</double,3,1>' do |value, type|
    t = type.new
    t.data = value.to_a
    t
end

Typelib.convert_to_ruby '/wrappers/Matrix</double,4,4>', Eigen::Matrix4 do |value|
    m = Eigen::Matrix4.new()
    m.from_a(value.data.to_a)
    m
end
Typelib.convert_from_ruby Eigen::Matrix4, '/wrappers/Matrix</double,4,4>' do |value, type|
    t = type.new
    t.data = value.to_a
    t
end

Typelib.convert_to_ruby '/wrappers/AngleAxisd', Eigen::AngleAxis do |value|
    Eigen::Quaternion.new(value.angle, *value.axis.to_a)
end
Typelib.convert_from_ruby Eigen::AngleAxis, '/wrappers/AngleAxisd' do |value, type|
    data = value.to_a
    t = type.new
    t.angle = data[0]
    t.axis = data[1, 3]
    t
end

Typelib.convert_to_ruby '/wrappers/Quaternion</double>', Eigen::Quaternion do |value|
    Eigen::Quaternion.new(value.re, *value.im.to_a)
end
Typelib.convert_from_ruby Eigen::Quaternion, '/wrappers/Quaternion</double>' do |value, type|
    data = value.to_a
    t = type.new
    t.re = data[0]
    t.im = data[1, 3]
    t
end

Typelib.specialize '/wrappers/MatrixX</double>' do
    def initialize
        self.rows = self.cols = 0
        super
    end
end
Typelib.convert_to_ruby '/wrappers/MatrixX</double>', Eigen::MatrixX do |value|
    m = Eigen::MatrixX.new(value.rows,value.cols)
    m.from_a(value.data.to_a,value.rows,value.cols)
    m
end
Typelib.convert_from_ruby Eigen::MatrixX, '/wrappers/MatrixX</double>' do |value, type|
    t = type.new
    t.rows = value.rows
    t.cols = value.cols
    t.data = value.to_a
    t
end

Typelib.convert_to_ruby '/wrappers/VectorX</double>', Eigen::VectorX do |value|
    m = Eigen::VectorX.new(value.data.size)
    m.from_a(value.data.to_a)
    m
end
Typelib.convert_from_ruby Eigen::VectorX, '/wrappers/VectorX</double>' do |value, type|
    t = type.new
    value.to_a.each {|v| t.data.push(v) }
    t
end
