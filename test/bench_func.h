#include <Eigen/Geometry>

typedef Eigen::Transform<float, 3, Eigen::Isometry, Eigen::AutoAlign> TransformFloatAlign;
typedef Eigen::Transform<float, 3, Eigen::Isometry, Eigen::DontAlign> TransformFloatNoAlign;
typedef Eigen::Transform<double, 3, Eigen::Isometry, Eigen::AutoAlign> TransformDoubleAlign;
typedef Eigen::Transform<double, 3, Eigen::Isometry, Eigen::DontAlign> TransformDoubleNoAlign;
typedef Eigen::Matrix<float, 3, 1, Eigen::AutoAlign> Vector3FloatAlign;
typedef Eigen::Matrix<float, 3, 1, Eigen::DontAlign> Vector3FloatNoAlign;
typedef Eigen::Matrix<float, 4, 1, Eigen::AutoAlign> Vector4FloatAlign;
typedef Eigen::Matrix<float, 4, 1, Eigen::DontAlign> Vector4FloatNoAlign;
typedef Eigen::Matrix<double, 3, 1, Eigen::AutoAlign> Vector3DoubleAlign;
typedef Eigen::Matrix<double, 3, 1, Eigen::DontAlign> Vector3DoubleNoAlign;
typedef Eigen::Matrix<double, 4, 1, Eigen::AutoAlign> Vector4DoubleAlign;
typedef Eigen::Matrix<double, 4, 1, Eigen::DontAlign> Vector4DoubleNoAlign;

Vector3FloatAlign mult_tv1(Vector3FloatAlign const& v, TransformFloatAlign const& t);
Vector3FloatNoAlign mult_tv2(Vector3FloatNoAlign const& v, TransformFloatNoAlign const& t);
Vector4FloatAlign mult_tv3(Vector4FloatAlign const& v, TransformFloatAlign const& t);
Vector4FloatNoAlign mult_tv4(Vector4FloatNoAlign const& v, TransformFloatNoAlign const& t);
Vector3DoubleAlign mult_tv5(Vector3DoubleAlign const& v, TransformDoubleAlign const& t);
Vector3DoubleNoAlign mult_tv6(Vector3DoubleNoAlign const& v, TransformDoubleNoAlign const& t);
Vector4DoubleAlign mult_tv7(Vector4DoubleAlign const& v, TransformDoubleAlign const& t);
Vector4DoubleNoAlign mult_tv8(Vector4DoubleNoAlign const& v, TransformDoubleNoAlign const& t);

TransformFloatAlign mult_tt1(TransformFloatAlign const& t1, TransformFloatAlign const& t2);
TransformFloatNoAlign mult_tt2(TransformFloatNoAlign const& t1, TransformFloatNoAlign const& t2);
TransformDoubleAlign mult_tt3(TransformDoubleAlign const& t1, TransformDoubleAlign const& t2);
TransformDoubleNoAlign mult_tt4(TransformDoubleNoAlign const& t1, TransformDoubleNoAlign const& t2);
