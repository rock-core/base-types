#include "bench_func.h"

Vector3FloatAlign mult_tv1(Vector3FloatAlign const& v, TransformFloatAlign const& t)
{
    return t * v;
}
Vector3FloatNoAlign mult_tv2(Vector3FloatNoAlign const& v, TransformFloatNoAlign const& t)
{
    return t * v;
}
Vector4FloatAlign mult_tv3(Vector4FloatAlign const& v, TransformFloatAlign const& t)
{
    return t * v;
}
Vector4FloatNoAlign mult_tv4(Vector4FloatNoAlign const& v, TransformFloatNoAlign const& t)
{
    return t * v;
}
Vector3DoubleAlign mult_tv5(Vector3DoubleAlign const& v, TransformDoubleAlign const& t)
{
    return t * v;
}
Vector3DoubleNoAlign mult_tv6(Vector3DoubleNoAlign const& v, TransformDoubleNoAlign const& t)
{
    return t * v;
}
Vector4DoubleAlign mult_tv7(Vector4DoubleAlign const& v, TransformDoubleAlign const& t)
{
    return t * v;
}
Vector4DoubleNoAlign mult_tv8(Vector4DoubleNoAlign const& v, TransformDoubleNoAlign const& t)
{
    return t * v;
}

TransformFloatAlign mult_tt1(TransformFloatAlign const& t1, TransformFloatAlign const& t2)
{
    return t1 * t2;
}
TransformFloatNoAlign mult_tt2(TransformFloatNoAlign const& t1, TransformFloatNoAlign const& t2)
{
    return t1 * t2;
}
TransformDoubleAlign mult_tt3(TransformDoubleAlign const& t1, TransformDoubleAlign const& t2)
{
    return t1 * t2;
}
TransformDoubleNoAlign mult_tt4(TransformDoubleNoAlign const& t1, TransformDoubleNoAlign const& t2)
{
    return t1 * t2;
}
