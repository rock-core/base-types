#include "rice/Class.hpp"
#include "rice/String.hpp"
#include "rice/Constructor.hpp"
#include "rice/Enum.hpp"

#include <base/geometry/Spline.hpp>

using namespace Rice;
using base::geometry::SplineBase;

typedef SplineBase::CoordinateType CoordinateType;

Rice::Enum<CoordinateType> coordinate_type_type;

template<>
CoordinateType from_ruby<CoordinateType>( Object x )
{
    Data_Object<CoordinateType> d( x, coordinate_type_type );
    return *d;
}

template<class Type>
static std::vector<Type> array_to_cpp(Array array)
{
    std::vector<Type> result;
    result.reserve(array.size());
    for (unsigned int i = 0; i < array.size(); ++i)
        result.push_back(from_ruby<Type>(array[i]));
    return result;
}

static std::vector<double> array_to_double_vector(Array array)
{
    return array_to_cpp<double>( array );
}

static Array double_vector_to_array(std::vector<double> const& v)
{
    return Array(v.begin(), v.end());
}

class RubySpline : public SplineBase
{
public:
    RubySpline(int dimension, double geometric_resolution, int order)
        : SplineBase(dimension, geometric_resolution, order) {}


    void do_interpolate(Array coordinates, Array values, Array coord_types)
    {
        interpolate(array_to_double_vector(coordinates), array_to_double_vector(values), array_to_cpp<CoordinateType>(coord_types) );
    }

    Array do_coordinates()
    {
        return double_vector_to_array(getCoordinates());
    }

    Array do_knots()
    {
        return double_vector_to_array(getKnots());
    }

    void do_reset(Array coordinates, Array knots, int kind)
    {
        SplineBase::reset(array_to_double_vector(coordinates), array_to_double_vector(knots), kind);
    }

    Array do_getPoint(double param, bool with_tangent)
    {
        std::vector<double> result;
        if (with_tangent)
        {
            result.resize(getDimension() * 2);
            getPointAndTangent(&result[0], param);
        }
        else
        {
            result.resize(getDimension());
            getPoint(&result[0], param);
        }

        return double_vector_to_array(result);
    }

    double do_findOneClosestPoint(Array _ref_point, double guess, double geores)
    {
        std::vector<double> ref_point = array_to_double_vector(_ref_point);
        return findOneClosestPoint(&ref_point[0], guess, geores);
    }

    double do_length(double start, double end, double geores) const
    {
        if (getDimension() == 3)
        {
            return reinterpret_cast<base::geometry::Spline<3> const*>(this)->length(start, end, geores);
        }
        else
        {
            throw std::runtime_error("cannot call #length on splines of dimension different than 3");
        }
    }

    Array do_findClosestPoints(Array _ref_point, double geores)
    {
        std::vector<double> ref_point = array_to_double_vector(_ref_point);
        std::vector<double> result_points;
        std::vector< std::pair<double, double> > result_curves;
        findClosestPoints(&ref_point[0], result_points, result_curves, geores);

        Array ruby_points = double_vector_to_array(result_points);
        Array ruby_curves;
        for (unsigned int i = 0; i < result_curves.size(); ++i)
        {
            Array pair;
            pair.push(result_curves[i].first);
            pair.push(result_curves[i].second);
            ruby_curves.push(pair);
        }

        Array result;
        result.push(ruby_points);
        result.push(ruby_curves);
        return result;
    }

    void initialize_copy(RubySpline const& other)
    {
        *this = other;
    }
};



void Init_spline_ext()
{
    Rice::Module rb_mSpline = Rice::define_module("SISL");

    typedef std::vector<double>(RubySpline::*SimplifySelector)(double);
    typedef void(SplineBase::*Append)(SplineBase const&,double);
    typedef double (SplineBase::*GetCurveLength)(double) const;

    coordinate_type_type =
	define_enum<CoordinateType>("CoordinateType", rb_mSpline)
	.define_value("ORDINARY_POINT", SplineBase::ORDINARY_POINT)
	.define_value("KNUCKLE_POINT", SplineBase::KNUCKLE_POINT)
	.define_value("DERIVATIVE_TO_NEXT", SplineBase::DERIVATIVE_TO_NEXT)
	.define_value("DERIVATIVE_TO_PRIOR", SplineBase::DERIVATIVE_TO_PRIOR)
	.define_value("SECOND_DERIVATIVE_TO_NEXT", SplineBase::SECOND_DERIVATIVE_TO_NEXT)
	.define_value("SECOND_DERIVATIVE_TO_PRIOR", SplineBase::SECOND_DERIVATIVE_TO_PRIOR)
	.define_value("TANGENT_POINT_FOR_NEXT", SplineBase::TANGENT_POINT_FOR_NEXT)
	.define_value("TANGENT_POINT_FOR_PRIOR", SplineBase::TANGENT_POINT_FOR_PRIOR);

    Data_Type<SplineBase> rb_SplineBase = define_class_under<SplineBase>(rb_mSpline, "SplineBase")
        .define_constructor(Constructor<SplineBase,int,double,int>(),
                (Arg("dimension"), Arg("geometric_resolution") = 0.1, Arg("order") = 3))
        .define_method("geometric_resolution=", &SplineBase::setGeometricResolution)
        .define_method("geometric_resolution", &SplineBase::getGeometricResolution)
        .define_method("order", &SplineBase::getCurveOrder)
        .define_method("empty?", &SplineBase::isEmpty)
        .define_method("singleton?", &SplineBase::isSingleton)
        .define_method("reverse", &SplineBase::reverse)
        .define_method("dimension", &SplineBase::getDimension)
        .define_method("point_count", &SplineBase::getPointCount)
        .define_method("curve_length", static_cast<GetCurveLength>(&SplineBase::getCurveLength),
                (Arg("relative_resolution") = 0.01))
        .define_method("curvature_max", &SplineBase::getCurvatureMax)
        .define_method("start_param", &SplineBase::getStartParam)
        .define_method("end_param", &SplineBase::getEndParam)
        .define_method("curvature_at", &SplineBase::getCurvature)
        .define_method("variation_of_curvature_at", &SplineBase::getVariationOfCurvature)
        .define_method("simplify", static_cast<SimplifySelector>(&SplineBase::simplify))
        .define_method("clear", &SplineBase::clear)
        .define_method("sisl_curve_type", &SplineBase::getSISLCurveType)
        .define_method("coordinate_stride", &SplineBase::getCoordinatesStride)
        .define_method("join", &SplineBase::join, (Arg("curve"), Arg("tolerance") = static_cast<double>(0), Arg("with_tangents") = true))
        .define_method("do_split", &SplineBase::split)
        .define_method("append", static_cast<Append>(&SplineBase::append),
                (Arg("spline"), Arg("tolerance") = static_cast<double>(1e-6)));

    Data_Type<RubySpline> rb_Spline = define_class_under<RubySpline, SplineBase>(rb_mSpline, "Spline")
        .define_constructor(Constructor<RubySpline,int,double,int>(),
                (Arg("dimension"), Arg("geometric_resolution") = 0.1, Arg("order") = 3))
        .define_method("initialize_copy", &RubySpline::initialize_copy)
        .define_method("do_interpolate", &RubySpline::do_interpolate)
        .define_method("coordinates", &RubySpline::do_coordinates)
        .define_method("knots", &RubySpline::do_knots)
        .define_method("do_length", &RubySpline::do_length)
        .define_method("do_find_one_closest_point", &RubySpline::do_findOneClosestPoint)
        .define_method("do_find_closest_points", &RubySpline::do_findClosestPoints)
        .define_method("reset", &RubySpline::do_reset, (Arg("coordinates"), Arg("knots"), Arg("kind") = -1))
        .define_method("get", &RubySpline::do_getPoint, (Arg("parameter"), Arg("with_tangent") = false));
}

