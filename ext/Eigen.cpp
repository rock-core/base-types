#include "rice/Class.hpp"
#include "rice/String.hpp"
#include "rice/Constructor.hpp"
#include "rice/Enum.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace Rice;

struct Vector3
{
    Eigen::Vector3d* v;

    Vector3(double x, double y, double z)
        : v(new Eigen::Vector3d(x, y, z)) {}
    Vector3(Eigen::Vector3d const& _v)
        : v(new Eigen::Vector3d(_v)) {}
    ~Vector3()
    { delete v; }

    double x() const { return v->x(); }
    double y() const { return v->y(); }
    double z() const { return v->z(); }
    void setX(double value) { v->x() = value; }
    void setY(double value) { v->y() = value; }
    void setZ(double value) { v->z() = value; }

    Vector3* operator + (Vector3 const& other) const
    { return new Vector3(*v + *other.v); }
    Vector3* operator - (Vector3 const& other) const
    { return new Vector3(*v - *other.v); }
    Vector3* negate() const
    { return new Vector3(-*v); }
    Vector3* scale(double value) const
    { return new Vector3(*v * value); }
};

struct Quaternion
{
    Eigen::Quaterniond* q;

    Quaternion(double w, double x, double y, double z)
        : q(new Eigen::Quaterniond(w, x, y, z)) { }

    Quaternion(Eigen::Quaterniond const& _q)
        : q(new Eigen::Quaterniond(_q)) {}

    ~Quaternion()
    { delete q; }

    double w() const { return q->w(); }
    double x() const { return q->x(); }
    double y() const { return q->y(); }
    double z() const { return q->z(); }
    void setW(double value) { q->w() = value; }
    void setX(double value) { q->x() = value; }
    void setY(double value) { q->y() = value; }
    void setZ(double value) { q->z() = value; }

    Quaternion* concatenate(Quaternion const& other) const
    { return new Quaternion((*q) * (*other.q)); }
    Vector3* transform(Vector3 const& v) const
    { return new Vector3((*q) * (*v.v)); }
    Quaternion* inverse() const
    { return new Quaternion(q->inverse()); }
    void normalize_bang()
    { q->normalize(); }
    Quaternion* normalize() const
    { 
        Eigen::Quaterniond q = *this->q;
        q.normalize();
        return new Quaternion(q);
    }

    void from_euler(Vector3 const& angles, int axis0, int axis1, int axis2)
    {
        *(this->q) =
            Eigen::AngleAxisd(angles.x(), Eigen::Vector3d::Unit(axis0)) *
            Eigen::AngleAxisd(angles.y(), Eigen::Vector3d::Unit(axis1)) *
            Eigen::AngleAxisd(angles.z(), Eigen::Vector3d::Unit(axis2));
    }

    bool isApprox(Quaternion const& other, double tolerance)
    {
        return q->isApprox(*other.q, tolerance);
    }

    Vector3* to_euler(int axis0, int axis1, int axis2)
    {
        return new Vector3(q->toRotationMatrix().eulerAngles(axis0, axis1, axis2));
    }
};

// The initialization method for this module
void Init_eigen_ext()
{
     Rice::Module rb_mEigen = define_module("Eigen");

     Data_Type<Vector3> rb_Vector3 = define_class_under<Vector3>(rb_mEigen, "Vector3")
       .define_constructor(Constructor<Vector3,double,double,double>())
       .define_method("x",  &Vector3::x)
       .define_method("y",  &Vector3::y)
       .define_method("z",  &Vector3::z)
       .define_method("x=", &Vector3::setX)
       .define_method("y=", &Vector3::setY)
       .define_method("z=", &Vector3::setZ)
       .define_method("+",  &Vector3::operator +)
       .define_method("-",  &Vector3::operator -)
       .define_method("-@", &Vector3::negate)
       .define_method("*",  &Vector3::scale);

     Data_Type<Quaternion> rb_Quaternion = define_class_under<Quaternion>(rb_mEigen, "Quaternion")
       .define_constructor(Constructor<Quaternion,double,double,double,double>())
       .define_method("w",  &Quaternion::w)
       .define_method("x",  &Quaternion::x)
       .define_method("y",  &Quaternion::y)
       .define_method("z",  &Quaternion::z)
       .define_method("w=", &Quaternion::setW)
       .define_method("x=", &Quaternion::setX)
       .define_method("y=", &Quaternion::setY)
       .define_method("z=", &Quaternion::setZ)
       .define_method("concatenate", &Quaternion::concatenate)
       .define_method("transform", &Quaternion::transform)
       .define_method("normalize!", &Quaternion::normalize_bang)
       .define_method("normalize", &Quaternion::normalize)
       .define_method("approx?", &Quaternion::isApprox)
       .define_method("to_euler", &Quaternion::to_euler)
       .define_method("from_euler", &Quaternion::from_euler);
}

