#include "rice/Class.hpp"
#include "rice/String.hpp"
#include "rice/Constructor.hpp"
#include "rice/Enum.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

using namespace Rice;

typedef Eigen::Matrix<double, 3, 1, Eigen::DontAlign>     Vector3d;
typedef Eigen::Matrix<double, 4, 4, Eigen::DontAlign>     Matrix4d;
typedef Eigen::Quaternion<double, Eigen::DontAlign>    Quaterniond;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::DontAlign>
                                                       MatrixXd;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::DontAlign>
                                                       VectorXd;
typedef Eigen::Transform< double, 3, Eigen::Isometry > Isometry3d;
typedef Eigen::Transform< double, 3, Eigen::Affine > Affine3d;
typedef Eigen::AngleAxis<double> AngleAxisd;

struct Vector3
{
    Vector3d* v;

    Vector3(double x, double y, double z)
        : v(new Vector3d(x, y, z)) {}
    Vector3(Vector3d const& _v)
        : v(new Vector3d(_v)) {}
    ~Vector3()
    { delete v; }

    double x() const { return v->x(); }
    double y() const { return v->y(); }
    double z() const { return v->z(); }
    void setX(double value) { v->x() = value; }
    void setY(double value) { v->y() = value; }
    void setZ(double value) { v->z() = value; }


    double norm() const { return v->norm(); }
    Vector3* normalize() const { return new Vector3(v->normalized()); }
    void normalizeBang() const { v->normalize(); }

    double get(int i) const { return (*v)[i]; }
    void set(int i, double value) { (*v)[i] = value; }

    Vector3* operator + (Vector3 const& other) const
    { return new Vector3(*v + *other.v); }
    Vector3* operator - (Vector3 const& other) const
    { return new Vector3(*v - *other.v); }

    Vector3* operator / (double scalar) const
    { return new Vector3(*v / scalar); }

    Vector3* negate() const
    { return new Vector3(-*v); }
    Vector3* scale(double value) const
    { return new Vector3(*v * value); }
    double dot(Vector3 const& other) const
    { return this->v->dot(*other.v); }
    Vector3* cross(Vector3 const& other) const
    { return new Vector3(this->v->cross(*other.v)); }
    bool operator ==(Vector3 const& other) const
    { return (*this->v) == (*other.v); }
    bool isApprox(Vector3 const& other, double tolerance)
    { return v->isApprox(*other.v, tolerance); }
};

struct VectorX {

    VectorXd* v;
    
    VectorX()
        : v(new VectorXd()) {}
    VectorX(VectorX const& v)
        : v(new VectorXd(*v.v)) {}
    VectorX(int n)
        : v(new VectorXd(n)) {}
    VectorX(VectorXd const& _v)
        : v(new VectorXd(_v)) {}
    ~VectorX()
    { delete v; }
    
    void resize(int n) { v->resize(n); }
    void conservativeResize(int n) { v->conservativeResize(n); }

    double norm() const { return v->norm(); }
    VectorX* normalize() const { return new VectorX(v->normalized()); }
    void normalizeBang() const { v->normalize(); }

    unsigned int size() { return v->size(); }

    double get(int i) const { return (*v)[i]; }
    void set(int i, double value) { (*v)[i] = value; }

    VectorX* operator + (VectorX const& other) const
    { return new VectorX(*v + *other.v); }
    VectorX* operator - (VectorX const& other) const
    { return new VectorX(*v - *other.v); }

    VectorX* operator / (double scalar) const
    { return new VectorX(*v / scalar); }
    
    VectorX* negate() const
    { return new VectorX(-*v); }

    VectorX* scale(double value) const
    { return new VectorX(*v * value); }

    double dot(VectorX const& other) const
    { return this->v->dot(*other.v); }

    bool operator ==(VectorX const& other) const
    { return (*this->v) == (*other.v); }

    bool isApprox(VectorX const& other, double tolerance)
    { return v->isApprox(*other.v, tolerance); }

};

struct Matrix4
{
    Matrix4d* mx;

    Matrix4() : mx(new Matrix4d()) {}

    Matrix4(Matrix4d const& _mx)
        : mx(new Matrix4d(_mx)) {}

    ~Matrix4()
    { delete mx; }

    double norm() const { return mx->norm(); }

    int rows() const { return mx->rows(); }
    int cols() const { return mx->cols(); }
    int size() const { return mx->size(); }

    double get(int i, int j ) const { return (*mx)(i,j); }
    void set(int i, int j, double value) { (*mx)(i,j) = value; }

    Matrix4* transpose() const
    { return new Matrix4(mx->transpose()); }

    Matrix4* operator + (Matrix4 const& other) const
    { return new Matrix4(*mx + *other.mx); }

    Matrix4* operator - (Matrix4 const& other) const
    { return new Matrix4(*mx - *other.mx); }

    Matrix4* operator / (double scalar) const
    { return new Matrix4(*mx / scalar); }

    Matrix4* negate() const
    { return new Matrix4(-*mx); }

    Matrix4* scale(double value) const
    { return new Matrix4(*mx * value); }

    Matrix4* dotM (Matrix4 const& other) const
    { return new Matrix4(*mx * (*other.mx)); }

    bool operator ==(Matrix4 const& other) const
    { return (*this->mx) == (*other.mx); }

    bool isApprox(Matrix4 const& other, double tolerance)
    { return mx->isApprox(*other.mx, tolerance); }
};

struct JacobiSVD {
    typedef Eigen::JacobiSVD<Eigen::MatrixXd::PlainObject> EigenT;
    EigenT* j;

    JacobiSVD( EigenT const& j )
    : j(new EigenT(j)) {}
    JacobiSVD( JacobiSVD const& j )
    : j(new EigenT(*j.j)) {}
    ~JacobiSVD()
    { delete j; }

    VectorX* solve(VectorX* y)
    { return new VectorX(j->solve(*y->v)); }
};

struct MatrixX {

    MatrixXd* m;

    MatrixX() : m(new MatrixXd()) {}
    MatrixX(const MatrixX& m) : m(new MatrixXd(*m.m)) {}
    MatrixX(int rows, int cols) : m(new MatrixXd(rows,cols)) {}
    MatrixX(const MatrixXd& _m) : m(new MatrixXd(_m)) {}
    ~MatrixX() { delete m; }

    void resize(int rows, int cols) { m->resize(rows,cols); }
    void conservativeResize(int rows, int cols) { m->conservativeResize(rows,cols); }

    double norm() const { return m->norm(); }

    unsigned int rows() const { return m->rows(); }
    unsigned int cols() const { return m->cols(); }
    unsigned int size() const { return m->size(); }

    double get(int i, int j ) const { return (*m)(i,j); }
    void set(int i, int j, double value) { (*m)(i,j) = value; }
    
    VectorX* getRow(int i) const { return new VectorX(m->row(i)); }
    void setRow(int i, const VectorX& v) { m->row(i) = *(v.v); }

    VectorX* getColumn(int j) const { return new VectorX(m->col(j)); }
    void setColumn(int j, const VectorX& v) { m->col(j) = *(v.v); }

    MatrixX* transpose() const
    { return new MatrixX(m->transpose()); }

    MatrixX* operator + (MatrixX const& other) const
    { return new MatrixX(*m + *other.m); }

    MatrixX* operator - (MatrixX const& other) const
    { return new MatrixX(*m - *other.m); }

    MatrixX* operator / (double scalar) const
    { return new MatrixX(*m / scalar); }

    MatrixX* negate() const
    { return new MatrixX(-*m); }
    
    MatrixX* scale (double scalar) const
    { return new MatrixX(*m * scalar); }

    VectorX* dotV (VectorX const& other) const
    { return new VectorX(*m * *other.v); }
    
    MatrixX* dotM (MatrixX const& other) const
    { return new MatrixX(*m * (*other.m)); }

    JacobiSVD* jacobiSvd(int flags = 0) const
    { return new JacobiSVD(m->jacobiSvd(flags)); }

    bool operator ==(MatrixX const& other) const
    { return (*this->m) == (*other.m); }

    bool isApprox(MatrixX const& other, double tolerance)
    { return m->isApprox(*other.m, tolerance); }
};

struct Quaternion
{
    Quaterniond* q;
    Quaternion(double w, double x, double y, double z)
        : q(new Quaterniond(w, x, y, z)) { }
    Quaternion(Quaternion const& q)
        : q(new Quaterniond(*q.q)) { }
    Quaternion(Quaterniond const& _q)
        : q(new Quaterniond(_q)) {}

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

    double norm() const { return q->norm(); }

    bool operator ==(Quaternion const& other) const
    { return x() == other.x() && y() == other.y() && z() == other.z() && w() == other.w(); }

    Quaternion* concatenate(Quaternion const& other) const
    { return new Quaternion((*q) * (*other.q)); }
    Vector3* transform(Vector3 const& v) const
    { return new Vector3((*q) * (*v.v)); }
    Quaternion* inverse() const
    { return new Quaternion(q->inverse()); }
    void normalizeBang()
    { q->normalize(); }
    Quaternion* normalize() const
    { 
        Quaterniond q = *this->q;
        q.normalize();
        return new Quaternion(q);
    }
    MatrixX* matrix() const
    {
        return new MatrixX(q->matrix());
    }

    void fromAngleAxis(double angle, Vector3 const& axis)
    {
	*(this->q) = 
            Eigen::AngleAxisd(angle, *axis.v);
    }

    void fromEuler(Vector3 const& angles, int axis0, int axis1, int axis2)
    {
        *(this->q) =
            Eigen::AngleAxisd(angles.x(), Eigen::Vector3d::Unit(axis0)) *
            Eigen::AngleAxisd(angles.y(), Eigen::Vector3d::Unit(axis1)) *
            Eigen::AngleAxisd(angles.z(), Eigen::Vector3d::Unit(axis2));
    }

    void fromMatrix(MatrixX const& matrix)
    {
	*(this->q) = 
            Quaterniond(Eigen::Matrix3d(*matrix.m));
    }

    bool isApprox(Quaternion const& other, double tolerance)
    {
        return q->isApprox(*other.q, tolerance);
    }

    Vector3* toEuler()
    {
        const Eigen::Matrix3d m = q->toRotationMatrix();
        double i = Eigen::Vector2d(m.coeff(2,2) , m.coeff(2,1)).norm();
        double y = atan2(-m.coeff(2,0), i);
        double x=0,z=0;
        if (i > Eigen::NumTraits<double>::dummy_precision()){
            x = ::atan2(m.coeff(1,0), m.coeff(0,0));
            z = ::atan2(m.coeff(2,1), m.coeff(2,2));
        }else{
            z = (m.coeff(2,0)>0?1:-1)* ::atan2(-m.coeff(0,1), m.coeff(1,1));
        }
        return new Vector3(x,y,z);
    }
};

struct AngleAxis
{
    AngleAxisd* aa;
    AngleAxis(double angle, Vector3 const& axis)
        : aa(new AngleAxisd(angle, Eigen::Vector3d(*axis.v))){}
    AngleAxis(AngleAxis const& aa)
        : aa(new AngleAxisd(*aa.aa)) { }
    AngleAxis(AngleAxisd const& _aa)
        : aa(new AngleAxisd(_aa)) {}

    ~AngleAxis()
    { delete aa; }

    bool operator ==(AngleAxis const& other) const
    { return angle() == other.angle() && axis() == other.axis(); }

    double angle() const { return aa->angle(); }
    Vector3* axis() const { return new Vector3(aa->axis()); }

    AngleAxis* concatenate(AngleAxis const& other) const
    { return new AngleAxis(static_cast<AngleAxisd>((*aa) * (*other.aa))); }

    Vector3* transform(Vector3 const& v) const
    { return new Vector3((*aa) * (*v.v)); }

    AngleAxis* inverse() const
    { return new AngleAxis(aa->inverse()); }

    MatrixX* matrix() const
    {
        return new MatrixX(aa->matrix());
    }

    void fromQuaternion(Quaternion const& q)
    {
        *(this->aa) = Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z());
    }

    void fromEuler(Vector3 const& angles, int axis0, int axis1, int axis2)
    {
        *(this->aa) =
            Eigen::AngleAxisd(angles.x(), Eigen::Vector3d::Unit(axis0)) *
            Eigen::AngleAxisd(angles.y(), Eigen::Vector3d::Unit(axis1)) *
            Eigen::AngleAxisd(angles.z(), Eigen::Vector3d::Unit(axis2));
    }

    void fromMatrix(MatrixX const& matrix)
    {
	    *(this->aa) =
            AngleAxisd(Eigen::Matrix3d(*matrix.m));
    }

    bool isApprox(AngleAxis const& other, double tolerance)
    {
        return aa->isApprox(*other.aa, tolerance);
    }

    Vector3* toEuler()
    {
        const Eigen::Matrix3d m = aa->toRotationMatrix();
        double i = Eigen::Vector2d(m.coeff(2,2) , m.coeff(2,1)).norm();
        double y = atan2(-m.coeff(2,0), i);
        double x=0,z=0;
        if (i > Eigen::NumTraits<double>::dummy_precision()){
            x = ::atan2(m.coeff(1,0), m.coeff(0,0));
            z = ::atan2(m.coeff(2,1), m.coeff(2,2));
        }else{
            z = (m.coeff(2,0)>0?1:-1)* ::atan2(-m.coeff(0,1), m.coeff(1,1));
        }
        return new Vector3(x,y,z);
    }
};

#include <iostream>

struct Isometry3
{
    Isometry3d *t;

    Isometry3() : t(new Isometry3d()) { t->setIdentity(); }
    Isometry3(const Isometry3& _m) : t(new Isometry3d(*_m.t)) {}
    Isometry3(const Isometry3d& _m) : t(new Isometry3d(_m)) {}
    ~Isometry3() { delete t; }

    Isometry3* inverse() const
    { return new Isometry3( t->inverse() ); }

    Vector3* translation() const
    { return new Vector3( t->translation() ); }

    Quaternion* rotation() const
    { return new Quaternion( Eigen::Quaterniond(t->linear()) ); }

    Isometry3* concatenate(Isometry3 const& other) const
    { return new Isometry3( *t * *other.t ); }

    Vector3* transform(Vector3 const& other) const
    { return new Vector3( *t * *other.v ); }

    MatrixX* matrix() const
    { return new MatrixX( t->matrix() ); }

    void translate( Vector3 const& other ) const
    { t->translate( *other.v ); }

    void pretranslate( Vector3 const& other ) const
    { t->pretranslate( *other.v ); }

    void rotate( Quaternion const& other ) const
    { t->rotate( *other.q ); }

    void prerotate( Quaternion const& other ) const
    { t->prerotate( *other.q ); }

    bool operator ==(Isometry3 const& other) const
    { return (*this->t).matrix() == (*other.t).matrix(); }

    bool isApprox(Isometry3 const& other, double tolerance)
    { return t->isApprox(*other.t, tolerance); }
};

struct Affine3
{
    Affine3d *t;

    Affine3() : t(new Affine3d()) { t->setIdentity(); }
    Affine3(const Affine3& _m) : t(new Affine3d(*_m.t)) {}
    Affine3(const Affine3d& _m) : t(new Affine3d(_m)) {}
    ~Affine3() { delete t; }

    Affine3* inverse() const
    { return new Affine3( t->inverse() ); }

    Vector3* translation() const
    { return new Vector3( t->translation() ); }

    Quaternion* rotation() const
    { return new Quaternion( Eigen::Quaterniond(t->linear()) ); }

    Affine3* concatenate(Affine3 const& other) const
    { return new Affine3( *t * *other.t ); }

    Vector3* transform(Vector3 const& other) const
    { return new Vector3( *t * *other.v ); }

    MatrixX* matrix() const
    { return new MatrixX( t->matrix() ); }

    void translate( Vector3 const& other ) const
    { t->translate( *other.v ); }

    void pretranslate( Vector3 const& other ) const
    { t->pretranslate( *other.v ); }

    void rotate( Quaternion const& other ) const
    { t->rotate( *other.q ); }

    void prerotate( Quaternion const& other ) const
    { t->prerotate( *other.q ); }

    bool operator ==(Affine3 const& other) const
    { return (*this->t).matrix() == (*other.t).matrix(); }

    bool isApprox(Affine3 const& other, double tolerance)
    { return t->isApprox(*other.t, tolerance); }
};


// The initialization method for this module
void Init_eigen_ext()
{
     Rice::Module rb_mEigen = define_module("Eigen");

     Data_Type<Vector3> rb_Vector3 = define_class_under<Vector3>(rb_mEigen, "Vector3")
       .define_constructor(Constructor<Vector3,double,double,double>(),
               (Arg("x") = static_cast<double>(0),
               Arg("y") = static_cast<double>(0),
               Arg("z") = static_cast<double>(0)))
       .define_method("__equal__",  &Vector3::operator ==)
       .define_method("norm",  &Vector3::norm)
       .define_method("normalize!",  &Vector3::normalizeBang)
       .define_method("normalize",  &Vector3::normalize)
       .define_method("[]",  &Vector3::get)
       .define_method("[]=",  &Vector3::set)
       .define_method("x",  &Vector3::x)
       .define_method("y",  &Vector3::y)
       .define_method("z",  &Vector3::z)
       .define_method("x=", &Vector3::setX)
       .define_method("y=", &Vector3::setY)
       .define_method("z=", &Vector3::setZ)
       .define_method("+",  &Vector3::operator +)
       .define_method("-",  &Vector3::operator -)
       .define_method("/",  &Vector3::operator /)
       .define_method("-@", &Vector3::negate)
       .define_method("*",  &Vector3::scale)
       .define_method("cross", &Vector3::cross)
       .define_method("dot",  &Vector3::dot)
       .define_method("approx?", &Vector3::isApprox, (Arg("v"), Arg("tolerance") = Eigen::NumTraits<double>::dummy_precision()));

     Data_Type<Quaternion> rb_Quaternion = define_class_under<Quaternion>(rb_mEigen, "Quaternion")
       .define_constructor(Constructor<Quaternion,double,double,double,double>())
       .define_method("__equal__", &Quaternion::operator ==)
       .define_method("w",  &Quaternion::w)
       .define_method("x",  &Quaternion::x)
       .define_method("y",  &Quaternion::y)
       .define_method("z",  &Quaternion::z)
       .define_method("w=", &Quaternion::setW)
       .define_method("x=", &Quaternion::setX)
       .define_method("y=", &Quaternion::setY)
       .define_method("z=", &Quaternion::setZ)
       .define_method("norm", &Quaternion::norm)
       .define_method("concatenate", &Quaternion::concatenate)
       .define_method("inverse", &Quaternion::inverse)
       .define_method("transform", &Quaternion::transform)
       .define_method("matrix", &Quaternion::matrix)
       .define_method("normalize!", &Quaternion::normalizeBang)
       .define_method("normalize", &Quaternion::normalize)
       .define_method("approx?", &Quaternion::isApprox, (Arg("q"), Arg("tolerance") = Eigen::NumTraits<double>::dummy_precision()))
       .define_method("to_euler", &Quaternion::toEuler)
       .define_method("from_euler", &Quaternion::fromEuler)
       .define_method("from_angle_axis", &Quaternion::fromAngleAxis)
       .define_method("from_matrix", &Quaternion::fromMatrix);

     Data_Type<AngleAxis> rb_AngleAxis = define_class_under<AngleAxis>(rb_mEigen, "AngleAxis")
       .define_constructor(Constructor<AngleAxis,double,Vector3 const&>())
       .define_method("__equal__", &AngleAxis::operator ==)
       .define_method("angle",  &AngleAxis::angle)
       .define_method("axis",  &AngleAxis::axis)
       .define_method("concatenate", &AngleAxis::concatenate)
       .define_method("inverse", &AngleAxis::inverse)
       .define_method("transform", &AngleAxis::transform)
       .define_method("matrix", &AngleAxis::matrix)
       .define_method("approx?", &AngleAxis::isApprox, (Arg("q"), Arg("tolerance") = Eigen::NumTraits<double>::dummy_precision()))
       .define_method("to_euler", &AngleAxis::toEuler)
       .define_method("from_euler", &AngleAxis::fromEuler)
       .define_method("from_quaternion", &AngleAxis::fromQuaternion)
       .define_method("from_matrix", &AngleAxis::fromMatrix);

     Data_Type<VectorX> rb_VectorX = define_class_under<VectorX>(rb_mEigen, "VectorX")
       .define_constructor(Constructor<VectorX,int>(),
               (Arg("rows") = static_cast<int>(0)))
       .define_method("resize", &VectorX::resize)
       .define_method("__equal__",  &VectorX::operator ==)
       .define_method("norm",  &VectorX::norm)
       .define_method("normalize!",  &VectorX::normalizeBang)
       .define_method("normalize",  &VectorX::normalize)
       .define_method("size", &VectorX::size)
       .define_method("[]",  &VectorX::get)
       .define_method("[]=",  &VectorX::set)
       .define_method("+",  &VectorX::operator +)
       .define_method("-",  &VectorX::operator -)
       .define_method("/",  &VectorX::operator /)
       .define_method("-@", &VectorX::negate)
       .define_method("*",  &VectorX::scale)
       .define_method("dot",  &VectorX::dot)
       .define_method("approx?", &VectorX::isApprox, (Arg("v"), Arg("tolerance") = Eigen::NumTraits<double>::dummy_precision()));

     Data_Type<Matrix4> rb_Matrix4 = define_class_under<Matrix4>(rb_mEigen, "Matrix4")
       .define_constructor(Constructor<Matrix4>())
       .define_method("__equal__",  &Matrix4::operator ==)
       .define_method("T", &Matrix4::transpose)
       .define_method("norm",  &Matrix4::norm)
       .define_method("rows", &Matrix4::rows)
       .define_method("cols", &Matrix4::cols)
       .define_method("size", &Matrix4::size)
       .define_method("[]",  &Matrix4::get)
       .define_method("[]=",  &Matrix4::set)
       .define_method("+",  &Matrix4::operator +)
       .define_method("-",  &Matrix4::operator -)
       .define_method("/",  &Matrix4::operator /)
       .define_method("-@", &Matrix4::negate)
       .define_method("*",  &Matrix4::scale)
       .define_method("dotM",  &Matrix4::dotM)
       .define_method("approx?", &Matrix4::isApprox, (Arg("m"), Arg("tolerance") = Eigen::NumTraits<double>::dummy_precision()));

     rb_mEigen.const_set("ComputeFullU", INT2FIX(Eigen::ComputeFullU));
     rb_mEigen.const_set("ComputeThinU", INT2FIX(Eigen::ComputeThinU));
     rb_mEigen.const_set("ComputeThinV", INT2FIX(Eigen::ComputeThinV));

     Data_Type<JacobiSVD> rb_JacobiSVD = define_class_under<JacobiSVD>(rb_mEigen, "JacobiSVD")
        .define_method("solve", &JacobiSVD::solve);

     Data_Type<MatrixX> rb_MatrixX = define_class_under<MatrixX>(rb_mEigen, "MatrixX")
       .define_constructor(Constructor<MatrixX,int,int>(),
               (Arg("rows") = static_cast<int>(0),
                Arg("cols") = static_cast<int>(0)))
       .define_method("resize", &MatrixX::resize)
       .define_method("__equal__",  &MatrixX::operator ==)
       .define_method("T", &MatrixX::transpose)
       .define_method("norm",  &MatrixX::norm)
       .define_method("rows", &MatrixX::rows)
       .define_method("cols", &MatrixX::cols)
       .define_method("size", &MatrixX::size)
       .define_method("[]",  &MatrixX::get)
       .define_method("[]=",  &MatrixX::set)
       .define_method("row", &MatrixX::getRow)
       .define_method("setRow", &MatrixX::setRow)
       .define_method("col", &MatrixX::getColumn)
       .define_method("setCol", &MatrixX::setColumn)
       .define_method("+",  &MatrixX::operator +)
       .define_method("-",  &MatrixX::operator -)
       .define_method("/",  &MatrixX::operator /)
       .define_method("-@", &MatrixX::negate)
       .define_method("*",  &MatrixX::scale)
       .define_method("dotV",  &MatrixX::dotV)
       .define_method("dotM",  &MatrixX::dotM)
       .define_method("jacobiSvd", &MatrixX::jacobiSvd, (Arg("flags") = 0))
       .define_method("approx?", &MatrixX::isApprox, (Arg("m"), Arg("tolerance") = Eigen::NumTraits<double>::dummy_precision()));

     Data_Type<Isometry3> rb_Isometry3 = define_class_under<Isometry3>(rb_mEigen, "Isometry3")
       .define_constructor(Constructor<Isometry3>())
       .define_method("__equal__",  &Isometry3::operator ==)
       .define_method("approx?", &Isometry3::isApprox, (Arg("i"), Arg("tolerance") = Eigen::NumTraits<double>::dummy_precision()))
       .define_method("inverse", &Isometry3::inverse)
       .define_method("translation", &Isometry3::translation)
       .define_method("rotation", &Isometry3::rotation)
       .define_method("concatenate", &Isometry3::concatenate)
       .define_method("transform", &Isometry3::transform)
       .define_method("matrix", &Isometry3::matrix)
       .define_method("translate", &Isometry3::translate)
       .define_method("pretranslate", &Isometry3::pretranslate)
       .define_method("rotate", &Isometry3::rotate)
       .define_method("prerotate", &Isometry3::prerotate);

     Data_Type<Affine3> rb_Affine3 = define_class_under<Affine3>(rb_mEigen, "Affine3")
       .define_constructor(Constructor<Affine3>())
       .define_method("__equal__",  &Affine3::operator ==)
       .define_method("approx?", &Affine3::isApprox, (Arg("i"), Arg("tolerance") = Eigen::NumTraits<double>::dummy_precision()))
       .define_method("inverse", &Affine3::inverse)
       .define_method("translation", &Affine3::translation)
       .define_method("rotation", &Affine3::rotation)
       .define_method("concatenate", &Affine3::concatenate)
       .define_method("transform", &Affine3::transform)
       .define_method("matrix", &Affine3::matrix)
       .define_method("translate", &Affine3::translate)
       .define_method("pretranslate", &Affine3::pretranslate)
       .define_method("rotate", &Affine3::rotate)
       .define_method("prerotate", &Affine3::prerotate);
}

