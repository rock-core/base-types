#include "TwistWithCovariance.hpp"

#include <iomanip> // std::setprecision
#include <Eigen/Core>   
#include <base/Float.hpp>

#include "Matrix.hpp"

namespace base {


typedef TwistWithCovariance::Covariance Covariance;

TwistWithCovariance::TwistWithCovariance(const Vector3d& vel, const Vector3d& rot)
    : vel(vel), rot(rot)
{
    this->invalidateCovariance();
}

TwistWithCovariance::TwistWithCovariance(const Vector3d& vel, const Vector3d& rot, const TwistWithCovariance::Covariance& cov)
    : vel(vel), rot(rot), cov(cov)
{

}

TwistWithCovariance::TwistWithCovariance(const Vector6d& velocity, const TwistWithCovariance::Covariance& cov)
{
    this->setVelocity(velocity);
    this->cov = cov;
}

const Vector3d& TwistWithCovariance::getTranslation() const
{
    return this->vel;
}

void TwistWithCovariance::setTranslation(const Vector3d& vel)
{
    this->vel = vel;
}

const Vector3d& TwistWithCovariance::getRotation() const
{
    return this->rot;
}

void TwistWithCovariance::setRotation(const Vector3d& rot)
{
    this->rot = rot;
}

const TwistWithCovariance::Covariance& TwistWithCovariance::getCovariance() const
{
    return this->cov;
}

void TwistWithCovariance::setCovariance(const TwistWithCovariance::Covariance& cov)
{
    this->cov = cov;
}

const Matrix3d TwistWithCovariance::getLinearVelocityCov() const
{
    return this->cov.block<3,3>(0,0);
}

void TwistWithCovariance::setLinearVelocityCov(const Matrix3d& cov)
{
    this->cov.block<3,3>(0,0) = cov;
}

const Matrix3d TwistWithCovariance::getAngularVelocityCov() const
{
    return this->cov.block<3,3>(3,3);
}

void TwistWithCovariance::setAngularVelocityCov(const Matrix3d& cov)
{
    this->cov.block<3,3>(3,3) = cov;
}

const Vector3d& TwistWithCovariance::getLinearVelocity() const
{
    return this->getTranslation(); 
}

void TwistWithCovariance::setLinearVelocity(const Vector3d& vel)
{
    return this->setTranslation(vel);
}

const Vector3d& TwistWithCovariance::getAngularVelocity() const
{
    return this->getRotation();
}

void TwistWithCovariance::setAngularVelocity(const Vector3d& rot)
{
    return this->setRotation(rot);
}

const Vector3d& TwistWithCovariance::translation() const
{
    return this->getTranslation();
}

const Vector3d& TwistWithCovariance::rotation() const
{
    return this->getRotation();
}

const Vector6d TwistWithCovariance::getVelocity() const
{
    Vector6d all_velocities;
    all_velocities.block<3,1>(0,0) = this->vel;
    all_velocities.block<3,1>(3,0) = this->rot;
    return all_velocities;
}

void TwistWithCovariance::setVelocity(const Vector6d& velocity)
{
    /** Linear velocity at first place, Angular velocity at second place **/
    this->vel = velocity.block<3,1>(0,0);
    this->rot = velocity.block<3,1>(3,0);
}

bool TwistWithCovariance::hasValidVelocity() const
{
    return this->vel.allFinite() && this->rot.allFinite();
}

void TwistWithCovariance::invalidateVelocity()
{
    this->vel = Vector3d::Ones() * unknown<double>();
    this->rot = Vector3d::Ones() * unknown<double>();
}

bool TwistWithCovariance::hasValidCovariance() const
{
    return this->cov.allFinite();
}

void TwistWithCovariance::invalidateCovariance()
{
    this->cov = Covariance::Ones() * unknown<double>();
}

void TwistWithCovariance::invalidate()
{
    this->invalidateVelocity();
    this->invalidateCovariance();
}

TwistWithCovariance TwistWithCovariance::Zero()
{
    return TwistWithCovariance(static_cast<const Vector3d>(Vector3d::Zero()), static_cast<const Vector3d>(Vector3d::Zero()));
}

double& TwistWithCovariance::operator[](int i)
{
    if (i<3)
        return this->vel(i);
    else
        return this->rot(i-3);
}

double TwistWithCovariance::operator[](int i) const
{
    if (i<3)
        return this->vel(i);
    else
        return this->rot(i-3);
}

TwistWithCovariance& TwistWithCovariance::operator+=(const TwistWithCovariance& arg)
{
    this->vel += arg.vel;
    this->rot += arg.rot;
    if (this->hasValidCovariance() && arg.hasValidCovariance())
    {
        this->cov += arg.cov;
        guaranteeSPD<Covariance>(this->cov);
    }
    return *this;
}

TwistWithCovariance operator+(TwistWithCovariance lhs, const TwistWithCovariance& rhs)
{
     return lhs += rhs;
}

TwistWithCovariance& TwistWithCovariance::operator-=(const TwistWithCovariance& arg)
{
    this->vel -= arg.vel;
    this->rot -= arg.rot;
    if (this->hasValidCovariance() && arg.hasValidCovariance())
    {
        this->cov += arg.cov;
        guaranteeSPD<Covariance>(this->cov);
    }

    return *this;
}

TwistWithCovariance operator-(TwistWithCovariance lhs, const TwistWithCovariance& rhs)
{
     return lhs -= rhs;
}

TwistWithCovariance operator*(const TwistWithCovariance& lhs, double rhs)
{
    if (!lhs.hasValidCovariance())
    {
        return TwistWithCovariance(static_cast<Vector3d>(lhs.vel*rhs), static_cast<Vector3d>(lhs.rot*rhs));
    }
    else
    {
        return TwistWithCovariance(static_cast<Vector3d>(lhs.vel*rhs), static_cast<Vector3d>(lhs.rot*rhs), static_cast<Covariance>((rhs*rhs)*lhs.cov));
    }
}

TwistWithCovariance operator*(double lhs, const TwistWithCovariance& rhs)
{
    if (!rhs.hasValidCovariance())
    {
        return TwistWithCovariance(static_cast<Vector3d>(lhs*rhs.vel), static_cast<Vector3d>(lhs*rhs.rot));
    }
    else
    {
        return TwistWithCovariance(static_cast<Vector3d>(lhs*rhs.vel), static_cast<Vector3d>(lhs*rhs.rot), static_cast<Covariance>((lhs*lhs)*rhs.cov));
    }
}

TwistWithCovariance operator*(const TwistWithCovariance& lhs, const TwistWithCovariance& rhs)
{
    TwistWithCovariance tmp;
    tmp.vel = lhs.rot.cross(rhs.vel)+lhs.vel.cross(rhs.rot);
    tmp.rot = lhs.rot.cross(rhs.rot);

    /** In case the two twist have uncertainty **/
    if (lhs.hasValidCovariance() && rhs.hasValidCovariance())
    {
        Eigen::Matrix<double, 3, 6> cross_jacob;
        Eigen::Matrix<double, 6, 6> cross_cov;

        /** Initialize covariance **/
        tmp.cov.setZero();

        cross_jacob = TwistWithCovariance::crossJacobian(lhs.rot, rhs.vel);
        cross_cov << lhs.cov.block<3,3>(3,3), Matrix3d::Zero(),
                    Matrix3d::Zero(), rhs.cov.block<3,3>(0,0);

        /** Linear velocity is at the first covariance block **/
        tmp.cov.block<3,3>(0,0) = cross_jacob * cross_cov * cross_jacob.transpose();

        cross_jacob = TwistWithCovariance::crossJacobian(lhs.vel, rhs.rot);
        cross_cov << lhs.cov.block<3,3>(0,0), Matrix3d::Zero(),
                    Matrix3d::Zero(),rhs.cov.block<3,3>(3,3);

        /** Linear velocity is at the first covariance block **/
        tmp.cov.block<3,3>(0,0) += cross_jacob * cross_cov * cross_jacob.transpose();

        cross_jacob = TwistWithCovariance::crossJacobian(lhs.rot, rhs.rot);
        cross_cov << lhs.cov.block<3,3>(3,3), Matrix3d::Zero(),
                    Matrix3d::Zero(),rhs.cov.block<3,3>(3,3);

        /** Angular velocity is at the first covariance block **/
        tmp.cov.block<3,3>(3,3) = cross_jacob * cross_cov * cross_jacob.transpose();

        guaranteeSPD<Covariance>(tmp.cov);
    }

    return tmp;
}

TwistWithCovariance operator/(const TwistWithCovariance& lhs, double rhs)
{
     return TwistWithCovariance(static_cast<Vector3d>(lhs.vel/rhs), static_cast<Vector3d>(lhs.rot/rhs), static_cast<Covariance>((1.0/(rhs *rhs))*lhs.cov));
}

TwistWithCovariance operator-(const TwistWithCovariance& arg)
{
    return TwistWithCovariance(static_cast<Vector3d>(-arg.vel), static_cast<Vector3d>(-arg.rot), arg.cov);
}

Eigen::Matrix< double, int(3), int(6) > TwistWithCovariance::crossJacobian(const Vector3d& u, const Vector3d& v)
{
    Eigen::Matrix<double, 3, 6> cross_jacob;
    Matrix3d cross_u, cross_v;
    cross_u << 0.0, -u[2], u[1], u[2], 0.0, -u[0], -u[1], u[0], 0.0;
    cross_v << 0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0;
    cross_jacob << cross_u, cross_v;
    return cross_jacob;
}

std::ostream& operator<<(std::ostream& out, const TwistWithCovariance& twist)
{
    /** cout the 6D twist vector (rotational first and linear second) with its associated covariance matrix **/
    for (register unsigned short i=0; i<twist.getCovariance().rows(); ++i)
    {
        if (i<3)
        {
            out<<std::fixed<<std::setprecision(5)<<twist.vel[i]<<"\t|";
        }
        else
        {
            out<<std::fixed<<std::setprecision(5)<<twist.rot[i-3]<<"\t|";
        }
        for (register unsigned short j=0; j<twist.getCovariance().cols(); ++j)
        {
            out<<std::fixed<<std::setprecision(5)<<twist.getCovariance().row(i)[j]<<"\t";
        }
        out<<"\n";
    }
    out.unsetf(std::ios::floatfield);
    return out;
}



} //end namespace base