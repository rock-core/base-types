#include "TwistWithCovariance.hpp"

typedef base::TwistWithCovariance::Covariance Covariance;

base::TwistWithCovariance::TwistWithCovariance(const base::Vector3d& vel, const base::Vector3d& rot)
    : vel(vel), rot(rot)
{
    this->invalidateCovariance();
}

base::TwistWithCovariance::TwistWithCovariance(const base::Vector3d& vel, const base::Vector3d& rot, const base::TwistWithCovariance::Covariance& cov)
    : vel(vel), rot(rot), cov(cov)
{

}

base::TwistWithCovariance::TwistWithCovariance(const base::Vector6d& velocity, const base::TwistWithCovariance::Covariance& cov)
{
    this->setVelocity(velocity);
    this->cov = cov;
}

const base::Vector3d& base::TwistWithCovariance::getTranslation() const
{
    return this->vel;
}

void base::TwistWithCovariance::setTranslation(const base::Vector3d& vel)
{
    this->vel = vel;
}

const base::Vector3d& base::TwistWithCovariance::getRotation() const
{
    return this->rot;
}

void base::TwistWithCovariance::setRotation(const base::Vector3d& rot)
{
    this->rot = rot;
}

const base::TwistWithCovariance::Covariance& base::TwistWithCovariance::getCovariance() const
{
    return this->cov;
}

void base::TwistWithCovariance::setCovariance(const base::TwistWithCovariance::Covariance& cov)
{
    this->cov = cov;
}

const base::Matrix3d base::TwistWithCovariance::getLinearVelocityCov() const
{
    return this->cov.block<3,3>(0,0);
}

void base::TwistWithCovariance::setLinearVelocityCov(const base::Matrix3d& cov)
{
    this->cov.block<3,3>(0,0) = cov;
}

const base::Matrix3d base::TwistWithCovariance::getAngularVelocityCov() const
{
    return this->cov.block<3,3>(3,3);
}

void base::TwistWithCovariance::setAngularVelocityCov(const base::Matrix3d& cov)
{
    this->cov.block<3,3>(3,3) = cov;
}

const base::Vector3d& base::TwistWithCovariance::getLinearVelocity() const
{
    return this->getTranslation(); 
}

void base::TwistWithCovariance::setLinearVelocity(const base::Vector3d& vel)
{
    return this->setTranslation(vel);
}

const base::Vector3d& base::TwistWithCovariance::getAngularVelocity() const
{
    return this->getRotation();
}

void base::TwistWithCovariance::setAngularVelocity(const base::Vector3d& rot)
{
    return this->setRotation(rot);
}

const base::Vector3d& base::TwistWithCovariance::translation() const
{
    return this->getTranslation();
}

const base::Vector3d& base::TwistWithCovariance::rotation() const
{
    return this->getRotation();
}

const base::Vector6d base::TwistWithCovariance::getVelocity() const
{
    base::Vector6d all_velocities;
    all_velocities.block<3,1>(0,0) = this->vel;
    all_velocities.block<3,1>(3,0) = this->rot;
    return all_velocities;
}

void base::TwistWithCovariance::setVelocity(const base::Vector6d& velocity)
{
    /** Linear velocity at first place, Angular velocity at second place **/
    this->vel = velocity.block<3,1>(0,0);
    this->rot = velocity.block<3,1>(3,0);
}

bool base::TwistWithCovariance::hasValidVelocity() const
{
    return this->vel.allFinite() && this->rot.allFinite();
}

void base::TwistWithCovariance::invalidateVelocity()
{
    this->vel = base::Vector3d::Ones() * base::unknown<double>();
    this->rot = base::Vector3d::Ones() * base::unknown<double>();
}

bool base::TwistWithCovariance::hasValidCovariance() const
{
    return this->cov.allFinite();
}

void base::TwistWithCovariance::invalidateCovariance()
{
    this->cov = Covariance::Ones() * base::unknown<double>();
}

void base::TwistWithCovariance::invalidate()
{
    this->invalidateVelocity();
    this->invalidateCovariance();
}

base::TwistWithCovariance base::TwistWithCovariance::Zero()
{
    return TwistWithCovariance(static_cast<const base::Vector3d>(base::Vector3d::Zero()), static_cast<const base::Vector3d>(base::Vector3d::Zero()));
}

double& base::TwistWithCovariance::operator[](int i)
{
    if (i<3)
        return this->vel(i);
    else
        return this->rot(i-3);
}

double base::TwistWithCovariance::operator[](int i) const
{
    if (i<3)
        return this->vel(i);
    else
        return this->rot(i-3);
}

base::TwistWithCovariance& base::TwistWithCovariance::operator+=(const base::TwistWithCovariance& arg)
{
    this->vel += arg.vel;
    this->rot += arg.rot;
    if (this->hasValidCovariance() && arg.hasValidCovariance())
    {
        this->cov += arg.cov;
        base::guaranteeSPD<Covariance>(this->cov);
    }
    return *this;
}

base::TwistWithCovariance operator+(base::TwistWithCovariance lhs, const base::TwistWithCovariance& rhs)
{
     return lhs += rhs;
}

base::TwistWithCovariance& base::TwistWithCovariance::operator-=(const base::TwistWithCovariance& arg)
{
    this->vel -= arg.vel;
    this->rot -= arg.rot;
    if (this->hasValidCovariance() && arg.hasValidCovariance())
    {
        this->cov += arg.cov;
        base::guaranteeSPD<Covariance>(this->cov);
    }

    return *this;
}

base::TwistWithCovariance operator-(base::TwistWithCovariance lhs, const base::TwistWithCovariance& rhs)
{
     return lhs -= rhs;
}

base::TwistWithCovariance operator*(const base::TwistWithCovariance& lhs, double rhs)
{
    if (!lhs.hasValidCovariance())
    {
        return base::TwistWithCovariance(static_cast<base::Vector3d>(lhs.vel*rhs), static_cast<base::Vector3d>(lhs.rot*rhs));
    }
    else
    {
        return base::TwistWithCovariance(static_cast<base::Vector3d>(lhs.vel*rhs), static_cast<base::Vector3d>(lhs.rot*rhs), static_cast<Covariance>((rhs*rhs)*lhs.cov));
    }
}

base::TwistWithCovariance operator*(double lhs, const base::TwistWithCovariance& rhs)
{
    if (!rhs.hasValidCovariance())
    {
        return base::TwistWithCovariance(static_cast<base::Vector3d>(lhs*rhs.vel), static_cast<base::Vector3d>(lhs*rhs.rot));
    }
    else
    {
        return base::TwistWithCovariance(static_cast<base::Vector3d>(lhs*rhs.vel), static_cast<base::Vector3d>(lhs*rhs.rot), static_cast<Covariance>((lhs*lhs)*rhs.cov));
    }
}

base::TwistWithCovariance operator*(const base::TwistWithCovariance& lhs, const base::TwistWithCovariance& rhs)
{
    base::TwistWithCovariance tmp;
    tmp.vel = lhs.rot.cross(rhs.vel)+lhs.vel.cross(rhs.rot);
    tmp.rot = lhs.rot.cross(rhs.rot);

    /** In case the two twist have uncertainty **/
    if (lhs.hasValidCovariance() && rhs.hasValidCovariance())
    {
        Eigen::Matrix<double, 3, 6> cross_jacob;
        Eigen::Matrix<double, 6, 6> cross_cov;

        /** Initialize covariance **/
        tmp.cov.setZero();

        cross_jacob = base::TwistWithCovariance::crossJacobian(lhs.rot, rhs.vel);
        cross_cov << lhs.cov.block<3,3>(3,3), base::Matrix3d::Zero(),
                    base::Matrix3d::Zero(), rhs.cov.block<3,3>(0,0);

        /** Linear velocity is at the first covariance block **/
        tmp.cov.block<3,3>(0,0) = cross_jacob * cross_cov * cross_jacob.transpose();

        cross_jacob = base::TwistWithCovariance::crossJacobian(lhs.vel, rhs.rot);
        cross_cov << lhs.cov.block<3,3>(0,0), base::Matrix3d::Zero(),
                    base::Matrix3d::Zero(),rhs.cov.block<3,3>(3,3);

        /** Linear velocity is at the first covariance block **/
        tmp.cov.block<3,3>(0,0) += cross_jacob * cross_cov * cross_jacob.transpose();

        cross_jacob = base::TwistWithCovariance::crossJacobian(lhs.rot, rhs.rot);
        cross_cov << lhs.cov.block<3,3>(3,3), base::Matrix3d::Zero(),
                    base::Matrix3d::Zero(),rhs.cov.block<3,3>(3,3);

        /** Angular velocity is at the first covariance block **/
        tmp.cov.block<3,3>(3,3) = cross_jacob * cross_cov * cross_jacob.transpose();

        base::guaranteeSPD<Covariance>(tmp.cov);
    }

    return tmp;
}

base::TwistWithCovariance operator/(const base::TwistWithCovariance& lhs, double rhs)
{
     return base::TwistWithCovariance(static_cast<base::Vector3d>(lhs.vel/rhs), static_cast<base::Vector3d>(lhs.rot/rhs), static_cast<Covariance>((1.0/(rhs *rhs))*lhs.cov));
}

base::TwistWithCovariance operator-(const base::TwistWithCovariance& arg)
{
    return base::TwistWithCovariance(static_cast<base::Vector3d>(-arg.vel), static_cast<base::Vector3d>(-arg.rot), arg.cov);
}

Eigen::Matrix< double, int(3), int(6) > base::TwistWithCovariance::crossJacobian(const base::Vector3d& u, const base::Vector3d& v)
{
    Eigen::Matrix<double, 3, 6> cross_jacob;
    base::Matrix3d cross_u, cross_v;
    cross_u << 0.0, -u[2], u[1], u[2], 0.0, -u[0], -u[1], u[0], 0.0;
    cross_v << 0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0;
    cross_jacob << cross_u, cross_v;
    return cross_jacob;
}

std::ostream& base::operator<<(std::ostream& out, const base::TwistWithCovariance& twist)
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
    out.unsetf(std::ios_base::floatfield);
    return out;
}

























