#ifndef __BASE_TWIST_WITH_COVARIANCE_HPP__
#define __BASE_TWIST_WITH_COVARIANCE_HPP__

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <base/Float.hpp>
#include <base/Eigen.hpp>

namespace base {

    struct TwistWithCovariance
    {
    public:
	    typedef base::Matrix6d Covariance;

    public:
        /** Veloicty in m/s **/
        base::Vector3d vel;

        /** Rotation rate in rad/s **/
        base::Vector3d rot;

        /** Uncertainty **/
        Covariance cov;

    public:
        explicit TwistWithCovariance ( const base::Vector3d& vel = base::Vector3d::Zero(),  const base::Vector3d& rot = base::Vector3d::Zero() ):
            vel(vel), rot(rot) {this->invalidateUncertainty(); };

        TwistWithCovariance(const base::Vector3d& vel, const base::Vector3d& rot, const Covariance& cov):
            vel(vel), rot(rot), cov(cov) {};

        TwistWithCovariance(const base::Vector6d& velocity, const Covariance& cov)
        {
            /** Linear velocity at first place, Angular velocity at second place **/
            this->vel = velocity.block<3,1>(0,0);
            this->rot = velocity.block<3,1>(3,0);
            this->cov = cov;
        }

        /** Default std::cout function **/
        friend std::ostream & operator<<(std::ostream &out, const base::Vector6d& vel);

        /** Get and Set Methods **/
        const base::Vector3d& getTranslation() const { return this->vel; }
        void setTranslation(const base::Vector3d& trans) { this->vel = trans; }

        const base::Vector3d& getRotation() const { return this->rot; }
        void setRotation(const base::Vector3d& rot) { this->rot = rot; }

        const Covariance& getCovariance() const { return this->cov; }
        void setCovariance( const Covariance& cov ) { this->cov = cov; }

        const base::Vector3d& getLinearVelocity() const {return this->getTranslation(); }
        void setLinearVelocity(const base::Vector3d& trans) { return this->setTranslation(trans); }
        const base::Vector3d& getAngularVelocity() const {return this->getRotation(); }
        void setAngularVelocity(const base::Vector3d& rot) { return this->setRotation(rot); }

        const base::Vector3d& translation() const {return this->getTranslation(); }
        const base::Vector3d& rotation() const {return this->getRotation(); }

        void setVelocity(const base::Vector6d& velocity)
        {
            this->vel = velocity.block<3,1>(0,0);
            this->rot = velocity.block<3,1>(3,0);
        }

        /** Check Methods **/
        bool hasValidVelocity() const
        {
            return base::isnotnan(this->vel) && base::isnotnan(this->rot);
        }

        void invalidateVelocity()
        {
            this->vel = base::Vector3d::Ones() * base::unknown<double>();
            this->rot = base::Vector3d::Ones() * base::unknown<double>();
        }

        bool hasValidUncertainty() const { return base::isnotnan(this->cov); }
        void invalidateUncertainty()
        {
            this->cov = Covariance::Ones() * base::unknown<double>();
        }

        TwistWithCovariance Zero()
        {
            return TwistWithCovariance(static_cast<const base::Vector3d>(base::Vector3d::Zero()), static_cast<const base::Vector3d>(base::Vector3d::Zero()));
        }

        double& operator[](int i)
        {
            if (i<3)
                return this->vel(i);
            else
                return this->rot(i-3);
        }

        double operator[](int i) const
        {
            if (i<3)
                return this->vel(i);
            else
                return this->rot(i-3);
        }


        inline TwistWithCovariance& operator+=(const TwistWithCovariance& arg)
        {
            this->vel += arg.vel;
            this->rot += arg.rot;
            if (this->hasValidUncertainty() && arg.hasValidUncertainty())
            {
                this->cov += arg.cov;
                base::guaranteeSPD<Covariance>(this->cov);
            }
            return *this;
        }

        inline friend TwistWithCovariance operator+(TwistWithCovariance lhs,const TwistWithCovariance& rhs)
        {
            return lhs += rhs;
        }

        inline TwistWithCovariance& operator-=(const TwistWithCovariance& arg)
        {
            this->vel -= arg.vel;
            this->rot -= arg.rot;
            if (this->hasValidUncertainty() && arg.hasValidUncertainty())
            {
                this->cov += arg.cov;
                base::guaranteeSPD<Covariance>(this->cov);
            }

            return *this;
        }

        inline friend TwistWithCovariance operator-(TwistWithCovariance lhs, const TwistWithCovariance& rhs)
        {
            return lhs -= rhs;
        }

        inline friend TwistWithCovariance operator*(const TwistWithCovariance& lhs,double rhs)
        {
            if (!lhs.hasValidUncertainty())
            {
                return TwistWithCovariance(static_cast<base::Vector3d>(lhs.vel*rhs),static_cast<base::Vector3d>(lhs.rot*rhs));
            }
            else
            {
                return TwistWithCovariance(static_cast<base::Vector3d>(lhs.vel*rhs),static_cast<base::Vector3d>(lhs.rot*rhs), static_cast<Covariance>((rhs*rhs)*lhs.cov));
            }
        }

        inline friend TwistWithCovariance operator*(double lhs, const TwistWithCovariance& rhs)
        {
            if (!rhs.hasValidUncertainty())
            {
                return TwistWithCovariance(static_cast<base::Vector3d>(lhs*rhs.vel),static_cast<base::Vector3d>(lhs*rhs.rot));
            }
            else
            {
                return TwistWithCovariance(static_cast<base::Vector3d>(lhs*rhs.vel),static_cast<base::Vector3d>(lhs*rhs.rot), static_cast<Covariance>((lhs*lhs)*rhs.cov));
            }
        }


        //Spatial Cross product
        inline friend TwistWithCovariance operator*(const TwistWithCovariance& lhs, const TwistWithCovariance& rhs)
        {
            TwistWithCovariance tmp;
            tmp.vel = lhs.rot.cross(rhs.vel)+lhs.vel.cross(rhs.rot);
            tmp.rot = lhs.rot.cross(rhs.rot);

            /** In case the two twist have uncertainty **/
            if (lhs.hasValidUncertainty() && rhs.hasValidUncertainty())
            {
                Eigen::Matrix<double, 3, 6> cross_jacob;
                Eigen::Matrix<double, 6, 6> cross_cov;

                /** Initialize covariance **/
                tmp.cov.setZero();

                cross_jacob = TwistWithCovariance::crossJacobian(lhs.rot, rhs.vel);
                cross_cov << lhs.cov.block<3,3>(3,3), base::Matrix3d::Zero(),
                            base::Matrix3d::Zero(), rhs.cov.block<3,3>(0,0);

                /** Linear fist block **/
                tmp.cov.block<3,3>(0,0) = cross_jacob * cross_cov * cross_jacob.transpose();

                cross_jacob = TwistWithCovariance::crossJacobian(lhs.vel, rhs.rot);
                cross_cov << lhs.cov.block<3,3>(0,0), base::Matrix3d::Zero(),
                           base::Matrix3d::Zero(),rhs.cov.block<3,3>(3,3);

                /** Linear fist block **/
                tmp.cov.block<3,3>(0,0) += cross_jacob * cross_cov * cross_jacob.transpose();

                cross_jacob = TwistWithCovariance::crossJacobian(lhs.rot, rhs.rot);
                cross_cov << lhs.cov.block<3,3>(3,3), base::Matrix3d::Zero(),
                           base::Matrix3d::Zero(),rhs.cov.block<3,3>(3,3);

                /** Angular second block **/
                tmp.cov.block<3,3>(3,3) = cross_jacob * cross_cov * cross_jacob.transpose();

                base::guaranteeSPD<Covariance>(tmp.cov);
            }

            return tmp;
        }

        inline friend TwistWithCovariance operator/(const TwistWithCovariance& lhs,double rhs)
        {
            return TwistWithCovariance(static_cast<base::Vector3d>(lhs.vel/rhs), static_cast<base::Vector3d>(lhs.rot/rhs), static_cast<Covariance>((1.0/(rhs *rhs))*lhs.cov));
        }


        /** unary - **/
        inline friend TwistWithCovariance operator-(const TwistWithCovariance& arg)
        {
            return TwistWithCovariance(static_cast<base::Vector3d>(-arg.vel),static_cast<base::Vector3d>(-arg.rot), arg.cov);
        }

        static Eigen::Matrix<double, 3, 6> crossJacobian(const base::Vector3d& u, const base::Vector3d& v)
        {
            Eigen::Matrix<double, 3, 6> cross_jacob;
            base::Matrix3d cross_u, cross_v;
            cross_u << 0.0, -u[2], u[1], u[2], 0.0, -u[0], -u[1], u[0], 0.0;
            cross_v << 0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0;
            cross_jacob << cross_u, cross_v;
            return cross_jacob;
        }

    };

    /** Default std::cout function
    */
    inline std::ostream & operator<<(std::ostream &out, const base::TwistWithCovariance& twist)
    {
        out << twist.vel <<"\n"<< twist.rot << std::endl;
        if (twist.hasValidUncertainty())
        {
            out << twist.getCovariance() << "\n";
        }
        return out;
    };

} // namespaces

#endif
