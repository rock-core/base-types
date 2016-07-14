#ifndef __BASE_TWIST_WITH_COVARIANCE_HPP__
#define __BASE_TWIST_WITH_COVARIANCE_HPP__

#include <iomanip> // std::setprecision

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <base/Float.hpp>
#include <base/Eigen.hpp>

namespace base {

    // Guarantee Semi-Positive Definite (SPD) matrix.
    template <typename _MatrixType>
    static _MatrixType guaranteeSPD (const _MatrixType &A)
    {
        _MatrixType spdA;
        Eigen::VectorXd s;
        s.resize(A.rows(), 1);

        /**
        * Single Value Decomposition
        */
        Eigen::JacobiSVD <Eigen::MatrixXd > svdOfA (A, Eigen::ComputeThinU | Eigen::ComputeThinV);

        s = svdOfA.singularValues(); //!eigenvalues

        for (register int i=0; i<s.size(); ++i)
        {
            if (s(i) < 0.00)
                s(i) = 0.00;
        }
        spdA = svdOfA.matrixU() * s.matrix().asDiagonal() * svdOfA.matrixV().transpose();

        return spdA;
    };

    
    struct TwistWithCovariance
    {
    public:
	    typedef base::Matrix6d Covariance;

    public:

        /** Velocity in m/s **/
        base::Vector3d vel;

        /** Rotation rate in rad/s **/
        base::Vector3d rot;

        /** Uncertainty **/
        Covariance cov;

    public:
        explicit TwistWithCovariance (const base::Vector3d& vel = base::Vector3d::Zero(), const base::Vector3d& rot = base::Vector3d::Zero() ):
             vel(vel), rot(rot){this->invalidateCovariance(); };

        TwistWithCovariance(const base::Vector3d& vel, const base::Vector3d& rot, const Covariance& cov):
             vel(vel), rot(rot), cov(cov) {};

        TwistWithCovariance(const base::Vector6d& velocity, const Covariance& cov)
        {
            this->setVelocity(velocity);
            this->cov = cov;
        }

        /** Default std::cout function **/
        friend std::ostream & operator<<(std::ostream &out, const base::Vector6d& vel);

        /** Get and Set Methods **/
        const base::Vector3d& getTranslation() const { return this->vel; }
        void setTranslation(const base::Vector3d& vel) { this->vel = vel; }

        const base::Vector3d& getRotation() const { return this->rot; }
        void setRotation(const base::Vector3d& rot) { this->rot = rot; }

        const Covariance& getCovariance() const { return this->cov; }
        void setCovariance(const Covariance& cov) { this->cov = cov; }

        const base::Matrix3d getLinearVelocityCov() const { return this->cov.block<3,3>(0,0); }
        void setLinearVelocityCov(const base::Matrix3d& cov) { this->cov.block<3,3>(0,0) = cov; }
        const base::Matrix3d getAngularVelocityCov() const { return this->cov.block<3,3>(3,3); }
        void setAngularVelocityCov(const base::Matrix3d& cov) { this->cov.block<3,3>(3,3) = cov; }

        const base::Vector3d& getLinearVelocity() const {return this->getTranslation(); }
        void setLinearVelocity(const base::Vector3d& vel) { return this->setTranslation(vel); }
        const base::Vector3d& getAngularVelocity() const {return this->getRotation(); }
        void setAngularVelocity(const base::Vector3d& rot) { return this->setRotation(rot); }

        const base::Vector3d& translation() const {return this->getTranslation(); }
        const base::Vector3d& rotation() const {return this->getRotation(); }

        const base::Vector6d getVelocity() const
        {
            base::Vector6d all_velocities;
            all_velocities.block<3,1>(0,0) = this->vel;
            all_velocities.block<3,1>(3,0) = this->rot;
            return all_velocities;
        }

        void setVelocity(const base::Vector6d& velocity)
        {
            /** Linear velocity at first place, Angular velocity at second place **/
            this->vel = velocity.block<3,1>(0,0);
            this->rot = velocity.block<3,1>(3,0);
        }

        /** Check Methods **/
        bool hasValidVelocity() const
        {
            return this->vel.allFinite() && this->rot.allFinite();
        }

        void invalidateVelocity()
        {
            this->vel = base::Vector3d::Ones() * base::unknown<double>();
            this->rot = base::Vector3d::Ones() * base::unknown<double>();
        }

        bool hasValidCovariance() const { return this->cov.allFinite(); }
        void invalidateCovariance()
        {
            this->cov = Covariance::Ones() * base::unknown<double>();
        }

        void invalidate()
        {
            this->invalidateVelocity();
            this->invalidateCovariance();
        }

        static TwistWithCovariance Zero()
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
            if (this->hasValidCovariance() && arg.hasValidCovariance())
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
            if (this->hasValidCovariance() && arg.hasValidCovariance())
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
            if (!lhs.hasValidCovariance())
            {
                return TwistWithCovariance(static_cast<base::Vector3d>(lhs.vel*rhs), static_cast<base::Vector3d>(lhs.rot*rhs));
            }
            else
            {
                return TwistWithCovariance(static_cast<base::Vector3d>(lhs.vel*rhs), static_cast<base::Vector3d>(lhs.rot*rhs), static_cast<Covariance>((rhs*rhs)*lhs.cov));
            }
        }

        inline friend TwistWithCovariance operator*(double lhs, const TwistWithCovariance& rhs)
        {
            if (!rhs.hasValidCovariance())
            {
                return TwistWithCovariance(static_cast<base::Vector3d>(lhs*rhs.vel), static_cast<base::Vector3d>(lhs*rhs.rot));
            }
            else
            {
                return TwistWithCovariance(static_cast<base::Vector3d>(lhs*rhs.vel), static_cast<base::Vector3d>(lhs*rhs.rot), static_cast<Covariance>((lhs*lhs)*rhs.cov));
            }
        }


        //Spatial Cross product
        inline friend TwistWithCovariance operator*(const TwistWithCovariance& lhs, const TwistWithCovariance& rhs)
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
                cross_cov << lhs.cov.block<3,3>(3,3), base::Matrix3d::Zero(),
                            base::Matrix3d::Zero(), rhs.cov.block<3,3>(0,0);

                /** Linear velocity is at the first covariance block **/
                tmp.cov.block<3,3>(0,0) = cross_jacob * cross_cov * cross_jacob.transpose();

                cross_jacob = TwistWithCovariance::crossJacobian(lhs.vel, rhs.rot);
                cross_cov << lhs.cov.block<3,3>(0,0), base::Matrix3d::Zero(),
                           base::Matrix3d::Zero(),rhs.cov.block<3,3>(3,3);

                /** Linear velocity is at the first covariance block **/
                tmp.cov.block<3,3>(0,0) += cross_jacob * cross_cov * cross_jacob.transpose();

                cross_jacob = TwistWithCovariance::crossJacobian(lhs.rot, rhs.rot);
                cross_cov << lhs.cov.block<3,3>(3,3), base::Matrix3d::Zero(),
                           base::Matrix3d::Zero(),rhs.cov.block<3,3>(3,3);

                /** Angular velocity is at the first covariance block **/
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
            return TwistWithCovariance(static_cast<base::Vector3d>(-arg.vel), static_cast<base::Vector3d>(-arg.rot), arg.cov);
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
    };

} // namespaces

#endif
