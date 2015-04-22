#ifndef __BASE_TWIST_HPP__
#define __BASE_TWIST_HPP__

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <base/Eigen.hpp>

namespace base {

    struct Twist
    {
    public:
	    typedef base::Matrix6d Covariance;

    public:
        base::Vector3d vel;
        base::Vector3d rot;
        Covariance cov;

    public:
        explicit Twist ( const base::Vector3d& vel = base::Vector3d::Zero(),  const base::Vector3d& rot = base::Vector3d::Zero() ):
            vel(vel), rot(rot) {this->invalidateUncertainty(); };

        Twist(const base::Vector3d& vel, const base::Vector3d& rot, const Covariance& cov):
            vel(vel), rot(rot), cov(cov) {};

        Twist(const base::Vector6d& velocity, const Covariance& cov)
        {
            this->vel = velocity.block<3,1>(0,0);//Linear velocity at first place
            this->rot = velocity.block<3,1>(3,0);//Angular velocity at second place
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
        bool hasValidVelocity() const { return base::isnotnan(this->vel) && base::isnotnan(this->rot); }
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

        Twist Zero()
        {
            return Twist();
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


        inline Twist& operator+=(const Twist& arg)
        {
            this->vel += arg.vel;
            this->rot += arg.rot;
            return *this;
        }

        inline friend Twist operator+(Twist lhs,const Twist& rhs)
        {
            return lhs += rhs;
        }

        inline Twist& operator-=(const Twist& arg)
        {
            this->vel -= arg.vel;
            this->rot -= arg.rot;
            return *this;
        }

        inline friend Twist operator-(Twist lhs, const Twist& rhs)
        {
            return lhs -= rhs;
        }

        //Spatial Cross product
        inline friend Twist operator*(const Twist& lhs, const Twist& rhs)
        {
            Twist tmp;
            tmp.vel = lhs.rot.cross(rhs.vel)+lhs.vel.cross(rhs.rot);
            tmp.rot = lhs.rot.cross(rhs.rot);
            return tmp;
        }

        /** unary - **/
        inline friend Twist operator-(const Twist& arg)
        {
            return Twist(static_cast<base::Vector3d>(-arg.vel),static_cast<base::Vector3d>(-arg.rot));
        }

    };

    /** Default std::cout function
    */
    inline std::ostream & operator<<(std::ostream &out, const base::Twist& twist)
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
