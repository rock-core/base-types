#ifndef __BASE_TWIST_WITH_COVARIANCE_HPP__
#define __BASE_TWIST_WITH_COVARIANCE_HPP__

#include <base/Eigen.hpp>

namespace base {

    
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
        explicit TwistWithCovariance (const base::Vector3d& vel = base::Vector3d::Zero(), const base::Vector3d& rot = base::Vector3d::Zero() );

        TwistWithCovariance(const base::Vector3d& vel, const base::Vector3d& rot, const Covariance& cov);

        TwistWithCovariance(const base::Vector6d& velocity, const Covariance& cov);

        /** Default std::cout function **/
        friend std::ostream & operator<<(std::ostream &out, const base::Vector6d& vel);

        /** Get and Set Methods **/
        const base::Vector3d& getTranslation() const;
        void setTranslation(const base::Vector3d& vel);

        const base::Vector3d& getRotation() const;
        void setRotation(const base::Vector3d& rot);

        const Covariance& getCovariance() const;
        void setCovariance(const Covariance& cov);

        const base::Matrix3d getLinearVelocityCov() const;
        void setLinearVelocityCov(const base::Matrix3d& cov);
        const base::Matrix3d getAngularVelocityCov() const;
        void setAngularVelocityCov(const base::Matrix3d& cov);

        const base::Vector3d& getLinearVelocity() const;
        void setLinearVelocity(const base::Vector3d& vel);
        const base::Vector3d& getAngularVelocity() const;
        void setAngularVelocity(const base::Vector3d& rot);

        const base::Vector3d& translation() const;
        const base::Vector3d& rotation() const;

        const base::Vector6d getVelocity() const;
            

        void setVelocity(const base::Vector6d& velocity);

        /** Check Methods **/

        bool hasValidVelocity() const;

        void invalidateVelocity();

        bool hasValidCovariance() const;
        void invalidateCovariance();


        void invalidate();

        static TwistWithCovariance Zero();
        
        double& operator[](int i);

        double operator[](int i) const;


        TwistWithCovariance& operator+=(const TwistWithCovariance& arg);


        friend TwistWithCovariance operator+(TwistWithCovariance lhs,const TwistWithCovariance& rhs);

        TwistWithCovariance& operator-=(const TwistWithCovariance& arg);

        friend TwistWithCovariance operator-(TwistWithCovariance lhs, const TwistWithCovariance& rhs);

        friend TwistWithCovariance operator*(const TwistWithCovariance& lhs,double rhs);

        friend TwistWithCovariance operator*(double lhs, const TwistWithCovariance& rhs);

        //Spatial Cross product
        friend TwistWithCovariance operator*(const TwistWithCovariance& lhs, const TwistWithCovariance& rhs);

        friend TwistWithCovariance operator/(const TwistWithCovariance& lhs,double rhs);


        /** unary - **/
        friend TwistWithCovariance operator-(const TwistWithCovariance& arg);

        static Eigen::Matrix<double, 3, 6> crossJacobian(const base::Vector3d& u, const base::Vector3d& v);

    };

    /** Default std::cout function
    */
    std::ostream & operator<<(std::ostream &out, const base::TwistWithCovariance& twist);

} // namespaces

#endif
