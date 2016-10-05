#ifndef __BASE_TRANSFORM_WITH_COVARIANCE_HPP__
#define __BASE_TRANSFORM_WITH_COVARIANCE_HPP__

#include <base/Pose.hpp>
#include <base/Eigen.hpp>

namespace base {

    /** 
     * Class which represents a 3D Transformation with associated uncertainty information.
     *
     * The uncertainty is represented as a 6x6 matrix, which is the covariance
     * matrix of the [r t] representation of the error. Here r is the rotation orientation
     * part expressed as a scaled axis of orientation, and t the translational
     * component.
     *
     * The uncertainty information is optional. The hasValidCovariance() method can
     * be used to see if uncertainty information is associated with the class.
     */
    class TransformWithCovariance
    {

    public:
	    typedef base::Matrix6d Covariance;

    public:
        /** The transformation is represented 6D vector [translation orientation]
        * Here orientation is the rotational part expressed as a quaternion
        * orientation, and t the translational component.
        */
        base::Position translation;

        base::Quaterniond orientation;

        /** The uncertainty is represented as a 6x6 matrix, which is the covariance
         * matrix of the [translation orientation] representation of the error.
         */
        Covariance cov;

    public:
        TransformWithCovariance()
            : translation(Position::Zero()) , orientation(Quaterniond::Identity())
        {
            this->invalidateCovariance();
        }
      
        explicit TransformWithCovariance( const base::Affine3d& trans)
        {
            this->setTransform(trans);
            this->invalidateCovariance();
        }

        TransformWithCovariance( const base::Affine3d& trans, const Covariance& cov )
        {
            this->setTransform(trans);
            this->cov = cov;
        }

        TransformWithCovariance(const base::Position& translation, const base::Quaterniond& orientation)
            : translation(translation), orientation(orientation)
        {
            this->invalidateCovariance();
        }

        TransformWithCovariance(const base::Position& translation, const base::Quaterniond& orientation, const Covariance& cov )
            : translation(translation), orientation(orientation), cov(cov)
        {

        }

        static TransformWithCovariance Identity()
        {
            return TransformWithCovariance();
        }
	
	
        /** Default std::cout function
        */
        friend std::ostream & operator<<(std::ostream &out, const TransformWithCovariance& trans);

        /** performs a composition of this transform with the transform given.
         * The result is another transform with result = this * trans
         */
        TransformWithCovariance composition( const TransformWithCovariance& trans ) const;

        /** performs an inverse composition of two transformations.
         * The result is such that result * trans = this. Note that this is different from
         * calling result = this * inv(trans), in the way the uncertainties are handled.
         */
        TransformWithCovariance compositionInv( const TransformWithCovariance& trans ) const;

        /** Same as compositionInv, just that the result is such that trans * result = this.
         * Note that this is different from calling result = inv(trans) * this, 
         * in the way the uncertainties are handled.
         */
        TransformWithCovariance preCompositionInv( const TransformWithCovariance& trans ) const;

        /** alias for the composition of two transforms
         */
        TransformWithCovariance operator*( const TransformWithCovariance& trans ) const;

        /** performs a composition of this transform with a given point with covariance.
         */
        std::pair<Eigen::Vector3d, Eigen::Matrix3d>
        composePointWithCovariance( const Eigen::Vector3d& point, const Eigen::Matrix3d& cov ) const;
	
        TransformWithCovariance inverse() const;

        const Covariance& getCovariance() const
        {
            return this->cov;
        }

        void setCovariance( const Covariance& cov )
        {
            this->cov = cov;
        }

        const base::Matrix3d getTranslationCov() const
        {
            return this->cov.topLeftCorner<3,3>();
        }
        
        void setTranslationCov(const base::Matrix3d& cov)
        {
            this->cov.topLeftCorner<3,3>() = cov;
        }

        const base::Matrix3d getOrientationCov() const
        {
            return this->cov.bottomRightCorner<3,3>();
        }
        
        void setOrientationCov(const base::Matrix3d& cov)
        {
            this->cov.bottomRightCorner<3,3>() = cov;
        }

        const base::Affine3d getTransform() const
        {
            Affine3d trans (this->orientation);
            trans.translation() = this->translation;
            return trans;
        }

        void setTransform( const base::Affine3d& trans )
        {
            this->translation = trans.translation();
            this->orientation = Quaterniond(trans.rotation());
        }

        const base::Orientation &getOrientation() const
        {
            return this->orientation;
        }

        void setOrientation(const base::Orientation & q)
        {
            this->orientation = q;
        }

        bool hasValidTransform() const
        {
            return !translation.hasNaN() && !orientation.coeffs().hasNaN();
        }

        void invalidateTransform()
        {
            translation = Position::Ones() * NaN<double>();
            orientation.coeffs() = Vector4d::Ones() * NaN<double>();
        }

        /** @warning This method is computationally expensive. Use with care! */
        bool hasValidCovariance() const
        {
            return !cov.hasNaN();
        }
        
        void invalidateCovariance()
        {
            cov = Covariance::Ones() * NaN<double>();
        }

    protected:
        // The uncertainty transformations are implemented according to: 
        // Pennec X, Thirion JP. A framework for uncertainty and validation of 3-D
        // registration methods based on points and frames. International Journal of
        // Computer Vion. 1997;25(3):203–229. Available at:
        // http://www.springerlink.com/index/JJ25N2Q23T402682.pdf.

        static Eigen::Quaterniond r_to_q( const Eigen::Vector3d& r );

        static Eigen::Vector3d q_to_r( const Eigen::Quaterniond& q );

        static inline double sign( double v );

        static Eigen::Matrix<double,3,3> skew_symmetric( const Eigen::Vector3d& r );

        static Eigen::Matrix<double,4,3> dq_by_dr( const Eigen::Quaterniond& q );

        static Eigen::Matrix<double,3,4> dr_by_dq( const Eigen::Quaterniond& q );

        static Eigen::Matrix<double,4,4> dq2q1_by_dq1( const Eigen::Quaterniond& q2 );

        static Eigen::Matrix<double,4,4> dq2q1_by_dq2( const Eigen::Quaterniond& q1 );

        static Eigen::Matrix<double,3,3> dr2r1_by_r1( const Eigen::Quaterniond& q, const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2 );

        static Eigen::Matrix<double,3,3> dr2r1_by_r2( const Eigen::Quaterniond& q, const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2 );

        static Eigen::Matrix<double,3,3> drx_by_dr( const Eigen::Quaterniond& q, const Eigen::Vector3d& x );
    };

    /** Default std::cout function
    */
    std::ostream & operator<<(std::ostream &out, const TransformWithCovariance& trans);
} // namespaces

#endif
