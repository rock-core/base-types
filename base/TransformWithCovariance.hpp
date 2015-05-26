#ifndef __BASE_TRANSFORM_WITH_COVARIANCE_HPP__
#define __BASE_TRANSFORM_WITH_COVARIANCE_HPP__

#include <iomanip> // std::setprecision

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <base/Eigen.hpp>
#include <base/samples/RigidBodyState.hpp> /** For backward compatibility with RBS **/

namespace base {

    /** 
     * Class which represents a 3D Transformation with associated uncertainty information.
     *
     * The uncertainty is represented as a 6x6 matrix, which is the covariance
     * matrix of the [r t] representation of the error. Here r is the rotational
     * part expressed as a scaled axis of rotation, and t the translational
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
        /** The transformation is represented 6D vector [rotation translation]
        * Here rotation is the rotational part expressed as a scaled axis of
        * rotation, and t the translational component.
        */
        base::AngleAxisd rotation;

        base::Position translation;

        /** The uncertainty is represented as a 6x6 matrix, which is the covariance
         * matrix of the [rotation translation] representation of the error.
         */
        Covariance cov;

    public:
        explicit TransformWithCovariance( const base::Affine3d& trans = base::Affine3d::Identity() )
            {this->setTransform(trans); this->invalidateCovariance();};

        TransformWithCovariance( const base::Affine3d& trans, const Covariance& cov )
            {this->setTransform(trans); this->cov = cov;};

        TransformWithCovariance( const base::AngleAxisd& rotation, const base::Position& translation ) :
            rotation(rotation), translation(translation){this->invalidateCovariance();};

        TransformWithCovariance( const base::AngleAxisd& rotation, const base::Position& translation, const Covariance& cov ) :
            rotation(rotation), translation(translation), cov(cov){};

        /** For backward compatibility with RBS **/
        explicit TransformWithCovariance( const base::samples::RigidBodyState& rbs )
        {
            this->operator=( rbs );
        };

        static TransformWithCovariance Identity()
        {
            return TransformWithCovariance();
        };
	
	
        /** Default std::cout function
        */
        friend std::ostream & operator<<(std::ostream &out, const TransformWithCovariance& trans);

        /** performs a composition of this transform with the transform given.
         * The result is another transform with result = this * trans
         */
        TransformWithCovariance composition( const TransformWithCovariance& trans ) const
        {
            return this->operator*( trans );
        };

        /** performs an inverse composition of two transformations.
         * The result is such that result * trans = this. Note that this is different from
         * calling result = this * inv(trans), in the way the uncertainties are handled.
         */
        TransformWithCovariance compositionInv( const TransformWithCovariance& trans ) const
        {
            const TransformWithCovariance &tf(*this);
            const TransformWithCovariance &t1(trans);
            base::AngleAxisd r2(tf.rotation * t1.rotation.inverse());
            base::Position p2(tf.translation + (tf.rotation * t1.inverse().translation));

            // short path if there is no uncertainty
            if( !t1.hasValidCovariance() && !tf.hasValidCovariance() )
                return TransformWithCovariance(r2, p2);

            // convert the rotations of the respective transforms into quaternions
            // in order to inverse the covariances, we need to get both the t1 and t2=[r2 p2] transformations
            // based on the composition tf = t2 * t1
            Eigen::Quaterniond q1( t1.rotation );
            Eigen::Quaterniond q2( r2 );
            Eigen::Quaterniond q( q2 * q1 );

            // initialize resulting covariance
            Eigen::Matrix<double,6,6> cov = Eigen::Matrix<double,6,6>::Zero();

            Eigen::Matrix<double,6,6> J1;
            J1 << dr2r1_by_r1(q, q1, q2), Eigen::Matrix3d::Zero(),
            Eigen::Matrix3d::Zero(), r2.toRotationMatrix();

            Eigen::Matrix<double,6,6> J2;
            J2 << dr2r1_by_r2(q, q1, q2), Eigen::Matrix3d::Zero(),
            drx_by_dr(q2, t1.translation), Eigen::Matrix3d::Identity();

            cov = J2.inverse() * ( tf.getCovariance() - J1 * t1.getCovariance() * J1.transpose() ) * J2.transpose().inverse();

            // and return the resulting uncertainty transform
            return TransformWithCovariance( r2, p2, cov );
        };

        /** Same as compositionInv, just that the result is such that trans * result = this.
         * Note that this is different from calling result = inv(trans) * this, 
         * in the way the uncertainties are handled.
         */
        TransformWithCovariance preCompositionInv( const TransformWithCovariance& trans ) const
        {
            const TransformWithCovariance &tf(*this);
            const TransformWithCovariance &t2(trans);
            base::AngleAxisd r1(t2.rotation.inverse() * tf.rotation);
            base::Position p1(t2.inverse().translation + (t2.rotation.inverse() * tf.translation));

            // short path if there is no uncertainty 
            if( !t2.hasValidCovariance() && !tf.hasValidCovariance() )
                return TransformWithCovariance( r1, p1 );

            // convert the rotations of the respective transforms into quaternions
            // in order to inverse the covariances, we need to get both the t1=[r1 p1] and t2 transformations
            // based on the composition tf = t2 * t1
            Eigen::Quaterniond q1(r1);
            Eigen::Quaterniond q2(t2.rotation);
            Eigen::Quaterniond q( q2 * q1 );

            // initialize resulting covariance
            Eigen::Matrix<double,6,6> cov = Eigen::Matrix<double,6,6>::Zero();

            Eigen::Matrix<double,6,6> J1;
            J1 << dr2r1_by_r1(q, q1, q2), Eigen::Matrix3d::Zero(),
            Eigen::Matrix3d::Zero(), t2.getTransform().linear();

            Eigen::Matrix<double,6,6> J2;
            J2 << dr2r1_by_r2(q, q1, q2), Eigen::Matrix3d::Zero(),
            drx_by_dr(q2, p1), Eigen::Matrix3d::Identity();

            cov = J1.inverse() * ( tf.getCovariance() - J2 * t2.getCovariance() * J2.transpose() ) * J1.transpose().inverse();

            // and return the resulting uncertainty transform
            return TransformWithCovariance( r1, p1, cov );
        };

        /** alias for the composition of two transforms
         */
        TransformWithCovariance operator*( const TransformWithCovariance& trans ) const
        {
            const TransformWithCovariance &t2(*this);
            const TransformWithCovariance &t1(trans);

            base::AngleAxisd t(t2.rotation * t1.rotation);
            base::Position p(t2.translation + (t2.rotation * t1.translation));

            // short path if there is no uncertainty 
            if( !t1.hasValidCovariance() && !t2.hasValidCovariance() )
            {
                return TransformWithCovariance(t, p);
            }

            // convert the rotations of the respective transforms into quaternions
            Eigen::Quaterniond q1( t1.rotation ), q2( t2.rotation );
            Eigen::Quaterniond q( t2.rotation * t1.rotation );

            // initialize resulting covariance
            Eigen::Matrix<double,6,6> cov = Eigen::Matrix<double,6,6>::Zero();

            // calculate the Jacobians (this is what all the above functions are for)
            // and add to the resulting covariance
            if( t1.hasValidCovariance() )
            {
                Eigen::Matrix<double,6,6> J1;
                J1 << dr2r1_by_r1(q, q1, q2), Eigen::Matrix3d::Zero(),
                Eigen::Matrix3d::Zero(), t2.getTransform().linear();

                cov += J1*t1.getCovariance()*J1.transpose();
            }

            if( t2.hasValidCovariance() )
            {
                Eigen::Matrix<double,6,6> J2;
                J2 << dr2r1_by_r2(q, q1, q2), Eigen::Matrix3d::Zero(),
                drx_by_dr(q2, t1.translation), Eigen::Matrix3d::Identity();

                cov += J2*t2.getCovariance()*J2.transpose();
            }

            // and return the resulting uncertainty transform
            return TransformWithCovariance(t, p, cov);
        };
	
        TransformWithCovariance inverse() const
        {
            // short path if there is no uncertainty
            if( !hasValidCovariance() )
                return TransformWithCovariance(this->rotation.inverse(), static_cast<base::Position>(-(this->rotation.inverse() * this->translation)));

            Eigen::Quaterniond q(this->rotation);
            Eigen::Vector3d t(this->translation);
            Eigen::Matrix<double,6,6> J;
            J << Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero(),
            drx_by_dr( q.inverse(), t ), q.toRotationMatrix().transpose();

            return TransformWithCovariance(this->rotation.inverse(),
                static_cast<base::Position>(-(this->rotation.inverse() * this->translation)),
                J*this->getCovariance()*J.transpose());
        };

        /** For backward compatibility with RBS **/
        TransformWithCovariance& operator=( const base::samples::RigidBodyState& rbs )
        {
            // extract the transform
            this->setTransform(rbs.getTransform());

            // and the covariance
            cov << rbs.cov_orientation, Eigen::Matrix3d::Zero(),
            Eigen::Matrix3d::Zero(), rbs.cov_position;

            return *this;
        };

        const Covariance& getCovariance() const { return this->cov; }
        void setCovariance( const Covariance& cov ) { this->cov = cov; }

        const base::Affine3d getTransform() const
        {
            base::Affine3d trans (this->rotation);
            trans.translation() = this->translation;
            return trans;
        }
        void setTransform( const base::Affine3d& trans )
        {
            this->rotation = base::AngleAxisd(trans.rotation());
            this->translation = trans.translation();
        }

        const base::Orientation getOrientation() const
        {
            return base::Orientation(this->rotation);
        }

        void setOrientation(const base::Orientation & q)
        {
            this->rotation = base::AngleAxisd(q);
        }

        bool hasValidTransform() const
        {
            return base::isnotnan(this->rotation.toRotationMatrix()) && base::isnotnan(this->translation);
        }

        void invalidateTransform()
        {
            base::Affine3d invalid_trans(base::AngleAxisd(base::NaN<double>(),
                        base::Vector3d::Ones() * base::NaN<double>()));
            this->setTransform(invalid_trans);
        }

        bool hasValidCovariance() const { return base::isnotnan(cov); }
        void invalidateCovariance()
        {
            cov = Covariance::Ones() * base::unknown<double>();
        }

    protected:
        // The uncertainty transformations are implemented according to: 
        // Pennec X, Thirion JP. A framework for uncertainty and validation of 3-D
        // registration methods based on points and frames. International Journal of
        // Computer Vion. 1997;25(3):203–229. Available at:
        // http://www.springerlink.com/index/JJ25N2Q23T402682.pdf.

        static Eigen::Quaterniond r_to_q( const Eigen::Vector3d& r )
        {
            double theta = r.norm();
            if( fabs(theta) > 1e-5 )
            return Eigen::Quaterniond( base::AngleAxisd( theta, r/theta ) );
            else
            return Eigen::Quaterniond::Identity();
        }

        static Eigen::Vector3d q_to_r( const Eigen::Quaterniond& q )
        {
            base::AngleAxisd aa( q );
            return aa.axis() * aa.angle();
        }

        static inline double sign( double v )
        {
            return v > 0.0 ? 1.0 : -1.0;
        }

        static Eigen::Matrix<double,3,3> skew_symmetric( const Eigen::Vector3d& r )
        {
            Eigen::Matrix3d res;
            res << 0, -r.z(), r.y(),
            r.z(), 0, -r.x(),
            -r.y(), r.x(), 0;
            return res;
        }

        static Eigen::Matrix<double,4,3> dq_by_dr( const Eigen::Quaterniond& q )
        {
            const Eigen::Vector3d r( q_to_r( q ) );

            const double theta = r.norm();
            const double kappa = 0.5 - theta*theta / 48.0; // approx. see Paper 
            const double lambda = 1.0/24.0*(1.0-theta*theta/40.0); // approx.
            Eigen::Matrix<double,4,3> res;
            res << - q.vec().transpose()/2.0,
            kappa * Eigen::Matrix3d::Identity() - lambda * r * r.transpose(); 	

            return res;
        }

        static Eigen::Matrix<double,3,4> dr_by_dq( const Eigen::Quaterniond& q )
        {
            const Eigen::Vector3d r( q_to_r( q ) );
            const double mu = q.vec().norm();
            const double tau = 2.0 * sign( q.w() ) * ( 1.0 + mu*mu/6.0 ); // approx
            const double nu = -2.0 * sign( q.w() ) * ( 2.0/3.0 + mu*mu/5.0 ); // approx

            Eigen::Matrix<double,3,4> res;
            res << -2*q.vec(), tau * Eigen::Matrix3d::Identity() + nu * q.vec() * q.vec().transpose();

            return res;
        }

        static Eigen::Matrix<double,4,4> dq2q1_by_dq1( const Eigen::Quaterniond& q2 )
        {
            Eigen::Matrix<double,4,4> res;
            res << 0, -q2.vec().transpose(),
            q2.vec(), skew_symmetric( q2.vec() );
            return Eigen::Matrix<double,4,4>::Identity() * q2.w() + res;
        }

        static Eigen::Matrix<double,4,4> dq2q1_by_dq2( const Eigen::Quaterniond& q1 )
        {
            Eigen::Matrix<double,4,4> res;
            res << 0, -q1.vec().transpose(),
            q1.vec(), -skew_symmetric( q1.vec() );
            return Eigen::Matrix<double,4,4>::Identity() * q1.w() + res;
        }

        static Eigen::Matrix<double,3,3> dr2r1_by_r1( const Eigen::Quaterniond& q, const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2 )
        {
            return Eigen::Matrix3d(
                dr_by_dq( q )
                * dq2q1_by_dq1( q2 )
                * dq_by_dr( q1 ) );
        }

        static Eigen::Matrix<double,3,3> dr2r1_by_r2( const Eigen::Quaterniond& q, const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2 )
        {
            return Eigen::Matrix3d(
                dr_by_dq( q )
                * dq2q1_by_dq2( q1 )
                * dq_by_dr( q2 ) );
        }

        static Eigen::Matrix<double,3,3> drx_by_dr( const Eigen::Quaterniond& q, const Eigen::Vector3d& x )
        {
            const Eigen::Vector3d r( q_to_r( q ) );
            const double theta = r.norm();
            const double alpha = 1.0 - theta*theta/6.0;
            const double beta = 0.5 - theta*theta/24.0;
            const double gamma = 1.0 / 3.0 - theta*theta/30.0;
            const double delta = -1.0 / 12.0 + theta*theta/180.0;

            return Eigen::Matrix3d(
                -skew_symmetric(x)*(gamma*r*r.transpose()
                - beta*skew_symmetric(r)+alpha*Eigen::Matrix3d::Identity())
                -skew_symmetric(r)*skew_symmetric(x)*(delta*r*r.transpose() 
                + 2.0*beta*Eigen::Matrix3d::Identity()) );
        }
    };

    /** Default std::cout function
    */
    inline std::ostream & operator<<(std::ostream &out, const TransformWithCovariance& trans)
    {
        /** cout the 6D pose vector (scaled axis rotation and translation) with its associated covariance matrix **/
        base::Vector3d scaled_axis;
        scaled_axis = trans.rotation.axis() * trans.rotation.angle();
        for (register unsigned short i=0; i<trans.getCovariance().rows(); ++i)
        {
            if (i<3)
            {
                out<<std::fixed<<std::setprecision(3)<<scaled_axis[i]<<"\t|";
            }
            else
            {
                out<<std::fixed<<std::setprecision(3)<<trans.translation[i-3]<<"\t|";
            }
            for (register unsigned short j=0; j<trans.getCovariance().cols(); ++j)
            {
                out<<std::fixed<<std::setprecision(3)<<trans.getCovariance().row(i)[j]<<"\t";
            }
            out<<"\n";
        }
        out.unsetf(std::ios_base::floatfield);
        return out;
    };
} // namespaces

#endif
