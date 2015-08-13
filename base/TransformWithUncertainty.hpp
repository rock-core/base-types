#ifndef __BASE_TRANSFORM_WITH_UNCERTAINTY_HPP__
#define __BASE_TRANSFORM_WITH_UNCERTAINTY_HPP__

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <base/samples/RigidBodyState.hpp>
#include <base/Eigen.hpp>
#include <base/Pose.hpp>

namespace base {

    /** 
     * Class which represents a 3D Transform with associated uncertainty information.
     *
     * The uncertainty is represented as a 6x6 matrix, which is the covariance
     * matrix of the [r t] representation of the error. Here r is the rotational
     * part expressed as a scaled axis of rotation, and t the translational
     * component.
     *
     * The uncertainty information is optional. The hasValidUncertainty() method can
     * be used to see if uncertainty information is associated with the class.
     */
    class TransformWithUncertainty
    {
    public:
	typedef base::Affine3d Transform;
	typedef base::Matrix6d Covariance;

	/** The transformation is represented as a 4x4 homogenous matrix. Both
	 * rotation and translation in 3D are represented.
	 */
	Transform trans;
	
	/** The uncertainty is represented as a 6x6 matrix, which is the covariance
	 * matrix of the [r t] representation of the error. Here r is the rotational
	 * part expressed as a scaled axis of rotation, and t the translational
	 * component.
	 */
	Covariance cov;

	explicit TransformWithUncertainty( const Transform& trans = Transform::Identity() ) : 
		trans( trans ) { invalidateUncertainty(); };

	TransformWithUncertainty( const Transform& trans, const Covariance& cov ) :
		trans( trans ), cov( cov ) {};

	explicit TransformWithUncertainty( const base::samples::RigidBodyState& rbs )
	{
	    this->operator=( rbs );
	};

	static TransformWithUncertainty Identity()
	{
	    return TransformWithUncertainty( Transform::Identity() );
	};
	
	
	/** Default std::cout function
	*/
	friend std::ostream & operator<<(std::ostream &out, const TransformWithUncertainty& trans);

	/** performs a composition of this transform with the transform given.
	 * The result is another transform with result = this * trans
	 */
	TransformWithUncertainty composition( const TransformWithUncertainty& trans ) const
	{
	    return this->operator*( trans );
	};

	/** performs an inverse composition of two transformations.
	 * The result is such that result * trans = this. Note that this is different from
	 * calling result = this * inv(trans), in the way the uncertainties are handled.
	 */
	TransformWithUncertainty compositionInv( const TransformWithUncertainty& trans ) const
	{
	    const TransformWithUncertainty &tf(*this);
	    const TransformWithUncertainty &t1(trans);
	    Eigen::Affine3d t2 = tf.getTransform() * t1.getTransform().inverse( Eigen::Isometry );

	    // short path if there is no uncertainty 
	    if( !t1.hasValidUncertainty() && !tf.hasValidUncertainty() )
		return TransformWithUncertainty( t2 );

	    // convert the rotations of the respective transforms into quaternions
	    // in order to inverse the covariances, we need to get both the t1 and t2 transformations
	    // based on the composition tf = t2 * t1
	    Eigen::Quaterniond 
		q1( t1.getTransform().linear() ),
		q2( t2.linear() );
	    Eigen::Quaterniond q( q2 * q1 );

	    // initialize resulting covariance
	    Eigen::Matrix<double,6,6> cov = Eigen::Matrix<double,6,6>::Zero();

	    Eigen::Matrix<double,6,6> J1;
	    J1 << dr2r1_by_r1(q, q1, q2), Eigen::Matrix3d::Zero(),
	    Eigen::Matrix3d::Zero(), t2.linear();

	    Eigen::Matrix<double,6,6> J2;
	    J2 << dr2r1_by_r2(q, q1, q2), Eigen::Matrix3d::Zero(),
	    drx_by_dr(q2, t1.getTransform().translation()), Eigen::Matrix3d::Identity();

	    cov = J2.inverse() * ( tf.getCovariance() - J1 * t1.getCovariance() * J1.transpose() ) * J2.transpose().inverse();

	    // and return the resulting uncertainty transform
	    return TransformWithUncertainty( 
		    t2, cov );
	};

	/** Same as compositionInv, just that the result is such that trans * result = this.
	 * Note that this is different from calling result = inv(trans) * this, 
	 * in the way the uncertainties are handled.
	 */
	TransformWithUncertainty preCompositionInv( const TransformWithUncertainty& trans ) const
	{
	    const TransformWithUncertainty &tf(*this);
	    const TransformWithUncertainty &t2(trans);
	    Eigen::Affine3d t1 = t2.getTransform().inverse( Eigen::Isometry ) * tf.getTransform(); 

	    // short path if there is no uncertainty 
	    if( !t2.hasValidUncertainty() && !tf.hasValidUncertainty() )
		return TransformWithUncertainty( t1 );

	    // convert the rotations of the respective transforms into quaternions
	    // in order to inverse the covariances, we need to get both the t1 and t2 transformations
	    // based on the composition tf = t2 * t1
	    Eigen::Quaterniond 
		q1( t1.linear() ),
		q2( t2.getTransform().linear() );
	    Eigen::Quaterniond q( q2 * q1 );

	    // initialize resulting covariance
	    Eigen::Matrix<double,6,6> cov = Eigen::Matrix<double,6,6>::Zero();

	    Eigen::Matrix<double,6,6> J1;
	    J1 << dr2r1_by_r1(q, q1, q2), Eigen::Matrix3d::Zero(),
	    Eigen::Matrix3d::Zero(), t2.getTransform().linear();

	    Eigen::Matrix<double,6,6> J2;
	    J2 << dr2r1_by_r2(q, q1, q2), Eigen::Matrix3d::Zero(),
	    drx_by_dr(q2, t1.translation()), Eigen::Matrix3d::Identity();

	    cov = J1.inverse() * ( tf.getCovariance() - J2 * t2.getCovariance() * J2.transpose() ) * J1.transpose().inverse();

	    // and return the resulting uncertainty transform
	    return TransformWithUncertainty( 
		    t1, cov );
	};

	/** alias for the composition of two transforms
	 */
	TransformWithUncertainty operator*( const TransformWithUncertainty& trans ) const
	{
	    const TransformWithUncertainty &t2(*this);
	    const TransformWithUncertainty &t1(trans);
	    // short path if there is no uncertainty 
	    if( !t1.hasValidUncertainty() && !t2.hasValidUncertainty() )
		return TransformWithUncertainty( t2.getTransform() * t1.getTransform() );

	    // convert the rotations of the respective transforms into quaternions
	    Eigen::Quaterniond 
		q1( t1.getTransform().linear() ),
		q2( t2.getTransform().linear() );
	    Eigen::Quaterniond q( q2 * q1 );

	    // initialize resulting covariance
	    Eigen::Matrix<double,6,6> cov = Eigen::Matrix<double,6,6>::Zero();

	    // calculate the Jacobians (this is what all the above functions are for)
	    // and add to the resulting covariance
	    if( t1.hasValidUncertainty() )
	    {
		Eigen::Matrix<double,6,6> J1;
		J1 << dr2r1_by_r1(q, q1, q2), Eigen::Matrix3d::Zero(),
		Eigen::Matrix3d::Zero(), t2.getTransform().linear();

		cov += J1*t1.getCovariance()*J1.transpose();
	    }

	    if( t2.hasValidUncertainty() )
	    {
		Eigen::Matrix<double,6,6> J2;
		J2 << dr2r1_by_r2(q, q1, q2), Eigen::Matrix3d::Zero(),
		drx_by_dr(q2, t1.getTransform().translation()), Eigen::Matrix3d::Identity();

		cov += J2*t2.getCovariance()*J2.transpose();
	    }

	    // and return the resulting uncertainty transform
	    return TransformWithUncertainty( 
		    t2.getTransform() * t1.getTransform(), cov );
	};
	
	TransformWithUncertainty inverse() const
	{
	    // short path if there is no uncertainty 
	    if( !hasValidUncertainty() )
		return TransformWithUncertainty( Transform( getTransform().inverse( Eigen::Isometry ) ) );

	    Eigen::Quaterniond q( getTransform().linear() );
	    Eigen::Vector3d t( getTransform().translation() );
	    Eigen::Matrix<double,6,6> J;
	    J << Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero(),
		drx_by_dr( q.inverse(), t ), q.toRotationMatrix().transpose();

	    return TransformWithUncertainty(
		    Eigen::Affine3d( getTransform().inverse( Eigen::Isometry ) ),
		    J*getCovariance()*J.transpose() );
	};

	TransformWithUncertainty& operator=( const base::samples::RigidBodyState& rbs )
	{
	    // extract the transform
	    trans = rbs.getTransform();

	    // and the covariance
	    cov << rbs.cov_orientation, Eigen::Matrix3d::Zero(),
		Eigen::Matrix3d::Zero(), rbs.cov_position;

	    return *this;
	};

	void copyToRigidBodyState( base::samples::RigidBodyState& rbs ) const
	{
	    base::Pose pose( getTransform() );
	    rbs.position = pose.position;
	    rbs.orientation = pose.orientation;
	    rbs.cov_orientation = getCovariance().topLeftCorner<3,3>();
	    rbs.cov_position = getCovariance().bottomRightCorner<3,3>();
	}

	const Covariance& getCovariance() const { return cov; }
	void setCovariance( const Covariance& cov ) { this->cov = cov; }
	const Transform& getTransform() const { return trans; }
	void setTransform( const Transform& trans ) { this->trans = trans; }

	bool hasValidUncertainty() const { return base::isnotnan(cov); }
	void invalidateUncertainty()
        {
            cov = Covariance::Ones() * base::unknown<double>();
        }

    protected:
	// The uncertainty transformations are implemented according to: 
	// Pennec X, Thirion JP. A framework for uncertainty and validation of 3-D
	// registration methods based on points and frames. International Journal of
	// Computer Vision. 1997;25(3):203–229. Available at:
	// http://www.springerlink.com/index/JJ25N2Q23T402682.pdf.

	static Eigen::Quaterniond r_to_q( const Eigen::Vector3d& r )
	{
	    double theta = r.norm();
	    if( fabs(theta) > 1e-5 )
		return Eigen::Quaterniond( Eigen::AngleAxisd( theta, r/theta ) );
	    else
		return Eigen::Quaterniond::Identity();
	}

	static Eigen::Vector3d q_to_r( const Eigen::Quaterniond& q )
	{
	    Eigen::AngleAxisd aa( q );
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
    inline std::ostream & operator<<(std::ostream &out, const TransformWithUncertainty& trans)
    {
	out << trans.getTransform().matrix() << "\n";
	if (trans.hasValidUncertainty())
	{
	    out << trans.getCovariance().topLeftCorner<3,3>() << "\n";
	    out << trans.getCovariance().bottomRightCorner<3,3>() << "\n";
	}
	return out;
    };
} // namespaces

#endif