#include "TransformWithCovariance.hpp"

#include <iomanip> // std::setprecision

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <base/Float.hpp>

base::TransformWithCovariance::TransformWithCovariance() 
    : translation(base::Position::Zero()) , orientation(base::Quaterniond::Identity())
{
    this->invalidateCovariance();
}

base::TransformWithCovariance::TransformWithCovariance(const base::Affine3d& trans)
{
    this->setTransform(trans);
    this->invalidateCovariance();
}

base::TransformWithCovariance::TransformWithCovariance(const base::Affine3d& trans, const base::TransformWithCovariance::Covariance& cov)
{
    this->setTransform(trans);
    this->cov = cov;
}

base::TransformWithCovariance::TransformWithCovariance(const base::Position& translation, const base::Quaterniond& orientation)
    : translation(translation), orientation(orientation)
{
    this->invalidateCovariance();
}

base::TransformWithCovariance::TransformWithCovariance(const base::Position& translation, const base::Quaterniond& orientation, const base::TransformWithCovariance::Covariance& cov)
    : translation(translation), orientation(orientation), cov(cov)
{

}

base::TransformWithCovariance base::TransformWithCovariance::Identity()
{
    return TransformWithCovariance();
}

base::TransformWithCovariance base::TransformWithCovariance::composition(const base::TransformWithCovariance& trans) const
{
    return this->operator*( trans );
}

base::TransformWithCovariance base::TransformWithCovariance::compositionInv(const base::TransformWithCovariance& trans) const
{
    const TransformWithCovariance &tf(*this);
    const TransformWithCovariance &t1(trans);
    base::Position p2(tf.translation + (tf.orientation * t1.inverse().translation));
    Eigen::Quaterniond q2( tf.orientation * t1.orientation.inverse());

    // short path if there is no uncertainty
    if( !t1.hasValidCovariance() && !tf.hasValidCovariance() )
        return TransformWithCovariance(p2, q2);

    // convert the orientations of the respective transforms into quaternions
    // in order to inverse the covariances, we need to get both the t1 and t2=[r2 p2] transformations
    // based on the composition tf = t2 * t1
    Eigen::Quaterniond q1( t1.orientation );
    Eigen::Quaterniond q( q2 * q1 );

    // initialize resulting covariance
    Eigen::Matrix<double,6,6> cov = Eigen::Matrix<double,6,6>::Zero();

    Eigen::Matrix<double,6,6> J1;
    J1 << q2.toRotationMatrix(), Eigen::Matrix3d::Zero(),
    Eigen::Matrix3d::Zero(), dr2r1_by_r1(q, q1, q2);

    Eigen::Matrix<double,6,6> J2;
    J2 << Eigen::Matrix3d::Identity(), drx_by_dr(q2, t1.translation),
    Eigen::Matrix3d::Zero(), dr2r1_by_r2(q, q1, q2);

    cov = J2.inverse() * ( tf.getCovariance() - J1 * t1.getCovariance() * J1.transpose() ) * J2.transpose().inverse();

    // and return the resulting uncertainty transform
    return TransformWithCovariance( p2, q2, cov );
}

base::TransformWithCovariance base::TransformWithCovariance::preCompositionInv(const base::TransformWithCovariance& trans) const
{
    const TransformWithCovariance &tf(*this);
    const TransformWithCovariance &t2(trans);
    base::Position p1(t2.inverse().translation + (t2.orientation.inverse() * tf.translation));
    Eigen::Quaterniond q1(t2.orientation.inverse() * tf.orientation);

    // short path if there is no uncertainty 
    if( !t2.hasValidCovariance() && !tf.hasValidCovariance() )
        return TransformWithCovariance( p1, q1 );

    // convert the orientations of the respective transforms into quaternions
    // in order to inverse the covariances, we need to get both the t1=[p1 q1] and t2 transformations
    // based on the composition tf = t2 * t1
    Eigen::Quaterniond q2(t2.orientation);
    Eigen::Quaterniond q( q2 * q1 );

    // initialize resulting covariance
    Eigen::Matrix<double,6,6> cov = Eigen::Matrix<double,6,6>::Zero();

    Eigen::Matrix<double,6,6> J1;
    J1 << t2.getTransform().linear(), Eigen::Matrix3d::Zero(),
    Eigen::Matrix3d::Zero(), dr2r1_by_r1(q, q1, q2);

    Eigen::Matrix<double,6,6> J2;
    J2 << Eigen::Matrix3d::Identity(), drx_by_dr(q2, p1),
    Eigen::Matrix3d::Zero(), dr2r1_by_r2(q, q1, q2);

    cov = J1.inverse() * ( tf.getCovariance() - J2 * t2.getCovariance() * J2.transpose() ) * J1.transpose().inverse();

    // and return the resulting uncertainty transform
    return TransformWithCovariance( p1, q1, cov );
}


base::TransformWithCovariance base::TransformWithCovariance::operator*(const base::TransformWithCovariance& trans) const
{
    const TransformWithCovariance &t2(*this);
    const TransformWithCovariance &t1(trans);

    const base::Quaterniond t(t2.orientation * t1.orientation);
    const base::Position p(t2.translation + (t2.orientation * t1.translation));

    // short path if there is no uncertainty 
    if( !t1.hasValidCovariance() && !t2.hasValidCovariance() )
    {
        return TransformWithCovariance(p, t);
    }

    // convert the orientations of the respective transforms into quaternions
    const Eigen::Quaterniond q1( t1.orientation ), q2( t2.orientation );
    const Eigen::Quaterniond q( t2.orientation * t1.orientation );

    // initialize resulting covariance
    Eigen::Matrix<double,6,6> cov = Eigen::Matrix<double,6,6>::Zero();

    // calculate the Jacobians (this is what all the above functions are for)
    // and add to the resulting covariance
    if( t1.hasValidCovariance() )
    {
        Eigen::Matrix<double,6,6> J1;
        J1 << t2.getTransform().linear(), Eigen::Matrix3d::Zero(),
        Eigen::Matrix3d::Zero(), dr2r1_by_r1(q, q1, q2);

        cov += J1*t1.getCovariance()*J1.transpose();
    }

    if( t2.hasValidCovariance() )
    {
        Eigen::Matrix<double,6,6> J2;
        J2 << Eigen::Matrix3d::Identity(), drx_by_dr(q2, t1.translation),
        Eigen::Matrix3d::Zero(), dr2r1_by_r2(q, q1, q2);

        cov += J2*t2.getCovariance()*J2.transpose();
    }

    // and return the resulting uncertainty transform
    return TransformWithCovariance(p, t, cov);
}

base::TransformWithCovariance base::TransformWithCovariance::inverse() const
{
    // short path if there is no uncertainty
    if( !hasValidCovariance() )
        return TransformWithCovariance(static_cast<base::Position>(-(this->orientation.inverse() * this->translation)), this->orientation.inverse());

    Eigen::Quaterniond q(this->orientation);
    Eigen::Vector3d t(this->translation);
    Eigen::Matrix<double,6,6> J;
    J << q.toRotationMatrix().transpose(), drx_by_dr( q.inverse(), t ),
    Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Identity();

    return TransformWithCovariance(static_cast<base::Position>(-(this->orientation.inverse() * this->translation)),
                                this->orientation.inverse(),
                                J*this->getCovariance()*J.transpose());
}

const base::TransformWithCovariance::Covariance& base::TransformWithCovariance::getCovariance() const
{
    return this->cov;
}

void base::TransformWithCovariance::setCovariance(const base::TransformWithCovariance::Covariance& cov)
{
    this->cov = cov;
}

const base::Matrix3d base::TransformWithCovariance::getTranslationCov() const
{
    return this->cov.topLeftCorner<3,3>();
}

void base::TransformWithCovariance::setTranslationCov(const base::Matrix3d& cov)
{
    this->cov.topLeftCorner<3,3>() = cov;
}

const base::Matrix3d base::TransformWithCovariance::getOrientationCov() const
{
    return this->cov.bottomRightCorner<3,3>();
}

void base::TransformWithCovariance::setOrientationCov(const base::Matrix3d& cov)
{
    this->cov.bottomRightCorner<3,3>() = cov;
}

const base::Affine3d base::TransformWithCovariance::getTransform() const
{
    base::Affine3d trans (this->orientation);
    trans.translation() = this->translation;
    return trans;
}

void base::TransformWithCovariance::setTransform(const base::Affine3d& trans)
{
    this->translation = trans.translation();
    this->orientation = base::Quaterniond(trans.rotation());
}

const base::Orientation base::TransformWithCovariance::getOrientation() const
{
    return base::Orientation(this->orientation);
}

void base::TransformWithCovariance::setOrientation(const base::Orientation& q)
{
    this->orientation = base::Quaterniond(q);
}

bool base::TransformWithCovariance::hasValidTransform() const
{
    return !translation.hasNaN() && !orientation.coeffs().hasNaN();
}

void base::TransformWithCovariance::invalidateTransform()
{
    translation = base::Position::Ones() * base::NaN<double>();
    orientation.coeffs() = base::Vector4d::Ones() * base::NaN<double>();
}

bool base::TransformWithCovariance::hasValidCovariance() const
{
    return !cov.hasNaN();
}

void base::TransformWithCovariance::invalidateCovariance()
{
    cov = Covariance::Ones() * base::NaN<double>();
}

Eigen::Quaterniond base::TransformWithCovariance::r_to_q(const Eigen::Vector3d& r)
{
    double theta = r.norm();
    if( fabs(theta) > 1e-5 )
        return Eigen::Quaterniond( base::AngleAxisd( theta, r/theta ) );
    else
        return Eigen::Quaterniond::Identity();
}

Eigen::Vector3d base::TransformWithCovariance::q_to_r(const Eigen::Quaterniond& q)
{
    base::AngleAxisd aa( q );
    return aa.axis() * aa.angle();
}

double base::TransformWithCovariance::sign(double v)
{
    return v > 0.0 ? 1.0 : -1.0;
}

Eigen::Matrix< double, int(3), int(3) > base::TransformWithCovariance::skew_symmetric(const Eigen::Vector3d& r)
{
    Eigen::Matrix3d res;
    res << 0, -r.z(), r.y(),
    r.z(), 0, -r.x(),
    -r.y(), r.x(), 0;
    return res;
}

Eigen::Matrix< double, int(4), int(3) > base::TransformWithCovariance::dq_by_dr(const Eigen::Quaterniond& q)
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

Eigen::Matrix< double, int(3), int(4) > base::TransformWithCovariance::dr_by_dq(const Eigen::Quaterniond& q)
{
    const Eigen::Vector3d r( q_to_r( q ) );
    const double mu = q.vec().norm();
    const double tau = 2.0 * sign( q.w() ) * ( 1.0 + mu*mu/6.0 ); // approx
    const double nu = -2.0 * sign( q.w() ) * ( 2.0/3.0 + mu*mu/5.0 ); // approx

    Eigen::Matrix<double,3,4> res;
    res << -2*q.vec(), tau * Eigen::Matrix3d::Identity() + nu * q.vec() * q.vec().transpose();

    return res;
}

Eigen::Matrix< double, int(4), int(4) > base::TransformWithCovariance::dq2q1_by_dq1(const Eigen::Quaterniond& q2)
{
    Eigen::Matrix<double,4,4> res;
    res << 0, -q2.vec().transpose(),
    q2.vec(), skew_symmetric( q2.vec() );
    return Eigen::Matrix<double,4,4>::Identity() * q2.w() + res;
}

Eigen::Matrix< double, int(4), int(4) > base::TransformWithCovariance::dq2q1_by_dq2(const Eigen::Quaterniond& q1)
{
    Eigen::Matrix<double,4,4> res;
    res << 0, -q1.vec().transpose(),
    q1.vec(), -skew_symmetric( q1.vec() );
    return Eigen::Matrix<double,4,4>::Identity() * q1.w() + res;
}

Eigen::Matrix< double, int(3), int(3) > base::TransformWithCovariance::dr2r1_by_r1(const Eigen::Quaterniond& q, const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2)
{
    return Eigen::Matrix3d(
    dr_by_dq( q )
    * dq2q1_by_dq1( q2 )
    * dq_by_dr( q1 ) );
}

Eigen::Matrix< double, int(3), int(3) > base::TransformWithCovariance::dr2r1_by_r2(const Eigen::Quaterniond& q, const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2)
{
    return Eigen::Matrix3d(
    dr_by_dq( q )
    * dq2q1_by_dq2( q1 )
    * dq_by_dr( q2 ) );
}

Eigen::Matrix< double, int(3), int(3) > base::TransformWithCovariance::drx_by_dr(const Eigen::Quaterniond& q, const Eigen::Vector3d& x)
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

std::ostream& base::operator<<(std::ostream& out, const base::TransformWithCovariance& trans)
{
    /** cout the 6D pose vector (translation and scaled axis orientation) with its associated covariance matrix **/
    base::Vector3d scaled_axis;
    base::AngleAxisd angle_axis (trans.orientation);
    scaled_axis = angle_axis.axis() * angle_axis.angle();
    for (register unsigned short i=0; i<trans.getCovariance().rows(); ++i)
    {
        if (i<3)
        {
            out<<std::fixed<<std::setprecision(5)<<trans.translation[i]<<"\t|";
        }
        else
        {
            out<<std::fixed<<std::setprecision(5)<<scaled_axis[i-3]<<"\t|";
        }
        for (register unsigned short j=0; j<trans.getCovariance().cols(); ++j)
        {
            out<<std::fixed<<std::setprecision(5)<<trans.getCovariance().row(i)[j]<<"\t";
        }
        out<<"\n";
    }
    out.unsetf(std::ios_base::floatfield);
    return out;
}























