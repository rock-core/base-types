#include "Pose.hpp"

namespace base 
{

base::Vector3d getEuler(const base::Orientation& orientation)
{
    const Eigen::Matrix3d m = orientation.toRotationMatrix();
    double x = base::Vector2d(m.coeff(2,2) , m.coeff(2,1)).norm();
    base::Vector3d res(0,::atan2(-m.coeff(2,0), x),0);
    if (x > Eigen::NumTraits<double>::dummy_precision()){
        res[0] = ::atan2(m.coeff(1,0), m.coeff(0,0));
        res[2] = ::atan2(m.coeff(2,1), m.coeff(2,2));
    }else{
        res[0] = 0;
        res[2] = (m.coeff(2,0)>0?1:-1)* ::atan2(-m.coeff(0,1), m.coeff(1,1));
    }
    return res;
}

base::Vector3d getEuler(const base::AngleAxisd& orientation)
{
    const Eigen::Matrix3d m = orientation.toRotationMatrix();
    double x = base::Vector2d(m.coeff(2,2) , m.coeff(2,1)).norm();
    base::Vector3d res(0,::atan2(-m.coeff(2,0), x),0);
    if (x > Eigen::NumTraits<double>::dummy_precision()){
        res[0] = ::atan2(m.coeff(1,0), m.coeff(0,0));
        res[2] = ::atan2(m.coeff(2,1), m.coeff(2,2));
    }else{
        res[0] = 0;
        res[2] = (m.coeff(2,0)>0?1:-1)* ::atan2(-m.coeff(0,1), m.coeff(1,1));
    }
    return res;
}

double getYaw(const base::Orientation& orientation)
{
    return base::getEuler(orientation)[0];
}

double getYaw(const base::AngleAxisd& orientation)
{
    return base::getEuler(orientation)[0];
}

double getPitch(const base::Orientation& orientation)
{
    return base::getEuler(orientation)[1];
}

double getPitch(const base::AngleAxisd& orientation)
{
    return base::getEuler(orientation)[1];
}

double getRoll(const base::Orientation& orientation)
{
    return base::getEuler(orientation)[2];
}

double getRoll(const base::AngleAxisd& orientation)
{
    return base::getEuler(orientation)[2];
}

base::Orientation removeYaw(const base::Orientation& orientation)
{
    return Eigen::AngleAxisd( -getYaw(orientation), Eigen::Vector3d::UnitZ()) * orientation;
}

base::Orientation removeYaw(const base::AngleAxisd& orientation)
{
    return Eigen::AngleAxisd( -getYaw(orientation), Eigen::Vector3d::UnitZ()) * orientation;
}

base::Orientation removePitch(const base::Orientation& orientation)
{
    return Eigen::AngleAxisd( -getPitch(orientation), Eigen::Vector3d::UnitY()) * orientation;
}

base::Orientation removePitch(const base::AngleAxisd& orientation)
{
    return Eigen::AngleAxisd( -getPitch(orientation), Eigen::Vector3d::UnitY()) * orientation;
}

base::Orientation removeRoll(const base::Orientation& orientation)
{
    return Eigen::AngleAxisd( -getRoll(orientation), Eigen::Vector3d::UnitX()) * orientation;
}

base::Orientation removeRoll(const base::AngleAxisd& orientation)
{
    return Eigen::AngleAxisd( -getRoll(orientation), Eigen::Vector3d::UnitX()) * orientation;
}


PoseUpdateThreshold::PoseUpdateThreshold()
{
}

PoseUpdateThreshold::PoseUpdateThreshold(double _distance, double _angle): distance( _distance ), angle( _angle )
{
}

bool PoseUpdateThreshold::test( double other_distance, double other_angle )
{
    return other_distance > distance || other_angle > angle;
}

bool PoseUpdateThreshold::test( const Eigen::Affine3d& pdelta )
{
    return test( pdelta.translation().norm(), Eigen::AngleAxisd( pdelta.linear() ).angle() );
}

bool PoseUpdateThreshold::test(const Eigen::Affine3d& a2b, const Eigen::Affine3d& aprime2b)
{
    return test( a2b.inverse() * aprime2b );
}


void Pose::fromVector6d(const Vector6d& v)
{
    const Eigen::Vector3d saxis = v.head<3>();
    if( saxis.norm() > 1e-9 )
        orientation = Eigen::AngleAxisd( saxis.norm(), saxis.normalized() );
    else
        orientation = Eigen::Quaterniond::Identity();

    position = v.tail<3>();
}

Vector6d Pose::toVector6d() const
{
    Vector6d res;
    Eigen::AngleAxisd aa(orientation);
    res.head<3>() = aa.axis() * aa.angle();
    res.tail<3>() = position;

    return res;
}

std::ostream& operator << (std::ostream& io, base::Pose const& pose)
{
    io << "Position "
        << pose.position.transpose()
        << " Orientation (RPY)" 
        << getRoll(pose.orientation) << " " 
        << getPitch(pose.orientation) << " " 
        << getYaw(pose.orientation);
    return io;
}

std::ostream& operator << (std::ostream& io, base::Pose2D const& pose)
{

    io << "Position "
        << pose.position.transpose()
        << " Orientation (Theta) " 
        << pose.orientation ;
    return io;
}



}