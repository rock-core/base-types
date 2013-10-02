#include "Uncertainty.hpp"

#include <osg/Group>
#include <osg/Geode>
#include <osg/Point>
#include <osg/Geometry>
#include <osg/Drawable>
#include <osg/ShapeDrawable>

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

using namespace vizkit3d;

template <class Scalar>
osg::Vec3 eigen2osg( const typename Eigen::Matrix<Scalar,3,1> &v ) { return osg::Vec3( v.x(), v.y(), v.z() ); }

template <class Scalar>
osg::Quat eigen2osg( const typename Eigen::Quaternion<Scalar> &q ) { return osg::Quat( q.x(), q.y(), q.z(), q.w() ); }

Uncertainty::Uncertainty()
    : rand_gen(42u), rand_norm(rand_gen, boost::normal_distribution<>(0,1.0) ),
    m_showSamples( false ), num_samples( 500 ), dim(0)
{
    geode = new osg::Geode();
    osg::StateSet* stategeode = geode->getOrCreateStateSet();
    stategeode->setMode( GL_LIGHTING, osg::StateAttribute::OFF );

    addChild( geode );
}

void Uncertainty::setMean( const Eigen::Vector2d& mean )
{
    Eigen::Vector3d m;
    m << mean, 0.0;
    setPosition( eigen2osg( m ) );

    redraw(2);
}

void Uncertainty::setMean( const Eigen::Vector3d& mean )
{
    setPosition( eigen2osg( mean ) );

    redraw(3);
}

void Uncertainty::setCovariance( const Eigen::Matrix2d& cov )
{
    // get the rotation and scaling of the ellipsoid from the covariance matrix
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> ev( cov );
    Eigen::Rotation2D<double> rm(0);
    rm.fromRotationMatrix( ev.eigenvectors() );

    Eigen::Quaterniond q(  
		Eigen::Quaterniond( Eigen::AngleAxisd( rm.angle(), Eigen::Vector3d::UnitZ() ) ) );
    setAttitude( eigen2osg( q ) );

    Eigen::Vector3d s;
    s << ev.eigenvalues(), 1.0;
    setScale( eigen2osg( Eigen::Vector3d( s.array().sqrt() ) ) );

    redraw(2);
}

void Uncertainty::setCovariance( const Eigen::Matrix3d& cov )
{
    // get the rotation and scaling of the ellipsoid from the covariance matrix
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> ev( cov );
    Eigen::Matrix3d e_vec = ev.eigenvectors();
    Eigen::Vector3d e_val = ev.eigenvalues();

    // the eigenvectors might not be right handed
    if( !e_vec.col(0).cross(e_vec.col(1)).isApprox( e_vec.col(2) ) )
    {
	e_vec.col(1).swap( e_vec.col(2) );
	e_val.row(1).swap( e_val.row(2) );
    }

    Eigen::Quaterniond q( e_vec );
    setAttitude( eigen2osg( q ) );
    
    /*
    std::cout << ev.eigenvectors() << std::endl;
    std::cout << q.toRotationMatrix() << std::endl;
    std::cout << "scale: " << ev.eigenvalues().cwise().sqrt().transpose() << std::endl;
    std::cout << "q scale: " << (q * ev.eigenvalues().cwise().sqrt()).transpose() << std::endl;
    */

    Eigen::Vector3d scale( Eigen::Vector3d( e_val.array().sqrt() ) );
    for( size_t i=0; i<3; ++i )
    {
	if( scale[i] != scale[i] ) 
	{
	    scale[i] = 0;
	}
    }

    setScale( eigen2osg(scale) );

    redraw(3);
}

void Uncertainty::redraw( int dim )
{
    if( this->dim != dim )
    {
	while(geode->removeDrawables(0));
	if( dim == 2 )
	{
	    addEllipse( 2 );
	}
	else
	{
	    addEllipse( 0 );
	    addEllipse( 1 );
	    addEllipse( 2 );
	}

	if( m_showSamples )
	    addSamples();

	this->dim = dim;
    }
}

void Uncertainty::addEllipse( int axis )
{
    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
    osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
    color->push_back(osg::Vec4(0,1,1,1));
    geom->setColorArray(color.get());
    geom->setColorBinding( osg::Geometry::BIND_OVERALL );

    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
    const int segments = 50;
    for( int i=0; i<segments; i++ )
    {
	double theta = (double)i/segments * 2.0 * M_PI;
	double x = cos( theta );
	double y = sin( theta );
	Eigen::Vector3d p;
	switch( axis ) 
	{
	    case 0: p << 0, x, y; break;
	    case 1: p << x, 0, y; break;
	    case 2: p << x, y, 0; break;
	}
	vertices->push_back( eigen2osg( p ) );
    }
    geom->setVertexArray(vertices);

    osg::ref_ptr<osg::DrawArrays> drawArrays = new osg::DrawArrays( osg::PrimitiveSet::LINE_LOOP, 0, vertices->size() );
    geom->addPrimitiveSet(drawArrays.get());

    geode->addDrawable(geom.get());    
}

void Uncertainty::addSamples()
{
    const size_t max_samples = num_samples;
    // create a new geometry object with the distribution give in the covariance
    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
    osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
    color->push_back(osg::Vec4(1,1,1,1));
    geom->setColorArray(color.get());
    geom->setColorBinding( osg::Geometry::BIND_OVERALL );

    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
    for(size_t i=0; i<max_samples; i++) 
    {
	Eigen::Vector3d n;
	for(int i=0;i<3;i++)
	    n[i] = rand_norm();

	vertices->push_back( eigen2osg(n) );
    }
    geom->setVertexArray(vertices);

    osg::ref_ptr<osg::DrawArrays> drawArrays = new osg::DrawArrays( osg::PrimitiveSet::POINTS, 0, vertices->size() );
    geom->addPrimitiveSet(drawArrays.get());

    geode->addDrawable(geom.get());    
}
