#ifndef __ENVIRE_VIZ_UNCERTAINTY__
#define __ENVIRE_VIZ_UNCERTAINTY__

#include <boost/random/linear_congruential.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/normal_distribution.hpp>

#include <osg/PositionAttitudeTransform>
#include <Eigen/Core>

namespace vizkit3d
{

class Uncertainty : public osg::PositionAttitudeTransform
{
public:
    Uncertainty();

    void setMean( const Eigen::Vector2d& mean );
    void setMean( const Eigen::Vector3d& mean );
    void setCovariance( const Eigen::Matrix2d& cov );
    void setCovariance( const Eigen::Matrix3d& cov );

    void showSamples() { m_showSamples = true; }
    void hideSamples() { m_showSamples = false; }

    void setNumSamples(size_t samples) { num_samples = samples; }

private:
    void redraw( int dim );
    void addEllipse( int axis );
    void addSamples();

    /** random number generator */
    boost::minstd_rand rand_gen;
    boost::variate_generator<boost::minstd_rand&, boost::normal_distribution<> > rand_norm;

    bool m_showSamples;
    osg::ref_ptr<osg::Geode> geode;

    size_t num_samples;
    int dim;
};

}

#endif
