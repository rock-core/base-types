#ifndef stereo_DistanceImageVisualization_H
#define stereo_DistanceImageVisualization_H

#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <base/samples/DistanceImage.hpp>

#include <boost/noncopyable.hpp>
#include "PointcloudVisualization.hpp"

namespace vizkit3d
{
    class DistanceImageVisualization
	: public PointcloudVisualization
        , boost::noncopyable
    {
	Q_OBJECT

    public:
        DistanceImageVisualization();
        ~DistanceImageVisualization();
	
	Q_INVOKABLE void updateData(const base::samples::DistanceImage& data); 
	Q_INVOKABLE void updateDistanceImage(const base::samples::DistanceImage& data)
        { updateData(data); }
	

    private:
    };
}
#endif
