#include "DistanceImageVisualization.hpp"


using namespace vizkit3d;

DistanceImageVisualization::DistanceImageVisualization()
{
}

DistanceImageVisualization::~DistanceImageVisualization()
{
}

void DistanceImageVisualization::updateData(const base::samples::DistanceImage& data)
{
    PointcloudVisualization::updateData(data.getPointCloud());
}

