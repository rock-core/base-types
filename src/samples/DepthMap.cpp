#include "DepthMap.hpp"

#include <base/Float.hpp>

namespace base { namespace samples {

void DepthMap::reset()
{
    timestamps.clear();
    vertical_projection = POLAR;
    horizontal_projection = POLAR;
    vertical_interval.clear();
    horizontal_interval.clear();
    vertical_size = 0;
    horizontal_size = 0;
    distances.clear();
    remissions.clear();
}

DepthMap::DepthMatrixMap DepthMap::getDistanceMatrixMap()
{
    return (DepthMatrixMap(distances.data(), vertical_size, horizontal_size));
}

DepthMap::DepthMatrixMapConst DepthMap::getDistanceMatrixMapConst() const
{
    return (DepthMatrixMapConst(distances.data(), vertical_size, horizontal_size));
}

DepthMap::DEPTH_MEASUREMENT_STATE DepthMap::getIndexState(std::size_t index) const
{
    if(index >= distances.size())
        throw std::out_of_range("Invalid measurement index given");
        
    return getMeasurementState(distances[index]);
}

DepthMap::DEPTH_MEASUREMENT_STATE DepthMap::getMeasurementState(DepthMap::uint32_t v_index, DepthMap::uint32_t h_index) const
{
    if(!checkSizeConfig())
        throw std::out_of_range("Vertical and horizontal size does not match the distance array size.");
    if(v_index >= vertical_size || h_index >= horizontal_size)
        throw std::out_of_range("Invalid vertical or horizontal index given.");
        
    return getMeasurementState(distances[getIndex(v_index,h_index)]);
}

DepthMap::DEPTH_MEASUREMENT_STATE DepthMap::getMeasurementState(DepthMap::scalar distance) const
{
    if(isNaN<scalar>(distance))
        return MEASUREMENT_ERROR;
    else if(isInfinity<scalar>(distance))
        return TOO_FAR;
    else if(distance <= 0.0)
        return TOO_NEAR;
    else
        return VALID_MEASUREMENT;
}

bool DepthMap::isIndexValid(std::size_t index) const
{
    if(index >= distances.size())
        throw std::out_of_range("Invalid measurement index given");
    
    return isMeasurementValid(distances[index]);
}

bool DepthMap::isMeasurementValid(DepthMap::uint32_t v_index, DepthMap::uint32_t h_index) const
{
    if(!checkSizeConfig())
        throw std::out_of_range("Vertical and horizontal size does not match the distance array size.");
    if(v_index >= vertical_size || h_index >= horizontal_size)
        throw std::out_of_range("Invalid vertical or horizontal index given.");
    
    return isMeasurementValid(distances[getIndex(v_index,h_index)]);
}

bool DepthMap::isMeasurementValid(DepthMap::scalar distance) const
{
    return getMeasurementState(distance) == VALID_MEASUREMENT;
}

std::size_t DepthMap::getIndex(DepthMap::uint32_t v_index, DepthMap::uint32_t h_index) const
{
    return ((size_t)v_index * (size_t)horizontal_size + (size_t)h_index);
}

bool DepthMap::checkSizeConfig() const
{
    return ((((size_t)vertical_size * (size_t)horizontal_size) == distances.size()) ? true : false);
}

}} //end namespace base::samples

