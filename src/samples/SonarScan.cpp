#include "SonarScan.hpp"

#include <stdint.h>
#include <memory.h>
#include <vector>
#include <string>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace base { namespace samples {

SonarScan::SonarScan()
    : number_of_beams(0)
    , number_of_bins(0)
    , sampling_interval(0)
    , speed_of_sound(0)
    , beamwidth_horizontal(Angle::fromRad(0))
    , beamwidth_vertical(Angle::fromRad(0))
    , memory_layout_column(true)
    , polar_coordinates(true)
{
    reset();
}

SonarScan::SonarScan(uint16_t number_of_beams, uint16_t number_of_bins, Angle start_bearing, Angle angular_resolution, bool memory_layout_column)
{
    init(number_of_beams,number_of_bins,start_bearing,angular_resolution,memory_layout_column);
}

SonarScan::SonarScan(const SonarScan& other, bool bcopy)
{
    init(other,bcopy);
}


SonarScan& SonarScan::operator=(const SonarScan& other)
{
    init(other,true);
    return *this;
}

void SonarScan::init(const SonarScan& other, bool bcopy)
{
    init(other.number_of_beams,other.number_of_bins,other.start_bearing,other.angular_resolution,other.memory_layout_column);
    time = other.time;

    beamwidth_vertical = other.beamwidth_vertical;
    beamwidth_horizontal = other.beamwidth_horizontal;
    sampling_interval = other.sampling_interval;
    speed_of_sound = other.speed_of_sound;
    polar_coordinates = other.polar_coordinates;
    if(bcopy){
        setData(other.getData());
        time_beams = other.time_beams;
    }
}

void SonarScan::init(uint16_t number_of_beams, uint16_t number_of_bins, Angle start_bearing, Angle angular_resolution, bool memory_layout_column, int val)
{
    //change size if the sonar scan does not fit
    if(this->number_of_beams != number_of_beams || this->number_of_bins !=  number_of_bins)
    {
        this->number_of_beams = number_of_beams;
        this->number_of_bins = number_of_bins;
        data.resize(number_of_beams*number_of_bins);
    }
    this->start_bearing = start_bearing;
    this->angular_resolution = angular_resolution;
    this->memory_layout_column = memory_layout_column;
    speed_of_sound = 0;
    beamwidth_horizontal = Angle::fromRad(0);
    beamwidth_vertical = Angle::fromRad(0);
    reset(val);
}

void SonarScan::reset(const int val)
{
    this->time = Time();
    if (this->data.size() > 0 && val >= 0) 
    {
        memset(&this->data[0], val%256, this->data.size());
    }
    time_beams.clear();
}

int SonarScan::beamIndexForBearing(const Angle bearing, bool range_check) const
{
    double temp_rad = (start_bearing-bearing).rad;
    int index = round(temp_rad/angular_resolution.rad);
    if(range_check && (index < 0 || index >= number_of_beams))
        return -1;
    return index;
}

bool SonarScan::hasSonarBeam(const SonarBeam& sonar_beam) const
{
    return hasSonarBeam(sonar_beam.bearing);
}

bool SonarScan::hasSonarBeam(const Angle bearing) const
{
    int index = beamIndexForBearing(bearing);
    if(index < 0)
        return false;

    //it is assumed that all data are set at once (imaging sonar)
    if(time_beams.empty()) 
        return true;

    //it is assumed that the data are set beam by beam (scanning sonar)
    if(time_beams[index].microseconds != 0)
        return true;
    return false;
}

void SonarScan::addSonarBeam(const SonarBeam& sonar_beam, bool resize)
{
    if(memory_layout_column)
        throw std::runtime_error("addSonarBeam: cannot add sonar beam because the memory layout is not supported. Call toggleMemoryLayout()");

    if(number_of_bins < sonar_beam.beam.size())
        throw std::runtime_error("addSonarBeam: cannot add sonar beam: too many bins");

    int index = beamIndexForBearing(sonar_beam.bearing,false);
    if(index < 0)
        throw std::runtime_error("addSonarBeam: negative index!");
    if(index >= number_of_beams)
    {
        if(!resize)
            throw std::runtime_error("addSonarBeam: bearing is out of range");
        number_of_beams = index+1;
        data.resize(number_of_beams*number_of_bins);
    }
    if(time_beams.size() != number_of_beams)
        time_beams.resize(number_of_beams);

    time_beams[index] = sonar_beam.time;
    sampling_interval = sonar_beam.sampling_interval;
    beamwidth_vertical = Angle::fromRad(sonar_beam.beamwidth_vertical);
    beamwidth_horizontal = Angle::fromRad(sonar_beam.beamwidth_horizontal);
    speed_of_sound = sonar_beam.speed_of_sound;
    memcpy(&data[index*number_of_bins],&sonar_beam.beam[0],sonar_beam.beam.size());
}

void SonarScan::getSonarBeam(const Angle bearing, SonarBeam& sonar_beam) const
{
    if(memory_layout_column)
        throw std::runtime_error("getSonarBeam: Wrong memory layout!");
    int index = beamIndexForBearing(bearing);
    if(index<0)
        throw std::runtime_error("getSonarBeam: No Data for the given bearing!");

    sonar_beam.beam.resize(number_of_bins);
    memcpy(&sonar_beam.beam[0],&data[number_of_bins*index],number_of_bins);
    if((int)time_beams.size() > index)
        sonar_beam.time = time_beams[index];
    else
        sonar_beam.time = time;
    sonar_beam.speed_of_sound = speed_of_sound;
    sonar_beam.beamwidth_horizontal = beamwidth_horizontal.rad;
    sonar_beam.beamwidth_vertical = beamwidth_vertical.rad;
    sonar_beam.sampling_interval = sampling_interval;
    sonar_beam.bearing = bearing;
}

void SonarScan::toggleMemoryLayout()
{
    std::vector<uint8_t> temp;
    temp.resize(data.size());

    if(memory_layout_column)
    {
        for(int row=0;row < number_of_beams;++row)
            for(int column=0;column < number_of_bins;++column)
                temp[row*number_of_bins+column] =data[column*number_of_beams+row]; 
    }
    else
    {
        for(int row=0;row < number_of_beams;++row)
            for(int column=0;column < number_of_bins;++column)
                temp[column*number_of_beams+row] =data[row*number_of_bins+column]; 
    }
    memory_layout_column = !memory_layout_column;
    data.swap(temp);
}

void SonarScan::swap(SonarScan& sonar_scan)
{
    //swap vector
    data.swap(sonar_scan.data);

    //copy values to temp
    Time temp_time = sonar_scan.time;
    Angle temp_beamwidth_vertical = sonar_scan.beamwidth_vertical;
    Angle temp_beamwidth_horizontal = sonar_scan.beamwidth_horizontal;
    double temp_sampling_interval = sonar_scan.sampling_interval;
    uint16_t temp_number_of_beams = sonar_scan.number_of_beams;
    uint16_t temp_number_of_bins = sonar_scan.number_of_bins;
    Angle temp_start_bearing = sonar_scan.start_bearing;
    Angle temp_angular_resolution = sonar_scan.angular_resolution;
    bool temp_memory_layout_column = sonar_scan.memory_layout_column;
    bool temp_polar_coordinates = sonar_scan.polar_coordinates;
    float temp_speed_of_sound = sonar_scan.speed_of_sound;

    //copy values
    sonar_scan.time = time;
    sonar_scan.beamwidth_vertical = beamwidth_vertical;
    sonar_scan.beamwidth_horizontal = beamwidth_horizontal;
    sonar_scan.sampling_interval = sampling_interval;
    sonar_scan.number_of_beams = number_of_beams;
    sonar_scan.number_of_bins = number_of_bins;
    sonar_scan.start_bearing = start_bearing;
    sonar_scan.angular_resolution = angular_resolution;
    sonar_scan.memory_layout_column = memory_layout_column;
    sonar_scan.polar_coordinates = polar_coordinates;
    sonar_scan.speed_of_sound = speed_of_sound;

    time = temp_time;
    beamwidth_vertical = temp_beamwidth_vertical;
    beamwidth_horizontal = temp_beamwidth_horizontal;
    sampling_interval = temp_sampling_interval;
    number_of_beams = temp_number_of_beams;
    number_of_bins = temp_number_of_bins;
    start_bearing = temp_start_bearing;
    angular_resolution = temp_angular_resolution;
    memory_layout_column = temp_memory_layout_column;
    polar_coordinates = temp_polar_coordinates;
    speed_of_sound = temp_speed_of_sound;
}

uint32_t SonarScan::getNumberOfBytes() const
{
    return data.size();
}

uint32_t SonarScan::getBinCount() const
{
    return number_of_beams * number_of_bins;
}

const std::vector< uint8_t >& SonarScan::getData() const
{
    return this->data;
}

Angle SonarScan::getEndBearing() const
{
    return start_bearing-angular_resolution*(number_of_beams-1);
}

Angle SonarScan::getStartBearing() const
{
    return start_bearing;
}

Angle SonarScan::getAngularResolution() const
{
    return angular_resolution;
}

double SonarScan::getSpatialResolution() const
{
    //the sampling interval includes the time for 
    //the sound traveling from the transmitter to the target an back
    //to the receiver
    return sampling_interval*0.5*speed_of_sound;
}

void SonarScan::setData(const std::vector< uint8_t >& data)
{
    this->data = data;
}

void SonarScan::setData(const char* data, uint32_t size)
{
    if (size != this->data.size())
    {
        std::cerr << "SonarScan: "
            << __FUNCTION__ << " (" << __FILE__ << ", line "
            << __LINE__ << "): " << "size mismatch in setData() ("
                                        << size << " != " << this->data.size() << ")"
                                        << std::endl;
        return;
    }
    memcpy(&this->data[0], data, size);
}

uint8_t* SonarScan::getDataPtr()
{
    return static_cast<uint8_t *>(&this->data[0]);
}

const uint8_t* SonarScan::getDataConstPtr() const
{
    return static_cast<const uint8_t *>(&this->data[0]);
}


}} //end namespace base::samples