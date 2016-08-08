#include "SonarBeam.hpp"

#include <vector>
#include <limits>

base::samples::SonarBeam::SonarBeam(const base::samples::SonarBeam& other)
{
    init(other);
}

double base::samples::SonarBeam::getSpatialResolution() const
{
    //the sampling interval includes the time for 
    //the sound traveling from the transiter to the target an back
    //to receiver
    return sampling_interval*0.5*speed_of_sound;
}

base::samples::SonarBeam& base::samples::SonarBeam::operator=(const base::samples::SonarBeam& other)
{
    init(other);
    return *this;
}

void base::samples::SonarBeam::init(const base::samples::SonarBeam& other)
{
    time = other.time;
    bearing = other.bearing;
    sampling_interval = other.sampling_interval;
    speed_of_sound = other.speed_of_sound;
    beamwidth_horizontal = other.beamwidth_horizontal;
    beamwidth_vertical = other.beamwidth_vertical;
    beam = other.beam;
}

void base::samples::SonarBeam::swap(base::samples::SonarBeam& other)
{
    Time temp_time = time;
    Angle temp_bearing = bearing;
    double temp_sampling_interval = sampling_interval;
    float temp_speed_of_sound = speed_of_sound;
    float temp_beamwidth_horizontal = beamwidth_horizontal;
    float temp_beamwidth_vertical= beamwidth_vertical;

    time = other.time;
    bearing = other.bearing;
    sampling_interval = other.sampling_interval;
    speed_of_sound = other.speed_of_sound;
    beamwidth_horizontal = other.beamwidth_horizontal;
    beamwidth_vertical = other.beamwidth_vertical;
    beam.swap(other.beam);

    other.time = temp_time;
    other.bearing = temp_bearing;
    other.sampling_interval = temp_sampling_interval;
    other.speed_of_sound = temp_speed_of_sound;
    other.beamwidth_horizontal = temp_beamwidth_horizontal;
    other.beamwidth_vertical = temp_beamwidth_vertical;
}




