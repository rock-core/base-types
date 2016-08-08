#include "Sonar.hpp"

void base::samples::Sonar::resize(int bin_count, int beam_count, bool per_beam_timestamps)
{
    if (per_beam_timestamps)
        timestamps.resize(beam_count);
    else
        timestamps.clear();

    bearings.resize(beam_count, base::Angle::unknown());
    bins.resize(beam_count * bin_count, base::unknown<float>());
    this->bin_count = bin_count;
    this->beam_count = beam_count;
}

base::samples::Sonar base::samples::Sonar::fromSingleBeam(base::Time time, base::Time bin_duration, base::Angle beam_width, base::Angle beam_height, const std::vector< float >& bins, base::Angle bearing, float speed_of_sound)
{
    Sonar sample(time, bin_duration, bins.size(), beam_width, beam_height);
    sample.speed_of_sound = speed_of_sound;
    sample.pushBeam(bins, bearing);
    return sample;
}

base::Time base::samples::Sonar::getBinRelativeStartTime(unsigned int bin_idx) const
{
    return bin_duration * bin_idx;
}

base::Time base::samples::Sonar::getBeamAcquisitionStartTime(unsigned int beam) const
{
    if (timestamps.empty())
        return time;
    else
        return timestamps[beam];
}

base::Time base::samples::Sonar::getBinTime(unsigned int bin, unsigned int beam) const
{
    return getBeamAcquisitionStartTime(beam) + getBinRelativeStartTime(bin);
}

float base::samples::Sonar::getBinStartDistance(unsigned int bin) const
{
    return getBinRelativeStartTime(bin).toSeconds() * speed_of_sound;
}

void base::samples::Sonar::setRegularBeamBearings(base::Angle start, base::Angle interval)
{
    base::Angle angle(start);
    bearings.resize(beam_count);
    for (uint32_t i = 0; i < beam_count; ++i, angle += interval)
        bearings[i] = angle;
}

void base::samples::Sonar::pushBeam(const std::vector< float >& bins)
{
    if (!timestamps.empty())
        throw std::invalid_argument("cannot call pushBeam(bins): the structure uses per-beam timestamps, use pushBeams(time, bins) instead");

    pushBeamBins(bins);
}

void base::samples::Sonar::pushBeam(const std::vector< float >& bins, base::Angle bearing)
{
    pushBeam(bins);
    bearings.push_back(bearing);
}

void base::samples::Sonar::pushBeam(const base::Time& beam_time, const std::vector< float >& beam_bins)
{ 
    pushBeamBins(beam_bins);
    timestamps.push_back(beam_time);
}

void base::samples::Sonar::pushBeam(const base::Time& beam_time, const std::vector< float >& beam_bins, base::Angle bearing)
{
    pushBeam(beam_time, beam_bins);
    bearings.push_back(bearing);
}

void base::samples::Sonar::pushBeamBins(const std::vector< float >& beam_bins)
{
    if (beam_bins.size() != bin_count)
        throw std::invalid_argument("pushBeam: the provided beam does not match the expected bin_count");
    bins.insert(bins.end(), beam_bins.begin(), beam_bins.end());
    beam_count++;
}

void base::samples::Sonar::setBeam(unsigned int beam, const std::vector< float >& bins)
{
    if (!timestamps.empty())
        throw std::invalid_argument("cannot call setBeam(bins): the structure uses per-beam timestamps, use setBeams(time, bins) instead");

    setBeamBins(beam, bins);
}

void base::samples::Sonar::setBeam(unsigned int beam, const std::vector< float >& bins, base::Angle bearing)
{
    setBeam(beam, bins);
    bearings[beam] = bearing;
}

void base::samples::Sonar::setBeam(unsigned int beam, const base::Time& beam_time, const std::vector< float >& beam_bins)
{
    setBeamBins(beam, beam_bins);
    timestamps[beam] = beam_time;
}

void base::samples::Sonar::setBeam(unsigned int beam, const base::Time& beam_time, const std::vector< float >& beam_bins, base::Angle bearing)
{
    setBeam(beam, beam_time, beam_bins);
    bearings[beam] = bearing;
}

void base::samples::Sonar::setBeamBins(int beam, const std::vector< float >& beam_bins)
{
    if (beam_bins.size() != bin_count)
        throw std::invalid_argument("pushBeam: the provided beam does not match the expected bin_count");
    std::copy(beam_bins.begin(), beam_bins.end(), bins.begin() + beam * bin_count);
}

base::Angle base::samples::Sonar::getBeamBearing(unsigned int beam) const
{
    return bearings[beam];
}

std::vector< float > base::samples::Sonar::getBeamBins(unsigned int beam) const
{
    std::vector<float> bins;
    getBeamBins(beam, bins);
    return bins;
}

void base::samples::Sonar::getBeamBins(unsigned int beam, std::vector< float >& beam_bins) const
{
    beam_bins.resize(bin_count);
    std::vector<float>::const_iterator ptr = bins.begin() + beam * bin_count;
    std::copy(ptr, ptr + bin_count, beam_bins.begin());
}

base::samples::Sonar base::samples::Sonar::getBeam(unsigned int beam) const
{
    return fromSingleBeam(
        getBeamAcquisitionStartTime(beam),
        bin_duration,
        beam_width,
        beam_height,
        getBeamBins(beam),
        getBeamBearing(beam),
        speed_of_sound);
}

void base::samples::Sonar::validate()
{
    if (bin_count * beam_count != bins.size())
        throw std::logic_error("the number of elements in 'bins' does not match the bin and beam counts");
    if (!timestamps.empty() && timestamps.size() != beam_count)
        throw std::logic_error("the number of elements in 'timestamps' does not match the beam count");
    if (bearings.size() != beam_count)
        throw std::logic_error("the number of elements in 'bearings' does not match the beam count");
}

base::samples::Sonar::Sonar(SonarScan const& old, float gain) 
    : time(old.time)
    , timestamps(old.time_beams)
    , bin_duration(base::Time::fromSeconds(old.getSpatialResolution() / old.speed_of_sound))
    , beam_width(old.beamwidth_horizontal)
    , beam_height(old.beamwidth_vertical)
    , speed_of_sound(old.speed_of_sound)
    , bin_count(old.number_of_bins)
    , beam_count(old.number_of_beams)
{
    if (!old.polar_coordinates)
            throw std::invalid_argument("there's not such thing as a non-polar sonar device, fix your driver");

    bins.resize(bin_count * beam_count);

    SonarScan scan(old);
    if (old.memory_layout_column)
        scan.toggleMemoryLayout();
    for (unsigned int i = 0; i < bins.size(); ++i)
        bins[i] = static_cast<float>(scan.data[i] * 1.0 / 255)  * gain;

    setRegularBeamBearings(old.getStartBearing(), old.getAngularResolution());
    validate();
}
        

base::samples::Sonar::Sonar(SonarBeam const& old, float gain)
    : time(old.time)
    , timestamps()
    , bin_duration(base::Time::fromSeconds(old.sampling_interval / 2.0))
    , beam_width(base::Angle::fromRad(old.beamwidth_horizontal))
    , beam_height(base::Angle::fromRad(old.beamwidth_vertical))
    , speed_of_sound(old.speed_of_sound)
    , bin_count(old.beam.size())
    , beam_count(0)
{
    std::vector<float> bins;
    bins.resize(bin_count);
    for (unsigned int i = 0; i < bin_count; ++i)
        bins[i] = static_cast<float>(old.beam[i] * 1.0 / 255) * gain;
    pushBeam(bins, old.bearing);
}

base::samples::SonarBeam base::samples::Sonar::toSonarBeam(float gain)
{
    base::samples::SonarBeam sonar_beam;
    sonar_beam.time = time;
    sonar_beam.speed_of_sound = speed_of_sound;
    sonar_beam.beamwidth_horizontal = beam_width.rad;
    sonar_beam.beamwidth_vertical = beam_height.rad;
    sonar_beam.bearing = bearings[0];
    sonar_beam.sampling_interval = bin_duration.toSeconds() * 2.0;

    // if any value of sonar data is higher than 1, normalize it
    std::vector<float> raw_data(bins.begin(), bins.end());
    std::vector<float>::iterator max = std::max_element(raw_data.begin(), raw_data.end());
    if (*max > 1)
        std::transform(raw_data.begin(), raw_data.end(), raw_data.begin(), std::bind2nd(std::divides<float>(), *max));

    std::transform(raw_data.begin(), raw_data.end(), raw_data.begin(), std::bind2nd(std::multiplies<float>(), 255 * gain));

    std::vector<uint8_t> data(raw_data.begin(), raw_data.end());
    sonar_beam.beam = data;
    return sonar_beam;
}

base::samples::SonarScan base::samples::Sonar::toSonarScan(float gain)
{
    base::samples::SonarScan sonar_scan;
    sonar_scan.time = time;
    sonar_scan.time_beams = timestamps;
    sonar_scan.speed_of_sound = speed_of_sound;
    sonar_scan.number_of_bins = bin_count;
    sonar_scan.number_of_beams = beam_count;
    sonar_scan.beamwidth_horizontal = beam_width;
    sonar_scan.beamwidth_vertical = beam_height;
    sonar_scan.start_bearing = bearings[0];
    sonar_scan.angular_resolution = base::Angle::fromRad(beam_width.rad / beam_count);
    sonar_scan.memory_layout_column = false;
    sonar_scan.polar_coordinates = true;

    // if any value of sonar data is higher than 1, normalize it
    std::vector<float> raw_data(bins.begin(), bins.end());
    std::vector<float>::iterator max = std::max_element(raw_data.begin(), raw_data.end());
    if (*max > 1)
        std::transform(raw_data.begin(), raw_data.end(), raw_data.begin(), std::bind2nd(std::divides<float>(), *max));

    std::transform(raw_data.begin(), raw_data.end(), raw_data.begin(), std::bind2nd(std::multiplies<float>(), 255 * gain));

    std::vector<uint8_t> data(raw_data.begin(), raw_data.end());
    sonar_scan.data = data;
    return sonar_scan;
}













