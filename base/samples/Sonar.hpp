#ifndef __BASE_SAMPLES_SONAR_HPP__
#define __BASE_SAMPLES_SONAR_HPP__

#include <vector>
#include <base/Float.hpp>
#include <base/Time.hpp>
#include <base/Angle.hpp>
#include <base/Deprecated.hpp>
BASE_TYPES_DEPRECATED_SUPPRESS_START
#include <base/samples/SonarBeam.hpp>
#include <base/samples/SonarScan.hpp>
BASE_TYPES_DEPRECATED_SUPPRESS_STOP

namespace base { namespace samples {

/** Representation of the data acquired by a sonar
 *
 * The data structure can be used to represent data from both mechanical
 * scanning and multibeam sonars. Let's see now the most common use-cases for
 * the structure
 *
 * In the mechanical-scanning sonar case, one would usually create one structure
 * per beam. The fromBeam helper can be used to create and initialize the
 * structure easily:
 *
 * @code
 * Sonar single_beam = Sonar::fromBeam(timestamp, bins, bearing);
 * @endcode
 *
 * In the multibeam case, one would initialize the structure and then fill it
 * with beams
 *
 * @code
 * Sonar multibeam(timestamp, bin_count);
 * for (int i = 0; i < bin_count; ++i)
 *    multibeam.pushBeam(bin_data_for_beam_i);
 * multibeam.setRegularBeamBearings(start_bearing, interval_bearing);
 * @encode
 *
 * A scanning sonar that would happen to be static can also have its beams
 * "aggregated" in a single structure, for instance if some multibeam-specific
 * algorithm can be used to process it. In this case, one timestamp should be
 * provided by beam, and a 'global' timestamp usually will need to be picked
 * in order for e.g. the stream aligner to work.
 *
 * @code
 * Sonar static_scanning_sonar(timestamp, bin_count);
 * for (int i = 0; i < bin_count; ++i)
 *    multibeam.pushBeam(time_for_beam_i, bin_data_for_beam_i);
 * multibeam.setRegularBeamBearings(start_bearing, interval_bearing);
 * @encode
 */
struct Sonar
{
public:
    /** A reference timestamp for the structure
     *
     * This is a bit of a bummer. When aggregating scanning sonar data, this
     * field has not much sense. However, aggregating scanning sonar is
     * meaningful only if the sonar does not move, so it is often possible to
     * find a timestamp that does make sense, but YMMV
     *
     * Advice: always set to a meaningful value as a lot of processing-related
     * algorithms expect one value. If you can't have *one*, then split your
     * scanning sonar into separate Sonar structure (one per beam).
     */
    base::Time time;

    /** The time at which the beam(s) acquisition started
     *
     * This is useful only if each beam have different acquisition times. If
     * not, the beam time is taken from the time field.
     *
     * @see getBeamAcquisitionStartTime 
     */
    std::vector<base::Time> timestamps;
    
    /** "Size" of one bin
     *
     * Sonars basically "slice" the echo return in the time domain, in what we
     * call "bins". Each bin has a starting point that is equal to
     *
     *     bin_duration * bin_index + beam_acquisition_time
     *
     * and is itself bin_duration wide. Conversion to the space domain requires
     * to know the speed of sound.
     *
     * @see getBeamAcquisitionStartTime getBinStartTime getBinStartDistance
     */
    base::Time bin_duration;
    
    /** Opening of the beam orthogonal to the device's Z direction
     *
     * This is usually along the scanning axis (for a scanning sonar), or in the
     * common plane of all the beams (for a multibeam sonar)
     */
    base::Angle beam_width;
    
    /** Opening of the beam along the device's Z direction
     *
     * This is usually orthogonal to the scanning axis (for a scanning sonar),
     * or orthogonal to the common plane of all the beams (for a multibeam sonar)
     */
    base::Angle beam_height;

    /** Each beam's bearing
     *
     * This is the position of the beam center with respect to the "front" of
     * the sonar (which is at 0)
     */
    std::vector<base::Angle> bearings;

    /** The speed of sound in water at the time of acquisition and in m/s*/
    float speed_of_sound;
    
    /** Number of bins in a beam */
    uint32_t bin_count;
    
    /** Number of beams in the structure */
    uint32_t beam_count;

    /** The bin values
     *
     * Bin values must be normalized, i.e. gain effects must be corrected and -
     * as much as made possible by the sonar's specification - converted to a
     * linear scale where 1.0 means that the signal received by the transducer
     * is equal to the signal transmittd.
     *
     * The data is stored beam-first, that is the data for beam N starts at N *
     * bin_count and ends at (N + 1) * bin_count
     */
    std::vector<float> bins;

    static float getSpeedOfSoundInWater() { return 1497.0; }

    Sonar()
        : speed_of_sound(getSpeedOfSoundInWater())
        , bin_count(0)
        , beam_count(0) {}

    Sonar(base::Time time, base::Time bin_duration, int bin_count, base::Angle beam_width, base::Angle beam_height)
        : time(time)
        , bin_duration(bin_duration)
        , beam_width(beam_width)
        , beam_height(beam_height)
        , speed_of_sound(getSpeedOfSoundInWater())
        , bin_count(bin_count)
        , beam_count(0)
    {
    }

    Sonar(base::Time time, base::Time bin_duration, int bin_count, base::Angle beam_width, base::Angle beam_height,
            int beam_count, bool per_beam_timestamps)
        : time(time)
        , bin_duration(bin_duration)
        , beam_width(beam_width)
        , beam_height(beam_height)
        , speed_of_sound(getSpeedOfSoundInWater())
        , bin_count(bin_count)
        , beam_count(beam_count)
    {
        resize(bin_count, beam_count, per_beam_timestamps);
    }

    void resize(int bin_count, int beam_count, bool per_beam_timestamps)
    {
        if (per_beam_timestamps)
            timestamps.resize(beam_count);

        bins.resize(beam_count * bin_count, base::unknown<float>());
        bearings.resize(beam_count, base::Angle::unknown());
    }

    /** Initializes a Sonar structure to represent a single beam
     */
    static Sonar fromSingleBeam(base::Time time, base::Time bin_duration, base::Angle beam_width, base::Angle beam_height,
            std::vector<float> const& bins, base::Angle bearing = base::Angle())
    {
        Sonar sample(time, bin_duration, bins.size(), beam_width, beam_height);
        sample.pushBeam(bins, bearing);
        return sample;
    }

    /** The start of a bin in the time domain, relative to the beam's
     * acquisition time
     */
    base::Time getBinRelativeStartTime(unsigned int bin_idx) const
    {
        return bin_duration * bin_idx;
    }

    /** The acquisition start of a beam
     */
    base::Time getBeamAcquisitionStartTime(unsigned int beam) const
    {
        if (timestamps.empty())
            return time;
        else
            return timestamps[beam];
    }

    /** The start of a bin in the time domain, absolute
     */
    base::Time getBinTime(unsigned int bin, unsigned int beam) const
    {
        return getBeamAcquisitionStartTime(beam) + getBinRelativeStartTime(bin);
    }

    /** Returns the distance of the start of one bin relative to the sonar's
     * emission point */
    base::Time getBinStartDistance(int bin) const
    {
        return getBinRelativeStartTime(bin) * speed_of_sound;
    }

    /** Sets the bearing field for a regular sampling
     *
     * beam_count must be properly set (through e.g. resize()) before calling
     * this. One usually calls it at the end of the beam construction.
     *
     * @arg start the start position
     * @arg interval the interval between two beams
     *
     */
    void setRegularBeamBearings(base::Angle start, base::Angle interval)
    {
        base::Angle angle(start);
        bearings.resize(beam_count);
        for (uint32_t i = 0; i < beam_count; ++i, angle += interval)
            bearings[i] = angle;
    }

    /** Add data for one beam
     *
     * @raise std::invalid_argument if the structure is using per-beam
     *   timestamps. In this case, use the overload that sets the beam time as
     *   well
     */
    void pushBeam(std::vector<float> const& bins)
    {
        if (!timestamps.empty())
            throw std::invalid_argument("cannot call pushBeam(bins): the structure uses per-beam timestamps, use pushBeams(time, bins) instead");

        pushBeamBins(bins);
    }

    /** Add data for one beam
     */
    void pushBeam(std::vector<float> const& bins, base::Angle bearing)
    {
        pushBeam(bins);
        bearings.push_back(bearing);
    }

    /** Add data for one beam
     */
    void pushBeam(base::Time const& beam_time, std::vector<float> const& beam_bins)
    {
        pushBeamBins(beam_bins);
        timestamps.push_back(beam_time);
    }

    /** Add data for one beam
     */
    void pushBeam(base::Time const& beam_time, std::vector<float> const& beam_bins, base::Angle bearing)
    {
        pushBeam(beam_time, beam_bins);
        bearings.push_back(bearing);
    }

    /** Adds a set of bins to the bin set, updating beam_bins
     *
     * One usually does not use this method directly, but one of the overloaded
     * pushBeam method
     *
     * @raise std::invalid_argument if the number of bins in the argument does
     *   not match bin_count
     */
    void pushBeamBins(std::vector<float> const& beam_bins)
    {
        if (beam_bins.size() != bin_count)
            throw std::invalid_argument("pushBeam: the provided beam does not match the expected bin_count");
        bins.insert(bins.end(), beam_bins.begin(), beam_bins.end());
        beam_count++;
    }

    /** Set data for one beam
     *
     * @raise std::invalid_argument if the structure is using per-beam
     *   timestamps. In this case, use the overload that sets the beam time as
     *   well
     */
    void setBeam(unsigned int beam, std::vector<float> const& bins)
    {
        if (!timestamps.empty())
            throw std::invalid_argument("cannot call setBeam(bins): the structure uses per-beam timestamps, use setBeams(time, bins) instead");

        setBeamBins(beam, bins);
    }

    /** Add data for one beam
     */
    void setBeam(unsigned int beam, std::vector<float> const& bins, base::Angle bearing)
    {
        setBeam(beam, bins);
        bearings[beam] = bearing;
    }

    /** Add data for one beam
     */
    void setBeam(unsigned int beam, base::Time const& beam_time, std::vector<float> const& beam_bins)
    {
        setBeamBins(beam, beam_bins);
        timestamps[beam] = beam_time;
    }

    /** Add data for one beam
     */
    void setBeam(unsigned int beam, base::Time const& beam_time, std::vector<float> const& beam_bins, base::Angle bearing)
    {
        setBeam(beam, beam_time, beam_bins);
        bearings[beam] = bearing;
    }

    /** Adds a set of bins to the bin set, updating beam_bins
     *
     * One usually does not use this method directly, but one of the overloaded
     * setBeam method
     *
     * @raise std::invalid_argument if the number of bins in the argument does
     *   not match bin_count
     */
    void setBeamBins(int beam, std::vector<float> const& beam_bins)
    {
        if (beam_bins.size() != bin_count)
            throw std::invalid_argument("pushBeam: the provided beam does not match the expected bin_count");
        std::copy(beam_bins.begin(), beam_bins.end(), bins.begin() + beam * bin_count);
    }

    /** Returns the bearing of a given beam
     *
     * This is the bearing of the center of the beam. A zero bearing means the
     * front of the device
     */
    base::Angle getBeamBearing(unsigned int beam) const
    {
        return bearings[beam];
    }

    /** Returns the bins of a given beam */
    std::vector<float> getBeamBins(unsigned int beam) const
    {
        std::vector<float> bins;
        getBeamBins(beam, bins);
        return bins;
    }

    /** Copies the bins of a given beam */
    void getBeamBins(unsigned int beam, std::vector<float>& beam_bins) const
    {
        beam_bins.resize(bin_count);
        std::vector<float>::const_iterator ptr = bins.begin() + beam * bin_count;
        std::copy(ptr, ptr + bin_count, beam_bins.begin());
    }

    /** Returns the data structure that represents a single beam */
    Sonar getBeam(unsigned int beam) const
    {
        return fromSingleBeam(
                getBeamAcquisitionStartTime(beam),
                bin_duration,
                beam_width,
                beam_height,
                getBeamBins(beam),
                getBeamBearing(beam));
    }

    /** Verify this structure's consistency
     */
    void validate()
    {
        if (bin_count * beam_count != bins.size())
            throw std::logic_error("the number of elements in 'bins' does not match the bin and beam counts");
        if (!timestamps.empty() && timestamps.size() != beam_count)
            throw std::logic_error("the number of elements in 'timestamps' does not match the beam count");
        if (bearings.size() != beam_count)
            throw std::logic_error("the number of elements in 'bearings' does not match the beam count");
    }

BASE_TYPES_DEPRECATED_SUPPRESS_START
    explicit Sonar(SonarScan const& old, float gain = 1)
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
            bins[i] = static_cast<float>(scan.data[i]) * gain;

        setRegularBeamBearings(old.getStartBearing(), old.getAngularResolution());
        validate();
    }

    explicit Sonar(SonarBeam const& old, float gain = 1)
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
        for (unsigned int i = 0; i < bin_count; ++i)
            bins[i] = static_cast<float>(old.beam[i]) * gain;
        pushBeam(bins, old.bearing);
    }
BASE_TYPES_DEPRECATED_SUPPRESS_STOP
};

}} // namespaces

#endif
