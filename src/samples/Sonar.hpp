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

    void resize(int bin_count, int beam_count, bool per_beam_timestamps);

    /** Initializes a Sonar structure to represent a single beam
     */
    static Sonar fromSingleBeam(base::Time time, base::Time bin_duration, base::Angle beam_width, base::Angle beam_height,
            std::vector<float> const& bins, base::Angle bearing = base::Angle(),
            float speed_of_sound = getSpeedOfSoundInWater());

    /** The start of a bin in the time domain, relative to the beam's
     * acquisition time
     */
    base::Time getBinRelativeStartTime(unsigned int bin_idx) const;

    /** The acquisition start of a beam
     */
    base::Time getBeamAcquisitionStartTime(unsigned int beam) const;

    /** The start of a bin in the time domain, absolute
     */
    base::Time getBinTime(unsigned int bin, unsigned int beam) const;

    /** Returns the distance of the start of one bin relative to the sonar's
     * emission point */
    float getBinStartDistance(unsigned int bin) const;

    /** Sets the bearing field for a regular sampling
     *
     * beam_count must be properly set (through e.g. resize()) before calling
     * this. One usually calls it at the end of the beam construction.
     *
     * @arg start the start position
     * @arg interval the interval between two beams
     *
     */
    void setRegularBeamBearings(base::Angle start, base::Angle interval);

    /** Add data for one beam
     *
     * @raise std::invalid_argument if the structure is using per-beam
     *   timestamps. In this case, use the overload that sets the beam time as
     *   well
     */
    void pushBeam(std::vector<float> const& bins);

    /** Add data for one beam
     */
    void pushBeam(std::vector<float> const& bins, base::Angle bearing);

    /** Add data for one beam
     */
    void pushBeam(base::Time const& beam_time, std::vector<float> const& beam_bins);


    /** Add data for one beam
     */
    void pushBeam(base::Time const& beam_time, std::vector<float> const& beam_bins, base::Angle bearing);

    /** Adds a set of bins to the bin set, updating beam_bins
     *
     * One usually does not use this method directly, but one of the overloaded
     * pushBeam method
     *
     * @raise std::invalid_argument if the number of bins in the argument does
     *   not match bin_count
     */
    void pushBeamBins(std::vector<float> const& beam_bins);

    /** Set data for one beam
     *
     * @raise std::invalid_argument if the structure is using per-beam
     *   timestamps. In this case, use the overload that sets the beam time as
     *   well
     */
    void setBeam(unsigned int beam, std::vector<float> const& bins);

    /** Add data for one beam
     */
    void setBeam(unsigned int beam, std::vector<float> const& bins, base::Angle bearing);

    /** Add data for one beam
     */
    void setBeam(unsigned int beam, base::Time const& beam_time, std::vector<float> const& beam_bins);

    /** Add data for one beam
     */
    void setBeam(unsigned int beam, base::Time const& beam_time, std::vector<float> const& beam_bins, base::Angle bearing);

    /** Adds a set of bins to the bin set, updating beam_bins
     *
     * One usually does not use this method directly, but one of the overloaded
     * setBeam method
     *
     * @raise std::invalid_argument if the number of bins in the argument does
     *   not match bin_count
     */
    void setBeamBins(int beam, std::vector<float> const& beam_bins);

    /** Returns the bearing of a given beam
     *
     * This is the bearing of the center of the beam. A zero bearing means the
     * front of the device
     */
    base::Angle getBeamBearing(unsigned int beam) const;

    /** Returns the bins of a given beam */
    std::vector<float> getBeamBins(unsigned int beam) const;

    /** Copies the bins of a given beam */
    void getBeamBins(unsigned int beam, std::vector<float>& beam_bins) const;

    /** Returns the data structure that represents a single beam */
    Sonar getBeam(unsigned int beam) const;

    /** Verify this structure's consistency
     */
    void validate();

BASE_TYPES_DEPRECATED_SUPPRESS_START
    explicit Sonar(SonarScan const& old, float gain = 1);

    explicit Sonar(SonarBeam const& old, float gain = 1);

    base::samples::SonarBeam toSonarBeam(float gain = 1);

    base::samples::SonarScan toSonarScan(float gain = 1);
BASE_TYPES_DEPRECATED_SUPPRESS_STOP
};

}} // namespaces

#endif
