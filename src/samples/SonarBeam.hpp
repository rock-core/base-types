#ifndef BASE_SAMPLES_SONARBEAM_H__
#define BASE_SAMPLES_SONARBEAM_H__

#include <boost/cstdint.hpp>
#include <base/Time.hpp>
#include <base/Angle.hpp>

namespace base { namespace samples {

    struct SonarBeam {
        typedef boost::uint8_t uint8_t;

        //timestamp of the sonar beam 
        Time time;

        //direction of the sonar beam in radians [-pi,+pi]
        //zero is at the front 
        Angle bearing;

        //sampling interval of each range bin in secs
        double sampling_interval;

        //speed of sound
        //this takes the medium into account 
        float speed_of_sound;

        //horizontal beamwidth of the sonar beam in radians
        float beamwidth_horizontal;

        //vertical beamwidth of the sonar beam in radians
        float beamwidth_vertical;

        //received echoes (bins) along the beam
        std::vector<uint8_t> beam;

        SonarBeam(): 
            sampling_interval(std::numeric_limits<double>::signaling_NaN()), 
            speed_of_sound(std::numeric_limits<float>::signaling_NaN()), 
            beamwidth_horizontal(std::numeric_limits<float>::signaling_NaN()), 
            beamwidth_vertical(std::numeric_limits<float>::signaling_NaN()){} 

        SonarBeam(const SonarBeam &other);
            
        //calculates the spatial resolution of the sonar beam in meter
        //this takes the sampling_interval and the speed of sound into account
        double getSpatialResolution()const;

        SonarBeam &operator=(const SonarBeam &other);

        void init(const SonarBeam &other);

        void swap(SonarBeam &other);
    };
}} 

#endif
