#ifndef BASE_SAMPLES_SONARBEAM_H__
#define BASE_SAMPLES_SONARBEAM_H__

#include <vector>
#include <boost/cstdint.hpp>
#include <base/Time.hpp>
#include <base/Angle.hpp>
#include <limits>

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

        SonarBeam(const SonarBeam &other)
        {
            init(other);
        }
            
        //calculates the spatial resolution of the sonar beam in meter
        //this takes the sampling_interval and the speed of sound into account
        double getSpatialResolution()const
        {
            //the sampling interval includes the time for 
            //the sound traveling from the transiter to the target an back
            //to receiver
            return sampling_interval*0.5*speed_of_sound;
        }

        SonarBeam &operator=(const SonarBeam &other)
        {
            init(other);
            return *this;
        }

        void init(const SonarBeam &other)
        {
            time = other.time;
            bearing = other.bearing;
            sampling_interval = other.sampling_interval;
            speed_of_sound = other.speed_of_sound;
            beamwidth_horizontal = other.beamwidth_horizontal;
            beamwidth_vertical = other.beamwidth_vertical;
            beam = other.beam;
        }

        void swap(SonarBeam &other)
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
    };
}} 

#endif
