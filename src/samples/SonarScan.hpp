/*! \file sonar_scan.h
  \brief container for sonar scan data 
  */

#ifndef BASE_SAMPLES_SONARSCAN_H__
#define BASE_SAMPLES_SONARSCAN_H__

#include <base/Time.hpp>
#include <base/Angle.hpp>
#include <base/samples/SonarBeam.hpp>

namespace base { namespace samples { 

    struct SonarScan
    {
        public:
            /**
             * Initialize the sonar scan
             */
            SonarScan();

            SonarScan(uint16_t number_of_beams,uint16_t number_of_bins,Angle start_bearing,Angle angular_resolution,bool memory_layout_column=true);

            //makes a copy of other
            SonarScan(const SonarScan &other,bool bcopy = true);

            SonarScan &operator=(const SonarScan &other);

            //makes a copy of other
            void init(const SonarScan &other,bool bcopy = true);

            void init(uint16_t number_of_beams, uint16_t number_of_bins, Angle start_bearing, Angle angular_resolution,bool memory_layout_column = true, int val=-1);

            // if val is negative the sonar scan data will not be initialized
            void reset(int const val = 0);

            //returns the index/column of the sonar beam which 
            //holds information about the given bearing
            //returns -1 if the sonar scan has no sonar beam for the given bearing 
            int beamIndexForBearing(const Angle bearing,bool range_check=true)const;

            //returns true if a sonar beam was already added for the given 
            //bearing 
            //internally time_beams is used for checking 
            bool hasSonarBeam(const base::samples::SonarBeam &sonar_beam)const;

            bool hasSonarBeam(const Angle bearing)const;


            //adds a sonar beam to the sonar scan
            //throws an exception if the sonar scan cannot hold the sonar beam and resize is set to false
            //otherwise the sonar scan will be resized (the start angle is not changed!)
            //
            //the memory layout must be one sonar beam per row otherwise the function will throw a std::runtime_error
            void addSonarBeam(const base::samples::SonarBeam &sonar_beam,bool resize=true);

            //returns a sonar beam for a given bearing 
            //throws an exception if the sonar scans holds no information for the given bearing
            //the memory layout must be one beam per row 
            //
            //an exception is thrown if the sonar beam holds no information about the given bearing
            void getSonarBeam(const Angle bearing,SonarBeam &sonar_beam)const;

            //this toggles the memory layout between one sonar beam per row and one sonar beam per column
            //to add sonar beams the memory layout must be one sonar beam per row 
            void toggleMemoryLayout();

            void swap(SonarScan &sonar_scan);

            uint32_t getNumberOfBytes() const;

            /**
             * Returns the total count of bins in this sonar scan
             * @return Returns the overall number of bins (number_of_beams * number_of_bins)
             */
            uint32_t getBinCount() const;

            const std::vector<uint8_t> &getData() const;


            Angle getEndBearing()const;

            Angle getStartBearing()const;

            Angle getAngularResolution()const;

            //calculates the spatial resolution of the sonar scan in meter
            //this takes the sampling_interval and the speed of sound into account
            double getSpatialResolution()const;

            void setData(const std::vector<uint8_t> &data);
            
            void setData(const char *data, uint32_t size);
          
            uint8_t *getDataPtr();
            
            const uint8_t *getDataConstPtr() const;

            //The time at which this sonar scan has been captured
            base::Time                  time;

            //The raw data of the sonar scan
            std::vector<uint8_t>    	data;

            //Time stamp for each sonar beam
            //if the time stamp is 1 January 1970 
            //the beam is regarded as not be set 
            //vector can be empty in this case time 
            //is used for all beams
            std::vector<base::Time>    	time_beams;

            //number of beams 
            //this can be interpreted as image width
            uint16_t                    number_of_beams;

            //number of bins 
            //this can be interpreted as image height 
            uint16_t                    number_of_bins;

            /** The angle at which the reading starts. Zero is at the front of
             * the device and turns counter-clockwise. 
             * This value is in radians
             *
             * All beams are stored from left to right to match the memory 
             * layout of an image !!! Therefore the end bearing is usually 
             * smaller than the start bearing
             */
            Angle start_bearing;

            // Angle difference between two beams in radians
            // The beams are stored from left to right 
            // to match the memory layout of an image 
            //
            // This value must be always positive !
            Angle angular_resolution;

            //sampling interval of each range bin in secs
            double sampling_interval;

            //speed of sound
            //this takes the medium into account 
            float speed_of_sound;

            //horizontal beam width of the sonar beam in radians
            Angle beamwidth_horizontal;

            //vertical beam width of the sonar beam in radians
            Angle beamwidth_vertical;

            //if set to true one sonar beam is stored per column
            //otherwise per row 
            bool memory_layout_column; 

            //if set to true the bins are interpreted in polar coordinates
            //otherwise in Cartesian coordinates
            //
            //Some imaging sonars store their data in Cartesian rather than 
            //polar coordinates (BlueView)
            bool polar_coordinates;
    };
}}

#endif

