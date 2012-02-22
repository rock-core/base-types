/*! \file sonar_scan.h
  \brief container for sonar scan data 
  */

#ifndef BASE_SAMPLES_SONARSCAN_H__
#define BASE_SAMPLES_SONARSCAN_H__

#include <stdint.h>
#include <memory.h>
#include <vector>
#include <string>
#include <iostream>
#include <sstream>
#include <stdexcept>

#include "base/time.h"
#include "base/angle.h"
#include "base/samples/sonar_beam.h"

namespace base { namespace samples { 

    struct SonarScan
    {
        public:
            /**
             * Initialize the sonar scan
             */
            SonarScan() :
                number_of_beams(0),number_of_bins(0),sampling_interval(0), speed_of_sound(0),memory_layout_column(true),polar_coordinates(true)
            {
                reset();
            }

            SonarScan(uint16_t number_of_beams,uint16_t number_of_bins,Angle start_bearing,Angle angular_resolution,bool memory_layout_column=true)
            {
                init(number_of_beams,number_of_bins,start_bearing,angular_resolution,memory_layout_column);
            }

            //makes a copy of other
            SonarScan(const SonarScan &other,bool bcopy = true)
            {
                init(other,bcopy);
            }

            SonarScan &operator=(const SonarScan &other)
            {
                init(other,true);
                return *this;
            }

            //makes a copy of other
            void init(const SonarScan &other,bool bcopy = true)
            {
                init(other.number_of_beams,other.number_of_bins,other.start_bearing,other.angular_resolution,other.memory_layout_column);
                beamwidth_vertical = other.beamwidth_vertical;
                beamwidth_horizontal = other.beamwidth_horizontal;
                sampling_interval = other.sampling_interval;
                if(bcopy)
                    setData(other.getData());
            }

            void init(uint16_t number_of_beams, uint16_t number_of_bins, Angle start_bearing, Angle angular_resolution,bool memory_layout_column = true, int val=-1)
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
                reset(val);
            }

            // if val is negative the sonar scan data will not be initialized
            void reset(int const val = 0)
            {
                this->time = base::Time();
                if (this->data.size() > 0 && val >= 0) 
                {
                    memset(&this->data[0], val%256, this->data.size());
                }
                time_beams.clear();
            }

            //returns the index/column of the sonar beam which 
            //holds information about the given bearing
            //returns -1 if the sonar scan has no sonar beam for the given bearing 
            int beamIndexForBearing(const Angle bearing,bool range_check=true)const
            {
               double temp_rad = (start_bearing-bearing).rad;
               if(temp_rad<0)
                   temp_rad+=2.0*M_PI;
               int index = round(temp_rad/angular_resolution.rad);
               if(range_check && (index < 0 || index >=(int) data.size()))
                   return -1;
               return index;
            }

            //returns true if a sonar beam was already added for the given 
            //bearing 
            //internally time_beams is used for checking 
            bool hasSonarBeam(const base::samples::SonarBeam &sonar_beam)const
            {
                return hasSonarBeam(sonar_beam.bearing);
            }

            bool hasSonarBeam(const Angle bearing)const
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


            //adds a sonar beam to the sonar scan
            //throws an exception if the sonar scan cannot hold the sonar beam and resize is set to false
            //otherwise the sonar scan will be resized (the start angle is not changed!)
            //
            //the memory layout must be one sonar beam per row otherwise the function will throw a std::runtime_error
            void addSonarBeam(const base::samples::SonarBeam &sonar_beam,bool resize=true)
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

            //returns a sonar beam for a given bearing 
            //throws an exception if the sonar scans holds no information for the given bearing
            //the memory layout must be one beam per row 
            //
            //an exception is thrown if the sonar beam holds no information about the given bearing
            void getSonarBeam(const Angle bearing,SonarBeam &sonar_beam)const
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

            //this toggles the memory layout between one sonar beam per row and one sonar beam per column
            //to add sonar beams the memory layout must be one sonar beam per row 
            void toggleMemoryLayout()
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

            void swap(SonarScan &sonar_scan)
            {
                //swap vector
                data.swap(sonar_scan.data);

                //copy values to temp
                base::Time temp_time = sonar_scan.time;
                Angle temp_beamwidth_vertical = sonar_scan.beamwidth_vertical;
                Angle temp_beamwidth_horizontal = sonar_scan.beamwidth_horizontal;
                double temp_sampling_interval = sonar_scan.sampling_interval;
                uint16_t temp_number_of_beams = sonar_scan.number_of_beams;
                uint16_t temp_number_of_bins = sonar_scan.number_of_bins;
                Angle temp_start_bearing = sonar_scan.start_bearing;
                Angle temp_angular_resolution = sonar_scan.angular_resolution;
                bool temp_memory_layout_column = sonar_scan.memory_layout_column;
                bool temp_polar_coordinates = sonar_scan.polar_coordinates;

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
            }

            inline uint32_t getNumberOfBytes() const {
                return data.size();
            }

            /**
             * Returns the total count of bins in this sonar scan
             * @return Returns the overall number of bins (number_of_beams * number_of_bins)
             */
            inline uint32_t getBinCount() const {
                return number_of_beams * number_of_bins;
            }

            inline const std::vector<uint8_t> &getData() const {
                return this->data;
            }


            Angle getEndBearing()const
            {
                return start_bearing-angular_resolution*(number_of_beams-1);
            }

            Angle getStartBearing()const
            {
                return start_bearing;
            }

            Angle getAngularResolution()const
            {
                return angular_resolution;
            }

            //calculates the spatial resolution of the sonar scan in meter
            //this takes the sampling_interval and the speed of sound into account
            double getSpatialResolution()const
            {
                //the sampling interval includes the time for 
                //the sound traveling from the transmitter to the target an back
                //to the receiver
                return sampling_interval*0.5*speed_of_sound;
            }

            inline void setData(const std::vector<uint8_t> &data) {
                this->data = data;
            }
            inline void setData(const char *data, uint32_t size) {
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

            inline uint8_t *getDataPtr() {
                return static_cast<uint8_t *>(&this->data[0]);
            }
            inline const uint8_t *getDataConstPtr() const {
                return static_cast<const uint8_t *>(&this->data[0]);
            }

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

            //check if opencv is present
#if defined( __OPENCV_CV_H__) ||defined (__OPENCV_CV_HPP__) || defined(_CV_H_) || defined(_CV_HPP_) || defined(__OPENCV_ALL_HPP__) ||defined(__OPENCV_OLD_CV_H__)
            inline cv::Mat convertToCvMat()
            {
                return cv::Mat(number_of_bins,number_of_beams, CV_8UC1, getDataPtr());
            }
            inline const cv::Mat convertToCvMat()const
            {
                return cv::Mat(number_of_bins,number_of_beams, CV_8UC1, (void*)getDataConstPtr());
            }
#else
#define convertToCvMat If_you_want_to_use_convertToCvMat_include_opencv_2_first
#endif
    };
}}

#endif

