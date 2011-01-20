#ifndef BASE_SAMPLES_SONAR_H__
#define BASE_SAMPLES_SONAR_H__

#ifdef __GCCXML__
#define EIGEN_DONT_VECTORIZE
#endif

#ifndef __orogen
#include <vector>
#include <boost/cstdint.hpp>
#include <Eigen/Geometry>
#include <stdexcept>
#endif

#include <base/time.h>

namespace base { namespace samples {
    
	struct SonarScan {
#ifndef __orogen
        typedef boost::uint8_t uint8_t;
#endif

        /** The timestamp of this reading. The timestamp is the time at which the
         * sonar send the Data to the PC,
         */
        Time time;

        /** The angle at which the range reading created. Zero is at the front of
         * the device and turns counter-clockwise. 
	 * This value is in radians
         */
        double angle;

	/**
	* The Time between two bins, with this and velocity of sonic in water the distance can be calculated
	*/
	double time_beetween_bins;

        /** The array one scanline at the angle, distance between to indexes is given trought  
         */
        std::vector<uint8_t> scanData;


#ifndef __orogen
        SonarScan()
            : angle(0), time_beetween_bins(0){}
	
	double getScale(double sonicVelocityinWater = 1500.0){
	  return  ((scanData.size()*time_beetween_bins)*sonicVelocityinWater/2.0)/scanData.size();
	}
            
#endif
    };
}} // namespaces

#endif
