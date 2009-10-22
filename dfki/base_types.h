#ifndef DFKI_BASE_TYPES_H__
#define DFKI_BASE_TYPES_H__

#ifndef __orogen
#include <sys/time.h>
#include <time.h>
#include <stdint.h>

#include <Eigen/Core>
#include <Eigen/Geometry> 
#endif

namespace DFKI {
    struct Time {
        /** The number of seconds */
        int seconds;
        /** The number of microseconds. This is always in [0, 1000000]. */
        int microseconds;

#ifndef __orogen
        Time()
            : seconds(0), microseconds(0) {}

        explicit Time(int seconds, int microseconds = 0)
            : seconds(seconds), microseconds(microseconds) {}

        /** Returns the current time */
        static Time now() {
            timeval t;
            gettimeofday(&t, 0);
            return Time(t.tv_sec, t.tv_usec);
        }

        bool operator < (Time const& ts) const
        { return seconds < ts.seconds || (seconds == ts.seconds && microseconds < ts.microseconds); }
        bool operator > (Time const& ts) const
        { return seconds > ts.seconds || (seconds == ts.seconds && microseconds > ts.microseconds); }
        bool operator == (Time const& ts) const
        { return seconds == ts.seconds && microseconds == ts.microseconds; }
        bool operator != (Time const& ts) const
        { return !(*this == ts); }
        bool operator >= (Time const& ts) const
        { return !(*this < ts); }
        bool operator <= (Time const& ts) const
        { return !(*this > ts); }
        Time operator - (Time const& ts) const
        {
            Time result;
            result.seconds      = seconds - ts.seconds;
            result.microseconds = microseconds - ts.microseconds;
            result.canonize();
            return result;
        }

        Time operator + (Time const& ts) const
        {
            Time result;
            result.seconds      = seconds + ts.seconds;
            result.microseconds = microseconds + ts.microseconds;
            result.canonize();
            return result;
        }

        Time operator / (int divider) const
        {
            Time result;
	    uint64_t timeInMicroSec = this->toMicroseconds();
	    
	    timeInMicroSec /= divider;

            int const UsecPerSec = 1000000;
            int64_t offset = timeInMicroSec / UsecPerSec;
            result.seconds = offset;
            timeInMicroSec -= offset * UsecPerSec;
	    result.microseconds = timeInMicroSec;	  
            return result;
        }

        /** True if this time is zero */
        bool isNull() const { return seconds == 0 && microseconds == 0; }

        /** Converts this time as a timeval object */
        timeval toTimeval() const
        {
            timeval tv = { seconds, microseconds };
            return tv;
        }

        /** Returns this time as a fractional number of seconds */
        double toSeconds() const
        {
            return static_cast<double>(seconds) + static_cast<double>(microseconds) / 1000000.0;
        }

        /** Returns this time as an integer number of milliseconds (thus dropping the microseconds) */
        uint64_t toMilliseconds() const
        {
            return static_cast<uint64_t>(seconds) * 1000 + static_cast<uint64_t>(microseconds) / 1000;
        }

        /** Returns this time as an integer number of microseconds */
        uint64_t toMicroseconds() const
        {
            return static_cast<uint64_t>(seconds) * 1000000 + static_cast<uint64_t>(microseconds);
        }

        static DFKI::Time fromMicroseconds(uint64_t value)
        {
            return DFKI::Time(value / 1000000ULL, value % 1000000ULL);
        }


    private:
        /** This method makes sure that the constraint on microseconds is met
         * (i.e. that microseconds is in [0, 1000000].
         */
        void canonize()
        {
	  int const UsecPerSec = 1000000;
	  int offset = microseconds / UsecPerSec;
	  seconds      += offset;
	  microseconds -= offset * UsecPerSec;

	  if(microseconds < 0) {
	    seconds--;
	    microseconds += UsecPerSec;
	  }
        }
#endif
    };


    /**
     * Wrapper class for Eigen to work around
     * the alignment problem.
     */ 
    struct Matrix3 
    {
      double data[9];
      

    };
    


    /**
     * Wrapper class for Eigen to work around
     * the alignment problem.
     */ 
    struct Vector3 {
      double data[3];
#ifndef __orogen
      Vector3() {
	data[0] = 0;
	data[1] = 0;
	data[2] = 0;
      };
      
      Vector3(Eigen::Vector3d vec) 
      {
	x() = vec.x();
	y() = vec.y();
	z() = vec.z();
      }
      
      double &x() 
      {
	return data[0];
      }

      const double &x() const
      {
	return data[0];
      }
      
      double &y() 
      {
	return data[1];
      }

      const double &y() const 
      {
	return data[1];
      }

      double &z() 
      {
	return data[2];
      }

      const double &z() const 
      {
	return data[2];
      }

      Eigen::Vector3d getEigenType() const {
	return Eigen::Vector3d(x(), y(), z());
      }
#endif
    };

    /**
     * Wrapper class for Eigen to work around
     * the alignment problem.
     */ 
    struct Quaternion {
      double data[4];
#ifndef __orogen
      Quaternion() {
	data[0] = 0;
	data[1] = 1;
	data[2] = 2;
	data[3] = 3;
      };
      
      Quaternion(Eigen::Quaterniond &q) 
      {
	x() = q.x();
	y() = q.y();
	z() = q.z();
	w() = q.w();
      }
      
      double &x() 
      {
	return data[0];
      }
      
      const double &x() const 
      {
	return data[0];
      }
      
      double &y() 
      {
	return data[1];
      }

      const double &y() const 
      {
	return data[1];
      }

      double &z() 
      {
	return data[2];
      }

      const double &z() const 
      {
	return data[2];
      }

      double &w() 
      {
	return data[3];
      }

      const double &w() const 
      {
	return data[3];
      }
      
      Eigen::Quaterniond getEigenType() const
      {
	return Eigen::Quaterniond(w(), x(), y(), z());
      }
#endif
    };

    struct Pose3D 
    {
      struct Vector3 position;
      struct Quaternion orientation;
    };
}

#endif
