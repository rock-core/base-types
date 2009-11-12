#ifndef DFKI_LINEAR_ALGEBRA_H__
#define DFKI_LINEAR_ALGEBRA_H__

#ifndef __orogen
#include <Eigen/Core>
#include <Eigen/Geometry> 
#endif

namespace DFKI {
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
      
      Vector3(const Eigen::Vector3d& vec) 
      {
	x() = vec.x();
	y() = vec.y();
	z() = vec.z();
      }
      
      Vector3 &operator=(const Eigen::Vector3d &vec) {
	x() = vec.x();
	y() = vec.y();
	z() = vec.z();
        return *this;
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
      // store as imaginary and real part, so it comes out clear in the pocosim logs
      double im[3];
      double re;

#ifndef __orogen
      Quaternion() {
	im[0] = 0;
	im[1] = 0;
	im[2] = 0;
	re = 1.0;
      };
      
      Quaternion(const Eigen::Quaterniond &q) 
      {
	x() = q.x();
	y() = q.y();
	z() = q.z();
	w() = q.w();
      }

      Quaternion &operator=(const Eigen::Quaterniond &q) {
	x() = q.x();
	y() = q.y();
	z() = q.z();
	w() = q.w();
        return *this;
      }
      
      double &x() 
      {
	return im[0];
      }
      
      const double &x() const 
      {
	return im[0];
      }
      
      double &y() 
      {
	return im[1];
      }

      const double &y() const 
      {
	return im[1];
      }

      double &z() 
      {
	return im[2];
      }

      const double &z() const 
      {
	return im[2];
      }

      double &w() 
      {
	return re; 
      }

      const double &w() const 
      {
	return re;
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
