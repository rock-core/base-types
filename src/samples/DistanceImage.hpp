#ifndef __BASE_SAMPLES_DISTANCE_IMAGE_H__
#define __BASE_SAMPLES_DISTANCE_IMAGE_H__

#include <base/Time.hpp>
#include <base/Eigen.hpp>
#include "Pointcloud.hpp"
#include <vector>
#include <boost/math/special_functions/fpclassify.hpp>

namespace base
{
namespace samples
{
    /** 
     * 2D array structure representing a distance image for a pinhole camera model.
     * 
     * The grid pixels are scaled such that (x*scale_x)+center_x = p_x are the
     * projective plane coordinates given a grid index x. This of course applies
     * to y as well.
     *
     * The data array is a row major flattened version of the image matrix,
     * giving the distance value d of the image points.  The relation is such
     * that for a point on the projection plane, the 3D point z can be calculated
     * as (p_x,p_y,1)*d = z.
     *
     * NOTE: unfortunately the scale and center is directly inverse to the convention
     * used in:
     * http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
     * for the time being, scale_x and scale_y are kept public for legacy code.
     * It is recommended to use the functions operating on it instead.
     */
    struct DistanceImage
    {
	/// original timestamp of the camera image
	base::Time time;

	/// width (x) value in pixels
	uint16_t width;
	/// height (y) value in pixels
	uint16_t height;

	typedef float scalar;
	/// scale value to apply to the x axis
	scalar scale_x;
	/// scale value to apply to the y axis
	scalar scale_y;

	/// center offset to apply to the x axis
	scalar center_x;
	/// center offset to apply to the y axis
	scalar center_y;

	/// distance values stored in row major order. NaN is used as the no value type.
	std::vector<scalar> data;

	DistanceImage(): width(0), height(0), scale_x(1.0), scale_y(1.0), center_x(0.0), center_y(0.0)
	{
	}

	DistanceImage(uint16_t width, uint16_t height): width(width), height(height), scale_x(1.0), scale_y(1.0), center_x(0.0), center_y(0.0)
	{
	}

	
	/** 
	 * resets all values in the distance image to nan
	 */
	void clear();

	/** 
	 * Transforms a x, y pixel coordinates into a 3d scene point, using the distance
	 * value of that point if available
	 *
	 * @param x - [in] x coordinate
	 * @param y - [in] y coordinate
	 * @param point - [out] resulting 3d scene point
	 * @result true if the scene point has a corresponding distance value
	 */
	template <class Scalar_>
	bool getScenePoint( size_t x, size_t y, Eigen::Matrix<Scalar_,3,1>& point ) const
	{
	    // check bounds. x and y are always positive
	    if( (x >= width) || (y >= height) )
		return false;

	    // only process vector if distance value is not NaN or inf
	    const scalar d = data[width*y+x];
	    if( boost::math::isnormal( d ) ) 
	    {
		point = Eigen::Matrix<Scalar_,3,1>( (x*scale_x)+center_x, (y*scale_y)+center_y, 1.0 );
		point *= d;
		return true;
	    }
	    return false;
	}

	/** transforms a 3d scene point onto an image coordinate
	 * 
	 * @param point - [in] 3d-scene point
	 * @param x - [out] x image coordinate 
	 * @param y - [out] y image coordinate 
	 */
	template <class Scalar_>
	bool getImagePoint( const Eigen::Matrix<Scalar_,3,1>& point, size_t &x, size_t &y ) const 
	{
	    if( point.z() <= 0 )
		return false;

	    size_t u = ((point.x() / point.z()) - center_x) / scale_x;
	    size_t v = ((point.y() / point.z()) - center_y) / scale_y;

	    // check bounds. u and v are always positive
	    if( (u >= width) || (v >= height) )
		return false;

	    x = u;
	    y = v;

	    return true;
	}

        base::samples::Pointcloud getPointCloud() const;
        
	/** 
	 * The intrinsic matrix has the following form
	 *
	 * | f_x  0    c_x |
	 * | 0    f_y  c_y |
	 * | 0    0    1   |
	 *
	 * where f_x and f_y are the focal length expressed in pixel
	 * and c_x and c_y are the center in pixel. This matrix can be 
	 * used to convert screen coordinates (x, y, z) into homogenous
	 * image coordinates (u, v, w) in the following way:
	 *
	 * (u,v,w) = F * (x, y, z)
	 *
	 * Note, that f and c are the inverse of the center and scale parameters.
	 *
	 * @return the intrinsic camera matrix
	 */
	template <class Scalar_>
	Eigen::Matrix<Scalar_,3,3> getIntrinsic() const
	{
	    // parameters in this class give the inverse
	    // so, we need to invert them
	    Eigen::Matrix<Scalar_,3,3> result;
	    result << 
		1.0/scale_x, 0, -center_x/scale_x,
		0, 1.0/scale_y, -center_y/scale_y,
		0, 0, 1;

	    return result;
	}

	/** @brief set the intrinsic camera parameters.
	 *
	 * see getIntrinsic() for a more detailed explanation.
	 *
	 * @param f_x focal length in x
	 * @param f_y focal length in y
	 * @param c_x center point in x
	 * @param c_y center point in y
	 */
	void setIntrinsic( double f_x, double f_y, double c_x, double c_y );

	/** @brief set the size of the distance image
	 *
	 * This will also rescale the internal data array to 
	 * hold the appropriate number of pixels.
	 *
	 * @param width width of the image
	 * @param height height of the image
	 */
	void setSize( uint16_t width, uint16_t height );
    };
}
}
#endif
