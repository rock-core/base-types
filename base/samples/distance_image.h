#ifndef __BASE_SAMPLES_DISTANCE_IMAGE_H__
#define __BASE_SAMPLES_DISTANCE_IMAGE_H__

#include <base/time.h>
#include <vector>
#include <Eigen/Core>
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
	    point = Eigen::Matrix<Scalar_,3,1>( (x*scale_x)+center_x, (y*scale_y)+center_y, 1.0 );

	    // check bounds
	    if( (x < 0 || x >= width) || (y < 0 || y >= height) ) 
		return false;

	    // only process vector if distance value is not NaN or inf
	    const float d = data[width*y+x];
	    if( boost::math::isnormal( d ) ) 
	    {
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

	    x = ((point.x() / point.z()) - center_x) / scale_x;
	    y = ((point.y() / point.z()) - center_y) / scale_y;

	    // check bounds
	    if( (x < 0 || x >= width) || (y < 0 || y >= height) ) 
		return false;

	    return true;
	}

	/** 
	 * The intrinsic matrix has the following form
	 *
	 * | f_x  0    c_x |
	 * | 0    f_y  c_y |
	 * | 0    0    1   |
	 *
	 * where f_x and f_y are the focal length expressed in pixel
	 * and c_x and c_y are the center in pixel.
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
    };
}
}
#endif
