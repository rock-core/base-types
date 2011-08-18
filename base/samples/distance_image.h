#ifndef __BASE_SAMPLES_DISTANCE_IMAGE_H__
#define __BASE_SAMPLES_DISTANCE_IMAGE_H__

#include <base/time.h>
#include <vector>

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
    };
}
}
#endif
