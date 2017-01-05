#ifndef __BASE_SAMPLES_DEPTH_MAP_HPP__
#define __BASE_SAMPLES_DEPTH_MAP_HPP__

#include <vector>
#include <Eigen/Geometry>

#include <base/Angle.hpp>
#include <base/Time.hpp>
#include <base-logging/Singleton.hpp>

#include <stdexcept>

#include <boost/cstdint.hpp>

namespace base { namespace samples {

/**
 * The DepthMap type provides distance and optional remission values in 3D.
 * The information is stored in a vector in a row major form, to simplify the usage as a distance image.
 * One row is a set of horizontal arranged single measurements with an identical vertical position or angle.
 * The distance and remission fields can therefore be seen as a image plane or a matrix. 
 * The horizontal and vertical intervals can be in polar or in planar form. In polar they are interpreted
 * as angles and in planar form they are interpreted as coordinates on a depth-image plane.
 * Horizontal intervals are always defined from left to right and vertical intervals are always
 * defined top down.
 * Each field of intervals can either have two entries, which define a range of regular arranged measurements,
 * or one entry per measurement to represent the irregular case.
 */
struct DepthMap
{
public:
    typedef float scalar;
    typedef boost::uint32_t uint32_t;
    typedef Eigen::Matrix<scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::DontAlign | Eigen::RowMajor> DepthMatrix;
    typedef const Eigen::Matrix<scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::DontAlign | Eigen::RowMajor> DepthMatrixConst;
    typedef Eigen::Map< DepthMatrix > DepthMatrixMap;
    typedef const Eigen::Map< DepthMatrixConst > DepthMatrixMapConst;
    
    enum DEPTH_MEASUREMENT_STATE 
    {
	VALID_MEASUREMENT  = 0,
	TOO_FAR            = 1,
	TOO_NEAR           = 2,
	MEASUREMENT_ERROR  = 3
    };
    
    enum PROJECTION_TYPE
    {
	POLAR,
	PLANAR
    };
    
    enum UNIT_AXIS
    {
	UNIT_X = 0,
	UNIT_Y = 1,
	UNIT_Z = 2
    };

    /** Reference timestamp for the depth map sample.
     * This timestamp is used for temporal alignment to other data samples
     * and transformations. 
     * It is important to always set here a meaningful value.
     */
    base::Time time;
    
    /** The timestamps can be either one timestamp for all measurements,
     * two for interpolation, per vertical entries, per horizontal entries
     * or one per measurement.
     */
    std::vector<base::Time> timestamps;
    
    /** Defines the vertical projection type of the depth map.
     * If polar, the vertical intervals are angular rotations around the Y-unit axis.
     * If planar, the vertical intervals are positions on an image plane coordinate frame.
     */
    PROJECTION_TYPE vertical_projection;
    
    /** Defines the horizontal projection type of the depth map.
     * If polar, the horizontal intervals are angular rotations around the Z-unit axis.
     * If planar, the horizontal intervals are positions on an image plane coordinate frame.
     */
    PROJECTION_TYPE horizontal_projection;

    /** The interval can describe a position on a planar plane or an
     * angle. The interval is interpreted as defined by the vertical projection type.
     * In planar projection mode the vertical intervals are y coordinates on an 
     * depth-image plane, with zero in the middle of the plane.
     * In polar projection mode the vertical intervals are angular rotations
     * around the Y-unit axis. Vertical angles must always be ordered from a smaller
     * to a higher value, since the rows of the data matrices are interpreted from the upper
     * to the lower row.
     * The field has either two or |vertical_size| entries. In the case of two 
     * entries the intervals are interpreted as upper and under boundaries. The 
     * transformation for each measurement will be interpolated.
     */
    std::vector<double> vertical_interval;
    
    /** The interval can describe a position on a planar plane or an
     * angle. The interval is interpreted as defined by the horizontal projection type.
     * In planar projection mode the horizontal intervals are x coordinates on an 
     * depth-image plane, with zero in the middle of the plane.
     * In polar projection mode the horizontal intervals are angular rotations
     * around the Z-unit axis. Horizontal angles must always be ordered from a higher
     * to a smaller value, since the columns of the data matrices are interpreted
     * from the left to the right.
     * The field has either two or |horizontal_size| entries. In the case of two 
     * entries the intervals are interpreted as left and right boundaries. The 
     * transformation for each measurement will be interpolated.
     */
    std::vector<double> horizontal_interval;
    
    /** Number of vertical depth samples */
    uint32_t vertical_size;
    
    /** Number of horizontal depth samples */
    uint32_t horizontal_size;

    /** The distance samples. The data is arranged in a row major order to simplify
     * the usage as a distance image. One row is a set of horizontal arranged single measurements.
     * The data for a measurement (vertical_index, horizontal_index) is in 
     * distances[(vertical_index * horizontal_size) + horizontal_index].
     */
    std::vector<scalar> distances;

    /** The remission samples. This field is optional.
     * The data is arranged in a row major order to simplify the usage as a remission image.
     * The data for a value (vertical_index, horizontal_index) is in 
     * remissions[(vertical_index * horizontal_size) + horizontal_index].
     */
    std::vector<scalar> remissions;

    DepthMap() : vertical_projection(POLAR), horizontal_projection(POLAR),
		vertical_size(0), horizontal_size(0) {}

    /** Reset the sample */
    void reset();
    
    /** Creates a mapping to a eigen matrix with dynamic sizes */
    DepthMatrixMap getDistanceMatrixMap();
    
    /** Creates a mapping to a const eigen matrix with dynamic sizes */
    DepthMatrixMapConst getDistanceMatrixMapConst() const;
    
    /** Returns the measurement state of a given index.
     * 
     * @param index of the measurement
     */
    DEPTH_MEASUREMENT_STATE getIndexState(size_t index) const;

    /** Returns the measurement state of a given vertical and horizontal index.
     *
     * @param v_index
     * @param h_index
     */
    DEPTH_MEASUREMENT_STATE getMeasurementState(uint32_t v_index, uint32_t h_index) const;

    /** Returns the measurement state of a given measurement 
     * 
     * @param distance measurement
     */
    DEPTH_MEASUREMENT_STATE getMeasurementState(scalar distance) const;

    /** Returns true if the measurement at the given index is valid.
     * 
     * @param index of the measurement
     */
    bool isIndexValid(size_t index) const;

    /** Returns true if the measurement at the given vertical and
     * horizontal index is valid.
     * 
     * @param v_index
     * @param h_index
     */
    bool isMeasurementValid(uint32_t v_index, uint32_t h_index) const;

    /** Returns true if the measurement is valid.
     * 
     * @param distance measurement
     */
    bool isMeasurementValid(scalar distance) const;

    /** Computes the index in the distance and remission vector 
     * of a given vertical and horizontal index.
     * Note that the data is stored in row major form.
     */
    size_t getIndex(uint32_t v_index, uint32_t h_index) const;
    
    /** Converts the depth map to a pointcloud in the coordinate system of the sensor 
     * (x-axis = forward, y-axis = to the left, z-axis = upwards).
     * If the resulting pointcloud should be associated with the remission values the invalid
     * measurements should not be skipped.
     * The template point type must be derived from a 3D eigen vector.
     * 
     * @param point_cloud returned pointcloud
     * @param use_lut true, if lookup table for single unit axis rotations shall be used
     * @param skip_invalid_measurements true, if invalid measurements should be skipped
     */
    template<typename T>
    void convertDepthMapToPointCloud(std::vector<T> &point_cloud, 
				     bool use_lut = false, 
				     bool skip_invalid_measurements = true) const
    {
	convertDepthMapToPointCloud(point_cloud, Eigen::Transform<typename T::Scalar,3,Eigen::Affine>::Identity(), 
				    use_lut,
				    skip_invalid_measurements);
    }

    /** Converts the depth map to a pointcloud according to the given transformation matrix.
     * If the transformation matrix is set to identity the depth map is converted into 
     * the coordinate system of the sensor (x-axis = forward, y-axis = to the left, z-axis = upwards).
     * If the resulting pointcloud should be associated with the remission values the invalid
     * measurements should not be skipped.
     * The template point type must be derived from a 3D eigen vector.
     * 
     * @param point_cloud returned pointcloud
     * @param transformation all points will be transformed using this transformation
     * @param use_lut true, if lookup table for single unit axis rotations shall be used
     * @param skip_invalid_measurements true, if invalid measurements should be skipped
     */
    template<typename T>
    void convertDepthMapToPointCloud(std::vector<T> &point_cloud,
				const Eigen::Transform<typename T::Scalar,3,Eigen::Affine>& transformation,
				bool use_lut = false, 
				bool skip_invalid_measurements = true) const
    {
	point_cloud.clear();
	
	// check row and col size
	if(!checkSizeConfig())
	    throw std::out_of_range("Number of rows and columns does not match the distance array size.");

	// check if nothing to do
	if(distances.empty())
	    return;

	// give the vector a hint about the size it might be
	point_cloud.reserve(distances.size());
	
	// precompute local transformations
	std::vector< Eigen::Transform<typename T::Scalar,3,Eigen::Affine>, Eigen::aligned_allocator< Eigen::Transform<typename T::Scalar,3,Eigen::Affine> > > rows2column;
	std::vector< Eigen::Transform<typename T::Scalar,3,Eigen::Affine>, Eigen::aligned_allocator< Eigen::Transform<typename T::Scalar,3,Eigen::Affine> > > columns2pointcloud;
	computeLocalTransformations(rows2column, columns2pointcloud, use_lut);
	
	// convert rows
	for(unsigned v = 0; v < vertical_size; v++)
	    convertSingleRow(point_cloud, v, rows2column[v], columns2pointcloud, transformation, skip_invalid_measurements);
    }

    /** Converts the depth map to a pointcloud according to the given transformation matrices.
     * On basis of the first and last transformation the transformations will be
     * interpolated and applied row-wise if the parameter apply_transforms_vertically is 
     * set to true, otherwise they will be interpolated and applied col-wise.
     * 
     * @param point_cloud returned pointcloud
     * @param first_transformation transformation of the first row or column
     * @param last_transformation translation of the last row or column
     * @param use_lut true, if lookup table for single unit axis rotations shall be used
     * @param skip_invalid_measurements true, if invalid measurements should be skipped
     * @param apply_transforms_vertically true, if transformations should be applied row-wise
     */
    template<typename T>
    void convertDepthMapToPointCloud(std::vector<T> &point_cloud,
				const Eigen::Transform<typename T::Scalar,3,Eigen::Affine>& first_transformation,
				const Eigen::Transform<typename T::Scalar,3,Eigen::Affine>& last_transformation,
				bool use_lut = false,
				bool skip_invalid_measurements = true,
				bool apply_transforms_vertically = true) const
    {
	point_cloud.clear();
	
	// check row and col size
	if(!checkSizeConfig())
	    throw std::out_of_range("Number of rows and columns does not match the distance array size.");

	// check if nothing to do
	if(distances.empty())
	    return;

	// give the vector a hint about the size it might be
	point_cloud.reserve(distances.size());
	
	// precompute local transformations
	std::vector< Eigen::Transform<typename T::Scalar,3,Eigen::Affine>, Eigen::aligned_allocator< Eigen::Transform<typename T::Scalar,3,Eigen::Affine> > > rows2column;
	std::vector< Eigen::Transform<typename T::Scalar,3,Eigen::Affine>, Eigen::aligned_allocator< Eigen::Transform<typename T::Scalar,3,Eigen::Affine> > > columns2pointcloud;
	computeLocalTransformations(rows2column, columns2pointcloud, use_lut);
	
	Eigen::Matrix<typename T::Scalar,3,1> translation_delta = last_transformation.translation() - first_transformation.translation();
	Eigen::Quaternion<typename T::Scalar> first_rotation = Eigen::Quaternion<typename T::Scalar>(first_transformation.linear());
	Eigen::Quaternion<typename T::Scalar> last_rotation = Eigen::Quaternion<typename T::Scalar>(last_transformation.linear());
	
	// apply global interpolated transformations
	if(!apply_transforms_vertically)
	{
	    for(unsigned h = 0; h < horizontal_size; h++)
	    {
		Eigen::Transform<typename T::Scalar,3,Eigen::Affine> transformation = 
		    Eigen::Transform<typename T::Scalar,3,Eigen::Affine>(first_rotation.slerp((double)h / (double)(horizontal_size-1), last_rotation));
		transformation.pretranslate(first_transformation.translation() + ((double)h / (double)(horizontal_size-1)) * translation_delta);
		columns2pointcloud[h] = transformation * columns2pointcloud[h];
	    }
	    
	    // convert rows
	    for(unsigned v = 0; v < vertical_size; v++)
		convertSingleRow(point_cloud, v, rows2column[v], columns2pointcloud, 
				 Eigen::Transform<typename T::Scalar,3,Eigen::Affine>::Identity(), 
				 skip_invalid_measurements);
	}
	else
	{
	    std::vector< Eigen::Transform<typename T::Scalar,3,Eigen::Affine>, Eigen::aligned_allocator< Eigen::Transform<typename T::Scalar,3,Eigen::Affine> > > pointcloud2world;
	    for(unsigned v = 0; v < vertical_size; v++)
	    {
		Eigen::Transform<typename T::Scalar,3,Eigen::Affine> transformation = 
		    Eigen::Transform<typename T::Scalar,3,Eigen::Affine>(first_rotation.slerp((double)v / (double)(vertical_size-1), last_rotation));
		transformation.pretranslate(first_transformation.translation() + ((double)v / (double)(vertical_size-1)) * translation_delta);
		pointcloud2world.push_back(transformation);
	    }
	    
	    // convert rows
	    for(unsigned v = 0; v < vertical_size; v++)
		convertSingleRow(point_cloud, v, rows2column[v], columns2pointcloud, 
				 pointcloud2world[v], skip_invalid_measurements);
	}
    }

    /** Converts the depth map to a pointcloud according to the given transformation matrices.
     * Each transformation in the given vector will be applied to each row if the parameter 
     * apply_transforms_vertically is set to true, otherwise it will be applied col-wise.
     * The transformations vector therefore must have either the same size of the columns 
     * or the rows.
     * 
     * @param point_cloud returned pointcloud
     * @param transformations vector of transformation matrices
     * @param use_lut true, if lookup table for single unit axis rotations shall be used
     * @param skip_invalid_measurements true, if invalid measurements should be skipped
     * @param apply_transforms_vertically true, if transformations should be applied row-wise
     */
    template<typename T>
    void convertDepthMapToPointCloud(std::vector<T> &point_cloud,
				const std::vector< Eigen::Transform<typename T::Scalar,3,Eigen::Affine>, Eigen::aligned_allocator< Eigen::Transform<typename T::Scalar,3,Eigen::Affine> > >& transformations,
				bool use_lut = false,
				bool skip_invalid_measurements = true,
				bool apply_transforms_vertically = true) const
    {
	point_cloud.clear();
	
	// check row and col size
	if(!checkSizeConfig())
	    throw std::out_of_range("Number of rows and columns does not match the distance array size.");

	// check if nothing to do
	if(distances.empty())
	    return;

	// give the vector a hint about the size it might be
	point_cloud.reserve(distances.size());
	
	// precompute local transformations
	std::vector< Eigen::Transform<typename T::Scalar,3,Eigen::Affine>, Eigen::aligned_allocator< Eigen::Transform<typename T::Scalar,3,Eigen::Affine> > > rows2column;
	std::vector< Eigen::Transform<typename T::Scalar,3,Eigen::Affine>, Eigen::aligned_allocator< Eigen::Transform<typename T::Scalar,3,Eigen::Affine> > > columns2pointcloud;
	computeLocalTransformations(rows2column, columns2pointcloud);
	
	// apply global transformations
	if(!apply_transforms_vertically)
	{
	    if(transformations.size() != horizontal_size)
		throw std::out_of_range("Invalid amount of transformations given");
	    for(unsigned h = 0; h < horizontal_size; h++)
		columns2pointcloud[h] = transformations[h] * columns2pointcloud[h];
	    
	    // convert rows
	    for(unsigned v = 0; v < vertical_size; v++)
		convertSingleRow(point_cloud, v, rows2column[v], columns2pointcloud, 
				 Eigen::Transform<typename T::Scalar,3,Eigen::Affine>::Identity(), 
				 skip_invalid_measurements);
	}
	else
	{
	    if(transformations.size() != vertical_size)
		throw std::out_of_range("Invalid amount of transformations given");
	    
	    // convert rows
	    for(unsigned v = 0; v < vertical_size; v++)
		convertSingleRow(point_cloud, v, rows2column[v], columns2pointcloud, 
				 transformations[v], 
				 skip_invalid_measurements);
	}
    }
    
private:
    template<typename T, UNIT_AXIS> class RotationLUT;

protected:    
    /** Helper method which converts a single row to a pointcloud. */
    template<typename T>
    void convertSingleRow(std::vector<T> &point_cloud, 
			    unsigned int row,
			    const Eigen::Transform<typename T::Scalar,3,Eigen::Affine>& row2column,
			    const std::vector< Eigen::Transform<typename T::Scalar,3,Eigen::Affine>, Eigen::aligned_allocator< Eigen::Transform<typename T::Scalar,3,Eigen::Affine> > >& columns2pointcloud,
			    const Eigen::Transform<typename T::Scalar,3,Eigen::Affine>& pointcloud2world,
			    bool skip_invalid_measurements) const
    {
	size_t row_index = (size_t)row * (size_t)horizontal_size;
	for(unsigned h = 0; h < horizontal_size; h++)
	{
	    scalar distance = distances[row_index + h];
	    if(isMeasurementValid(distance))
	    {
		T point(distance, 0.0, 0.0);
		point =  pointcloud2world * (columns2pointcloud[h] * row2column * point);
		point_cloud.push_back(point);
	    }
	    else if(!skip_invalid_measurements)
	    {
		point_cloud.push_back(T(base::unknown<typename T::Scalar>(), 
					base::unknown<typename T::Scalar>(), 
					base::unknown<typename T::Scalar>()));
	    }
	}
    }

    /** Helper method to compute the local rows2column and columns2pointcloud transformations. */
    template<typename T>
    void computeLocalTransformations(std::vector< Eigen::Transform<T,3,Eigen::Affine>, Eigen::aligned_allocator< Eigen::Transform<T,3,Eigen::Affine> > >& rows2column, 
				     std::vector< Eigen::Transform<T,3,Eigen::Affine>, Eigen::aligned_allocator< Eigen::Transform<T,3,Eigen::Affine> > >& columns2pointcloud,
				     bool use_lut = false) const
    {
	// check interval sizes
	if(vertical_interval.size() > 2 && vertical_interval.size() != vertical_size)
	{
	    std::string err_msg = "Number of vertical ";
	    err_msg += vertical_projection == POLAR ? "angles " : "distances ";
	    err_msg += "does no match the expected number of entries.";
	    throw std::invalid_argument(err_msg);
	}
	if(horizontal_interval.size() > 2 && horizontal_interval.size() != horizontal_size)
	{
	    std::string err_msg = "Number of horizontal ";
	    err_msg += horizontal_projection == POLAR ? "angles " : "distances ";
	    err_msg += "does no match the expected number of entries.";
	    throw std::invalid_argument(err_msg);
	}
	
	// compute row2column transformations
	rows2column.resize(vertical_size, Eigen::Transform<T,3,Eigen::Affine>::Identity());
	if(vertical_interval.size() == 1)
	{
	    // one planar translation or polar rotation for all vertical measurements
	    if(vertical_projection == PLANAR)
		rows2column.resize(vertical_size, Eigen::Transform<T,3,Eigen::Affine>(Eigen::AngleAxis<T>(
		    base::Angle::normalizeRad(atan(vertical_interval.front())), Eigen::Matrix<T,3,1>::UnitY())));
	    else if(vertical_projection == POLAR)
		rows2column.resize(vertical_size, Eigen::Transform<T,3,Eigen::Affine>(Eigen::AngleAxis<T>(
		    base::Angle::normalizeRad(vertical_interval.front()), Eigen::Matrix<T,3,1>::UnitY())));
	    else
		throw std::invalid_argument("Invalid argument for vertical projection type.");
	}
	else if(vertical_interval.size() == 2)
	{
	    // interpolate transformations between the given start and end values for all vertical measurements
	    std::vector<base::Angle> vertical_angles(vertical_size);
	    double step_resolution = computeResolution(vertical_interval, vertical_size, vertical_projection);
	    if(vertical_projection == PLANAR)
	    {	
		for(unsigned v = 0; v < vertical_size; v++)
		    vertical_angles[v] = base::Angle::fromRad(atan(vertical_interval.front() + ((double)v * step_resolution)));
	    }
	    else if(vertical_projection == POLAR)
	    {
		for(unsigned v = 0; v < vertical_size; v++)
		    vertical_angles[v] = base::Angle::fromRad(vertical_interval.front() + ((double)v * step_resolution));
	    }
	    else
		throw std::invalid_argument("Invalid argument for vertical projection type.");
	    
	    computeRotations(rows2column, vertical_angles, UNIT_Y, use_lut);
	}
	else if(vertical_interval.size() > 2)
	{
	    // use unique planar translation or polar rotation for each vertical measurement
	    std::vector<base::Angle> vertical_angles(vertical_size);
	    if(vertical_projection == PLANAR)
		for(unsigned v = 0; v < vertical_size; v++)
		    vertical_angles[v] = base::Angle::fromRad(atan(vertical_interval[v]));
	    else if(vertical_projection == POLAR)
	    {
		for(unsigned v = 0; v < vertical_size; v++)
		    vertical_angles[v] = base::Angle::fromRad(vertical_interval[v]);
	    }
	    else
		throw std::invalid_argument("Invalid argument for vertical projection type.");
	    
	    computeRotations(rows2column, vertical_angles, UNIT_Y, use_lut);
	}
	
	// compute columns2pointcloud transformations
	columns2pointcloud.resize(horizontal_size, Eigen::Transform<T,3,Eigen::Affine>::Identity());
	if(horizontal_interval.size() == 1)
	{
	    // one planar translation or polar rotation for all horizontal measurements
	    if(horizontal_projection == PLANAR)
		columns2pointcloud.resize(horizontal_size, Eigen::Transform<T,3,Eigen::Affine>(Eigen::AngleAxis<T>(
		    base::Angle::normalizeRad(atan(horizontal_interval.front())), Eigen::Matrix<T,3,1>::UnitZ())));
	    else if(horizontal_projection == POLAR)
		columns2pointcloud.resize(horizontal_size, Eigen::Transform<T,3,Eigen::Affine>(Eigen::AngleAxis<T>(
		    base::Angle::normalizeRad(horizontal_interval.front()), Eigen::Matrix<T,3,1>::UnitZ())));
	    else
		throw std::invalid_argument("Invalid argument for horizontal projection type.");
	}
	else if(horizontal_interval.size() == 2)
	{
	    // interpolate transformations between the given start and end values for all horizontal measurements
	    std::vector<base::Angle> horizontal_angles(horizontal_size);
	    double step_resolution = computeResolution(horizontal_interval, horizontal_size, horizontal_projection);
	    if(horizontal_projection == PLANAR)
	    {
		for(unsigned h = 0; h < horizontal_size; h++)
		    horizontal_angles[h] = base::Angle::fromRad(atan(-1.0 * (horizontal_interval.front() + ((double)h * step_resolution))));
	    }
	    else if(horizontal_projection == POLAR)
	    {
		for(unsigned h = 0; h < horizontal_size; h++)
		    horizontal_angles[h] = base::Angle::fromRad(horizontal_interval.front() + ((double)h * step_resolution));
	    }
	    else
		throw std::invalid_argument("Invalid argument for horizontal projection type.");
	    
	    computeRotations(columns2pointcloud, horizontal_angles, UNIT_Z, use_lut);
	}
	else if(horizontal_interval.size() > 2)
	{
	    // use unique planar translation or polar rotation for each horizontal measurement
	    std::vector<base::Angle> horizontal_angles(horizontal_size);
	    if(horizontal_projection == PLANAR)
		for(unsigned h = 0; h < horizontal_size; h++)
		    horizontal_angles[h] = base::Angle::fromRad(atan(-1.0 * horizontal_interval[h]));
	    else if(horizontal_projection == POLAR)
		for(unsigned h = 0; h < horizontal_size; h++)
		    horizontal_angles[h] = base::Angle::fromRad(horizontal_interval[h]);
	    else
		throw std::invalid_argument("Invalid argument for horizontal projection type.");
	    
	    computeRotations(columns2pointcloud, horizontal_angles, UNIT_Z, use_lut);
	}
    }
    
    /** Helper method to compute the rotations around an unit axis. */
    template<typename T>
    void computeRotations(std::vector< Eigen::Transform<T,3,Eigen::Affine>, Eigen::aligned_allocator< Eigen::Transform<T,3,Eigen::Affine> > >& rotations, 
			const std::vector<base::Angle>& angles, 
			UNIT_AXIS axis,
			bool use_lut = false) const
    {
	rotations.clear();
	rotations.resize(angles.size());
	
	if(use_lut)
	{
	    if(axis == UNIT_X)
	    {
		RotationLUT<T,UNIT_X>* lut = Singleton< RotationLUT<T,UNIT_X> >::getInstance();
		for(unsigned i = 0; i < angles.size(); i++)
		    rotations[i] = lut->getTransformation(angles[i].getRad());
	    }
	    else if(axis == UNIT_Y)
	    {
		RotationLUT<T,UNIT_Y>* lut = Singleton< RotationLUT<T,UNIT_Y> >::getInstance();
		for(unsigned i = 0; i < angles.size(); i++)
		    rotations[i] = lut->getTransformation(angles[i].getRad());
	    }
	    else if(axis == UNIT_Z)
	    {
		RotationLUT<T,UNIT_Z>* lut = Singleton< RotationLUT<T,UNIT_Z> >::getInstance();
		for(unsigned i = 0; i < angles.size(); i++)
		    rotations[i] = lut->getTransformation(angles[i].getRad());
	    }
	}
	else
	{
	    for(unsigned i = 0; i < angles.size(); i++)
		rotations[i] = Eigen::AngleAxis<T>(angles[i].getRad(), Eigen::Matrix<T,3,1>::Unit(axis));
	}
    };
    
    /** Computes the resolution in a given interval. */
    double computeResolution(const std::vector<double> &interval, uint32_t elements, PROJECTION_TYPE projection) const
    {
	if(interval.size() != 2)
	    throw std::invalid_argument("Expecting two interval values.");
	
	double step_resolution = 0.0;
	if(projection == PLANAR)
	{
	    double diff = interval.back() - interval.front();
	    step_resolution = diff / (double)(elements-1);
	}
	else if(projection == POLAR)
	{
	    if(interval.back() == interval.front())
		step_resolution = (2.0 * M_PI) / (double)(elements-1);
	    else
                step_resolution = (interval.back() - interval.front()) / (double)(elements-1);
	}
	else
	    throw std::invalid_argument("Invalid argument for projection type.");
	
	return step_resolution;
    };

    /** Checks if the vertical and horizontal sizes match the size of distance vector. */
    bool checkSizeConfig() const;

    
private:
    /** Lookup table for rotations around one unit axis */
    template<typename T, UNIT_AXIS unit_axis>
    class RotationLUT
    {
	static const unsigned resolution = 36000;
	
    public:
	Eigen::Transform<T,3,Eigen::Affine> getTransformation(double rad)
	{
	    uint16_t deg = (uint16_t)((rad < 0.0 ? rad + 2.0*M_PI: rad) * rad2deg);
	    return transformations[deg];
	};
	
	RotationLUT() : rad2deg( ((double)resolution) / (2.0*M_PI) ), deg2rad( (2.0*M_PI) / ((double)resolution) )
	{
	    transformations.resize(resolution);
	    for(unsigned i = 0; i < resolution; i++)
		transformations[i] = Eigen::Transform<T,3,Eigen::Affine>(Eigen::AngleAxis<T>((double)i * deg2rad, Eigen::Matrix<T,3,1>::Unit(unit_axis)));
	}
	virtual ~RotationLUT() {}
	
    private:
	double rad2deg;
	double deg2rad;
	std::vector< Eigen::Transform<T,3,Eigen::Affine>, Eigen::aligned_allocator< Eigen::Transform<T,3,Eigen::Affine> > > transformations;
    };
};

}} // namespaces

#endif
