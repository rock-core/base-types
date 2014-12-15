#ifndef __BASE_SAMPLES_DEPTH_MAP_HPP__
#define __BASE_SAMPLES_DEPTH_MAP_HPP__

#include <vector>
#include <Eigen/Geometry>

#include <base/Float.hpp>
#include <base/Time.hpp>
#include <base/Angle.hpp>

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
    
    /** The timestamps can be either one timestamp for all measurements,
     * two for interpolation, per vertical entries, per horizontal entries
     * or one per measurement.
     */
    std::vector<base::Time> timestamps;
    
    /** Defines the vertical projection type of the depth map.
     * If polar, the vertical intervals are angles in the range of (-PI, PI].
     * If planar, the vertical intervals are positions on an image plane coordinate frame.
     */
    PROJECTION_TYPE vertical_projection;
    
    /** Defines the horizontal projection type of the depth map.
     * If polar, the horizontal intervals are angles in the range of (-PI, PI].
     * If planar, the horizontal intervals are positions on an image plane coordinate frame.
     */
    PROJECTION_TYPE horizontal_projection;

    /** The interval can describe a position on a planar plane or an
     * angle. The interval is interpreted as defined by the vertical projection type.
     * In planar projection mode the vertical intervals are y coordinates on an 
     * depth-image plane.
     * In polar projection mode the vertical intervals are angular rotations
     * around the Y-unit axis.
     * The field has either two or |vertical_size| entries. In the case of two 
     * entries the intervals are interpreted as upper and under boundaries. The 
     * transformation for each measurement will be interpolated.
     */
    std::vector<double> vertical_interval;
    
    /** The interval can describe a position on a planar plane or an
     * angle. The interval is interpreted as defined by the horizontal projection type.
     * In planar projection mode the horizontal intervals are x coordinates on an 
     * depth-image plane.
     * In polar projection mode the horizontal intervals are angular rotations
     * around the Z-unit axis.
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
    void reset()
    {
	timestamps.clear();
	vertical_projection = POLAR;
	horizontal_projection = POLAR;
	vertical_interval.clear();
	horizontal_interval.clear();
	vertical_size = 0;
	horizontal_size = 0;
	distances.clear();
	remissions.clear();
    }
    
    /** Creates a mapping to a eigen matrix with dynamic sizes */
    inline DepthMatrixMap getDistanceMatrixMap() 
	{ return (DepthMatrixMap(distances.data(), vertical_size, horizontal_size)); }
    
    /** Creates a mapping to a const eigen matrix with dynamic sizes */
    inline DepthMatrixMapConst getDistanceMatrixMapConst() const 
	{ return (DepthMatrixMapConst(distances.data(), vertical_size, horizontal_size)); }
    
    /** Returns the measurement state of a given index.
     * 
     * @param index of the measurement
     */
    DEPTH_MEASUREMENT_STATE getIndexState(size_t index) const
    {
	if(index >= distances.size())
	    throw std::out_of_range("Invalid measurement index given");
	
	return getMeasurementState(distances[index]);
    }

    /** Returns the measurement state of a given vertrical and horizontal index.
     *
     * @param v_index
     * @param h_index
     */
    DEPTH_MEASUREMENT_STATE getMeasurementState(uint32_t v_index, uint32_t h_index) const
    {
	if(!checkSizeConfig())
	    throw std::out_of_range("Vertical and horizontal size does not match the distance array size.");
	if(v_index >= vertical_size || h_index >= horizontal_size)
	    throw std::out_of_range("Invalid vertrical or horizontal index given.");
	
	return getMeasurementState(distances[getIndex(v_index,h_index)]);
    }

    /** Returns the measurement state of a given measurement 
     * 
     * @param distance measurement
     */
    DEPTH_MEASUREMENT_STATE getMeasurementState(scalar distance) const
    {
	if(base::isNaN<scalar>(distance))
	    return MEASUREMENT_ERROR;
	else if(base::isInfinity<scalar>(distance))
	    return TOO_FAR;
	else if(distance <= 0.0)
	    return TOO_NEAR;
	else
	    return VALID_MEASUREMENT;
	    
    }

    /** Returns true if the measurement at the given index is valid.
     * 
     * @param index of the measurement
     */
    bool isIndexValid(size_t index) const
    {
	if(index >= distances.size())
	    throw std::out_of_range("Invalid measurement index given");
	
	return isMeasurementValid(distances[index]);
    }

    /** Returns true if the measurement at the given vertrical and 
     * horizontal index is valid.
     * 
     * @param v_index
     * @param h_index
     */
    bool isMeasurementValid(uint32_t v_index, uint32_t h_index) const
    {
	if(!checkSizeConfig())
	    throw std::out_of_range("Vertical and horizontal size does not match the distance array size.");
	if(v_index >= vertical_size || h_index >= horizontal_size)
	    throw std::out_of_range("Invalid vertrical or horizontal index given.");
	
	return isMeasurementValid(distances[getIndex(v_index,h_index)]);
    }

    /** Returns true if the measurement is valid.
     * 
     * @param distance measurement
     */
    bool isMeasurementValid(scalar distance) const
    {
	return getMeasurementState(distance) == VALID_MEASUREMENT;
    }

    /** Computes the index in the distance and remission vector 
     * of a given vertrical and horizontal index. 
     * Note that the data is stored in row major form.
     */
    inline size_t getIndex(uint32_t v_index, uint32_t h_index) const
    {
	return ((size_t)v_index * (size_t)horizontal_size + (size_t)h_index);
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
     * @param skip_invalid_measurements true, if invalid measurements should be skipped
     */
    template<typename T>
    void convertDepthMapToPointCloud(std::vector<T> &point_cloud,
				const Eigen::Transform<typename T::Scalar,3,Eigen::Affine>& transformation,
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
	std::vector< Eigen::Transform<typename T::Scalar,3,Eigen::Affine> > rows2column;
	std::vector< Eigen::Transform<typename T::Scalar,3,Eigen::Affine> > columns2pointcloud;
	computeLocalTransformations(rows2column, columns2pointcloud);
	
	// convert rows
	for(unsigned v = 0; v < vertical_size; v++)
	    convertSingleRow(point_cloud, v, rows2column[v], columns2pointcloud, transformation, skip_invalid_measurements);
    }

    /** Converts the depth map to a pointcloud according to the given transformation matrices.
     * On basis of the first and last transformation the transformations will be
     * interpolated and applied row-wise if the parameter apply_transforms_row_wise is 
     * set to true, otherwise they will be interpolated and applied col-wise.
     * 
     * @param point_cloud returned pointcloud
     * @param first_transformation transformation of the first row or column
     * @param last_transformation translation of the last row or column
     * @param skip_invalid_measurements true, if invalid measurements should be skipped
     * @param apply_transforms_row_wise true, if transformations should be applied row-wise
     */
    template<typename T>
    void convertDepthMapToPointCloud(std::vector<T> &point_cloud,
				const Eigen::Transform<typename T::Scalar,3,Eigen::Affine>& first_transformation,
				const Eigen::Transform<typename T::Scalar,3,Eigen::Affine>& last_transformation,
				bool skip_invalid_measurements = true,
				bool apply_transforms_row_wise = true) const
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
	std::vector< Eigen::Transform<typename T::Scalar,3,Eigen::Affine> > rows2column;
	std::vector< Eigen::Transform<typename T::Scalar,3,Eigen::Affine> > columns2pointcloud;
	computeLocalTransformations(rows2column, columns2pointcloud);
	
	Eigen::Matrix<typename T::Scalar,3,1> translation_delta = last_transformation.translation() - first_transformation.translation();
	Eigen::Quaterniond first_rotation = Eigen::Quaterniond(first_transformation.linear());
	Eigen::Quaterniond last_rotation = Eigen::Quaterniond(last_transformation.linear());
	
	// apply global interpolated transformations
	if(!apply_transforms_row_wise)
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
		convertSingleRow(point_cloud, v, rows2column[v], columns2pointcloud, Eigen::Transform<typename T::Scalar,3,Eigen::Affine>::Identity(), skip_invalid_measurements);
	}
	else
	{
	    std::vector< Eigen::Transform<typename T::Scalar,3,Eigen::Affine> > pointcloud2world;
	    for(unsigned v = 0; v < vertical_size; v++)
	    {
		Eigen::Transform<typename T::Scalar,3,Eigen::Affine> transformation = 
		    Eigen::Transform<typename T::Scalar,3,Eigen::Affine>(first_rotation.slerp((double)v / (double)(vertical_size-1), last_rotation));
		transformation.pretranslate(first_transformation.translation() + ((double)v / (double)(vertical_size-1)) * translation_delta);
		pointcloud2world.push_back(transformation);
	    }
	    
	    // convert rows
	    for(unsigned v = 0; v < vertical_size; v++)
		convertSingleRow(point_cloud, v, rows2column[v], columns2pointcloud, pointcloud2world[v], skip_invalid_measurements);
	}
    }

    /** Converts the depth map to a pointcloud according to the given transformation matrices.
     * Each transformation in the given vector will be applied to each row if the parameter 
     * apply_transforms_row_wise is set to true, otherwise it will be applied col-wise.
     * The transformations vector therefore must have either the same size of the columns 
     * or the rows.
     * 
     * @param point_cloud returned pointcloud
     * @param transformations vector of transformation matrices
     * @param skip_invalid_measurements true, if invalid measurements should be skipped
     * @param apply_transforms_row_wise true, if transformations should be applied row-wise
     */
    template<typename T>
    void convertDepthMapToPointCloud(std::vector<T> &point_cloud,
				const std::vector< Eigen::Transform<typename T::Scalar,3,Eigen::Affine> >& transformations,
				bool skip_invalid_measurements = true,
				bool apply_transforms_row_wise = true) const
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
	std::vector< Eigen::Transform<typename T::Scalar,3,Eigen::Affine> > rows2column;
	std::vector< Eigen::Transform<typename T::Scalar,3,Eigen::Affine> > columns2pointcloud;
	computeLocalTransformations(rows2column, columns2pointcloud);
	
	// apply global transformations
	if(!apply_transforms_row_wise)
	{
	    if(transformations.size() != horizontal_size)
		throw std::out_of_range("Invalid amount of transformations given");
	    for(unsigned h = 0; h < horizontal_size; h++)
		columns2pointcloud[h] = transformations[h] * columns2pointcloud[h];
	    
	    // convert rows
	    for(unsigned v = 0; v < vertical_size; v++)
		convertSingleRow(point_cloud, v, rows2column[v], columns2pointcloud, Eigen::Transform<typename T::Scalar,3,Eigen::Affine>::Identity(), skip_invalid_measurements);
	}
	else
	{
	    if(transformations.size() != vertical_size)
		throw std::out_of_range("Invalid amount of transformations given");
	    
	    // convert rows
	    for(unsigned v = 0; v < vertical_size; v++)
		convertSingleRow(point_cloud, v, rows2column[v], columns2pointcloud, transformations[v], skip_invalid_measurements);
	}
    }

protected:

    /** Helper method which converts a single row to a pointcloud. */
    template<typename T>
    void convertSingleRow(std::vector<T> &point_cloud, 
			    unsigned int row,
			    const Eigen::Transform<typename T::Scalar,3,Eigen::Affine>& row2column,
			    const std::vector< Eigen::Transform<typename T::Scalar,3,Eigen::Affine> >& columns2pointcloud,
			    const Eigen::Transform<typename T::Scalar,3,Eigen::Affine>& pointcloud2world,
			    bool skip_invalid_measurements) const
    {
	size_t row_index = (size_t)row * (size_t)horizontal_size;
	for(unsigned h = 0; h < horizontal_size; h++)
	{
	    if(isMeasurementValid(distances[row_index + h]))
	    {
		T point(distances[row_index + h], 0.0, 0.0);
		point =  pointcloud2world * (columns2pointcloud[h] * row2column * point);
		point_cloud.push_back(point);
	    }
	    else if(!skip_invalid_measurements)
	    {
		point_cloud.push_back(T(base::unknown<typename T::Scalar>(), base::unknown<typename T::Scalar>(), base::unknown<typename T::Scalar>()));
	    }
	}
    }

    /** Helper method to compute the local rows2column and columns2pointcloud transformations. */
    template<typename T>
    void computeLocalTransformations(std::vector< Eigen::Transform<T,3,Eigen::Affine> >& rows2column, 
				     std::vector< Eigen::Transform<T,3,Eigen::Affine> >& columns2pointcloud) const
    {
	// check interval sizes
	if(vertical_interval.size() > 2 && vertical_interval.size() != vertical_size)
	{
	    std::string err_msg = "Number of vertical ";
	    err_msg += vertical_projection == POLAR ? "angles" : "distances";
	    err_msg += "does no match the expected number of entries.";
	    throw std::invalid_argument(err_msg);
	}
	if(horizontal_interval.size() > 2 && horizontal_interval.size() != horizontal_size)
	{
	    std::string err_msg = "Number of horizontal ";
	    err_msg += horizontal_projection == POLAR ? "angles" : "distances";
	    err_msg += "does no match the expected number of entries.";
	    throw std::invalid_argument(err_msg);
	}

	// compute row2column transformations
	rows2column.resize(vertical_size);
	if(vertical_interval.empty())
	{
	    // no transformation set, use the identity for all vertical measurements
	    rows2column.resize(vertical_size, Eigen::Transform<T,3,Eigen::Affine>::Identity());
	}
	if(vertical_interval.size() == 1)
	{
	    // one planar translation or polar rotation for all vertical measurements
	    if(vertical_projection == PLANAR)
	    {
		Eigen::Transform<T,3,Eigen::Affine> trans = Eigen::Transform<T,3,Eigen::Affine>::Identity();
		trans.translation() = Eigen::Matrix<T,3,1>(0.0, 0.0, -1.0 * vertical_interval.front());
		rows2column.resize(vertical_size, trans);
	    }
	    else if(vertical_projection == POLAR)
		rows2column.resize(vertical_size, Eigen::Transform<T,3,Eigen::Affine>(Eigen::AngleAxis<T>(base::Angle::normalizeRad(vertical_interval.front()), Eigen::Matrix<T,3,1>::UnitY())));
	    else
		throw std::invalid_argument("Invalid argument for vertical projection type.");
	}
	if(vertical_interval.size() == 2)
	{
	    // interpolate transformations between the given start and end values for all vertical measurements
	    if(vertical_projection == PLANAR)
	    {
		double plane_resolution = computeResolution(vertical_interval, vertical_size, vertical_projection);
		Eigen::Transform<T,3,Eigen::Affine> trans = Eigen::Transform<T,3,Eigen::Affine>::Identity();
		for(unsigned v = 0; v < vertical_size; v++)
		{
		    trans.translation() = Eigen::Matrix<T,3,1>(0.0, 0.0, -1.0 * (vertical_interval.front() + ((double)v * plane_resolution)));
		    rows2column[v] = trans;
		}
	    }
	    else if(vertical_projection == POLAR)
	    {
		double angular_resolution = computeResolution(vertical_interval, vertical_size, vertical_projection);
		for(unsigned v = 0; v < vertical_size; v++)
		    rows2column[v] = Eigen::AngleAxis<T>(base::Angle::normalizeRad(vertical_interval.front() + (angular_resolution * (double)v)), Eigen::Matrix<T,3,1>::UnitY());
	    }
	    else
		throw std::invalid_argument("Invalid argument for vertical projection type.");
	}
	else
	{
	    // use unique planar translation or polar rotation for each vertical measurement
	    if(vertical_projection == PLANAR)
		for(unsigned v = 0; v < vertical_size; v++)
		{
		    Eigen::Transform<T,3,Eigen::Affine> trans = Eigen::Transform<T,3,Eigen::Affine>::Identity();
		    trans.translation() = Eigen::Matrix<T,3,1>(0.0, 0.0, -1.0 * vertical_interval[v]);
		    rows2column[v] = trans;
		}
	    else if(vertical_projection == POLAR)
		for(unsigned v = 0; v < vertical_size; v++)
		    rows2column[v] = Eigen::AngleAxis<T>(base::Angle::normalizeRad(vertical_interval[v]), Eigen::Matrix<T,3,1>::UnitY());
	    else
		throw std::invalid_argument("Invalid argument for vertical projection type.");
	}
	
	// compute columns2pointcloud transformations
	columns2pointcloud.resize(horizontal_size);
	if(horizontal_interval.empty())
	{
	    // no transformation set, use the identity for all horizontal measurements
	    columns2pointcloud.resize(horizontal_size, Eigen::Transform<T,3,Eigen::Affine>::Identity());
	}
	if(horizontal_interval.size() == 1)
	{
	    // one planar translation or polar rotation for all horizontal measurements
	    if(horizontal_projection == PLANAR)
	    {
		Eigen::Transform<T,3,Eigen::Affine> trans = Eigen::Transform<T,3,Eigen::Affine>::Identity();
		trans.translation() = Eigen::Matrix<T,3,1>(0.0, -1.0 * horizontal_interval.front(), 0.0);
		columns2pointcloud.resize(horizontal_size, trans);
	    }
	    else if(horizontal_projection == POLAR)
		columns2pointcloud.resize(horizontal_size, Eigen::Transform<T,3,Eigen::Affine>(Eigen::AngleAxis<T>(base::Angle::normalizeRad(horizontal_interval.front()), Eigen::Matrix<T,3,1>::UnitZ())));
	    else
		throw std::invalid_argument("Invalid argument for horizontal projection type.");
	}
	if(horizontal_interval.size() == 2)
	{
	    // interpolate transformations between the given start and end values for all horizontal measurements
	    if(horizontal_projection == PLANAR)
	    {
		double planar_resolution = computeResolution(horizontal_interval, horizontal_size, horizontal_projection);
		Eigen::Transform<T,3,Eigen::Affine> trans = Eigen::Transform<T,3,Eigen::Affine>::Identity();
		for(unsigned h = 0; h < horizontal_size; h++)
		{
		    trans.translation() = Eigen::Matrix<T,3,1>(0.0, -1.0 * (horizontal_interval.front() + ((double)h * planar_resolution)), 0.0);
		    columns2pointcloud[h] = trans;
		}
	    }
	    else if(horizontal_projection == POLAR)
	    {
		double angular_resolution = computeResolution(horizontal_interval, horizontal_size, horizontal_projection);
		for(unsigned h = 0; h < horizontal_size; h++)
		    columns2pointcloud[h] = Eigen::AngleAxis<T>(base::Angle::normalizeRad(horizontal_interval.front() - (angular_resolution * (double)h)), Eigen::Matrix<T,3,1>::UnitZ());
	    }
	    else
		throw std::invalid_argument("Invalid argument for horizontal projection type.");
	}
	else
	{
	    // use unique planar translation or polar rotation for each horizontal measurement
	    if(horizontal_projection == PLANAR)
		for(unsigned h = 0; h < horizontal_size; h++)
		{
		    Eigen::Transform<T,3,Eigen::Affine> trans = Eigen::Transform<T,3,Eigen::Affine>::Identity();
		    trans.translation() = Eigen::Matrix<T,3,1>(0.0, -1.0 * horizontal_interval[h], 0.0);
		    columns2pointcloud[h] = trans;
		}
	    else if(horizontal_projection == POLAR)
		for(unsigned h = 0; h < horizontal_size; h++)
		    columns2pointcloud[h] = Eigen::AngleAxis<T>(base::Angle::normalizeRad(horizontal_interval[h]), Eigen::Matrix<T,3,1>::UnitZ());
	    else
		throw std::invalid_argument("Invalid argument for horizontal projection type.");
	}
    }
    
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
	    {
		double diff = std::abs(base::Angle::normalizeRad(interval.back() - interval.front()));
		step_resolution = diff / (double)(elements-1);
	    }
	}
	else
	    throw std::invalid_argument("Invalid argument for projection type.");
	
	return step_resolution;
    };

    /** Checks if the vertical and horizontal sizes match the size of distance vector. */
    inline bool checkSizeConfig() const
    {
	return ((((size_t)vertical_size * (size_t)horizontal_size) == distances.size()) ? true : false);
    }
};

}} // namespaces

#endif