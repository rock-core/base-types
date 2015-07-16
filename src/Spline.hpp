#ifndef  _BASE_SPLINE_HPP_INC
#define  _BASE_SPLINE_HPP_INC

#include <vector>
#include <base/Eigen.hpp>
#include <stdexcept>
#include <algorithm>

struct SISLCurve;

namespace base {
namespace geometry {
    /** Representation and manipulation of a N-order, 3D NURBS curve
     *
     * The availability of this class is dependent on the availability of the
     * SISL library
     *
     * http://www.sintef.no/Informasjons--og-kommunikasjonsteknologi-IKT/Anvendt-matematikk/Fagomrader/Geometri/Prosjekter/The-SISL-Nurbs-Library/SISL-Homepage/
     */
    class SplineBase
    {
    public:
        SplineBase(SplineBase const& source);
        ~SplineBase();

        explicit SplineBase(int dimension,
                double geometric_resolution = 0.1, int order = 3);
        explicit SplineBase(double geometric_resolution, SISLCurve* curve);

        /** Changes the default geometric resolution */
        void setGeometricResolution(double _geores) { geometric_resolution = _geores; }
        double getGeometricResolution() const { return geometric_resolution; };

        /** Returns true if the curve is not yet initialized */
        bool isEmpty() const
        { return !getSISLCurve() && singleton.empty(); }

        /** Returns true if the curve is a point */
        bool isSingleton() const
        { return !singleton.empty(); }

        /** Returns the dimension of the space in which the curve lies */
        int    getDimension() const { return dimension; }
        /** Returns the number of points for this curve */
        int    getPointCount() const;
        /** Change the curve order. This is only propagated after interpolate()
         * is called
         */
        void setCurveOrder(int value) { curve_order = value; }
        /** Returns the order of the curve */
        int    getCurveOrder() const { return curve_order; }
        
        /** Returns the length of the curve in geometric space
         *
         * @param relative_error the acceptable error on the final result w.r.t.
         *   the real curve length
         */
        double getCurveLength(double relative_resolution = 0.01) const;
        double getCurveLength(double startParam, double relative_resolution) const;
        double getCurveLength(double startParam, double endParam, double relative_resolution) const;
        
        
        /** Returns the maximum curvature of the curve */
        double getCurvatureMax();
        double getStartParam() const { return start_param; };
        double getEndParam()   const { return end_param; };

        /** Return a pointer to the underlying SISL structure
         *
         * This pointer will be non-NULL only after interpolate() has been called
         * at least once.
         */
        SISLCurve const* getSISLCurve() const;

        /** Return a pointer to the underlying SISL structure
         *
         * This pointer will be non-NULL only after interpolate() has been called
         * at least once.
         */
        SISLCurve* getSISLCurve();
      
        /** Returns the curvature at the given position
         *
         * @throws out_of_range if _param is not in [start_param,
         * end_param] and runtime_error if SISL returns an error
         */
        double getCurvature(double _param) const;

        /** Returns the first order derivative of the curvature at the given
         * position
         *
         * @throws out_of_range if _param is not in [start_param,
         * end_param] and runtime_error if SISL returns an error
         */
        double getVariationOfCurvature(double _param);  // Variation of Curvature

        std::vector<double> getCoordinates() const;

        std::vector<double> getKnots() const;

        int getSISLCurveType() const;

        /** Display the curve properties on the given IO object */
        void printCurveProperties(std::ostream& io);

	/**
	 *  types to be used in the interpolate() method
	 */
	enum CoordinateType
	{
	    ORDINARY_POINT = 1,
	    KNUCKLE_POINT = 2,
	    DERIVATIVE_TO_NEXT = 3,
	    DERIVATIVE_TO_PRIOR = 4,
	    SECOND_DERIVATIVE_TO_NEXT = 5,
	    SECOND_DERIVATIVE_TO_PRIOR = 6,
	    TANGENT_POINT_FOR_NEXT = 13,
	    TANGENT_POINT_FOR_PRIOR = 14,
	};

        /** Generates the curve
         *
         * If the \c parameters array is given, it is the desired parameter for
         * the provided points. Otherwise, an automatic parametrization is
         * generated.
	 *
	 * @param coordinates - list of points/derivatives
	 * @param parameters - list of parameters. Needs to be the same size as
	 *        the number of actual points in the coordinates list, or zero.
	 * @param coord_types - needs to be of the same size as coordinates, or
	 *        zero. It marks the type of coordinate/derivative given by the
	 *        coordinates param.
         */
        void interpolate(std::vector<double> const& coordinates, 
                std::vector<double> const& parameters = std::vector<double>(), 
                std::vector<CoordinateType> const& coord_types = std::vector<CoordinateType>() );

        void interpolate(std::vector<double> const& coordinates, 
                std::vector<double> &parameterOut, 
                std::vector<double> const& parameterIn  = std::vector<double>(), 
                std::vector<CoordinateType> const& coord_types = std::vector<CoordinateType>() );

        /** Tests for intersection between two curves
         */
        bool testIntersection(SplineBase const& other, double resolution = 0.01) const;

        /** Reinitializes the curve */
        void clear();

        std::vector<double> simplify();
        std::vector<double> simplify(double tolerance);

        SplineBase const& operator = (SplineBase const& base);

        bool isNURBS() const;

        /** Replaces the current internal SISLCurve by a new one that contains
         * the given information
         *
         * The semantic of the three parameters is internal to SISL
         */
        void reset(std::vector<double> const& coordinates, std::vector<double> const& knots, int kind = -1);

        /** Reverses the direction of the curve
         */
        void reverse();

        void append(SplineBase const& other);

        /** Appends @c other at the end of @c this.
         *
         * \c other is translated so that other's start point is equal to \c
         * this's end point. The resulting curve is then appended to \c this
         *
         * @see join
         */
        void append(SplineBase const& other, double tolerance);

        /** Joins \c this and \c other
         *
         * Unlike \c append, if the distance between \c this's end point and \c
         * other's start point is greater than \c tolerance, the method computes
         * an intermediate curve that smoothly joins \c this and \c other.
         *
         * It retusn the parameter offset by which the parameters in \c other
         * have been translated. I.e. if 
         *
         * <code>
         * offset = curve.join(other, tol, true);
         * </code>
         *
         * Then other.get(t) == curve.get(t + offset)
         */
        double join(SplineBase const& other, double tolerance, bool with_tangents);

        /** Splits this curve in two parts, at the specified parameter
         *
         * \c this is the first part of the new pair. \c second_part is filled
         * with the second part
         */
        void split(SplineBase& second_part, double parameter);

        /** Crops this curve at the specified boundaries
         *
         * After this operation, +this+ will represent the part of the curve
         * that was in between the two given parameters
         */
        void crop(double start_t, double end_t);

        /**
         * Returns a new curve, that represents the
         * spline between start_t and end_t
         * */
        SplineBase *getSubSpline(double start_t, double end_t) const;
        
        int getCoordinatesStride() const
        {
            if (isNURBS()) return dimension + 1;
            else return dimension;
        }

        void setSingleton(double const* coordinates);

    protected:
	/**
	 * This function checks weather param is smaler or bigger
	 * than end and start param. It also sets param to start 
	 * or end, if the difference between param and start/end
	 * is smaler than equalDistance.
	 * 
	 * returns true if param is inside start/end.
	 * */
	bool checkAndNormalizeParam(double &param, double equalDistance = 0.001) const;

        /** Replaces the current internal SISLCurve by the provided one. The
         * SplineBase object takes ownership of the given curve.
         *
         * Is is used in mutating methods that need to create a new curve: i.e.
         * create a new curve, do SISL operation, replace current curve by new
         * curve by calling reset(new_curve)
         */
        void reset(SISLCurve* curve);
        void getPoint(double* result, double _param) const;
        void getPointAndTangent(double* result, double _param) const;

        void findPointIntersections(double const* _point,
                std::vector<double>& _result_points,
                std::vector< std::pair<double, double> >& _result_curves,
                double _geores) const;

        std::pair<double, bool> findOneLineIntersection(double const* _point,
                double const* _normal, double _guess, double _geores) const;
        void findLineIntersections(double const* _point, double const* _normal,
                std::vector<double>& _result_points,
                std::vector< std::pair<double, double> >& _result_curves,
                double _geores) const;

        void findSphereIntersections(double const* _center, double radius,
                std::vector<double>& points,
                std::vector< std::pair<double, double> >& segments, double _geores) const;

        /**
         * Warning, do not use this method, it is broken and returns the wrong result
         * 
         * */
        double findOneClosestPoint(double const* _pt, double _guess, double _geores) const;
        void findClosestPoints(double const* ref_point,
                std::vector<double>& _result_points,
                std::vector< std::pair<double, double> >& _result_curves,
                double _geores) const;

        double localClosestPointSearch(double const* ref_point,
                double _guess, double _start, double _end,
                double  _geores) const;

        void getPointAndTangentHelper(double* result, double _param, bool with_tangent) const;

        /** Helper function for findOneClosestPoint and findOneLineIntersection.
         * It returns the parameter in points and/or curves that is the closest
         * to the given guess
         *
         * The caller is in charge of making sure that at least one of \c points
         * or \c curves is not empty
         */
        double getResultClosestToGuess(double _guess, std::vector<double> points,
            std::vector< std::pair<double, double> > curves) const;

        //! available only in Spline<3>
        base::Matrix3d getFrenetFrame(double _param);
        //! available only in Spline<3>
        double getHeading(double _param);
        //! available only in Spline<3>
        double headingError(double _actHeading, double _param);
        //! available only in Spline<3>
        double distanceError(base::Vector3d _pt, double _param);
        //! available only in Spline<3>
        base::Vector3d poseError(base::Vector3d _pt, double _actZRot, double _st_para);
        //! available only in Spline<3>
        base::Vector3d poseError(base::Vector3d _pt, double _actZRot, double _st_para, double minParam);
    private:
        std::vector<double> singleton;

        int dimension;
        //! the underlying SISL curve
        SISLCurve *curve;

        //! the geometric resolution
        double geometric_resolution;
        //! the order of the NURBS curve that approximates the points
        int curve_order;
        //! the start parameter (usually 0.0)
        double start_param;
        //! the end parameter, as returned by SISL
        double end_param;

        //! if we have already calculated the maximum curvature
        bool has_curvature_max;
        //! maximum curvature in the curve
        double curvature_max;
    };

    /** Intermediate base class to add functionality that is specific to 3D
     * curves
     */
    class Spline3Base : public SplineBase
    {
    public:
        /** Pass-through constructor for Spline<3>. The check on dimensionality
         * is done by Spline<>
         */
        explicit Spline3Base(int dimension, double geometric_resolution, int order)
            : SplineBase(3, geometric_resolution, order) {}
        /** Pass-through constructor for Spline<3>. The check on dimensionality
         * is done by Spline<>
         */
        explicit Spline3Base(double geometric_resolution, SISLCurve* curve)
            : SplineBase(geometric_resolution, curve) {}
        Spline3Base(SplineBase const& source)
            : SplineBase(source) {}

        base::Matrix3d getFrenetFrame(double _param)
        { return SplineBase::getFrenetFrame(_param); }

        double getHeading(double _param)
        { return SplineBase::getHeading(_param); }

        double headingError(double _actZRot, double _param)
        { return SplineBase::headingError(_actZRot, _param); }

        double distanceError(base::Vector3d _pt, double _param)
        { return SplineBase::distanceError(_pt, _param); }

        /** Searches for the closest point in the curve, and returns the pose
         * error between the frenet frame on the curve and the given pose given
         * by _position and _heading.
         *
         * The returned vector is (distance_error, heading_error,
         * curve_parameter)
         */
        base::Vector3d poseError(base::Vector3d const& _position, double _heading, double _guess)
        { return SplineBase::poseError(_position, _heading, _guess); }
        
        /** Searches for the closest point in the curve, the it checks, if the closest point is
         * an advancement on the trajectory in respect to the given 'minParam'. If this is the 
         * case is uses the new point, else it uses minParam.
         * Returns the pose
         * error between the frenet frame on the curve and the given pose (determinded by the current param)
         * given by _position and _heading.
         *
         * The returned vector is (distance_error, heading_error,
         * curve_parameter)
         * */
        base::Vector3d poseError(base::Vector3d const& _position, double _heading, double _guess, double minParam)
        { return SplineBase::poseError(_position, _heading, _guess, minParam); }

    };

    template<int DIM> struct SplineBaseClass
    { typedef SplineBase type; };
    template<> struct SplineBaseClass<3>
    { typedef Spline3Base type; };

    template<int DIM>
    class Spline : public SplineBaseClass<DIM>::type
    {
    public:
        typedef typename SplineBaseClass<DIM>::type base_t;
        typedef Eigen::Matrix<double, DIM, 1, Eigen::DontAlign> vector_t;
        typedef Eigen::Matrix<double, DIM, 1, Eigen::AutoAlign> vector_ta;
        typedef Eigen::Transform<double, DIM, Eigen::Affine> transform_t;

        explicit Spline(double geometric_resolution = 0.1, int order = 3)
            : base_t(DIM, geometric_resolution, order) {}
        explicit Spline(double geometric_resolution, SISLCurve* curve)
            : base_t(geometric_resolution, curve)
        {
            if (this->getDimension() != DIM)
                throw std::runtime_error("trying to initialize a Spline<> class with a curve of wrong dimension");
        }
        Spline(SplineBase const& base)
            : base_t(base) {}

        /** Returns the geometric start point of the curve */
        vector_t getStartPoint() const
        { return getPoint(this->getStartParam()); }

        /** Returns the geometric end point of the curve */
        vector_t getEndPoint() const
        { return getPoint(this->getEndParam()); }

        /** Resets this curve to a singleton */
        void setSingleton(vector_t const& v)
        {
            SplineBase::setSingleton(v.data());
        }

        /** Returns the geometric point that lies on the curve at the given
         * parameter */
        vector_t getPoint(double _param) const
        {
            vector_t result;
            SplineBase::getPoint(result.data(), _param);
            return result;
        }

        std::vector<vector_t> getPoints(std::vector<double> const& parameters) const
        {
            std::vector<vector_t> result;
            result.reserve(parameters.size());
            for (unsigned int i = 0; i < parameters.size(); ++i)
                result.push_back(getPoint(parameters[i]));
            return result;
        }

        /** Private helper method for advance and length
         *
         * Iteratively computes a discretization of the curve from +start+ to
         * +end+, with two points in the discretization not being further apart
         * than _geores.
         *
         * It accumulates the distance while doing so. If, at some point in the
         * search, the accumulated distance \c cur_length gets bigger than \c
         * target, then the parameter at that point gets stored in result and
         * the method returns true.
         *
         * If it never happens, the method returns false. In this case, \c
         * result is set to \c end and \c cur_length is the approximate length
         * of the whole segment.
         */
        bool doAdvance(double& result, double& cur_length, double target, double start, vector_t const& start_p, double end, vector_t const& end_p, double _geores) const
        {
            double d = (start_p - end_p).norm();
            if (d < _geores)
            {
                if(target < 0)
                {
                    cur_length -= d;
                    if (cur_length < target)
                    {
                        result = end;
                        return true;
                    }
                }
                else
                {
                    cur_length += d;
                    if (cur_length > target)
                    {
                        result = end;
                        return true;
                    }
                }
                  
                return false;
            }

            double middle = (start + end) / 2;
            vector_t middle_p = getPoint(middle);

            if (doAdvance(result, cur_length, target, start, start_p, middle, middle_p, _geores))
                return true;
            if (doAdvance(result, cur_length, target, middle, middle_p, end, end_p, _geores))
                return true;

            return false;
        }

        /** Find a parameter separated from another by a given curve length
         *
         * Specifically, this method finds the parameter t1 so that the curve
         * length between t and t1 is in [length, length + _geores]
         *
         * If the end of the curve is reached first, then the parameter of the
         * end of the curve is returned.
         */
        std::pair<double, double> advance(double t, double length, double _geores) const
        {
            double result_t = 0;
            double result_d = 0;
            if(length < 0)
            {
                if (!doAdvance(result_t, result_d, length, t, getPoint(t), this->getStartParam(), getPoint(this->getStartParam()), _geores))
                    return std::make_pair(this->getStartParam(), result_d);
            }
            else
            {
                if (!doAdvance(result_t, result_d, length, t, getPoint(t), this->getEndParam(), getPoint(this->getEndParam()), _geores))
                    return std::make_pair(this->getEndParam(), result_d);
            }
            return std::make_pair(result_t, result_d);
        }


        /** Computes the length of a curve segment
         *
         * Specifically, this method finds the length of the curve by
         * discretizing it with a step length of _geores
         */
        double length(double start, double end, double _geores) const
        {
            double result_t = 0;
            double result_d = 0;
            doAdvance(result_t, result_d, std::numeric_limits<double>::infinity(), start, getPoint(start), end, getPoint(end), _geores);
            return result_d;
        }

        /** Returns the geometric point that lies on the curve at the given
         * parameter */
        std::pair<vector_t, vector_t> getPointAndTangent(double _param) const
        {
            double result[DIM * 2];
            SplineBase::getPointAndTangent(result, _param);
            vector_t point(result);
            vector_t tangent(result + DIM);
            return std::make_pair(point, tangent);
        }

        /** Returns a discretization of this curve so that two consecutive
         * points are separated by a curve length lower than _geores
         */
        std::vector<vector_t> sample(double _geores, std::vector<double>* parameters = 0, int max_recursion = 20) const
        {
            std::vector<vector_t> result;
            sample(result, _geores, parameters, max_recursion);
            return result;
        }

        /** Samples the curve so that the distance between two consecutive
         * points is always below _geores
         */
        void sample(std::vector<vector_t>& result, double _geores, std::vector<double>* parameters = 0, int max_recursion = 20) const
        {
            double start = this->getStartParam();
            vector_t start_p = this->getPoint(start);
            if (parameters)
                parameters->push_back(start);
            result.push_back(start_p);

            double end   = this->getEndParam();
            vector_t end_p = this->getPoint(end);
            sample(result, start, start_p, end, end_p, _geores, parameters, max_recursion);
        }

        /** Helper method for the other sample methods
         */
        void sample(std::vector<vector_t>& result, double start, vector_t const& start_p, double end, vector_t const& end_p, double _geores, std::vector<double>* parameters, int max_recursion = 20) const
        {
            if (max_recursion == 0 || (start_p - end_p).norm() < _geores)
            {
                if (parameters)
                    parameters->push_back(end);

                result.push_back(end_p);
                return;
            }

            double middle = (start + end) / 2;
            vector_t middle_p = getPoint(middle);
            sample(result, start, start_p, middle, middle_p, _geores, parameters, max_recursion - 1);
            sample(result, middle, middle_p, end, end_p, _geores, parameters, max_recursion - 1);
        }

        /** Compute the curve from the given set of points */
        void interpolate(std::vector<vector_t> const& points, 
		std::vector<double> const& parameters = std::vector<double>(),
		std::vector<SplineBase::CoordinateType> const& coord_types = std::vector<SplineBase::CoordinateType>() )
        {
            std::vector<double> coordinates;
            for (size_t i = 0; i < points.size(); ++i)
                coordinates.insert(coordinates.end(), points[i].data(), points[i].data() + DIM);
            SplineBase::interpolate(coordinates, parameters, coord_types);
        }

        void interpolate(std::vector<vector_t> const& points, 
                std::vector<double> &parametersOut,
                std::vector<double> const& parametersIn = std::vector<double>(),
                std::vector<SplineBase::CoordinateType> const& coord_types = std::vector<SplineBase::CoordinateType>() )
        {
            std::vector<double> coordinates;
            for (size_t i = 0; i < points.size(); ++i)
                coordinates.insert(coordinates.end(), points[i].data(), points[i].data() + DIM);
            SplineBase::interpolate(coordinates, parametersOut, parametersIn, coord_types);
        }

        void interpolate(std::vector<vector_ta> const& points, 
                std::vector<double> &parametersOut,
                std::vector<double> const& parametersIn = std::vector<double>(),
                std::vector<SplineBase::CoordinateType> const& coord_types = std::vector<SplineBase::CoordinateType>() )
        {
            std::vector<double> coordinates;
            for (size_t i = 0; i < points.size(); ++i)
                coordinates.insert(coordinates.end(), points[i].data(), points[i].data() + DIM);
            SplineBase::interpolate(coordinates, parametersOut, parametersIn, coord_types);
        }

        /** Compute the curve from the given set of points */
        void interpolate(std::vector<vector_ta> const& points, 
                std::vector<double> const& parameters = std::vector<double>(),
                std::vector<SplineBase::CoordinateType> const& coord_types = std::vector<SplineBase::CoordinateType>() )
        {
            std::vector<double> coordinates;
            for (size_t i = 0; i < points.size(); ++i)
                coordinates.insert(coordinates.end(), points[i].data(), points[i].data() + DIM);
            SplineBase::interpolate(coordinates, parameters, coord_types);
        }
        
        void interpolate(std::vector<double> const& coordinates, 
                std::vector<double> const& parameters = std::vector<double>(), 
                std::vector<SplineBase::CoordinateType> const& coord_types = std::vector<SplineBase::CoordinateType>() )
        {
            return SplineBase::interpolate(coordinates, parameters, coord_types);
        };

        void interpolate(std::vector<double> const& coordinates, 
                std::vector<double> &parameterOut, 
                std::vector<double> const& parameterIn  = std::vector<double>(), 
                std::vector<SplineBase::CoordinateType> const& coord_types = std::vector<SplineBase::CoordinateType>() )
        {
            return SplineBase::interpolate(coordinates, parameterOut, parameterIn, coord_types);
        };

        
        /** Returns the distance between the given point and the curve
         */
        double distanceTo(vector_t const& _pt) const
        {
            double closest = findOneClosestPoint(_pt);
            vector_t curve_p = getPoint(closest);
            return (_pt - curve_p).norm();
        }

        template<typename Test>
        std::pair<double, double> dichotomic_search(double start_t, double end_t, Test test, double resolution, double parameter_threshold) const
        {
            return this->dichotomic_search(
                    start_t, this->getPoint(start_t),
                    end_t, this->getPoint(end_t),
                    test, resolution, parameter_threshold);
        }

        /** Does a dichotomic search to find the first point in [start_t, end_t]
         * for which func returns true
         *
         * end_t might be lower than start_t, in which case the point returned
         * will be the last point in the curve
         */
        template<typename Test>
        std::pair<double, double> dichotomic_search(double start_t, vector_t const& start_p, double end_t, vector_t const& end_p,
                Test test, double resolution, double parameter_threshold) const
        {
            std::pair<bool, double> test_result =
                test(start_t, end_t, *this);
            if (!test_result.first)
                return std::make_pair(0, 0);

            if (fabs(end_t - start_t) < parameter_threshold || test_result.second < resolution)
                return std::make_pair(start_t, end_t);

            double middle_t = (start_t + end_t) / 2;
            vector_t middle_p = getPoint(middle_t);
            std::pair<double, double> result;

            result = dichotomic_search(start_t, start_p, middle_t, middle_p, test, resolution, parameter_threshold);
            if (result.first != result.second)
                return result;

            result = dichotomic_search(middle_t, middle_p, end_t, end_p, test, resolution, parameter_threshold);
            if (result.first != result.second)
                return result;

            throw std::runtime_error("cannot find a solution, but we should have. Is the provided function stable ?");
        }

        void findSphereIntersections(vector_t const& _center, double _radius,
                std::vector<double>& points, std::vector< std::pair<double, double> >& segments) const
        { return findSphereIntersections(_center, _radius, points, segments, SplineBase::getGeometricResolution()); }

        void findSphereIntersections(vector_t const& _center, double _radius,
                std::vector<double>& points, std::vector< std::pair<double, double> >& segments, double _geores) const
        { return SplineBase::findSphereIntersections(_center.data(), _radius,
                points, segments, _geores); }

        void findPointIntersections(vector_t const& _point,
                std::vector<double>& _result_points,
                std::vector< std::pair<double, double> >& _result_curves,
                double _geores) const
        { return SplineBase::findPointIntersections(_point.data(),
                _result_points, _result_curves, _geores); }

        /** \overload
         */
        std::pair<double, bool> findOneLineIntersection(vector_t const& _pt, vector_t const& _normal) const
        { return findOneLineIntersection(_pt, _normal, SplineBase::getGeometricResolution()); }

        /** \overload
         *
         * Calls findOneClosestPoint using the start parameter as the guess
         * parameter. I.e. it will always return the point closest to the start
         * of the curve.
         */
        std::pair<double, bool> findOneLineIntersection(vector_t const& _pt, vector_t const& _normal, double geometric_resolution) const
        { return findOneLineIntersection(_pt, _normal, SplineBase::getStartParam(), geometric_resolution); }

        /** Returns a single intersection point between this curve and the line
         * or plan defined by \c _pt and \c _normal
         *
         * This is a convenience method that calls findLineIntersections and
         * returns one single parameter in the values returned. The returned
         * parameter is the closes point closest to _guess.
         *
         * @return a (t, true) pair, with t being the parameter of the found intersection, if there is one. If
         * there is none, the (0, false) pair is returned
         */
        std::pair<double, bool> findOneLineIntersection(vector_t const& _pt, vector_t const& _normal, double _guess, double _geometric_resolution) const
        { return SplineBase::findOneLineIntersection(_pt.data(), _normal.data(), _guess, _geometric_resolution); }

        /** \overload
         */
        std::pair<double, bool> findOneLineIntersection(vector_t const& _pt, vector_t const& _normal, vector_t const& _guess, double _geometric_resolution) const
        {
            std::vector<double> points;
            std::vector< std::pair<double, double> > curves;
            findLineIntersections(_pt, _normal, points, curves, _geometric_resolution);

            if (points.empty() && curves.empty())
                return std::make_pair(0, false);

            for (unsigned int i = 0; i < curves.size(); ++i)
            {
                points.push_back(curves[i].first);
                points.push_back(curves[i].second);
            }

            double result_t = points[0];
            vector_t result_p = getPoint(result_t);
            double min_d = (result_p - _guess).norm();
            for (unsigned int i = 1; i < points.size(); ++i)
            {
                vector_t p = getPoint(points[i]);
                double   d = (p - _guess).norm();
                if (d < min_d)
                {
                    result_t = points[i];
                    result_p = p;
                    min_d    = d;
                }
            }
            return std::make_pair(result_t, true);
        }

        void findLineIntersections(vector_t const& _point, vector_t const& _normal,
                std::vector<double>& _result_points,
                std::vector< std::pair<double, double> >& _result_curves,
                double _geores) const
        {
            return SplineBase::findLineIntersections(_point.data(), _normal.data(),
                    _result_points, _result_curves, _geores);
        }


        void findSegmentIntersections(vector_t const& _p0, vector_t const& _p1,
                std::vector<double>& _result_points,
                std::vector< std::pair<double, double> >& _result_curves,
                double _geores) const
        {
            std::vector<double> points;
            std::vector< std::pair<double, double> > curves;

            vector_t p0p1 = (_p1 - _p0);
            double p0p1_length = p0p1.norm();
            p0p1.normalize();

            vector_t normal;
            if (p0p1.x() < p0p1.y() && p0p1.x() < p0p1.z())
                normal = p0p1.cross(Eigen::Vector3d::UnitX());
            else if (p0p1.y() < p0p1.x() && p0p1.y() < p0p1.z())
                normal = p0p1.cross(Eigen::Vector3d::UnitY());
            else if (p0p1.z() < p0p1.x() && p0p1.z() < p0p1.y())
                normal = p0p1.cross(Eigen::Vector3d::UnitY());
            findLineIntersections(_p0, normal, points, curves, _geores);

            for (unsigned int i = 0; i < points.size(); ++i)
            {
                vector_t p = getPoint(points[i]);
                double p_t = (p - _p0).dot(p0p1);
                if (p_t >=0 && p_t <= p0p1_length && fabs((p - _p0).dot(normal)) < _geores)
                    _result_points.push_back(points[i]);
            }

            for (unsigned int curve_idx = 0; curve_idx < curves.size(); ++curve_idx)
            {
                double curve_t[2] =
                    { curves[curve_idx].first,  curves[curve_idx].second };
                vector_t p[2];
                double segment_t[2];
                for (int i = 0; i < 2; ++i)
                {
                    p[i] = getPoint(curve_t[i]);
                    segment_t[i] = (p[i] - _p0).dot(p0p1);
                }

                double* min_t = std::min_element(segment_t, segment_t + 2);
                double* max_t = std::max_element(segment_t, segment_t + 2);
                if (*min_t <= p0p1_length && *max_t >= 0)
                {
                    std::pair<double, double> segment;
                    if (*min_t < 0)
                        segment.first = findOneClosestPoint(_p0);
                    else
                        segment.first = curve_t[min_t - segment_t];

                    if (*max_t > p0p1_length)
                        segment.second = findOneClosestPoint(_p1);
                    else
                        segment.second = curve_t[max_t - segment_t];

                    if (segment.first > segment.second)
                        std::swap(segment.first, segment.second);
                    _result_curves.push_back(segment);
                }
            }
        }

        /** Returns true if this curve intersects the given segment */
        bool isIntersectingSegment(vector_t const& _p0, vector_t const& _p1, double _geores)
        {
            std::vector<double> points;
            std::vector< std::pair<double, double> > curves;
            findSegmentIntersections(_p0, _p1, points, curves, _geores);
            return !points.empty() || !curves.empty();
        }

        /** \overload
         */
        double findOneClosestPoint(vector_t const& _pt) const
        { return findOneClosestPoint(_pt, SplineBase::getGeometricResolution()); }

        bool isCloser(const vector_t &p, const double &squaredDist, const double param, vector_t &pOfParam, double &squaredDistOfParam) const
        {
            vector_t curPoint = getPoint(param);
            double curSquaredDist = (curPoint - p).squaredNorm();
            if( curSquaredDist < squaredDist )
            {
                pOfParam = curPoint;
                squaredDistOfParam = curSquaredDist;
                return true;
            }
            
            return false;
        }
        
        /** 
         * This method return the closest point in the curve to the given
         * point _pt.
         */
        double findOneClosestPoint(vector_t const& _pt, double _geometric_resolution) const
        { 
            if (!SplineBase::getSISLCurve())
                return SplineBase::getStartParam();

            std::vector<double> points;
            std::vector< std::pair<double, double> > curves;
            findClosestPoints(_pt, points, curves, _geometric_resolution);
            
            vector_t closestPoint;
            double closestParam;
            double closestSquaredDist = std::numeric_limits< double >::max();
            if (points.empty())
            {
                if (curves.empty())
                    throw std::logic_error("no closest point returned by findClosestPoints");
                else
                {
                    closestPoint = getPoint(curves.front().first);
                    closestParam = curves.front().first;
                    closestSquaredDist = (_pt - closestPoint).squaredNorm();
                }
            }
            else
            {
                closestPoint = getPoint(points.front());
                closestParam = points.front();
                closestSquaredDist = (_pt - closestPoint).squaredNorm();
                for(std::vector<double>::iterator it = points.begin() + 1; it != points.end(); ++it) 
                {
                    if(isCloser(_pt, closestSquaredDist, *it, closestPoint, closestSquaredDist))
                        closestParam = *it;
                }
            }

            for (std::vector< std::pair<double, double> >::const_iterator it = curves.begin();
                    it != curves.end(); ++it)
            {
                if(isCloser(_pt, closestSquaredDist, it->first, closestPoint, closestSquaredDist))
                    closestParam = it->first;

                if(isCloser(_pt, closestSquaredDist, it->second, closestPoint, closestSquaredDist))
                    closestParam = it->second;
            }
            return closestParam;
        }

        /** Returns a single closest point to _pt
         *
         * This is a convenience method that calls findClosestPoints and returns
         * one single parameter in the values returned. 
         *
         * @return the parameter of the found closes point
         * @param _guess this paramter is ignored. It is only there for backward compability
         * @throw std::logic_error if no points have been found (should not happen)
         * @see localClosestPointSearch findClosestPoints
         */
        double findOneClosestPoint(vector_t const& _pt, double _guess, double _geometric_resolution) const
        { return findOneClosestPoint(_pt, _geometric_resolution); }

        /** \overload
         */
        void findClosestPoints(vector_t const& _pt,
                std::vector<double>& _points,
                std::vector< std::pair<double, double> >& _curves) const
        { return findClosestPoints(_pt, _points, _curves, SplineBase::getGeometricResolution()); }

        /**
         * Returns the single points and curve segments that are closest to
         * the given point.
         *
         */
        void findClosestPoints(vector_t const& _pt,
                std::vector<double>& _points,
                std::vector< std::pair<double, double> >& _curves,
                double _geores) const
        { return SplineBase::findClosestPoints(_pt.data(), _points, _curves, _geores); }

        /** \overload
         */
        double localClosestPointSearch(vector_t const& _pt, double _guess, double _start, double _end)
        { return localClosestPointSearch(_pt, _guess, _start, _end, SplineBase::getGeometricResolution()); }

        /** Performs a Newton search in the provided parametric interval, starting with the given guess. The order between _start and _end can be arbitrary.
         *
         * This method is subject to local minima problems
         */
        double localClosestPointSearch(vector_t const& _pt, double _guess, double _start, double _end, double _geores)
        { return SplineBase::localClosestPointSearch(_pt.data(), _guess, _start, _end, _geores); }

        template<typename Transform>
        void transform(Transform const& t)
        {
            if (SplineBase::isEmpty())
                return;
            else if (SplineBase::isSingleton())
            {
                vector_t v = getPoint(0);
                v = t * v;
                std::vector<double> coordinates(v.data(), v.data() + DIM);
                SplineBase::reset(coordinates, std::vector<double>());
            }
            else
            {
                std::vector<double> const& current_coordinates = SplineBase::getCoordinates();
                std::vector<double> coordinates(current_coordinates.begin(), current_coordinates.end());
                int stride = SplineBase::getCoordinatesStride();

                vector_t v;
                for (unsigned int i = 0; i < current_coordinates.size(); i += stride)
                {
                    memcpy(v.data(), &coordinates[i], sizeof(double) * DIM);
                    v = t * v;
                    memcpy(&coordinates[i], v.data(), sizeof(double) * DIM);
                }
                SplineBase::reset(coordinates, SplineBase::getKnots());
            }
        }
    };

    // This is for GCCXML parsing
    struct __gccxml_workaround_spline {
        base::geometry::Spline<1> instanciation1;
        base::geometry::Spline<2> instanciation2;
        base::geometry::Spline<3> instanciation3;
    };
    typedef base::geometry::Spline<1> Spline1;
    typedef base::geometry::Spline<2> Spline2;
    typedef base::geometry::Spline<3> Spline3;
    
    inline std::ostream& operator << (std::ostream& io, base::geometry::Spline<3> const& s)
    {
	io << "Length " << s.getCurveLength() 
	    << " start " << s.getStartPoint().transpose() << " end " << s.getEndPoint().transpose()
	    << " startParam " << s.getStartParam() << " endParam " << s.getEndParam();

	return io;
    }
    
} // geometry
} // base
#endif
