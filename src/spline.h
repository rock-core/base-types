#ifndef  _BASE_SPLINE_HPP_INC
#define  _BASE_SPLINE_HPP_INC

#include <vector>
#include <eigen2/Eigen/Core>

#include <boost/utility/enable_if.hpp>

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
        virtual ~SplineBase();

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
        /** Returns the length of the curve in geometric space */
        double getCurveLength();
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

        /** Returns the length-to-parametric scale
         *
         * I.e. it returns the number of parametric units that lie in one
         * curve length unit
         */
        double getUnitParameter();
      
        /** Returns the curvature at the given position
         *
         * @throws out_of_range if _param is not in [start_param,
         * end_param] and runtime_error if SISL returns an error
         */
        double getCurvature(double _param);

        /** Returns the first order derivative of the curvature at the given
         * position
         *
         * @throws out_of_range if _param is not in [start_param,
         * end_param] and runtime_error if SISL returns an error
         */
        double getVariationOfCurvature(double _param);  // Variation of Curvature
                
        /** Returns the maximum curvature in a curve
         *
         * Iterates through the curve and return the maximum curvature found
         *  Checks every delLen for the parameter
         */
        double findCurvatureMax(); 

        std::vector<double> getCoordinates() const;

        std::vector<double> getKnots() const;

        /** Display the curve properties on the given IO object */
        void printCurveProperties(std::ostream& io);

        /** Generates the curve
         *
         * If the \c parameters array is given, it is the desired parameter for
         * the provided points. Otherwise, an automatic parametrization is
         * generated.
         */
        void interpolate(std::vector<double> const& coordinates, std::vector<double> const& parameters = std::vector<double>());

        /** Reinitializes the curve */
        void clear();

        std::vector<double> simplify();
        std::vector<double> simplify(double tolerance);

        SplineBase const& operator = (SplineBase const& base);

        bool isNURBS() const;

        void reset(std::vector<double> const& coordinates, std::vector<double> const& knots, int kind = -1);

        int getCoordinatesStride() const
        {
            if (isNURBS()) return dimension + 1;
            else return dimension;
        }

    protected:
        void getPoint(double* result, double _param);
        double findOneClosestPoint(double const* _pt, double _guess, double _geores);
        void findClosestPoints(double const* ref_point,
                std::vector<double>& _result_points,
                std::vector< std::pair<double, double> >& _result_curves,
                double _geores);

        double localClosestPointSearch(double* ref_point,
                double _guess, double _start, double _end,
                double  _geores);

        //! available only in Spline<3>
        Eigen::Matrix3d getFrenetFrame(double _param);
        //! available only in Spline<3>
        double getHeading(double _param);
        //! available only in Spline<3>
        double headingError(double _actHeading, double _param);
        //! available only in Spline<3>
        double distanceError(Eigen::Vector3d _pt, double _param);
        //! available only in Spline<3>
        Eigen::Vector3d poseError(Eigen::Vector3d _pt, double _actZRot, double _st_para);

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

        //! if we have already calculated the curve length
        bool has_curve_length;
        //! the cache value for the curve length in geometric space
        double curve_length; // Length of the curve

        //! if we have already calculated the maximum curvature
        bool has_curvature_max;
        //! maximum curvature in the curve
        double curvature_max;
    };

    class Spline3Base : public SplineBase
    {
    public:
        explicit Spline3Base(int dimension, double geometric_resolution, int order)
            : SplineBase(dimension, geometric_resolution, order) {}
        explicit Spline3Base(double geometric_resolution, SISLCurve* curve)
            : SplineBase(geometric_resolution, curve) {}
        Spline3Base(SplineBase const& source)
            : SplineBase(source) {}

        Eigen::Matrix3d getFrenetFrame(double _param)
        { return SplineBase::getFrenetFrame(_param); }

        double getHeading(double _param)
        { return SplineBase::getHeading(_param); }

        double headingError(double _actZRot, double _param)
        { return SplineBase::headingError(_actZRot, _param); }

        double distanceError(Eigen::Vector3d _pt, double _param)
        { return SplineBase::distanceError(_pt, _param); }

        /** Searches for the closest point in the curve, and returns the pose
         * error between the frenet frame on the curve and the given pose given
         * by _position and _heading.
         *
         * The returned vector is (distance_error, heading_error,
         * curve_parameter)
         */
        Eigen::Vector3d poseError(Eigen::Vector3d _position, double _heading, double _guess)
        { return SplineBase::poseError(_position, _heading, _guess); }
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
        typedef Eigen::Matrix<double, DIM, 1> vector_t;
        typedef Eigen::Transform<double, DIM> transform_t;

        explicit Spline(double geometric_resolution = 0.1, int order = 3)
            : base_t(DIM, geometric_resolution, order) {}
        explicit Spline(double geometric_resolution, SISLCurve* curve)
            : base_t(geometric_resolution, curve) {}
        Spline(SplineBase const& base)
            : base_t(base) {}

        /** Returns the geometric point that lies on the curve at the given
         * parameter */
        vector_t getPoint(double _param)
        { 
            vector_t result;
            SplineBase::getPoint(result.data(), _param);
            return result;
        }

        /** Compute the curve from the given set of points */
        void interpolate(std::vector<vector_t> const& points, std::vector<double> const& parameters = std::vector<double>())
        {
            std::vector<double> coordinates;
            for (size_t i = 0; i < points.size(); ++i)
                coordinates.insert(coordinates.end(), points[i].data(), points[i].data() + DIM);
            SplineBase::interpolate(coordinates, parameters);
        }

        /** \overload
         */
        double findOneClosestPoint(vector_t const& _pt)
        { return findOneClosestPoint(_pt, SplineBase::getGeometricResolution()); }

        /** \overload
         *
         * Calls findOneClosestPoint using the start parameter as the guess
         * parameter. I.e. it will always return the point closest to the start
         * of the curve.
         */
        double findOneClosestPoint(vector_t const& _pt, double _geometric_resolution)
        { return findOneClosestPoint(_pt, SplineBase::getStartParam(), _geometric_resolution); }

        /** Returns a single closest point to _pt
         *
         * This is a convenience method that calls findClosestPoints and returns
         * one single parameter in the values returned. The returned parameter
         * is the closes point closest to _guess.
         *
         * @return the parameter of the found closes point
         * @throw std::logic_error if no points have been found (should not happen)
         * @see localClosestPointSearch findClosestPoints
         */
        double findOneClosestPoint(vector_t const& _pt, double _guess, double _geometric_resolution)
        { return SplineBase::findOneClosestPoint(_pt.data(), _guess, _geometric_resolution); }

        /** \overload
         */
        void findClosestPoints(vector_t const& _pt,
                std::vector<double>& _points,
                std::vector< std::pair<double, double> >& _curves)
        { return findClosestPoints(_pt, _points, _curves, SplineBase::getGeometricResolution()); }

        /**
         * Returns the single points and curve segments that are closest to
         * the given point.
         *
         */
        void findClosestPoints(vector_t const& _pt,
                std::vector<double>& _points,
                std::vector< std::pair<double, double> >& _curves,
                double _geores)
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

    typedef Spline<3> NURBSCurve3D;
} // geometry
} // base
#endif
