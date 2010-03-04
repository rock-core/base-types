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
        SplineBase();
    public:
        SplineBase(SplineBase const& source);
        virtual ~SplineBase();

        explicit SplineBase(int dimension,
                double geometric_resolution = 0.1, int order = 3);
        explicit SplineBase(double geometric_resolution, SISLCurve* curve);

        /** Changes the default geometric resolution */
        void setGeometricResolution(double _geores) { geometric_resolution = _geores; }
        double getGeometricResolution() const { return geometric_resolution; };

        /** Returns the dimension of the space in which the curve lies */
        int    getDimension() const { return dimension; }
        /** Returns the number of points for this curve */
        int    getPointCount() const;
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


        /** Generates the curve
         *
         * If the \c parameters array is given, it is the desired parameter for
         * the provided points. Otherwise, an automatic parametrization is
         * generated.
         */
        void interpolate(std::vector<double> const& coordinates, std::vector<double> const& parameters = std::vector<double>());

        /** Display the curve properties on the given IO object */
        void printCurveProperties(std::ostream& io);

        /** Reinitializes the curve */
        void clear();

        std::vector<double> simplify();
        std::vector<double> simplify(double tolerance);

        SplineBase const& operator = (SplineBase const& base);

    protected:
        void getPoint(double* result, double _param);
        double findOneClosestPoint(double const* _pt, double _geores);
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
        Eigen::Vector3d poseError(Eigen::Vector3d _pt, double _actZRot, double _st_para, double _len_tol);;


    private:
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

    template<int DIM>
    class Spline : public SplineBase
    {
    public:
        typedef Eigen::Matrix<double, DIM, 1> vector_t;

        explicit Spline(double geometric_resolution = 0.1, int order = 3)
            : SplineBase(DIM, geometric_resolution, order) {}
        explicit Spline(double geometric_resolution, SISLCurve* curve)
            : SplineBase(geometric_resolution, curve) {}
        Spline(SplineBase const& base)
            : SplineBase(base) {}

        /** Returns the geometric point that lies on the curve at the given
         * parameter */
        vector_t getPoint(double _param)
        { 
            vector_t result;
            SplineBase::getPoint(result.data(), _param);
        }

        /** \overload
         */
        double findOneClosestPoint(vector_t const& _pt)
        { return findOneClosestPoint(_pt, getGeometricResolution()); }

        /** Compute the curve from the given set of points */
        void interpolate(std::vector<vector_t> const& points, std::vector<double> const& parameters = std::vector<double>())
        {
            std::vector<double> coordinates;
            for (size_t i = 0; i < points.size(); ++i)
                coordinates.insert(coordinates.end(), points[i].data(), points[i].data() + DIM);
            SplineBase::interpolate(coordinates, parameters);
        }

        /** Returns a single closest point to _pt
         *
         * This is a convenience method that calls findClosestPoints
         *
         * @return the parameter of the found closes point
         * @throw std::runtime_error if no points have been found (should not happen)
         * @see localClosestPointSearch findClosestPoints
         */
        double findOneClosestPoint(vector_t const& _pt, double _geometric_resolution)
        { return SplineBase::findOneClosestPoint(_pt.data(), _geometric_resolution); }

        /** \overload
         */
        void findClosestPoints(vector_t const& _pt,
                std::vector<double>& _points,
                std::vector< std::pair<double, double> >& _curves)
        { return findClosestPoints(_pt, _points, _curves, getGeometricResolution()); }

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
        { return localClosestPointSearch(_pt, _guess, _start, _end, getGeometricResolution()); }

        /** Performs a Newton search in the provided parametric interval, starting with the given guess.
         *
         * This method is subject to local minima problems
         */
        double localClosestPointSearch(vector_t const& _pt, double _guess, double _start, double _end, double _geores)
        { return SplineBase::localClosestPointSearch(_pt.data(), _guess, _start, _end, _geores); }

        Eigen::Matrix3d getFrenetFrame(double _param,
                typename boost::enable_if_c<DIM == 3>::type* enabler = 0)
        { return SplineBase::getFrenetFrame(_param); }

        double getHeading(double _param,
                typename boost::enable_if_c<DIM == 3>::type* enabler = 0)
        { return SplineBase::getHeading(_param); }

        double headingError(double _actHeading, double _param,
                typename boost::enable_if_c<DIM == 3>::type* enabler = 0)
        { return SplineBase::headingError(_actHeading, _param); }

        double distanceError(Eigen::Vector3d _pt, double _param,
                typename boost::enable_if_c<DIM == 3>::type* enabler = 0)
        { return SplineBase::distanceError(_pt, _param); }

        Eigen::Vector3d poseError(Eigen::Vector3d _pt, double _actZRot, double _st_para, double _len_tol,
                typename boost::enable_if_c<DIM == 3>::type* enabler = 0)
        { return SplineBase::poseError(_pt, _actZRot, _st_para, _len_tol); }
    };

    typedef Spline<3> NURBSCurve3D;
} // geometry
} // base
#endif
