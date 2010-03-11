#ifndef  _NURBS_CURVE3D_HPP_INC
#define  _NURBS_CURVE3D_HPP_INC

#include <vector>
#include <eigen2/Eigen/Core>
#include <math.h>

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
    class NURBSCurve3D
    {
        static const int DIM = 3;

	public:
	    explicit NURBSCurve3D(double geometric_resolution = 0.1, int order = 3,
                    std::vector<Eigen::Vector3d> const& points = std::vector<Eigen::Vector3d>());
	    explicit NURBSCurve3D(double geometric_resolution, int order,
                    std::vector<Eigen::Vector3d> const& points,
                    SISLCurve* curve);
            NURBSCurve3D(NURBSCurve3D const& source);
	    ~NURBSCurve3D();

            /** Changes the default geometric resolution */
            void setGeometricResolution(double _geores) { geometric_resolution = _geores; }
            double getGeometricResolution() const { return geometric_resolution; };

            std::vector<Eigen::Vector3d> const& getPoints() const
            { return points; }

            /** Returns the number of points for this curve */
	    int    getPointCount() const { return points.size(); };
            int    getCurveOrder() const { return curve_order; }
            /** Returns the length of the curve in geometric space */
	    double getCurveLength();
            /** Returns the maximum curvature of the curve */
	    double getCurvatureMax();
	    double getStartParam() const { return start_param; };
	    double getEndParam()   const { return end_param; };

            /** Return a pointer to the underlying SISL structure
             *
             * This pointer will be non-NULL only after update() has been called
             * at least once.
             */
            SISLCurve const* getSISLCurve() const;

            /** Return a pointer to the underlying SISL structure
             *
             * This pointer will be non-NULL only after update() has been called
             * at least once.
             */
            SISLCurve* getSISLCurve();

            /** Returns the length-to-parametric scale
             *
             * I.e. it returns the number of parametric units that lie in one
             * curve length unit
             */
	    double getUnitParameter();
   	  
            /** Returns the geometric point that lies on the curve at the given
             * parameter */
	    Eigen::Vector3d getPoint(double _param);

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

            /** \overload
             */
            double findOneClosestPoint(Eigen::Vector3d const& _pt);

            /** Returns a single closest point to _pt
             *
             * This is a convenience method that calls findClosestPoints
             *
             * @return the parameter of the found closes point
             * @throw std::runtime_error if no points have been found (should not happen)
             * @see localClosestPointSearch findClosestPoints
             */
            double findOneClosestPoint(Eigen::Vector3d const& _pt, double _geometric_resolution);

            /** \overload
             */
            void findClosestPoints(Eigen::Vector3d const& _pt,
                    std::vector<double>& _points,
                    std::vector< std::pair<double, double> >& _curves);

            /**
             * Returns the single points and curve segments that are closest to
             * the given point.
             *
             */
            void findClosestPoints(Eigen::Vector3d const& _pt,
                    std::vector<double>& _points,
                    std::vector< std::pair<double, double> >& _curves,
                    double _geores);

            /** \overload
             */
            double localClosestPointSearch(Eigen::Vector3d const& _pt, double _guess, double _start, double _end);

            /** Performs a Newton search in the provided parametric interval, starting with the given guess.
             *
             * This method is subject to local minima problems
             */
            double localClosestPointSearch(Eigen::Vector3d const& _pt, double _guess, double _start, double _end, double _geores);

	    /** Computes the Frenet frame at a particular parameter */
	    Eigen::Matrix3d getFrenetFrame(double _param);

            /** Returns the curve heading at the given parametric position
             *
             * The heading is defined as the angle between the tangent projected
             * on the X-Y axis and the X axis itself.
             */
            double getHeading(double _param);

	    void printCurveProperties();


	    /** Calculates the heading error */
	    double headingError(double _actZRot, double _param);
	    /** Calculates the distance error */
	    double distanceError(Eigen::Vector3d _pt, double _param);
	    /** Calculates the pose error */
	    Eigen::Vector3d poseError(Eigen::Vector3d _pt, double _actZRot, double _start_param, double _length_tol);

	    /** Add pointst to generate the curve */
	    void addPoint(Eigen::Vector3d pt);

	    /** Generates the curve */
	    void update();

            /** Reinitializes the curve */
            void clear();

            std::vector<double> simplify();
            std::vector<double> simplify(double tolerance);

	private:
            //! the underlying SISL curve
	    SISLCurve *curve;
            //! the list of points from which we generated the curve
	    std::vector<Eigen::Vector3d> points;

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

} // geometry
} // base
#endif
