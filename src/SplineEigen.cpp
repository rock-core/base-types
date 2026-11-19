// Parts of base::geometry::Spline that are specific to the Eigen-based
// implementation, i.e. when base/types is built without SISL. The shared
// parts are in Spline.cpp
#include "Spline.hpp"

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <stdexcept>
#include <vector>
#include <sstream>
#include <boost/lexical_cast.hpp>

#include <iostream>

using namespace std;
using boost::lexical_cast;
using namespace Eigen;



namespace {

/** Plain representation of a (non-rational) B-spline curve, used by the
 * low-level algorithms below
 *
 * For n vertices and order k, the knot vector t has n + k entries and the
 * parameter domain is [t[k-1], t[n]]. Vertices are stored one after the
 * other.
 */
struct Curve
{
    int dim;
    int k;
    vector<double> t;
    vector<double> P;

    int n() const { return P.size() / dim; }
    double start() const { return t[k - 1]; }
    double end() const { return t[n()]; }
};

/** Returns the index s of the knot span [t[s], t[s+1]] that should be used
 * to evaluate the curve at u
 *
 * From the right, it is the largest s in [k-1, n-1] with t[s] <= u. From the
 * left, it is the smallest s in [k-1, n-1] with u <= t[s+1].
 */
int findSpan(vector<double> const& t, int k, int n, double u, bool from_left)
{
    vector<double>::const_iterator begin = t.begin() + k;
    vector<double>::const_iterator end = t.begin() + n;
    if (from_left)
        return (lower_bound(begin, end, u) - t.begin()) - 1;
    else
        return (upper_bound(begin, end, u) - t.begin()) - 1;
}

/** Computes the values and derivatives of the k basis functions that are
 * non-zero on the knot span s
 *
 * This is algorithm A2.3 of "The NURBS Book" (Piegl & Tiller). Row d of the
 * result contains the d-th derivatives of N_{s-k+1}, ..., N_{s}.
 *
 * The matrix types are template parameters, so that the common case (low
 * orders) can be computed without heap allocations
 */
template<typename Square, typename Vector, typename Result>
void basisDerivatives(vector<double> const& t, int k, int s, double u, int nder, Result& ders)
{
    int const p = k - 1;
    Square ndu(k, k);
    Vector left(k), right(k);
    ndu(0, 0) = 1.0;
    for (int j = 1; j <= p; ++j)
    {
        left(j) = u - t[s + 1 - j];
        right(j) = t[s + j] - u;
        double saved = 0.0;
        for (int r = 0; r < j; ++r)
        {
            // Lower triangle: knot differences
            ndu(j, r) = right(r + 1) + left(j - r);
            double temp = ndu(r, j - 1) / ndu(j, r);
            // Upper triangle: basis functions
            ndu(r, j) = saved + right(r + 1) * temp;
            saved = left(j - r) * temp;
        }
        ndu(j, j) = saved;
    }

    ders.setZero(nder + 1, k);
    for (int j = 0; j <= p; ++j)
        ders(0, j) = ndu(j, p);

    int const max_der = min(nder, p);
    Square a(2, k);
    for (int r = 0; r <= p; ++r)
    {
        int s1 = 0, s2 = 1;
        a(0, 0) = 1.0;
        for (int kk = 1; kk <= max_der; ++kk)
        {
            double d = 0.0;
            int const rk = r - kk;
            int const pk = p - kk;
            if (r >= kk)
            {
                a(s2, 0) = a(s1, 0) / ndu(pk + 1, rk);
                d = a(s2, 0) * ndu(rk, pk);
            }
            int const j1 = (rk >= -1) ? 1 : -rk;
            int const j2 = (r - 1 <= pk) ? kk - 1 : p - r;
            for (int j = j1; j <= j2; ++j)
            {
                a(s2, j) = (a(s1, j) - a(s1, j - 1)) / ndu(pk + 1, rk + j);
                d += a(s2, j) * ndu(rk + j, pk);
            }
            if (r <= pk)
            {
                a(s2, kk) = -a(s1, kk - 1) / ndu(pk + 1, r);
                d += a(s2, kk) * ndu(r, pk);
            }
            ders(kk, r) = d;
            swap(s1, s2);
        }
    }

    double factor = p;
    for (int kk = 1; kk <= max_der; ++kk)
    {
        ders.row(kk) *= factor;
        factor *= (p - kk);
    }
}

//! Maximum order for which basis functions are computed without heap allocation
int const SMALL_ORDER = 8;
typedef Matrix<double, Dynamic, Dynamic, 0, SMALL_ORDER, SMALL_ORDER> SmallMatrix;
typedef Matrix<double, Dynamic, 1, 0, SMALL_ORDER, 1> SmallVector;

MatrixXd basisDerivatives(vector<double> const& t, int k, int s, double u, int nder)
{
    MatrixXd ders;
    basisDerivatives<MatrixXd, VectorXd>(t, k, s, u, nder, ders);
    return ders;
}

/** Computes the curve derivatives from the basis function derivatives */
template<typename Derivatives>
void combineVertices(Derivatives const& ders, vector<double> const& P, int dim, int k, int s,
        int nder, double* result)
{
    fill(result, result + (nder + 1) * dim, 0.0);
    for (int d = 0; d <= nder; ++d)
    {
        for (int j = 0; j < k; ++j)
        {
            double const b = ders(d, j);
            double const* vertex = &P[(s - k + 1 + j) * dim];
            for (int c = 0; c < dim; ++c)
                result[d * dim + c] += b * vertex[c];
        }
    }
}

/** Evaluates the curve and its first nder derivatives at u
 *
 * @param result array of (nder + 1) * dim values
 */
void evaluateCurve(vector<double> const& t, vector<double> const& P, int dim, int k,
        double u, int nder, bool from_left, double* result)
{
    int const n = P.size() / dim;
    int const s = findSpan(t, k, n, u, from_left);
    if (k <= SMALL_ORDER && nder < SMALL_ORDER)
    {
        SmallMatrix ders;
        basisDerivatives<SmallMatrix, SmallVector>(t, k, s, u, nder, ders);
        combineVertices(ders, P, dim, k, s, nder, result);
    }
    else
        combineVertices(basisDerivatives(t, k, s, u, nder), P, dim, k, s, nder, result);

    // The end points of a clamped curve are its first and last vertices.
    // Return them exactly, to avoid rounding errors.
    if (u == t[k - 1] && t[0] == t[k - 1])
        copy(P.begin(), P.begin() + dim, result);
    else if (u == t[n] && t[n] == t.back())
        copy(P.end() - dim, P.end(), result);
}

void evaluateCurve(Curve const& c, double u, int nder, bool from_left, double* result)
{
    evaluateCurve(c.t, c.P, c.dim, c.k, u, nder, from_left, result);
}

int multiplicity(vector<double> const& t, double u)
{
    return upper_bound(t.begin(), t.end(), u) - lower_bound(t.begin(), t.end(), u);
}

/** Inserts the knot u once (Boehm's algorithm) */
void insertKnot(Curve& c, double u)
{
    int const n = c.n();
    int const p = c.k - 1;
    int const dim = c.dim;
    int const s = findSpan(c.t, c.k, n, u, false);
    vector<double> Q((n + 1) * dim);
    for (int i = 0; i <= n; ++i)
    {
        double* q = &Q[i * dim];
        if (i <= s - p)
            copy(&c.P[i * dim], &c.P[i * dim] + dim, q);
        else if (i >= s + 1)
            copy(&c.P[(i - 1) * dim], &c.P[(i - 1) * dim] + dim, q);
        else
        {
            double const alpha = (u - c.t[i]) / (c.t[i + p] - c.t[i]);
            for (int d = 0; d < dim; ++d)
                q[d] = alpha * c.P[i * dim + d] + (1 - alpha) * c.P[(i - 1) * dim + d];
        }
    }
    c.t.insert(c.t.begin() + s + 1, u);
    c.P.swap(Q);
}

/** Splits the curve at u, which must be within the knot range
 *
 * The knot u is inserted until it has a multiplicity equal to the order.
 * The left part (which might be empty, i.e. have no vertices, if u is a
 * clamped start of the curve) is returned in \c left, the right part in \c
 * right.
 */
void splitCurve(Curve const& c, double u, Curve& left, Curve& right)
{
    Curve full = c;
    for (int m = multiplicity(full.t, u); m < full.k; ++m)
        insertKnot(full, u);

    int const a = lower_bound(full.t.begin(), full.t.end(), u) - full.t.begin();
    left.dim = right.dim = c.dim;
    left.k = right.k = c.k;
    left.t.assign(full.t.begin(), full.t.begin() + a + c.k);
    left.P.assign(full.P.begin(), full.P.begin() + a * c.dim);
    right.t.assign(full.t.begin() + a, full.t.end());
    right.P.assign(full.P.begin() + a * c.dim, full.P.end());
}

bool isClampedAt(Curve const& c, double u)
{
    return multiplicity(c.t, u) >= c.k;
}

/** Restricts the curve to [a, b], with a < b, and makes sure that the result
 * has a clamped knot vector
 */
void cropCurve(Curve& c, double a, double b)
{
    Curve left, right;
    if (!(a == c.start() && isClampedAt(c, a)))
    {
        splitCurve(c, a, left, right);
        c = right;
    }
    if (!(b == c.end() && isClampedAt(c, b)))
    {
        splitCurve(c, b, left, right);
        c = left;
    }
}

void clampCurve(Curve& c)
{
    cropCurve(c, c.start(), c.end());
}

/** Turns the direction of the curve, keeping its parameter domain */
void reverseCurve(Curve& c)
{
    double const sum = c.start() + c.end();
    vector<double> t(c.t.size());
    for (size_t i = 0; i < t.size(); ++i)
        t[i] = sum - c.t[c.t.size() - 1 - i];

    int const n = c.n();
    vector<double> P(c.P.size());
    for (int i = 0; i < n; ++i)
        copy(&c.P[(n - 1 - i) * c.dim], &c.P[(n - i) * c.dim], &P[i * c.dim]);

    c.t.swap(t);
    c.P.swap(P);
}

template<typename Matrix>
MatrixXd solveSparse(Matrix const& A, MatrixXd const& B)
{
    SparseLU< SparseMatrix<double>, COLAMDOrdering<int> > solver;
    solver.analyzePattern(A);
    solver.factorize(A);
    if (solver.info() != Success)
        throw std::runtime_error("singular system");
    MatrixXd X = solver.solve(B);
    if (solver.info() != Success || !X.allFinite())
        throw std::runtime_error("singular system");
    return X;
}

/** Fills one row of a collocation matrix with the d-th derivatives of the
 * basis functions at u
 */
void addCollocationRow(vector< Triplet<double> >& triplets, int row,
        vector<double> const& t, int k, int n, double u, int d, bool from_left)
{
    int const s = findSpan(t, k, n, u, from_left);
    MatrixXd const ders = basisDerivatives(t, k, s, u, d);
    for (int j = 0; j < k; ++j)
    {
        if (ders(d, j) != 0)
            triplets.push_back(Triplet<double>(row, s - k + 1 + j, ders(d, j)));
    }
}

/** Raises the order of a clamped curve without changing its shape
 *
 * The curve must be clamped, i.e. have knots of multiplicity k at both ends
 */
void elevateOrder(Curve& c, int new_k)
{
    int const delta = new_k - c.k;
    if (delta <= 0)
        return;

    // Each distinct knot gets its multiplicity increased by delta, which
    // preserves the continuity at the knots
    vector<double> t;
    for (size_t i = 0; i < c.t.size(); )
    {
        double const value = c.t[i];
        size_t j = i;
        while (j < c.t.size() && c.t[j] == value)
            ++j;
        t.insert(t.end(), j - i + delta, value);
        i = j;
    }

    // The original curve is part of the new spline space: interpolating it
    // at the Greville abscissae of the new basis reproduces it exactly
    int const n = t.size() - new_k;
    double const end = c.end();
    vector< Triplet<double> > triplets;
    MatrixXd B(n, c.dim);
    VectorXd point(c.dim);
    for (int i = 0; i < n; ++i)
    {
        double u = 0;
        for (int j = 1; j < new_k; ++j)
            u += t[i + j];
        u /= (new_k - 1);

        bool const from_left = (u >= end);
        addCollocationRow(triplets, i, t, new_k, n, u, 0, from_left);
        evaluateCurve(c, u, 0, from_left, point.data());
        B.row(i) = point.transpose();
    }
    SparseMatrix<double> A(n, n);
    A.setFromTriplets(triplets.begin(), triplets.end());
    MatrixXd X = solveSparse(A, B);

    // The curve is clamped, i.e. its end points are its first and last
    // vertices. Keep them exact.
    for (int d = 0; d < c.dim; ++d)
    {
        X(0, d) = c.P[d];
        X(n - 1, d) = c.P[c.P.size() - c.dim + d];
    }

    c.k = new_k;
    c.t.swap(t);
    c.P.resize(n * c.dim);
    for (int i = 0; i < n; ++i)
        for (int d = 0; d < c.dim; ++d)
            c.P[i * c.dim + d] = X(i, d);
}

/** Computes the knot vector for an open interpolating curve
 *
 * This is a port of SISL's s1902: the knots are placed at the parameters
 * (even orders) or between them (odd orders).
 *
 * @param epar sorted parameters of the interpolation conditions. Parameters
 *   are repeated for derivative conditions.
 */
vector<double> interpolationKnots(vector<double> const& epar, int k)
{
    int const in = epar.size();
    int const kk2 = k / 2;
    double const tval1 = epar[0];
    double const tval2 = epar[in - 1];

    int smult = 1;
    while (smult < in && epar[smult - 1] == epar[smult])
        ++smult;
    int emult = 1;
    for (int i = in - 2; i >= 0 && epar[i] == epar[i + 1]; --i)
        ++emult;
    if (smult >= in)
        throw std::runtime_error("all interpolation conditions have the same parameter");

    vector<double> interior;
    int const kstart = smult + max(0, kk2 - smult);
    int const kstop = in - emult - max(0, kk2 - emult);
    if (k % 2 == 0)
    {
        double dummy1 = 0.5 * (epar[smult] + tval1);
        double dummy2 = 0.5 * (epar[in - emult - 1] + tval2);
        if (dummy1 == dummy2)
        {
            dummy1 = (epar[smult] + tval1 + tval1) / 3;
            dummy2 = (epar[in - emult - 1] + tval2 + tval2) / 3;
        }
        for (int i = 0; i < smult - kk2; ++i)
            interior.push_back(dummy1);
        for (int i = kstart; i < kstop; ++i)
            interior.push_back(epar[i]);
        for (int i = 0; i < emult - kk2; ++i)
            interior.push_back(dummy2);
    }
    else
    {
        if (smult - kk2 > 0)
        {
            double const delta = (epar[smult] - tval1) / (smult - kk2 + 1);
            for (int i = 0; i < smult - kk2; ++i)
                interior.push_back(tval1 + (i + 1) * delta);
        }
        for (int i = kstart; i < kstop; ++i)
            interior.push_back(0.5 * (epar[i] + epar[i + 1]));
        if (emult - kk2 > 0)
        {
            double const delta = (tval2 - epar[in - emult - 1]) / (emult - kk2 + 1);
            for (int i = 0; i < emult - kk2; ++i)
                interior.push_back(epar[in - emult - 1] + (i + 1) * delta);
        }
    }

    // For odd orders, the loops above produce one knot too many. SISL
    // overwrites it with the end knots, i.e. drops it.
    if (static_cast<int>(interior.size()) < in - k)
        throw std::logic_error("interpolationKnots: not enough interior knots");

    vector<double> knots(k, tval1);
    knots.insert(knots.end(), interior.begin(), interior.begin() + (in - k));
    knots.insert(knots.end(), k, tval2);
    return knots;
}

double squaredDistance(double const* a, double const* b, int dim)
{
    double result = 0;
    for (int i = 0; i < dim; ++i)
        result += (a[i] - b[i]) * (a[i] - b[i]);
    return result;
}

/** Relative resolutions used by SISL in comparisons */
double const REL_COMP_RES = 1e-15;
double const REL_PAR_RES = 1e-12;

/** SISL's test for zero length (DEQUAL(length, 0)) */
bool isZeroLength(double length)
{
    return fabs(length) <= REL_PAR_RES * std::max(fabs(length), 1.0);
}

/** Computes the parameter step of the Newton iteration for the closest point
 * problem. Port of s1771_s9del from SISL
 *
 * @param eco the difference vector between the point and the curve
 * @param eco1 the first derivative of the curve
 * @param eco2 the second derivative of the curve
 */
double newtonClosestPointStep(double const* eco, double const* eco1, double const* eco2, int dim)
{
    Map<VectorXd const> d(eco, dim), d1(eco1, dim), d2(eco2, dim);
    double const ttol = 1e-10;
    double t1 = d.dot(d1);
    double const t3 = d1.dot(d1);
    double const t2 = t3 - d.dot(d2);
    double const t4 = -2 * d1.dot(d2);
    double const tmax = std::max(std::max(fabs(t1), fabs(t2)), std::max(fabs(t3), fabs(t4)));
    if (tmax <= REL_COMP_RES)
        return 0;
    else if (fabs(t4) / tmax < ttol)
    {
        // The second degree part is degenerated
        if (fabs(t2) / tmax < ttol)
        {
            if (fabs(t3) / tmax < ttol)
                return 0;
            else
                return t1 / t3;
        }
        else
            return t1 / t2;
    }

    double t5 = t2 * t2 - 2 * t4 * t1;
    if (t5 < 0)
        return t1 / t3;

    double t6 = sqrt(t5);
    t5 = (t2 + t6) / t4;
    t6 = (t2 - t6) / t4;
    t1 *= t3;
    // We have two solutions and we want to use the one with the same sign as
    // t1/t3. If both solutions have the same sign we use the one with the
    // smallest value.
    if (t1 < 0)
    {
        if (t5 <= 0 && t6 <= 0)
            return std::max(t5, t6);
        else if (t5 <= 0)
            return t5;
        else if (t6 <= 0)
            return t6;
        else
            return std::min(t5, t6);
    }
    else if (t1 > 0)
    {
        if (t5 >= 0 && t6 >= 0)
            return std::min(t5, t6);
        else if (t5 >= 0)
            return t5;
        else if (t6 >= 0)
            return t6;
        else
            return std::max(t5, t6);
    }
    else
        return std::min(fabs(t5), fabs(t6));
}

/** Golden section search for a minimum of f on [a, b] */
template<typename F>
double goldenSectionMinimum(F const& f, double a, double b)
{
    double const ratio = (sqrt(5.0) - 1) / 2;
    double x1 = b - ratio * (b - a), x2 = a + ratio * (b - a);
    double f1 = f(x1), f2 = f(x2);
    for (int i = 0; i < 200 && (b - a) > 1e-14 * (1 + fabs(a) + fabs(b)); ++i)
    {
        if (f1 < f2)
        {
            b = x2; x2 = x1; f2 = f1;
            x1 = b - ratio * (b - a);
            f1 = f(x1);
        }
        else
        {
            a = x1; x1 = x2; f1 = f2;
            x2 = a + ratio * (b - a);
            f2 = f(x2);
        }
    }
    return (a + b) / 2;
}

/** Finds a root of f in [a, b], given that fa and fb have opposite signs */
template<typename F>
double bracketedRoot(F const& f, double a, double fa, double b, double fb)
{
    for (int i = 0; i < 200; ++i)
    {
        double const m = (a + b) / 2;
        if ((b - a) <= 1e-14 * (1 + fabs(a) + fabs(b)))
            return m;
        double const fm = f(m);
        if (fm == 0)
            return m;
        if ((fm < 0) == (fa < 0))
        {
            a = m; fa = fm;
        }
        else
        {
            b = m; fb = fm;
        }
    }
    return (a + b) / 2;
}

/** Finds the roots of f at the given sampling parameters
 *
 * Roots are detected by sign changes between samples. In addition, local
 * minima of |f| that are below touch_tolerance are reported as roots, to
 * detect tangential intersections.
 */
template<typename F>
vector<double> findRoots(F const& f, vector<double> const& us, double touch_tolerance)
{
    vector<double> values(us.size());
    for (size_t i = 0; i < us.size(); ++i)
        values[i] = f(us[i]);

    vector<double> roots;
    for (size_t i = 0; i < us.size(); ++i)
    {
        if (values[i] == 0)
        {
            roots.push_back(us[i]);
            continue;
        }

        if (i + 1 < us.size() && values[i + 1] != 0 && (values[i] < 0) != (values[i + 1] < 0))
            roots.push_back(bracketedRoot(f, us[i], values[i], us[i + 1], values[i + 1]));

        // Local minimum of |f| among the samples. Look for the minimum of
        // |f| in between the neighboring samples: it is either a dip that
        // crosses zero (two roots), or a tangential contact
        double const v = fabs(values[i]);
        bool const left_ok = (i == 0) || (v <= fabs(values[i - 1]) && (values[i - 1] < 0) == (values[i] < 0));
        bool const right_ok = (i + 1 == us.size()) || (v <= fabs(values[i + 1]) && (values[i + 1] < 0) == (values[i] < 0));
        if (left_ok && right_ok)
        {
            size_t const ia = (i == 0) ? 0 : i - 1;
            size_t const ib = (i + 1 == us.size()) ? i : i + 1;
            double const sign = (values[i] < 0) ? -1 : 1;
            double const u = goldenSectionMinimum([&f, sign](double x) { return sign * f(x); }, us[ia], us[ib]);
            double const fu = f(u);
            if (sign * fu < 0)
            {
                roots.push_back(bracketedRoot(f, us[ia], values[ia], u, fu));
                roots.push_back(bracketedRoot(f, u, fu, us[ib], values[ib]));
            }
            else if (fabs(fu) <= touch_tolerance)
                roots.push_back(u);
        }
    }
    return roots;
}

/** Sorts the parameters and removes the ones that are closer than tolerance */
void uniqueParameters(vector<double>& params, double tolerance)
{
    sort(params.begin(), params.end());
    vector<double> result;
    for (size_t i = 0; i < params.size(); ++i)
    {
        if (result.empty() || params[i] - result.back() > tolerance)
            result.push_back(params[i]);
    }
    params.swap(result);
}

/** Distance between the segments [p0, p1] and [q0, q1] */
double segmentDistance(VectorXd const& p0, VectorXd const& p1, VectorXd const& q0, VectorXd const& q1)
{
    VectorXd const d1 = p1 - p0, d2 = q1 - q0, r = p0 - q0;
    double const a = d1.squaredNorm(), e = d2.squaredNorm(), f = d2.dot(r);
    double s = 0, t = 0;
    if (a <= 1e-300 && e <= 1e-300)
        return r.norm();
    if (a <= 1e-300)
        t = std::min(1.0, std::max(0.0, f / e));
    else
    {
        double const c = d1.dot(r);
        if (e <= 1e-300)
            s = std::min(1.0, std::max(0.0, -c / a));
        else
        {
            double const b = d1.dot(d2);
            double const denom = a * e - b * b;
            if (denom != 0)
                s = std::min(1.0, std::max(0.0, (b * f - c * e) / denom));
            t = (b * s + f) / e;
            if (t < 0)
            {
                t = 0;
                s = std::min(1.0, std::max(0.0, -c / a));
            }
            else if (t > 1)
            {
                t = 1;
                s = std::min(1.0, std::max(0.0, (b - c) / a));
            }
        }
    }
    return ((p0 + d1 * s) - (q0 + d2 * t)).norm();
}

/** Embeds the first and second (and optionally third) derivatives of a
 * curve in 3D, using SISL's convention
 *
 * One-dimensional curves are handled as the graph (u, x(u)).
 */
void canonicalDerivatives(double const* derivs, int dim, int nder, Vector3d* result)
{
    if (dim > 3)
        throw std::runtime_error("this operation is only available for curves of dimension 1 to 3");

    for (int d = 1; d <= nder; ++d)
    {
        Vector3d& v = result[d - 1];
        v.setZero();
        if (dim == 1)
        {
            v.x() = (d == 1) ? 1.0 : 0.0;
            v.y() = derivs[d];
        }
        else
        {
            for (int c = 0; c < dim; ++c)
                v[c] = derivs[d * dim + c];
        }
    }
}

}

namespace base { namespace geometry {

void SplineBase::setCurve(int order, vector<double> const& new_knots,
        vector<double> const& new_vertices)
{
    if (order < 1)
        throw std::invalid_argument("the curve order must be strictly positive");
    if (new_vertices.size() % dimension != 0)
        throw std::invalid_argument("the number of coordinates is not a multiple of the curve dimension");

    size_t const n = new_vertices.size() / dimension;
    if (n < static_cast<size_t>(order))
        throw std::invalid_argument("a curve of order " + lexical_cast<string>(order) +
                " needs at least " + lexical_cast<string>(order) + " vertices, got " +
                lexical_cast<string>(n));
    if (new_knots.size() != n + order)
        throw std::invalid_argument("expected " + lexical_cast<string>(n + order) +
                " knots, got " + lexical_cast<string>(new_knots.size()));
    for (size_t i = 1; i < new_knots.size(); ++i)
    {
        if (!(new_knots[i - 1] <= new_knots[i]))
            throw std::invalid_argument("the knot vector is not sorted");
    }
    if (!(new_knots[order - 1] < new_knots[n]))
        throw std::invalid_argument("the parameter range of the curve is empty");

    singleton.clear();
    knots = new_knots;
    vertices = new_vertices;
    curve_order = order;
    start_param = knots[order - 1];
    end_param = knots[n];
    has_curvature_max = false;
}

void SplineBase::evaluate(double* result, double param, int derivatives, bool from_left) const
{
    int const n = vertices.size() / dimension;
    int const k = knots.size() - n;
    evaluateCurve(knots, vertices, dimension, k, param, derivatives, from_left, result);
}

double SplineBase::integrateLength(double start, double end, double relative_resolution) const
{
    if (end < start)
        swap(start, end);

    int const n = vertices.size() / dimension;
    int const k = knots.size() - n;
    if (dimension == 1)
    {
        // The length of a one-dimensional curve is its total variation.
        // Compute it exactly by splitting the curve where its derivative
        // changes sign
        vector<double> derivs(2);
        auto derivative = [&](double u) {
            evaluate(derivs.data(), u, 1, false);
            return derivs[1];
        };
        vector<double> breaks = findRoots(derivative, getSamplingParameters(std::max(8, 4 * k)), 0);
        breaks.erase(std::remove_if(breaks.begin(), breaks.end(),
                    [start, end](double u) { return u <= start || u >= end; }), breaks.end());
        breaks.push_back(start);
        breaks.push_back(end);
        sort(breaks.begin(), breaks.end());

        double total = 0, previous;
        evaluate(&previous, breaks[0], 0, false);
        for (size_t i = 1; i < breaks.size(); ++i)
        {
            double value;
            evaluate(&value, breaks[i], 0, true);
            total += fabs(value - previous);
            previous = value;
        }
        return total;
    }

    // 5-point Gauss-Legendre quadrature, applied adaptively on each knot span
    static const double nodes[5] = {
        -0.9061798459386640, -0.5384693101056831, 0.0,
        0.5384693101056831, 0.9061798459386640 };
    static const double weights[5] = {
        0.2369268850561891, 0.4786286704993665, 0.5688888888888889,
        0.4786286704993665, 0.2369268850561891 };

    vector<double> derivs(2 * dimension);
    auto gauss = [&](double a, double b) {
        double const half = (b - a) / 2, mid = (a + b) / 2;
        double sum = 0;
        for (int i = 0; i < 5; ++i)
        {
            evaluate(derivs.data(), mid + half * nodes[i], 1, false);
            sum += weights[i] * Map<VectorXd>(&derivs[dimension], dimension).norm();
        }
        return sum * half;
    };

    struct Interval { double a, b, value; int depth; };
    double total = 0;
    for (int s = k - 1; s < n; ++s)
    {
        double const a = std::max(start, knots[s]);
        double const b = std::min(end, knots[s + 1]);
        if (!(a < b))
            continue;

        vector<Interval> stack;
        stack.push_back(Interval{a, b, gauss(a, b), 0});
        while (!stack.empty())
        {
            Interval i = stack.back();
            stack.pop_back();
            double const m = (i.a + i.b) / 2;
            double const left = gauss(i.a, m), right = gauss(m, i.b);
            double const refined = left + right;
            if (i.depth >= 20 || fabs(refined - i.value) <= relative_resolution * 0.1 * fabs(refined) + 1e-15)
                total += refined;
            else
            {
                stack.push_back(Interval{i.a, m, left, i.depth + 1});
                stack.push_back(Interval{m, i.b, right, i.depth + 1});
            }
        }
    }
    return total;
}

bool SplineBase::isNURBS() const
{
    return false;
}

std::vector<double> SplineBase::getKnots() const
{
    return knots;
}

int SplineBase::getSISLCurveType() const
{
    return hasCurve() ? 1 : 0;
}

std::vector<double> SplineBase::getSamplingParameters(int intervals_per_span) const
{
    vector<double> result;
    int const n = vertices.size() / dimension;
    int const k = knots.size() - n;
    for (int s = k - 1; s < n; ++s)
    {
        double const a = knots[s], b = knots[s + 1];
        if (!(a < b))
            continue;
        if (result.empty())
            result.push_back(a);
        for (int i = 1; i <= intervals_per_span; ++i)
            result.push_back(a + (b - a) * i / intervals_per_span);
    }
    return result;
}

std::vector<double> SplineBase::findDistanceMinima(double const* point) const
{
    int const n = vertices.size() / dimension;
    int const k = knots.size() - n;
    vector<double> const us = getSamplingParameters(std::max(8, 4 * k));

    vector<double> distances(us.size());
    vector<double> p(dimension);
    for (size_t i = 0; i < us.size(); ++i)
    {
        evaluate(p.data(), us[i], 0, false);
        distances[i] = squaredDistance(p.data(), point, dimension);
    }

    // Refine the local minima of the samples. Since the knots are part of
    // the samples, each sampling interval is within a single knot span, where
    // the curve is smooth. The minimum might be at a knot where the curve
    // is not differentiable, so refine on both sides separately.
    vector<double> result;
    for (size_t i = 0; i < us.size(); ++i)
    {
        bool const left_ok = (i == 0) || distances[i] <= distances[i - 1];
        bool const right_ok = (i + 1 == us.size()) || distances[i] <= distances[i + 1];
        if (!left_ok || !right_ok)
            continue;

        double best = us[i], best_distance = distances[i];
        for (int side = 0; side < 2; ++side)
        {
            if ((side == 0 && i == 0) || (side == 1 && i + 1 == us.size()))
                continue;
            double const a = (side == 0) ? us[i - 1] : us[i];
            double const b = (side == 0) ? us[i] : us[i + 1];
            double const u = localClosestPointSearch(point, us[i], a, b, 0);
            evaluate(p.data(), u, 0, side == 0);
            double const d = squaredDistance(p.data(), point, dimension);
            if (d < best_distance)
            {
                best = u;
                best_distance = d;
            }
        }
        result.push_back(best);
    }
    uniqueParameters(result, 1e-9 * (end_param - start_param));
    return result;
}

double SplineBase::curvatureImpl(double _param) const
{
    // Same computation as SISL's s2550: length of the curvature vector, with
    // the derivatives evaluated from the right
    vector<double> derivs(3 * dimension);
    evaluate(derivs.data(), _param, 2, false);
    Map<VectorXd> d1(&derivs[dimension], dimension);
    Map<VectorXd> d2(&derivs[2 * dimension], dimension);

    double const length = d1.norm();
    if (isZeroLength(length))
        return 0;
    VectorXd const tangent = d1 / length;
    VectorXd const curvature = (d2 / length - tangent * (d2.dot(tangent) / length)) / length;
    return curvature.norm();
}

double SplineBase::variationOfCurvatureImpl(double _param) const
{
    // Same computation as SISL's s2556, generalized to any dimension using
    // (a x b).(c x d) = (a.c)(b.d) - (a.d)(b.c)
    vector<double> derivs(4 * dimension);
    evaluate(derivs.data(), _param, 3, false);

    VectorXd d1, d2, d3;
    if (dimension == 1)
    {
        d1 = Vector2d(1, derivs[1]);
        d2 = Vector2d(0, derivs[2]);
        d3 = Vector2d(0, derivs[3]);
    }
    else
    {
        d1 = Map<VectorXd>(&derivs[dimension], dimension);
        d2 = Map<VectorXd>(&derivs[2 * dimension], dimension);
        d3 = Map<VectorXd>(&derivs[3 * dimension], dimension);
    }

    double const a2 = d1.squaredNorm();
    double const a1dota2 = d1.dot(d2);
    double const norm_crpr12 = sqrt(std::max(0.0, a2 * d2.squaredNorm() - a1dota2 * a1dota2));
    double const crpr13_dot_crpr12 = a2 * d3.dot(d2) - d1.dot(d2) * d3.dot(d1);
    if (a2 == 0 || norm_crpr12 == 0)
        return 0;

    return (crpr13_dot_crpr12 / norm_crpr12 - 3.0 * a1dota2 * norm_crpr12 / a2) / (a2 * a2);
}

void SplineBase::interpolateImpl(vector<double> const& points,
        vector<double>& parametersOut, vector<double> const& parametersIn,
        vector<int> const& point_types)
{
    unsigned int const point_count = point_types.size();

    auto error = [&](std::string const& reason) {
        std::ostringstream str;
        for (unsigned int i = 0; i < point_count; ++i)
        {
            str << " (" << points[i * dimension];
            for (int c = 1; c < dimension; ++c)
                str << " " << points[i * dimension + c];
            str << ")";
        }

        if (!parametersIn.empty())
        {
            str << " with parameters ";
            for (unsigned int c = 0; c < parametersIn.size(); ++c)
                str << " " << parametersIn[c];
        }

        return std::runtime_error(
            "base::geometry::SplineBase::interpolate(): "
            "cannot create a spline interpolating the required points (" + reason + ")" + str.str());
    };

    int const order = curve_order;
    if (order < 1)
        throw std::invalid_argument("base::geometry::SplineBase::interpolate(): the curve order must be strictly positive");

    // Transform the conditions into points (type 0) and derivatives of order
    // d to the prior (type d) or next (type -d) point. Same as SISL's s1906
    vector<double> values(points);
    vector<int> types(point_count);
    for (unsigned int i = 0; i < point_count; ++i)
    {
        double* value = &values[i * dimension];
        switch (point_types[i])
        {
            case ORDINARY_POINT:
            case KNUCKLE_POINT:
                types[i] = 0; break;
            case DERIVATIVE_TO_NEXT:
                types[i] = -1; break;
            case DERIVATIVE_TO_PRIOR:
                types[i] = 1; break;
            case SECOND_DERIVATIVE_TO_NEXT:
                types[i] = -2; break;
            case SECOND_DERIVATIVE_TO_PRIOR:
                types[i] = 2; break;
            case TANGENT_POINT_FOR_NEXT:
                if (i + 1 == point_count)
                    throw std::invalid_argument("base::geometry::SplineBase::interpolate(): TANGENT_POINT_FOR_NEXT cannot be the last entry");
                for (int c = 0; c < dimension; ++c)
                    value[c] = points[(i + 1) * dimension + c] - points[i * dimension + c];
                types[i] = -1;
                break;
            case TANGENT_POINT_FOR_PRIOR:
                if (i == 0)
                    throw std::invalid_argument("base::geometry::SplineBase::interpolate(): TANGENT_POINT_FOR_PRIOR cannot be the first entry");
                for (int c = 0; c < dimension; ++c)
                    value[c] = points[i * dimension + c] - points[(i - 1) * dimension + c];
                types[i] = 1;
                break;
            default:
                throw std::invalid_argument("base::geometry::SplineBase::interpolate(): invalid coordinate type " + lexical_cast<string>(point_types[i]));
        }
    }

    // Select the conditions that are used, and associate them with the
    // points. Same as SISL's s1905: derivatives that are higher than the
    // curve degree are ignored, as well as derivatives of an order that is
    // already given in the same block. A block is made of the conditions
    // between two points, the second point and the derivatives to the prior
    // point that follow it.
    struct Condition
    {
        int index;
        int derivative;
        bool from_left;
        //! index of the point whose parameter is used for this condition
        int point;
    };
    vector<Condition> conditions;
    vector<int> point_indexes;

    unsigned int first = 0;
    while (first < point_count && types[first] != 0)
        ++first;
    if (first == point_count)
        throw std::invalid_argument("base::geometry::SplineBase::interpolate(): no point given, only derivatives");

    vector<bool> used(order, false);
    used[0] = true;
    point_indexes.push_back(first);
    conditions.push_back(Condition{static_cast<int>(first), 0, false, 0});
    for (int i = static_cast<int>(first) - 1; i >= 0 && types[i] < 0; --i)
    {
        int const d = -types[i];
        if (d >= order || used[d])
            continue;
        used[d] = true;
        conditions.push_back(Condition{i, d, true, 0});
    }
    unsigned int i = first + 1;
    for (; i < point_count && types[i] > 0; ++i)
    {
        int const d = types[i];
        if (d >= order || used[d])
            continue;
        used[d] = true;
        conditions.push_back(Condition{static_cast<int>(i), d, true, 0});
    }

    while (i < point_count)
    {
        fill(used.begin(), used.end(), false);
        int const block = point_indexes.size();
        bool before_point = true;
        vector<Condition> block_conditions;
        for (; i < point_count && (before_point || types[i] > 0); ++i)
        {
            int const d = abs(types[i]);
            if (d == 0)
            {
                before_point = false;
                point_indexes.push_back(i);
            }
            if (d >= order || used[d])
                continue;
            used[d] = true;

            // derivatives to the prior point that are before the point of
            // the block get the parameter of the prior point
            int const point = (before_point && types[i] > 0) ? block - 1 : block;
            conditions.push_back(Condition{static_cast<int>(i), d, types[i] > 0, point});
        }
        if (before_point)
            throw std::invalid_argument("base::geometry::SplineBase::interpolate(): derivatives given after the last point");
    }

    // Compute the parameters of the points
    size_t const points_count = point_indexes.size();
    if (points_count < 2)
        throw error("need at least two points");
    vector<double> point_params(points_count);
    if (parametersIn.empty())
    {
        // Chord length parametrization, like SISL's s1909
        point_params[0] = start_param;
        for (size_t p = 1; p < points_count; ++p)
        {
            point_params[p] = point_params[p - 1] + sqrt(squaredDistance(
                &values[point_indexes[p - 1] * dimension],
                &values[point_indexes[p] * dimension], dimension));
        }
    }
    else
    {
        if (parametersIn.size() != points_count)
        {
            throw std::invalid_argument("base::geometry::SplineBase::interpolate(): expected " +
                    lexical_cast<string>(points_count) + " parameters (one per point), got " +
                    lexical_cast<string>(parametersIn.size()));
        }
        point_params = parametersIn;
    }
    for (size_t p = 1; p < points_count; ++p)
    {
        if (!(point_params[p - 1] < point_params[p]))
            throw error("the point parameters are not strictly increasing");
    }

    // Generate the knots and solve the interpolation problem
    int const condition_count = conditions.size();
    vector<double> epar(condition_count);
    for (int c = 0; c < condition_count; ++c)
        epar[c] = point_params[conditions[c].point];
    sort(epar.begin(), epar.end());

    Curve curve;
    curve.dim = dimension;
    curve.k = std::min(order, condition_count);
    curve.t = interpolationKnots(epar, curve.k);

    vector< Triplet<double> > triplets;
    MatrixXd B(condition_count, dimension);
    for (int c = 0; c < condition_count; ++c)
    {
        Condition const& cond = conditions[c];
        if (cond.derivative >= curve.k)
            throw error("derivative of order " + lexical_cast<string>(cond.derivative) +
                    " given for a curve of order " + lexical_cast<string>(curve.k));
        addCollocationRow(triplets, c, curve.t, curve.k, condition_count,
                point_params[cond.point], cond.derivative, cond.from_left);
        B.row(c) = Map<RowVectorXd>(&values[cond.index * dimension], dimension);
    }
    SparseMatrix<double> A(condition_count, condition_count);
    A.setFromTriplets(triplets.begin(), triplets.end());

    MatrixXd X;
    try { X = solveSparse(A, B); }
    catch(std::runtime_error const& e) { throw error(e.what()); }

    curve.P.resize(condition_count * dimension);
    for (int c = 0; c < condition_count; ++c)
        for (int d = 0; d < dimension; ++d)
            curve.P[c * dimension + d] = X(c, d);

    // Like SISL, use a lower order if there are not enough conditions, and
    // raise the order afterwards
    elevateOrder(curve, order);
    setCurve(curve.k, curve.t, curve.P);

    parametersOut = epar;
    parametersOut.erase(std::unique(parametersOut.begin(), parametersOut.end()), parametersOut.end());
}

void SplineBase::resetImpl(std::vector<double> const& coordinates,
        std::vector<double> const& new_knots, int kind)
{
    if (kind == -1)
        kind = getSISLCurveType();

    std::vector<double> polynomial;
    if (kind == 1 || kind == 3)
        polynomial = coordinates;
    else if (kind == 2 || kind == 4)
    {
        // Rational curve, the coordinates are given in homogeneous form. We
        // can only handle it if all the weights are equal
        int const stride = dimension + 1;
        if (coordinates.size() % stride != 0)
            throw std::invalid_argument("the number of coordinates is not a multiple of the curve dimension + 1");
        double const w0 = coordinates[dimension];
        for (size_t i = 0; i < coordinates.size(); i += stride)
        {
            double const w = coordinates[i + dimension];
            if (fabs(w - w0) > 1e-12 * fabs(w0))
                throw std::invalid_argument("rational curves (NURBS) are not supported");
            for (int c = 0; c < dimension; ++c)
                polynomial.push_back(coordinates[i + c] / w);
        }
    }
    else
        throw std::invalid_argument("unsupported curve kind " + lexical_cast<string>(kind));

    int const n = polynomial.size() / dimension;
    setCurve(static_cast<int>(new_knots.size()) - n, new_knots, polynomial);
}

void SplineBase::findClosestPointsImpl(double const* ref_point,
        vector<double>& _result_points, double _geores) const
{
    vector<double> const minima = findDistanceMinima(ref_point);
    vector<double> distances(minima.size());
    vector<double> p(dimension);
    double min_distance = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < minima.size(); ++i)
    {
        evaluate(p.data(), minima[i], 0, minima[i] >= end_param);
        distances[i] = sqrt(squaredDistance(p.data(), ref_point, dimension));
        min_distance = std::min(min_distance, distances[i]);
    }

    for (size_t i = 0; i < minima.size(); ++i)
    {
        if (distances[i] <= min_distance + std::max(0.0, _geores))
            _result_points.push_back(minima[i]);
    }
}

double SplineBase::localClosestPointSearchImpl(double const* ref_point, double _guess,
        double _start, double _end) const
{
    // Port of SISL's s1771 (called by s1774): Newton iteration on the
    // distance function, with a step computed by solving a second degree
    // equation, and step halving if the distance increases
    double const astart = std::min(std::max(_start, start_param), end_param);
    double const aend = std::min(std::max(_end, start_param), end_param);
    double const adel = end_param - start_param;
    int const dim = dimension;

    // SISL evaluates from the right. Use the left side at the end of the
    // search interval, as the interval might end at a knot where the curve
    // is not smooth
    vector<double> sval(3 * dim), sdiff(dim);
    auto distanceAt = [&](double u) {
        evaluate(sval.data(), u, 2, u >= aend && aend > astart);
        for (int c = 0; c < dim; ++c)
            sdiff[c] = ref_point[c] - sval[c];
        return sqrt(squaredDistance(ref_point, sval.data(), dim));
    };
    auto clampStep = [&](double u, double& step) {
        if (u + step < astart)
            step = astart - u;
        else if (u + step > aend)
            step = aend - u;
    };

    double anext = std::min(std::max(_guess, start_param), end_param);
    distanceAt(anext);
    double td = newtonClosestPointStep(sdiff.data(), &sval[dim], &sval[2 * dim], dim);
    clampStep(anext, td);

    double aprev = HUGE_VAL;
    int kdiv = 0, kdiv2 = 0;
    for (int it = 0; it < 20; ++it)
    {
        double const cdist = distanceAt(anext + td);
        if (cdist - aprev <= REL_COMP_RES)
        {
            if (kdiv2 > 4)
                break;
            if (cdist - aprev >= 0)
                kdiv2++;
            kdiv = 0;
            aprev = cdist;
            anext += td;
            td = newtonClosestPointStep(sdiff.data(), &sval[dim], &sval[2 * dim], dim);
            clampStep(anext, td);
        }
        else
        {
            kdiv++;
            if (kdiv > 3)
                break;
            td /= 2;
        }
        if (fabs(td / std::max(fabs(anext), adel)) <= REL_COMP_RES)
            break;
    }
    return anext;
}

void SplineBase::findPointIntersectionsImpl(double const* _point,
        std::vector<double>& _result_points, double _geores) const
{
    vector<double> const minima = findDistanceMinima(_point);
    vector<double> p(dimension);
    for (size_t i = 0; i < minima.size(); ++i)
    {
        evaluate(p.data(), minima[i], 0, minima[i] >= end_param);
        if (sqrt(squaredDistance(p.data(), _point, dimension)) <= _geores)
            _result_points.push_back(minima[i]);
    }
}

void SplineBase::findLineIntersectionsImpl(double const* _point, double const* _normal,
        std::vector<double>& _result_points,
        std::vector< std::pair<double, double> >& _result_curves,
        double _geores) const
{
    int const n = vertices.size() / dimension;
    int const k = knots.size() - n;
    Map<VectorXd const> const normal(_normal, dimension);
    Map<VectorXd const> const point(_point, dimension);

    // Since the signed distance to the line/plane is linear, it is itself a
    // B-spline whose coefficients are the distances of the vertices. Parts of
    // the curve that lie in the line/plane are the knot spans whose
    // coefficients are all zero
    double scale = 1;
    vector<double> coefficients(n);
    for (int i = 0; i < n; ++i)
    {
        Map<VectorXd const> v(&vertices[i * dimension], dimension);
        coefficients[i] = (v - point).dot(normal);
        scale = std::max(scale, v.cwiseAbs().maxCoeff());
    }
    double const zero_tolerance = 1e-12 * scale * normal.norm();
    for (int s = k - 1; s < n; ++s)
    {
        if (!(knots[s] < knots[s + 1]))
            continue;
        bool in_plane = true;
        for (int j = s - k + 1; j <= s && in_plane; ++j)
            in_plane = fabs(coefficients[j]) <= zero_tolerance;
        if (!in_plane)
            continue;
        if (!_result_curves.empty() && _result_curves.back().second == knots[s])
            _result_curves.back().second = knots[s + 1];
        else
            _result_curves.push_back(make_pair(knots[s], knots[s + 1]));
    }

    vector<double> derivs(dimension);
    auto f = [&](double u) {
        evaluate(derivs.data(), u, 0, false);
        return (Map<VectorXd>(derivs.data(), dimension) - point).dot(normal);
    };
    vector<double> roots = findRoots(f, getSamplingParameters(std::max(8, 4 * k)),
            _geores * normal.norm());
    uniqueParameters(roots, 1e-9 * (end_param - start_param));
    for (size_t i = 0; i < roots.size(); ++i)
    {
        bool in_curve = false;
        for (size_t c = 0; c < _result_curves.size() && !in_curve; ++c)
            in_curve = (_result_curves[c].first <= roots[i] && roots[i] <= _result_curves[c].second);
        if (!in_curve)
            _result_points.push_back(roots[i]);
    }
}

void SplineBase::findSphereIntersectionsImpl(double const* _center, double radius,
        std::vector<double>& _result_points, double _geores) const
{
    int const n = vertices.size() / dimension;
    int const k = knots.size() - n;
    vector<double> p(dimension);
    auto f = [&](double u) {
        evaluate(p.data(), u, 0, false);
        return squaredDistance(p.data(), _center, dimension) - radius * radius;
    };
    vector<double> roots = findRoots(f, getSamplingParameters(std::max(8, 4 * k)),
            _geores * (2 * radius + _geores));
    uniqueParameters(roots, 1e-9 * (end_param - start_param));
    _result_points = roots;
}

void SplineBase::appendImpl(SplineBase const& other)
{
    if (other.getDimension() != dimension)
        throw std::runtime_error("cannot append curves of different dimensions");

    // Like SISL's s1715: other is translated so that its start point is
    // this curve's end point, and its parameters are shifted so that they
    // follow the ones of this curve
    int const dim = dimension;
    int const n1 = vertices.size() / dim, n2 = other.vertices.size() / dim;
    Curve first = { dim, static_cast<int>(knots.size()) - n1, knots, vertices };
    Curve second = { dim, static_cast<int>(other.knots.size()) - n2, other.knots, other.vertices };
    clampCurve(first);
    clampCurve(second);
    int const k = std::max(first.k, second.k);
    elevateOrder(first, k);
    elevateOrder(second, k);

    double const shift = first.end() - second.start();
    Curve result = first;
    // The joint gets a multiplicity of k - 1, i.e. the curve is C0 there
    result.t.pop_back();
    for (size_t i = k; i < second.t.size(); ++i)
        result.t.push_back(second.t[i] + shift);

    double const* last = &first.P[(first.n() - 1) * dim];
    double const* other_first = &second.P[0];
    for (int i = 1; i < second.n(); ++i)
    {
        for (int c = 0; c < dim; ++c)
            result.P.push_back(second.P[i * dim + c] - other_first[c] + last[c]);
    }
    setCurve(result.k, result.t, result.P);
}

void SplineBase::reverse()
{
    if (!hasCurve())
        return;

    int const n = vertices.size() / dimension;
    Curve c = { dimension, static_cast<int>(knots.size()) - n, knots, vertices };
    reverseCurve(c);
    setCurve(c.k, c.t, c.P);
}

bool SplineBase::testIntersection(SplineBase const& other, double resolution) const
{
    if (!hasCurve() || !other.hasCurve())
        return false;
    if (other.getDimension() != dimension)
        throw std::runtime_error("cannot test the intersection of curves of different dimensions");

    // Approximate both curves by polylines whose segments are not longer
    // than the resolution, and look for segments that are closer than the
    // resolution
    auto polyline = [resolution](SplineBase const& curve) {
        int const dim = curve.getDimension();
        int const n = curve.vertices.size() / dim;
        int const k = curve.knots.size() - n;
        vector<double> const us = curve.getSamplingParameters(std::max(8, 4 * k));
        vector<VectorXd> result;
        VectorXd p(dim), next(dim);
        curve.evaluate(p.data(), us[0], 0, false);
        result.push_back(p);
        for (size_t i = 1; i < us.size(); ++i)
        {
            curve.evaluate(next.data(), us[i], 0, true);
            int const steps = std::max(1, static_cast<int>(ceil((next - p).norm() / resolution)) * 2);
            for (int s = 1; s <= steps; ++s)
            {
                VectorXd q(dim);
                curve.evaluate(q.data(), us[i - 1] + (us[i] - us[i - 1]) * s / steps, 0, true);
                result.push_back(q);
            }
            p = next;
        }
        return result;
    };

    vector<VectorXd> const a = polyline(*this);
    vector<VectorXd> const b = polyline(other);
    for (size_t i = 1; i < a.size(); ++i)
    {
        VectorXd const a_min = (a[i - 1].cwiseMin(a[i]).array() - resolution).matrix();
        VectorXd const a_max = (a[i - 1].cwiseMax(a[i]).array() + resolution).matrix();
        for (size_t j = 1; j < b.size(); ++j)
        {
            if ((b[j - 1].cwiseMax(b[j]).array() < a_min.array()).any() ||
                (b[j - 1].cwiseMin(b[j]).array() > a_max.array()).any())
                continue;
            if (segmentDistance(a[i - 1], a[i], b[j - 1], b[j]) <= resolution)
                return true;
        }
    }
    return false;
}

vector<double> SplineBase::simplify(double tolerance)
{
    if (!hasCurve())
    {
        std::vector<double> result;
        result.push_back(0);
        return result;
    }

    // Approximates the curve by least-squares fits with uniform knots and
    // increasing number of vertices, until the error is within the tolerance.
    // The end points are kept.
    int const dim = dimension;
    int const n = vertices.size() / dim;
    int const k = knots.size() - n;
    vector<double> const us = getSamplingParameters(4 * k + 4);
    int const sample_count = us.size();
    MatrixXd samples(sample_count, dim);
    for (int i = 0; i < sample_count; ++i)
    {
        VectorXd p(dim);
        evaluate(p.data(), us[i], 0, i == sample_count - 1);
        samples.row(i) = p.transpose();
    }

    auto fit = [&](int m, Curve& result) {
        result.dim = dim;
        result.k = k;
        result.t.assign(k, start_param);
        for (int i = 1; i < m - k + 1; ++i)
            result.t.push_back(start_param + (end_param - start_param) * i / (m - k + 1));
        result.t.insert(result.t.end(), k, end_param);

        vector< Triplet<double> > triplets;
        for (int i = 0; i < sample_count; ++i)
            addCollocationRow(triplets, i, result.t, k, m, us[i], 0, i == sample_count - 1);
        SparseMatrix<double> A(sample_count, m);
        A.setFromTriplets(triplets.begin(), triplets.end());

        // Fix the end points, and solve for the other vertices
        RowVectorXd const first = samples.row(0), last = samples.row(sample_count - 1);
        MatrixXd X;
        if (m > 2)
        {
            MatrixXd const rhs = samples - VectorXd(A.col(0)) * first - VectorXd(A.col(m - 1)) * last;
            SparseMatrix<double> const inner = A.middleCols(1, m - 2);
            SparseMatrix<double> const normal = inner.transpose() * inner;
            SimplicialLDLT< SparseMatrix<double> > solver(normal);
            if (solver.info() != Success)
                return false;
            X = solver.solve(inner.transpose() * rhs);
            if (solver.info() != Success || !X.allFinite())
                return false;
        }

        result.P.resize(m * dim);
        for (int c = 0; c < dim; ++c)
        {
            result.P[c] = first(c);
            result.P[(m - 1) * dim + c] = last(c);
            for (int i = 1; i < m - 1; ++i)
                result.P[i * dim + c] = X(i - 1, c);
        }
        return true;
    };
    auto error = [&](Curve const& c) {
        vector<double> maxerr(dim, 0.0);
        VectorXd p(dim);
        for (int i = 0; i < sample_count; ++i)
        {
            evaluateCurve(c, us[i], 0, i == sample_count - 1, p.data());
            for (int d = 0; d < dim; ++d)
                maxerr[d] = std::max(maxerr[d], fabs(p(d) - samples(i, d)));
        }
        return maxerr;
    };
    auto within = [&](vector<double> const& maxerr) {
        for (int d = 0; d < dim; ++d)
            if (maxerr[d] > tolerance)
                return false;
        return true;
    };

    int lo = std::max(k, 2), hi = n;
    Curve best;
    vector<double> best_error(dim, 0.0);
    bool found = false;
    while (lo < hi)
    {
        int const m = (lo + hi) / 2;
        Curve candidate;
        vector<double> candidate_error;
        if (fit(m, candidate) && within(candidate_error = error(candidate)))
        {
            hi = m;
            best = candidate;
            best_error = candidate_error;
            found = true;
        }
        else
            lo = m + 1;
    }

    if (!found || best.n() >= n)
        return vector<double>(dim, 0.0);

    setCurve(best.k, best.t, best.P);
    return best_error;
}

Matrix3d SplineBase::getFrenetFrame(double _param) const
{
    if (!hasCurve())
        throw std::runtime_error("getFrenetFrame() called on an empty or degenerate curve");
    if (!checkAndNormalizeParam(_param))
        throw std::out_of_range("_param is not in the [start_param, end_param] range");

    // Same computation as SISL's s2559
    vector<double> derivs(3 * dimension);
    evaluate(derivs.data(), _param, 2, false);
    Eigen::Vector3d d[2];
    canonicalDerivatives(derivs.data(), dimension, 2, d);

    Eigen::Vector3d t = Eigen::Vector3d::Zero(), n = Eigen::Vector3d::Zero(), b = Eigen::Vector3d::Zero();
    double const length = d[0].norm();
    if (length != 0)
        t = d[0] / length;
    // Like SISL, the normal and binormal are zero if the curvature is zero
    Eigen::Vector3d const cross = d[0].cross(d[1]);
    if (!isZeroLength(cross.norm()))
    {
        b = cross.normalized();
        n = b.cross(t);
    }

    // Writes the frame to a matrix
    Matrix3d frame;
    frame << t[0], t[1], t[2], n[0], n[1], n[2], b[0], b[1], b[2];

    return frame;
}

void SplineBase::cropImpl(double start_t, double end_t)
{
    double a = std::min(start_t, end_t), b = std::max(start_t, end_t);
    if (!checkAndNormalizeParam(a, 0) || !checkAndNormalizeParam(b, 0))
        throw std::runtime_error("failed to crop the curve at between " + boost::lexical_cast<std::string>(start_t) + " and " + boost::lexical_cast<std::string>(end_t));

    // Like SISL's s1712, the curve is turned if end_t < start_t
    int const n = vertices.size() / dimension;
    Curve c = { dimension, static_cast<int>(knots.size()) - n, knots, vertices };
    cropCurve(c, a, b);
    if (end_t < start_t)
        reverseCurve(c);
    setCurve(c.k, c.t, c.P);
}

void SplineBase::splitImpl(SplineBase& second_part, double _param)
{
    int const n = vertices.size() / dimension;
    Curve c = { dimension, static_cast<int>(knots.size()) - n, knots, vertices };
    clampCurve(c);
    Curve part1, part2;
    splitCurve(c, _param, part1, part2);
    setCurve(part1.k, part1.t, part1.P);
    second_part.setCurve(part2.k, part2.t, part2.P);
}

SplineBase *SplineBase::getSubSpline(double start_t, double end_t) const
{
    if (!hasCurve() || start_t == end_t)
        return NULL;

    double a = std::min(start_t, end_t), b = std::max(start_t, end_t);
    if (!checkAndNormalizeParam(a, 0) || !checkAndNormalizeParam(b, 0))
        return NULL;

    SplineBase* result = new SplineBase(*this);
    result->crop(start_t, end_t);
    return result;
}

void SplineBase::derive(unsigned int order, SplineBase& result) const
{
    if (!hasCurve())
        throw std::invalid_argument("base::geometry::SplineBase::derive(): cannot derive a singleton");
    if (result.getDimension() != dimension)
        throw std::invalid_argument("base::geometry::SplineBase::derive(): the result curve has a different dimension");

    int const dim = dimension;
    vector<double> t = knots, P = vertices;
    int k = knots.size() - vertices.size() / dim;
    for (unsigned int d = 0; d < order; ++d)
    {
        if (k == 1)
        {
            // The derivative of a piecewise constant curve is zero
            t.clear();
            t.push_back(start_param);
            t.push_back(end_param);
            P.assign(dim, 0.0);
            break;
        }

        int const n = P.size() / dim;
        int const p = k - 1;
        vector<double> Q((n - 1) * dim);
        for (int i = 0; i < n - 1; ++i)
        {
            double const denominator = t[i + k] - t[i + 1];
            for (int c = 0; c < dim; ++c)
            {
                Q[i * dim + c] = (denominator > 0) ?
                    p * (P[(i + 1) * dim + c] - P[i * dim + c]) / denominator : 0;
            }
        }
        t = vector<double>(t.begin() + 1, t.end() - 1);
        P.swap(Q);
        --k;
    }

    result.setCurve(k, t, P);
}

}} //end namespace base::geometry
