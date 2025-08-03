/*------------------------------------------------------------------------------
 * @file AlignLib.cpp
 * @brief Alignment and Navigation Library - Core Algorithm Implementation
 *
 * This file implements the core algorithms of the PPOI_Nav navigation and integrated positioning system,
 * including the concrete implementation of classes and functions related to matrices, vectors, quaternions,
 * filtering, statistics, inertial navigation, GNSS integration, ring buffers, and more.
 * It mainly covers details such as inertial navigation, GNSS observation processing, filtering and state estimation,
 * attitude/velocity/position interpolation, sliding window mean, extremum statistics, IMU modeling,
 * SINS/GNSS loose integration, odometer constraints, and other algorithmic aspects.
 *
 * @author  Yejin
 * @date    2020-2025
 * @version 1.0
 * @copyright Copyright (c) 2020-2025 PPOI_Nav Project
 *                          PSINS: A C++ Inertial Navigation Library, https://psins.org.cn/
 *
 * @mainpage PPOI_Nav Alignment and Navigation Library - Algorithm Implementation
 * @section intro_sec Introduction
 * This source file provides all core algorithm implementations for the PPOI_Nav navigation and integrated positioning system,
 * including mathematical foundations, inertial/GNSS integration, filtering and state estimation, data structures, and utility tools.
 * It is suitable for high-precision navigation computation, integrated positioning, sensor modeling, observation data processing,
 * coordinate transformation, and similar scenarios.
 *
 * @section class_sec Classes & Functions Overview
 * - mat3, quat, vect, mat: Mathematical foundation classes and matrix/vector/quaternion operations
 * - earth: Earth model and navigation parameter updates
 * - maxmin, maxminn: Extremum and mean statistics
 * - STA_VAR, STA_VARS: Mean and variance statistics
 * - STA_AVPI: Attitude/velocity/position ring buffer and interpolation
 * - IMU: Inertial sensor modeling and data processing
 * - INS: Inertial navigation system state and update
 * - IIR, IIRV3: 1D and 3D IIR filter implementations
 * - odo: Odometer data and processing
 * - ZIHR: Zero-integrated heading rate detection
 * - KalmanFilter, EKFTDKF: Kalman filter and extended/distributed filter implementations
 * - SINSGNSSLOOSE, SINSGNSSBorad, SINSGNSSPOST: SINS/GNSS loose integration, with odometer and post-processing
 * - RingMemory: Fixed-length ring buffer implementation
 * - Smooth: Sliding window mean filter implementation
 * - Others: Utility functions, GNSS observation processing, coordinate transformation, physical constants, etc.
 *
 * @section history_sec History
 * - 2020-2021: Initial version, basic algorithm implementation
 * - 2022-2023: Major refactoring and documentation improvement
 * - 2024-2025: Feature expansion and performance optimization
 *
 * @section reference_sec References
 * - [0] PSINS: A C++ Inertial Navigation Library, https://psins.org.cn/
 * - [1] Farrell, J. Aided Navigation: GPS with High Rate Sensors, 2008.
 * - [2] Titterton, D. H., & Weston, J. L. Strapdown Inertial Navigation Technology, 2004.
 * - [3] RTKLIB: An Open Source Program Package for GNSS Positioning, http://www.rtklib.com/
 * - [4] Groves, P. D. Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems, 2013.
 *
 * @section usage_sec Usage
 * Simply include AlignLib.h and link this source file to obtain all core algorithm implementations
 * of the PPOI_Nav navigation and integrated positioning system.
 * Suitable for navigation computation, integrated positioning, filtering and estimation,
 * observation data processing, coordinate transformation, and related modules.
 *
 * @defgroup math_core Mathematical Foundations
 * @defgroup nav_core Navigation Core
 * @defgroup filter_core Filtering and Statistics
 * @defgroup util_core Utilities and Tools
 *----------------------------------------------------------------------------*/

#include "stdarg.h"
#include "math.h"
#include "string.h"
#include <stdio.h>
#include "../include/AlgLib.h"


#define e      sqrt(2*f0_earth-f0_earth*f0_earth)
#define e2     e*e
#define RP     (1-f0_earth)*RE
#define ep     sqrt(RP*RE-RP*RP)/RP
#define ep2    ep*ep


vect3 O31(0.0);               /*the define of vector 3x1 with {0,0,0}*/
mat3  I33(1, 0, 0, 0, 1, 0, 0, 0, 1);
quat  qI(1, 0, 0, 0);

static STA_VAR ant_angle(5, 100); /* show the var with the ant angle ensure ant is stale*/
static STA_VAR trj_angle(5, 100); /* show the var with the trj angle ensure trj is stale*/
static int flag_align = 0;     /* flag show the coast align used acutly*/
static int cnt_align = 0;      /* the cnt using in estimating the bias process*/

/* range limiting function ----------------------------------------------------
* limit a value within a specified range
* args   : double     val      I   input value
*          double     minVal   I   minimum limit
*          double     maxVal   I   maximum limit
* return : value within the range [minVal, maxVal]
*-----------------------------------------------------------------------------*/
double range(double val, double minVal, double maxVal) {
	double res;

	if (val < minVal)
	{
		res = minVal;
	}
	else if (val > maxVal)
	{
		res = maxVal;
	}
	else
	{
		res = val;
	}
	return res;
}

/* sign function ----------------------------------------------------------------
* determine the sign of a given value
* args   : double     val      I   input value
*          double     eps      I   threshold for determining zero (default: EPS)
* return : 1 if val > eps
*         -1 if val < -eps
*          0 if -eps <= val <= eps
*-----------------------------------------------------------------------------*/
int sign(double val, double eps) {
	int s;

	if (val < -eps)
	{
		s = -1;
	}
	else if (val > eps)
	{
		s = 1;
	}
	else
	{
		s = 0;
	}
	return s;
}

/* extended atan2 function ----------------------------------------------------
* calculate atan2 with extended functionality
* args   : double     y        I   y-coordinate
*          double     x        I   x-coordinate
* return : angle in radians
*-----------------------------------------------------------------------------*/
double atan2Ex(double y, double x) {
	double res;
	if ((sign(y) == 0) && (sign(x) == 0)) {
		res = 0.0;
	}
	else {
		res = atan2(y, x);
	}
	return res;
}
/* polynomial evaluation -------------------------------------------------------
 * Evaluates a polynomial at a given point.
 * args   : const double* p    I   coefficients of the polynomial
 *          int    order       I   order of the polynomial
 *          double x           I   point at which to evaluate
 * return : double             O   value of the polynomial at x
 *-----------------------------------------------------------------------------*/
double polyval(const double* p, int order, double x)
{
	double y = p[0];
	for (int k = 1; k <= order; k++)  y = y * x + p[k];
	return y;
}

/* calculate 1-norm of an array -----------------------------------------------
* calculate the 1-norm (sum of absolute values) of an array
* args   : const double* pd    I   pointer to the array
*          int         n       I   number of elements in the array
* return : 1-norm of the array
*-----------------------------------------------------------------------------*/
double norm1(const double* pd, int n)
{
	double n1 = 0.0; const double* pn = &pd[n];
	for (; pd < pn; pd++) {
		if (*pd > 0.0) n1 += *pd; else n1 -= *pd;
	}
	return n1;
}

/* calculate Euclidean norm of an array ---------------------------------------
* calculate the Euclidean norm of an array
* args   : const double* pd    I   pointer to the array
*          int         n       I   number of elements in the array
* return : Euclidean norm of the array
*-----------------------------------------------------------------------------*/
double norm(const double* pd, int n)
{
	double n2 = 0.0; const double* pn = &pd[n];
	for (; pd < pn; pd++) {
		n2 += *pd * (*pd);
	}
	return sqrt(n2);
}

/* calculate infinity norm of an array ----------------------------------------
* calculate the infinity norm (maximum absolute value) of an array
* args   : const double* pd    I   pointer to the array
*          int         n       I   number of elements in the array
* return : infinity norm of the array
*-----------------------------------------------------------------------------*/
double normInf(const double* pd, int n)
{
	double ninf = 0.0; const double* pn = &pd[n];
	for (; pd < pn; pd++) {
		if (*pd > ninf) ninf = *pd; else if (-*pd > ninf) ninf = -*pd;
	}
	return ninf;
}

/* calculate first-order Markov white-noise variance ---------------------------
* calculate the first-order Markov white-noise variance
* args   : vect3      sR       I   noise standard deviation
*          vect3      tau      I   correlation time
* return : Markov white-noise variance
*-----------------------------------------------------------------------------*/
double MKQt(double sR, double tau)
{
	return sR * sR * 2.0 / tau;
}

/* calculate first-order Markov white-noise variance ---------------------------
* calculate the first-order Markov white-noise variance
* args   : vect3      sR       I   noise standard deviation
*          vect3      tau      I   correlation time
* return : Markov white-noise variance
*-----------------------------------------------------------------------------*/
vect3 MKQt(const vect3& sR, const vect3& tau)
{
	return vect3(sR.i * sR.i * 2.0 / tau.i, sR.j * sR.j * 2.0 / tau.j, sR.k * sR.k * 2.0 / tau.k);
}

/* default constructor for vect3 -----------------------------------------------
 * Initializes a vect3 object with zero values.
 * args   : none
 * return : none
 *----------------------------------------------------------------------------- */
vect3::vect3(void)
{
}
/* constructor for vect3 with scalar input -------------------------------------
 * Initializes a vect3 object with the same value for all components.
 * args   : double    xyz      I   value for all components
 * return : none
 *----------------------------------------------------------------------------- */
vect3::vect3(double xyz)
{
	i = j = k = xyz;
}
/* constructor for vect3 with individual components ---------------------------
 * Initializes a vect3 object with specified values for each component.
 * args   : double    xx       I   x-component
 *          double    yy       I   y-component
 *          double    zz       I   z-component
 * return : none
 *----------------------------------------------------------------------------- */
vect3::vect3(double xx, double yy, double zz)
{
	i = xx, j = yy, k = zz;
}
/* constructor for vect3 from a double array -----------------------------------
 * Initializes a vect3 object from a double array.
 * args   : const double* pdata I   pointer to the array
 * return : none
 *----------------------------------------------------------------------------- */
vect3::vect3(const double *pdata)
{
	i = *pdata++, j = *pdata++, k = *pdata;
}
/* constructor for vect3 from a float array ------------------------------------
 * Initializes a vect3 object from a float array.
 * args   : const float* pdata I   pointer to the array
 * return : none
 *----------------------------------------------------------------------------- */
vect3::vect3(const float *pdata)
{
	i = *pdata++, j = *pdata++, k = *pdata;
}

/* check if a vect3 is zero ----------------------------------------------------
 * Checks if all components of a vect3 are within a tolerance of zero.
 * args   : const vect3 &v     I   input vector
 *          double    eps      I   tolerance
 * return : int                 O   1 if all components are zero, 0 otherwise
 *----------------------------------------------------------------------------- */
int IsZeros(const vect3 &v, double eps)
{
	return (v.i<eps&&v.i>-eps && v.j<eps&&v.j>-eps && v.k<eps&&v.k>-eps);
}
/* check if a scalar is zero ---------------------------------------------------
 * Checks if a scalar value is within a tolerance of zero.
 * args   : const double& val  I   input value
 *          double    eps      I   tolerance
 * return : int                 O   1 if value is zero, 0 otherwise
 *----------------------------------------------------------------------------- */
int IsZero(const double& val, double eps) {
	return (val<eps&&val>-eps);
}
/* calculate yaw difference ----------------------------------------------------
* calculate the difference between two yaw angles
* args   : double     yaw      I   first yaw angle (rad)
*          double     yaw0     I   second yaw angle (rad)
* return : difference between yaw and yaw0 (rad)
*-----------------------------------------------------------------------------*/
double diffYaw(double yaw, double yaw0)
{
	double dyaw = yaw - yaw0;
	if (dyaw >= PI) dyaw -= _2PI;
	else if (dyaw <= -PI) dyaw += _2PI;
	return dyaw;
}
/* check if a vect3 has zero x and y components --------------------------------
 * Checks if the x and y components of a vect3 are within a tolerance of zero.
 * args   : const vect3 &v     I   input vector
 *          double    eps      I   tolerance
 * return : bool               O   true if x and y are zero, false otherwise
 *----------------------------------------------------------------------------- */
uint8_t IsZerosXY(const vect3& v, double eps)
{
	return (v.i<eps&&v.i>-eps && v.j<eps&&v.j>-eps);
}

/* check if a vect3 contains NaN values ----------------------------------------
 * Checks if any component of a vect3 is NaN.
 * args   : const vect3 &v     I   input vector
 * return : bool               O   true if any component is NaN, false otherwise
 *----------------------------------------------------------------------------- */
uint8_t IsNaN(const vect3& v)
{
	return 0; //(_isnan(i) || _isnan(j) || _isnan(k));
}

/* addition operator for vect3 ------------------------------------------------
 * Adds two vect3 objects component-wise.
 * args   : const vect3 &v     I   vector to add
 * return : vect3              O   result of the addition
 *-----------------------------------------------------------------------------*/
vect3 vect3::operator+(const vect3 &v) const
{
	return vect3(this->i + v.i, this->j + v.j, this->k + v.k);
}
/* subtraction operator for vect3 ----------------------------------------------
 * Subtracts one vect3 object from another component-wise.
 * args   : const vect3 &v     I   vector to subtract
 * return : vect3              O   result of the subtraction
 *----------------------------------------------------------------------------- */
vect3 vect3::operator-(const vect3 &v) const
{
	return vect3(this->i - v.i, this->j - v.j, this->k - v.k);
}
/* cross product operator for vect3 --------------------------------------------
 * Computes the cross product of two vect3 objects.
 * args   : const vect3 &v     I   vector to compute cross product with
 * return : vect3              O   result of the cross product
 *-----------------------------------------------------------------------------
 */
vect3 vect3::operator*(const vect3 &v) const
{
	return vect3(this->j*v.k - this->k*v.j, this->k*v.i - this->i*v.k, this->i*v.j - this->j*v.i);
}
/* scalar multiplication operator for vect3 ------------------------------------
 * Multiplies a vect3 object by a scalar.
 * args   : double    f        I   scalar to multiply by
 * return : vect3              O   result of the multiplication
 *-----------------------------------------------------------------------------
 */
vect3 vect3::operator*(double f) const
{
	return vect3(i*f, j*f, k*f);
}

/* matrix multiplication operator for vect3 ------------------------------------
 * Multiplies a vect3 object by a mat3 object.
 * args   : const mat3 &m      I   matrix to multiply by
 * return : vect3              O   result of the multiplication
 *-----------------------------------------------------------------------------
 */
vect3 vect3::operator*(const mat3 &m) const
{
	return vect3(i*m.e00 + j*m.e10 + k*m.e20, i*m.e01 + j*m.e11 + k*m.e21, i*m.e02 + j*m.e12 + k*m.e22);
}

/* scalar division operator for vect3 ------------------------------------------
 * Divides a vect3 object by a scalar.
 * args   : double    f        I   scalar to divide by
 * return : vect3              O   result of the division
 *-----------------------------------------------------------------------------
 */
vect3 vect3::operator/(double f) const
{
	return vect3(i / f, j / f, k / f);
}

/* component-wise division operator for vect3 ----------------------------------
 * Divides a vect3 object by another vect3 object component-wise.
 * args   : const vect3 &v     I   vector to divide by
 * return : vect3              O   result of the division
 *-----------------------------------------------------------------------------
 */
vect3 vect3::operator/(const vect3 &v) const
{
	return vect3(i / v.i, j / v.j, k / v.k);
}

/* assignment operator for vect3 with scalar -----------------------------------
 * Assigns the same scalar value to all components of a vect3 object.
 * args   : double    f        I   scalar value to assign
 * return : vect3&             O   reference to the modified vect3 object
 *-----------------------------------------------------------------------------
 */
vect3& vect3::operator=(double f)
{
	i = j = k = f;
	return *this;
}

/* assignment operator for vect3 with array ------------------------------------
 * Assigns values from a double array to a vect3 object.
 * args   : const double *pf   I   pointer to the array
 * return : vect3&             O   reference to the modified vect3 object
 *-----------------------------------------------------------------------------
 */
vect3& vect3::operator=(const double *pf)
{
	i = *pf++, j = *pf++, k = *pf;
	return *this;
}

/* compound addition operator for vect3 ----------------------------------------
 * Adds another vect3 object to the current vect3 object component-wise.
 * args   : const vect3 &v     I   vector to add
 * return : vect3&             O   reference to the modified vect3 object
 *-----------------------------------------------------------------------------
 */
vect3& vect3::operator+=(const vect3 &v)
{
	i += v.i, j += v.j, k += v.k;
	return *this;
}

/* compound subtraction operator for vect3 -------------------------------------
 * Subtracts another vect3 object from the current vect3 object component-wise.
 * args   : const vect3 &v     I   vector to subtract
 * return : vect3&             O   reference to the modified vect3 object
 *-----------------------------------------------------------------------------
 */
vect3& vect3::operator-=(const vect3 &v)
{
	i -= v.i, j -= v.j, k -= v.k;
	return *this;
}

/* compound scalar multiplication operator for vect3 ---------------------------
 * Multiplies the current vect3 object by a scalar.
 * args   : double    f        I   scalar to multiply by
 * return : vect3&             O   reference to the modified vect3 object
 *-----------------------------------------------------------------------------
 */
vect3& vect3::operator*=(double f)
{
	i *= f, j *= f, k *= f;
	return *this;
}

/* compound scalar division operator for vect3 ---------------------------------
 * Divides the current vect3 object by a scalar.
 * args   : double    f        I   scalar to divide by
 * return : vect3&             O   reference to the modified vect3 object
 *-----------------------------------------------------------------------------
 */
vect3& vect3::operator/=(double f)
{
	i /= f, j /= f, k /= f;
	return *this;
}

/* compound component-wise division operator for vect3 -------------------------
 * Divides the current vect3 object by another vect3 object component-wise.
 * args   : const vect3 &v     I   vector to divide by
 * return : vect3&             O   reference to the modified vect3 object
 *-----------------------------------------------------------------------------
 */
vect3& vect3::operator/=(const vect3 &v)
{
	i /= v.i, j /= v.j, k /= v.k;
	return *this;
}

/* scalar multiplication operator for vect3 (reversed) -------------------------
 * Multiplies a scalar by a vect3 object.
 * args   : double    f        I   scalar to multiply by
 *          const vect3 &v     I   vector to multiply
 * return : vect3              O   result of the multiplication
 *-----------------------------------------------------------------------------
 */
vect3 operator*(double f, const vect3 &v)
{
	return vect3(v.i*f, v.j*f, v.k*f);
}

/* negation operator for vect3 ------------------------------------------------
 * Negates all components of a vect3 object.
 * args   : const vect3 &v     I   vector to negate
 * return : vect3              O   negated vector
 *-----------------------------------------------------------------------------
 */
vect3 operator-(const vect3 &v)
{
	return vect3(-v.i, -v.j, -v.k);
}

/* outer product of two vect3 objects ------------------------------------------
 * Computes the outer product of two vect3 objects.
 * args   : const vect3 &v1    I   first vector
 *          const vect3 &v2    I   second vector
 * return : mat3               O   result of the outer product
 *-----------------------------------------------------------------------------
 */
mat3 vxv(const vect3 &v1, const vect3 &v2)
{
	return mat3(v1.i*v2.i, v1.i*v2.j, v1.i*v2.k,
		v1.j*v2.i, v1.j*v2.j, v1.j*v2.k,
		v1.k*v2.i, v1.k*v2.j, v1.k*v2.k);
}

/* square root of a vect3 ------------------------------------------------------
 * Computes the square root of each component of a vect3 object.
 * args   : const vect3 &v     I   input vector
 * return : vect3              O   vector with square root of each component
 *-----------------------------------------------------------------------------
 */
vect3 sqrt(const vect3 &v)
{
	return vect3(sqrt(v.i), sqrt(v.j), sqrt(v.k));
}

/* power of a vect3 ------------------------------------------------------------
 * Raises each component of a vect3 object to a given power.
 * args   : const vect3 &v     I   input vector
 *          int    k           I   power to raise to
 * return : vect3              O   vector with each component raised to power k
 *-----------------------------------------------------------------------------
 */
vect3 pow(const vect3 &v, int k)
{
	vect3 pp = v;
	for (int i = 1; i<k; i++)
	{
		pp.i *= v.i, pp.j *= v.j, pp.k *= v.k;
	}
	return pp;
}

/* absolute value of a vect3 ---------------------------------------------------
 * Computes the absolute value of each component of a vect3 object.
 * args   : const vect3 &v     I   input vector
 * return : vect3              O   vector with absolute value of each component
 *-----------------------------------------------------------------------------
 */
vect3 abs(const vect3 &v)
{
	vect3 res;
	res.i = v.i>0.0 ? v.i : -v.i;
	res.j = v.j>0.0 ? v.j : -v.j;
	res.k = v.k>0.0 ? v.k : -v.k;
	return res;
}

/* L2 norm of a vect3 ----------------------------------------------------------
 * Computes the L2 norm (Euclidean norm) of a vect3 object.
 * args   : const vect3 &v     I   input vector
 * return : double             O   L2 norm of the vector
 *-----------------------------------------------------------------------------
 */
double norm(const vect3 &v)
{
	return sqrt(v.i*v.i + v.j*v.j + v.k*v.k);
}

/* infinity norm of a vect3 ----------------------------------------------------
 * Computes the infinity norm (maximum absolute value) of a vect3 object.
 * args   : const vect3 &v     I   input vector
 * return : double             O   infinity norm of the vector
 *-----------------------------------------------------------------------------
 */
double normInf(const vect3 &v)
{
	double i = v.i>0 ? v.i : -v.i,
		j = v.j>0 ? v.j : -v.j,
		k = v.k>0 ? v.k : -v.k;
	if (i>j)	return i>k ? i : k;
	else    return j>k ? j : k;
}

/* L2 norm of the x and y components of a vect3 --------------------------------
 * Computes the L2 norm of the x and y components of a vect3 object.
 * args   : const vect3 &v     I   input vector
 * return : double             O   L2 norm of the x and y components
 *-----------------------------------------------------------------------------
 */
double normXY(const vect3 &v)
{
	return sqrt(v.i*v.i + v.j*v.j);
}

/* infinity norm of the x and y components of a vect3 --------------------------
 * Computes the infinity norm of the x and y components of a vect3 object.
 * args   : const vect3 &v     I   input vector
 * return : double             O   infinity norm of the x and y components
 *-----------------------------------------------------------------------------
 */
double normXYInf(const vect3 &v)
{
	double i = v.i>0 ? v.i : -v.i,
		j = v.j>0 ? v.j : -v.j;
	return i>j ? i : j;
}

/* dot product of two vect3 objects -------------------------------------------
 * Computes the dot product of two vect3 objects.
 * args   : const vect3 &v1    I   first vector
 *          const vect3 &v2    I   second vector
 * return : double             O   dot product of the two vectors
 *-----------------------------------------------------------------------------
 */
double dot(const vect3 &v1, const vect3 &v2)
{
	return (v1.i*v2.i + v1.j*v2.j + v1.k*v2.k);
}

/* component-wise multiplication of two vect3 objects -------------------------
 * Computes the component-wise multiplication of two vect3 objects.
 * args   : const vect3 &v1    I   first vector
 *          const vect3 &v2    I   second vector
 * return : vect3              O   component-wise product of the two vectors
 *-----------------------------------------------------------------------------
 */
vect3 dotmul(const vect3 &v1, const vect3 &v2)
{
	return vect3(v1.i*v2.i, v1.j*v2.j, v1.k*v2.k);
}

/* rotation vector to quaternion conversion -----------------------------------
 * Converts a rotation vector to a quaternion.
 * args   : const vect3 &rv     I   rotation vector
 * return : quat                O   quaternion representation of the rotation vector
 *-----------------------------------------------------------------------------*/
quat rv2q(const vect3 &rv)
{
#define F1	(   2 * 1)		// define: Fk=2^k*k!
#define F2	(F1*2 * 2)
#define F3	(F2*2 * 3)
#define F4	(F3*2 * 4)
#define F5	(F4*2 * 5)
	double n2 = rv.i*rv.i + rv.j*rv.j + rv.k*rv.k, c, f;
	if (n2<(PI / 180.0*PI / 180.0))	// 0.017^2
	{
		double n4 = n2*n2;
		c = 1.0 - n2*(1.0 / F2) + n4*(1.0 / F4);
		f = 0.5 - n2*(1.0 / F3) + n4*(1.0 / F5);
	}
	else
	{
		double n_2 = sqrt(n2) / 2.0;
		c = cos(n_2);
		f = sin(n_2) / n_2*0.5;
	}
	return quat(c, f*rv.i, f*rv.j, f*rv.k);
}

/* quaternion to rotation vector conversion -----------------------------------
 * Converts a quaternion to a rotation vector.
 * args   : const quat &qnb     I   quaternion
 * return : vect3              O   rotation vector representation of the quaternion
 *-----------------------------------------------------------------------------*/
mat3 q2mat(const quat &qnb)
{
	double	q11 = qnb.q0*qnb.q0, q12 = qnb.q0*qnb.q1, q13 = qnb.q0*qnb.q2, q14 = qnb.q0*qnb.q3,
		q22 = qnb.q1*qnb.q1, q23 = qnb.q1*qnb.q2, q24 = qnb.q1*qnb.q3,
		q33 = qnb.q2*qnb.q2, q34 = qnb.q2*qnb.q3,
		q44 = qnb.q3*qnb.q3;
	mat3 Cnb;
	Cnb.e00 = q11 + q22 - q33 - q44, Cnb.e01 = 2 * (q23 - q14), Cnb.e02 = 2 * (q24 + q13),
		Cnb.e10 = 2 * (q23 + q14), Cnb.e11 = q11 - q22 + q33 - q44, Cnb.e12 = 2 * (q34 - q12),
		Cnb.e20 = 2 * (q24 - q13), Cnb.e21 = 2 * (q34 + q12), Cnb.e22 = q11 - q22 - q33 + q44;
	return Cnb;
}

/* rotation vector to matrix conversion --------------------------------------
 * Converts a rotation vector to a rotation matrix.
 * args   : const vect3 &rv     I   rotation vector
 * return : mat3                O   rotation matrix representation of the rotation vector
 *-----------------------------------------------------------------------------*/
mat3 rv2m(const vect3 &rv)
{
	return q2mat(rv2q(rv));
}

/* calculate askew-symmetric matrix --------------------------------------------
* calculate the askew-symmetric matrix of a vector
* args   : vect3      v        I   input vector
* return : askew-symmetric matrix
*-----------------------------------------------------------------------------*/
mat3 askew(const vect3& v)
{
	return mat3(0, -v.k, v.j,
		v.k, 0.0, -v.i,
		-v.j, v.i, 0);
}

/* calculate sine of angle between two vectors ---------------------------------
* calculate the absolute value of sine of the angle between two vectors
* args   : vect3      v1       I   first input vector
*          vect3      v2       I   second input vector
* return : |sin(angle(v1, v2))|
*-----------------------------------------------------------------------------*/
double sinAng(const vect3& v1, const vect3& v2)
{
	if (IsZeros(v1) || IsZeros(v2)) return 0.0;
	return norm(v1 * v2) / (norm(v1) * norm(v2));
}

/* convert Direction Cosine Matrix (DCM) to quaternion ------------------------
* convert a Direction Cosine Matrix (DCM) to a quaternion
* args   : mat3      Cnb      I   Direction Cosine Matrix (DCM)
* return : quaternion
*-----------------------------------------------------------------------------*/
quat m2qua(const mat3& Cnb)
{
	double q0, q1, q2, q3, qq4;
	if (Cnb.e00 >= Cnb.e11 + Cnb.e22)
	{
		q1 = 0.5 * sqrt(1 + Cnb.e00 - Cnb.e11 - Cnb.e22);  qq4 = 4 * q1;
		q0 = (Cnb.e21 - Cnb.e12) / qq4; q2 = (Cnb.e01 + Cnb.e10) / qq4; q3 = (Cnb.e02 + Cnb.e20) / qq4;
	}
	else if (Cnb.e11 >= Cnb.e00 + Cnb.e22)
	{
		q2 = 0.5 * sqrt(1 - Cnb.e00 + Cnb.e11 - Cnb.e22);  qq4 = 4 * q2;
		q0 = (Cnb.e02 - Cnb.e20) / qq4; q1 = (Cnb.e01 + Cnb.e10) / qq4; q3 = (Cnb.e12 + Cnb.e21) / qq4;
	}
	else if (Cnb.e22 >= Cnb.e00 + Cnb.e11)
	{
		q3 = 0.5 * sqrt(1 - Cnb.e00 - Cnb.e11 + Cnb.e22);  qq4 = 4 * q3;
		q0 = (Cnb.e10 - Cnb.e01) / qq4; q1 = (Cnb.e02 + Cnb.e20) / qq4; q2 = (Cnb.e12 + Cnb.e21) / qq4;
	}
	else
	{
		q0 = 0.5 * sqrt(1 + Cnb.e00 + Cnb.e11 + Cnb.e22);  qq4 = 4 * q0;
		q1 = (Cnb.e21 - Cnb.e12) / qq4; q2 = (Cnb.e02 - Cnb.e20) / qq4; q3 = (Cnb.e10 - Cnb.e01) / qq4;
	}
	double nq = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 /= nq; q1 /= nq; q2 /= nq; q3 /= nq;
	return quat(q0, q1, q2, q3);
}

/* convert Euler angles to Direction Cosine Matrix (DCM) -----------------------
* convert a set of Euler angles to a Direction Cosine Matrix (DCM)
* args   : vect3      att      I   Euler angles {roll, pitch, yaw} (radians)
* return : Direction Cosine Matrix (DCM)
*-----------------------------------------------------------------------------*/
mat3 a2mat(const vect3& att)
{
	double	si = sin(att.i), ci = cos(att.i),
		sj = sin(att.j), cj = cos(att.j),
		sk = sin(att.k), ck = cos(att.k);
	mat3 Cnb;
	Cnb.e00 = cj * ck - si * sj * sk;	Cnb.e01 = -ci * sk;	Cnb.e02 = sj * ck + si * cj * sk;
	Cnb.e10 = cj * sk + si * sj * ck;	Cnb.e11 = ci * ck;	Cnb.e12 = sj * sk - si * cj * ck;
	Cnb.e20 = -ci * sj;				Cnb.e21 = si;		Cnb.e22 = ci * cj;
	return Cnb;
}

/* calculate magnetic yaw ------------------------------------------------------
* calculate the yaw angle based on magnetic field measurements
* args   : vect3      mag      I   magnetic field vector
*          vect3      att      I   attitude {roll, pitch, yaw} (radians)
*          double     declination I magnetic declination (radians)
* return : magnetic yaw angle (radians)
*-----------------------------------------------------------------------------*/
double MagYaw(const vect3& mag, const vect3& att, double declination)
{
	vect3 attH(att.i, att.j, 0.0);
	vect3 magH = a2mat(attH) * mag;
	double yaw = 0.0;
	//	if(attH.i<(80.0*DEG)&&attH.i>-(80.0*DEG))
	{
		yaw = atan2Ex(magH.i, magH.j) + declination;
		if (yaw > PI)       yaw -= _2PI;
		else if (yaw < -PI) yaw += _2PI;
	}
	return yaw;
}

/* sort vector elements -------------------------------------------------------
* sort the elements of a vector in ascending order
* args   : vect3      v        I   input vector
* return : sorted vector
*-----------------------------------------------------------------------------*/
vect3 sort(const vect3& v)
{
	vect3 vtmp = v;
	if (vtmp.i < vtmp.j) swapt(vtmp.i, vtmp.j, double);
	if (vtmp.i < vtmp.k) swapt(vtmp.i, vtmp.k, double);
	if (vtmp.j < vtmp.k) swapt(vtmp.j, vtmp.k, double);
	return vtmp;
}

/* default constructor --------------------------------------------------------
* initialize quaternion to (1, 0, 0, 0)
* args   : none
* return : none
*-----------------------------------------------------------------------------*/
quat::quat(void)
{
}

/* parameterized constructor --------------------------------------------------
* initialize quaternion with specified components
* args   : double     qq0      I   scalar part
*          double     qq1      I   first vector part (default: 0.0)
*          double     qq2      I   second vector part (default: 0.0)
*          double     qq3      I   third vector part (default: 0.0)
* return : none
*-----------------------------------------------------------------------------*/
quat::quat(double qq0, const vect3& qqv)
{
	q0 = qq0, q1 = qqv.i, q2 = qqv.j, q3 = qqv.k;
}

/* quaternion from scalar and vector ------------------------------------------
* initialize quaternion from scalar and 3D vector
* args   : double     qq0      I   scalar part
*          vect3      qqv      I   vector part
* return : none
*-----------------------------------------------------------------------------*/
quat::quat(double qq0, double qq1, double qq2, double qq3)
{
	q0 = qq0, q1 = qq1, q2 = qq2, q3 = qq3;
}

/* quaternion from array ------------------------------------------------------
* initialize quaternion from array of 4 doubles
* args   : const double* pdata I   pointer to 4 double values
* return : none
*-----------------------------------------------------------------------------*/
quat::quat(const double* pdata)
{
	q0 = *pdata++, q1 = *pdata++, q2 = *pdata++, q3 = *pdata++;
}

/* add misalignment vector ----------------------------------------------------
* add a misalignment vector to the quaternion (true quaternion add misalign angles)
* args   : const vect3& phi    I   misalignment vector
* return : resulting quaternion
*-----------------------------------------------------------------------------*/
quat quat::operator+(const vect3& phi) const
{
	quat qtmp = rv2q(-phi);
	return qtmp * (*this);
}

/* subtract misalignment vector -----------------------------------------------
* subtract a misalignment vector from the quaternion (calculated quaternion delete misalign angles)
* args   : const vect3& phi    I   misalignment vector
* return : resulting quaternion
*-----------------------------------------------------------------------------*/
quat quat::operator-(const vect3& phi) const
{
	quat qtmp = rv2q(phi);
	return qtmp * (*this);
}

/* get misalignment angles ----------------------------------------------------
* get misalignment angles from calculated quaternion and true quaternion
* args   : const quat& quat    I   true quaternion
* return : misalignment vector
*-----------------------------------------------------------------------------*/
vect3 quat::operator-(const quat& quat0) const
{
	quat dq;

	dq = quat0 * (~(*this));
	if (dq.q0 < 0)
	{
		dq.q0 = -dq.q0, dq.q1 = -dq.q1, dq.q2 = -dq.q2, dq.q3 = -dq.q3;
	}
	double n2 = acos(dq.q0), f;
	if (sign(n2) != 0)
	{
		f = 2.0 / (sin(n2) / n2);
	}
	else
	{
		f = 2.0;
	}
	return vect3(dq.q1, dq.q2, dq.q3) * f;
}

/* quaternion multiplication --------------------------------------------------
* multiply two quaternions
* args   : const quat& q       I   quaternion to multiply
* return : resulting quaternion
*-----------------------------------------------------------------------------*/
quat quat::operator*(const quat& quat0) const
{
	quat qtmp;
	qtmp.q0 = q0 * quat0.q0 - q1 * quat0.q1 - q2 * quat0.q2 - q3 * quat0.q3;
	qtmp.q1 = q0 * quat0.q1 + q1 * quat0.q0 + q2 * quat0.q3 - q3 * quat0.q2;
	qtmp.q2 = q0 * quat0.q2 + q2 * quat0.q0 + q3 * quat0.q1 - q1 * quat0.q3;
	qtmp.q3 = q0 * quat0.q3 + q3 * quat0.q0 + q1 * quat0.q2 - q2 * quat0.q1;
	return qtmp;
}

/* quaternion multiplication assignment ---------------------------------------
* multiply this quaternion by another quaternion
* args   : const quat& q       I   quaternion to multiply
* return : reference to this quaternion
*-----------------------------------------------------------------------------*/
quat& quat::operator*=(const quat& quat0)
{
	return (*this = *this * quat0);
}

/* subtract misalignment vector assignment ------------------------------------
* subtract a misalignment vector from this quaternion
* args   : const vect3& phi    I   misalignment vector
* return : reference to this quaternion
*-----------------------------------------------------------------------------*/
quat& quat::operator-=(const vect3& phi)
{
	quat qtmp = rv2q(phi);
	return (*this = qtmp * (*this));
}

/* quaternion conjugate -------------------------------------------------------
* compute the conjugate of the quaternion
* args   : const quat& q       I   input quaternion
* return : conjugated quaternion
*-----------------------------------------------------------------------------*/
quat operator~(const quat& q)
{
	return quat(q.q0, -q.q1, -q.q2, -q.q3);
}

/* quaternion-vector multiplication -------------------------------------------
* rotate a vector by the quaternion
* args   : const vect3& v      I   vector to rotate
* return : rotated vector
*-----------------------------------------------------------------------------*/
vect3 quat::operator*(const vect3& v) const
{
	quat qtmp;
	vect3 vtmp;
	qtmp.q0 = -q1 * v.i - q2 * v.j - q3 * v.k;
	qtmp.q1 = q0 * v.i + q2 * v.k - q3 * v.j;
	qtmp.q2 = q0 * v.j + q3 * v.i - q1 * v.k;
	qtmp.q3 = q0 * v.k + q1 * v.j - q2 * v.i;
	vtmp.i = -qtmp.q0 * q1 + qtmp.q1 * q0 - qtmp.q2 * q3 + qtmp.q3 * q2;
	vtmp.j = -qtmp.q0 * q2 + qtmp.q2 * q0 - qtmp.q3 * q1 + qtmp.q1 * q3;
	vtmp.k = -qtmp.q0 * q3 + qtmp.q3 * q0 - qtmp.q1 * q2 + qtmp.q2 * q1;
	return vtmp;
}

/* convert quaternion to Euler angles ------------------------------------------
* convert a quaternion to a set of Euler angles
* args   : quat       qnb      I   quaternion
* return : Euler angles {roll, pitch, yaw} (radians)
*-----------------------------------------------------------------------------*/
vect3 q2att(const quat& qnb)
{
	double	q11 = qnb.q0 * qnb.q0, q12 = qnb.q0 * qnb.q1, q13 = qnb.q0 * qnb.q2, q14 = qnb.q0 * qnb.q3,
		q22 = qnb.q1 * qnb.q1, q23 = qnb.q1 * qnb.q2, q24 = qnb.q1 * qnb.q3,
		q33 = qnb.q2 * qnb.q2, q34 = qnb.q2 * qnb.q3,
		q44 = qnb.q3 * qnb.q3;
	vect3 att;
	att.i = asinEx(2 * (q34 + q12));
	att.j = atan2Ex(-2 * (q24 - q13), q11 - q22 - q33 + q44);
	att.k = atan2Ex(-2 * (q23 - q14), q11 - q22 + q33 - q44);
	return att;
}

/* convert Euler angles to quaternion ------------------------------------------
* convert a set of Euler angles to a quaternion
* args   : double     pitch    I   pitch angle (radians)
*          double     roll     I   roll angle (radians)
*          double     yaw      I   yaw angle (radians)
* return : quaternion
*-----------------------------------------------------------------------------*/
quat a2qua(double pitch, double roll, double yaw)
{
	pitch /= 2.0, roll /= 2.0, yaw /= 2.0;
	double	sp = sin(pitch), sr = sin(roll), sy = sin(yaw),
		cp = cos(pitch), cr = cos(roll), cy = cos(yaw);
	quat qnb;
	qnb.q0 = cp * cr * cy - sp * sr * sy;
	qnb.q1 = sp * cr * cy - cp * sr * sy;
	qnb.q2 = cp * sr * cy + sp * cr * sy;
	qnb.q3 = cp * cr * sy + sp * sr * cy;
	return qnb;
}

/* convert Euler angles to quaternion ------------------------------------------
* convert a set of Euler angles to a quaternion
* args   : vect3      att      I   Euler angles {roll, pitch, yaw} (radians)
* return : quaternion
*-----------------------------------------------------------------------------*/
quat a2qua(const vect3& att)
{
	return a2qua(att.i, att.j, att.k);
}

/* set yaw angle --------------------------------------------------------------
* set the yaw (heading) angle of the quaternion, keeping roll and pitch unchanged
* args   : double     yaw      I   yaw angle (radians, default: 0.0)
* return : none
*-----------------------------------------------------------------------------*/
void quat::SetYaw(double yaw)
{
	vect3 att = q2att(*this);
	att.k = yaw;
	*this = a2qua(att);
}

/* normalize quaternion -------------------------------------------------------
* normalize the quaternion to unit length
* args   : quat*      q        O   pointer to output normalized quaternion (optional)
* return : none
*-----------------------------------------------------------------------------*/
void normlize(quat* q)
{
	double nq = sqrt(q->q0 * q->q0 + q->q1 * q->q1 + q->q2 * q->q2 + q->q3 * q->q3);
	q->q0 /= nq, q->q1 /= nq, q->q2 /= nq, q->q3 /= nq;
}

/* convert quaternion to rotation vector ---------------------------------------
* convert a quaternion to a rotation vector
* args   : quat       q        I   quaternion
* return : rotation vector
*-----------------------------------------------------------------------------*/
vect3 q2rv(const quat& q)
{
	quat dq;
	dq = q;
	if (dq.q0 < 0) { dq.q0 = -dq.q0, dq.q1 = -dq.q1, dq.q2 = -dq.q2, dq.q3 = -dq.q3; }
	if (dq.q0 > 1.0) dq.q0 = 1.0;
	double n2 = acos(dq.q0), f;
	if (n2 > 1.0e-20)
	{
		f = 2.0 / (sin(n2) / n2);
	}
	else
	{
		f = 2.0;
	}
	return vect3(dq.q1, dq.q2, dq.q3) * f;
}

/* calculate quaternion difference --------------------------------------------
* calculate the difference between two quaternions
* args   : quat       qcalcu   I   calculated quaternion
*          quat       qreal    I   real quaternion
* return : rotation vector representing the difference
*-----------------------------------------------------------------------------*/
vect3 qq2phi(const quat& qcalcu, const quat& qreal)
{
	return q2rv(qreal * (~qcalcu));
}

/* add misalignment vector to quaternion ---------------------------------------
* add a misalignment vector to a quaternion
* args   : quat       q        I   input quaternion
*          vect3      mu       I   misalignment vector
* return : resulting quaternion
*-----------------------------------------------------------------------------*/
quat addmu(const quat& q, const vect3& mu)
{
	return q * rv2q(mu);
}

/* flip quaternion representation ----------------------------------------------
* flip the quaternion representation of an attitude
* args   : quat       q        I   input quaternion
* return : flipped quaternion
*-----------------------------------------------------------------------------*/
quat UpDown(const quat& q)
{
	vect3 att = q2att(q);
	att.i = -att.i; att.j += PI;
	return a2qua(att);
}

//***************************  class mat3  *********************************/
/* default constructor --------------------------------------------------------
* initialize all elements of the matrix to zero
* args   : none
* return : none
*-----------------------------------------------------------------------------*/
mat3::mat3(void)
{
}

/* diagonal initialization constructor ----------------------------------------
* initialize the matrix as a diagonal matrix with the given value
* args   : double     xyz      I   value for diagonal elements
* return : none
*-----------------------------------------------------------------------------*/
mat3::mat3(double xyz)
{
	e00 = e01 = e02 = e10 = e11 = e12 = e20 = e21 = e22 = xyz;
}

/* array initialization constructor (double) ----------------------------------
* initialize the matrix from a double array (row-major order)
* args   : const double* pxyz  I   pointer to 9 double values
* return : none
*-----------------------------------------------------------------------------*/
mat3::mat3(const double* pxyz)
{
	e00 = *pxyz++, e01 = *pxyz++, e02 = *pxyz++,
		e10 = *pxyz++, e11 = *pxyz++, e12 = *pxyz++,
		e20 = *pxyz++, e21 = *pxyz++, e22 = *pxyz;
}

/* array initialization constructor (float) -----------------------------------
* initialize the matrix from a float array (row-major order)
* args   : const float* pxyz   I   pointer to 9 float values
* return : none
*-----------------------------------------------------------------------------*/
mat3::mat3(const float* pxyz)
{
	e00 = *pxyz++, e01 = *pxyz++, e02 = *pxyz++,
		e10 = *pxyz++, e11 = *pxyz++, e12 = *pxyz++,
		e20 = *pxyz++, e21 = *pxyz++, e22 = *pxyz;
}

/* diagonal elements constructor ----------------------------------------------
* initialize the matrix as a diagonal matrix with specified diagonal values
* args   : double     xx       I   value for (0,0)
*          double     yy       I   value for (1,1)
*          double     zz       I   value for (2,2)
* return : none
*-----------------------------------------------------------------------------*/
mat3::mat3(double xx, double yy, double zz)
{
	e00 = xx, e11 = yy, e22 = zz;
	e01 = e02 = e10 = e12 = e20 = e21 = 0.0;
}

/* full elements constructor --------------------------------------------------
* initialize the matrix with all 9 elements specified (row-major order)
* args   : double     xx,xy,xz I   first row elements
*          double     yx,yy,yz I   second row elements
*          double     zx,zy,zz I   third row elements
* return : none
*-----------------------------------------------------------------------------*/
mat3::mat3(double xx, double xy, double xz,
	double yx, double yy, double yz,
	double zx, double zy, double zz)
{
	e00 = xx, e01 = xy, e02 = xz; e10 = yx, e11 = yy, e12 = yz; e20 = zx, e21 = zy, e22 = zz;
}

/* vector rows/columns constructor --------------------------------------------
* initialize the matrix from three vectors as rows or columns
* args   : vect3      v0       I   first vector
*          vect3      v1       I   second vector
*          vect3      v2       I   third vector
*          bool       isrow    I   true: use as rows; false: use as columns (default: true)
* return : none
*-----------------------------------------------------------------------------*/
mat3::mat3(const vect3& v0, const vect3& v1, const vect3& v2, bool isrow)
{
	if (isrow) {
		e00 = v0.i, e01 = v0.j, e02 = v0.k;
		e10 = v1.i, e11 = v1.j, e12 = v1.k;
		e20 = v2.i, e21 = v2.j, e22 = v2.k;
	}
	else {
		e00 = v0.i, e01 = v1.i, e02 = v2.i;
		e10 = v0.j, e11 = v1.j, e12 = v2.j;
		e20 = v0.k, e21 = v1.k, e22 = v2.k;
	}
}

/* determine level attitude using single vector --------------------------------
* determine the level attitude using a single vector
* args   : vect3      fb       I   body-frame vector
*          double     yaw0     I   initial yaw angle (radians)
*          vect3      fn       I   navigation-frame vector
* return : level attitude {roll, pitch, yaw} (radians)
*-----------------------------------------------------------------------------*/
vect3 sv2att(const vect3& fb, double yaw0, const vect3& fn)
{
	vect3 phi = fb * fn;
	double afa = acos(dot(fn, fb) / norm(fn) / norm(fb)), nphi = norm(phi);
	vect3 att = q2att(rv2q(phi * (nphi < 1.0e-10 ? 1.0 : afa / nphi)));   att.k = yaw0;
	return att;  // return C^n_b
}

/* convert Direction Cosine Matrix (DCM) to Euler angles -----------------------
* convert a Direction Cosine Matrix (DCM) to a set of Euler angles
* args   : mat3       Cnb      I   Direction Cosine Matrix (DCM)
* return : Euler angles {roll, pitch, yaw} (radians)
*-----------------------------------------------------------------------------*/
vect3 m2att(const mat3& Cnb)
{
	vect3 att;
	att.i = asinEx(Cnb.e21);
	att.j = atan2Ex(-Cnb.e20, Cnb.e22);
	att.k = atan2Ex(-Cnb.e01, Cnb.e11);
	return att;
}

/* determine attitude using double vectors -------------------------------------
* determine the attitude using two pairs of vectors
* args   : vect3      va1      I   first vector in frame A
*          vect3      va2      I   second vector in frame A
*          vect3      vb1      I   first vector in frame B
*          vect3      vb2      I   second vector in frame B
* return : attitude {roll, pitch, yaw} (radians)
*-----------------------------------------------------------------------------*/
vect3 dv2att(const vect3& va1, const vect3& va2, const vect3& vb1, const vect3& vb2)
{
	vect3 a = va1 * va2, b = vb1 * vb2, aa = a * va1, bb = b * vb1;
	if (IsZeros(va1) || IsZeros(a) || IsZeros(aa) || IsZeros(vb1) || IsZeros(b) || IsZeros(bb)) return O31;
	mat3 Ma(va1 / norm(va1), a / norm(a), aa / norm(aa)), Mb(vb1 / norm(vb1), b / norm(b), bb / norm(bb));
	return m2att((~Ma) * (Mb));  // return C^a_b -> att
}


/* convert ENU velocity to attitude --------------------------------------------
* convert East-North-Up (ENU) velocity to attitude (pitch and yaw)
* args   : vect3      vn       I   ENU velocity vector
* return : attitude {pitch, yaw} (radians)
*-----------------------------------------------------------------------------*/
vect3 vn2att(const vect3& vn)
{
	double vel = normXY(vn);
	if (vel < 1.0e-6) return O31;
	return vect3(atan2(vn.k, vel), 0, atan2(-vn.i, vn.j));
}

/* convert ENU velocity components to attitude ---------------------------------
* convert East and North velocity components to attitude (pitch and yaw)
* args   : double     vel_east I   East velocity component
*          double     vel_north I  North velocity component
* return : attitude {pitch, yaw} (radians)
*-----------------------------------------------------------------------------*/
double vn2att(const double vel_east, const double vel_north) {
	double vel = vel_east * vel_east + vel_north * vel_north;
	if (vel < 3) return 0;
	return atan2(-vel_east, vel_north);
}

/* matrix subtraction ---------------------------------------------------------
* subtract two matrices element-wise
* args   : const mat3& m       I   matrix to subtract
* return : result of subtraction
*-----------------------------------------------------------------------------*/
mat3 operator-(const mat3& m)
{
	return mat3(-m.e00, -m.e01, -m.e02, -m.e10, -m.e11, -m.e12, -m.e20, -m.e21, -m.e22);
}

/* matrix transposition -------------------------------------------------------
* transpose the matrix
* args   : const mat3& m      I   matrix to transpose
* return : transposed matrix
*-----------------------------------------------------------------------------*/
mat3 operator~(const mat3& m)
{
	return mat3(m.e00, m.e10, m.e20, m.e01, m.e11, m.e21, m.e02, m.e12, m.e22);
}

/* unary minus (negation) -----------------------------------------------------
* negate all elements of the matrix
* args   : const mat3& m      I   matrix to negate
* return : negated matrix
*-----------------------------------------------------------------------------*/
mat3 mat3::operator*(const mat3& mat) const
{
	mat3 mtmp;
	mtmp.e00 = e00 * mat.e00 + e01 * mat.e10 + e02 * mat.e20;
	mtmp.e01 = e00 * mat.e01 + e01 * mat.e11 + e02 * mat.e21;
	mtmp.e02 = e00 * mat.e02 + e01 * mat.e12 + e02 * mat.e22;
	mtmp.e10 = e10 * mat.e00 + e11 * mat.e10 + e12 * mat.e20;
	mtmp.e11 = e10 * mat.e01 + e11 * mat.e11 + e12 * mat.e21;
	mtmp.e12 = e10 * mat.e02 + e11 * mat.e12 + e12 * mat.e22;
	mtmp.e20 = e20 * mat.e00 + e21 * mat.e10 + e22 * mat.e20;
	mtmp.e21 = e20 * mat.e01 + e21 * mat.e11 + e22 * mat.e21;
	mtmp.e22 = e20 * mat.e02 + e21 * mat.e12 + e22 * mat.e22;
	return mtmp;
}

/* matrix addition ------------------------------------------------------------
* add two matrices element-wise
* args   : const mat3& m       I   matrix to add
* return : result of addition
*-----------------------------------------------------------------------------*/
mat3 mat3::operator+(const mat3& mat) const
{
	mat3 mtmp;
	mtmp.e00 = e00 + mat.e00;  mtmp.e01 = e01 + mat.e01;  mtmp.e02 = e02 + mat.e02;
	mtmp.e10 = e10 + mat.e10;  mtmp.e11 = e11 + mat.e11;  mtmp.e12 = e12 + mat.e12;
	mtmp.e20 = e20 + mat.e20;  mtmp.e21 = e21 + mat.e21;  mtmp.e22 = e22 + mat.e22;
	return mtmp;
}

/* matrix addition assignment -------------------------------------------------
* add another matrix to this matrix (element-wise)
* args   : const mat3& m       I   matrix to add
* return : reference to this matrix
*-----------------------------------------------------------------------------*/
mat3& mat3::operator+=(const mat3& mat)
{
	this->e00 += mat.e00;  this->e01 += mat.e01;  this->e02 += mat.e02;
	this->e10 += mat.e10;  this->e11 += mat.e11;  this->e12 += mat.e12;
	this->e20 += mat.e20;  this->e21 += mat.e21;  this->e22 += mat.e22;
	return *this;
}

/* matrix + diag(vector) ------------------------------------------------------
* add a vector to the diagonal elements of the matrix
* args   : const vect3& v      I   vector to add to diagonal
* return : resulting matrix
*-----------------------------------------------------------------------------*/
mat3 mat3::operator+(const vect3& v) const
{
	mat3 mtmp = *this;
	mtmp.e00 += v.i;  mtmp.e11 += v.j;  mtmp.e22 += v.k;
	return mtmp;
}

/* matrix + diag(vector) assignment -------------------------------------------
* add a vector to the diagonal elements of this matrix
* args   : const vect3& v      I   vector to add to diagonal
* return : reference to this matrix
*-----------------------------------------------------------------------------*/
mat3& mat3::operator+=(const vect3& v)
{
	this->e00 += v.i;  this->e11 += v.j;  this->e22 += v.k;
	return *this;
}

/* matrix subtraction ---------------------------------------------------------
* subtract two matrices element-wise
* args   : const mat3& m       I   matrix to subtract
* return : result of subtraction
*-----------------------------------------------------------------------------*/
mat3 mat3::operator-(const mat3& mat) const
{
	mat3 mtmp;
	mtmp.e00 = e00 - mat.e00;  mtmp.e01 = e01 - mat.e01;  mtmp.e02 = e02 - mat.e02;
	mtmp.e10 = e10 - mat.e10;  mtmp.e11 = e11 - mat.e11;  mtmp.e12 = e12 - mat.e12;
	mtmp.e20 = e20 - mat.e20;  mtmp.e21 = e21 - mat.e21;  mtmp.e22 = e22 - mat.e22;
	return mtmp;
}

/* matrix scaling -------------------------------------------------------------
* multiply all elements by a scalar
* args   : double     f        I   scale factor
* return : scaled matrix
*-----------------------------------------------------------------------------*/
mat3 mat3::operator*(double f) const
{
	return mat3(e00 * f, e01 * f, e02 * f, e10 * f, e11 * f, e12 * f, e20 * f, e21 * f, e22 * f);
}


/* set row from vector --------------------------------------------------------
* set the i-th row of the matrix from a vector
* args   : int        i        I   row index (0~2)
*          vect3      v        I   vector to set
* return : none
*-----------------------------------------------------------------------------*/
void mat3::SetRow(int i, const vect3& v)
{
	double* p = &e00 + i * 3;
	*p = v.i, * (p + 1) = v.j, * (p + 2) = v.k;
}

/* set column from vector -----------------------------------------------------
* set the i-th column of the matrix from a vector
* args   : int        i        I   column index (0~2)
*          vect3      v        I   vector to set
* return : none
*-----------------------------------------------------------------------------*/
void mat3::SetClm(int i, const vect3& v)
{
	double* p = &e00 + i;
	*p = v.i, * (p + 3) = v.j, * (p + 6) = v.k;
}

/* get row as vector ----------------------------------------------------------
* get the i-th row of the matrix as a vector
* args   : int        i        I   row index (0~2)
* return : row vector
*-----------------------------------------------------------------------------*/
vect3 mat3::GetRow(int i) const
{
	const double* p = &e00 + i * 3;
	return vect3(*p, *(p + 1), *(p + 2));
}

/* get column as vector -------------------------------------------------------
* get the i-th column of the matrix as a vector
* args   : int        i        I   column index (0~2)
* return : column vector
*-----------------------------------------------------------------------------*/
vect3 mat3::GetClm(int i) const
{
	const double* p = &e00 + i;
	return vect3(*p, *(p + 3), *(p + 6));
}

/* rotation by axis -----------------------------------------------------------
* generate a rotation matrix for rotation about x/y/z axis by angle
* args   : double     angle    I   rotation angle (radians)
*          char       axis     I   axis ('x','y','z')
* return : rotation matrix
*-----------------------------------------------------------------------------*/
mat3 Rot(double angle, char axis)
{
	double s = sin(angle), c = cos(angle);
	if (axis == 'x' || axis == 'X')		return mat3(1, 0, 0, 0, c, -s, 0, s, c);
	else if (axis == 'y' || axis == 'Y')	return mat3(c, 0, s, 0, 1, 0, -s, 0, c);
	else							return mat3(c, -s, 0, s, c, 0, 0, 0, 1);
}

/* re-arrange row/column by index ---------------------------------------------
* re-arrange the matrix rows/columns according to ijk index
* args   : const mat3& m      I   input matrix
*          int        ijk     I   index for re-arrangement
* return : rearranged matrix
*-----------------------------------------------------------------------------*/
mat3 rcijk(const mat3& m, int ijk)
{
	switch (ijk)
	{
		//	case 012: return m; break;
	case 021: return mat3(m.e00, m.e02, m.e01, m.e20, m.e22, m.e21, m.e10, m.e12, m.e11);
	case 102: return mat3(m.e11, m.e10, m.e12, m.e01, m.e00, m.e02, m.e21, m.e20, m.e22);
	case 120: return mat3(m.e11, m.e12, m.e10, m.e21, m.e22, m.e20, m.e01, m.e02, m.e00);
	case 201: return mat3(m.e22, m.e20, m.e21, m.e02, m.e00, m.e01, m.e12, m.e10, m.e11);
	case 210: return mat3(m.e22, m.e21, m.e20, m.e12, m.e11, m.e10, m.e02, m.e01, m.e00);
	}
	return m;
}

/* calculate trace of matrix multiplication -----------------------------------
* calculate the trace of the product of two matrices
* args   : mat3      m1       I   first input matrix
*          mat3      m2       I   second input matrix (default: identity matrix)
* return : trace of the product of m1 and m2
*-----------------------------------------------------------------------------*/
double trMMT(const mat3& m1, const mat3& m2)
{
	const mat3* pm2;
	pm2 = (&m2 == &I33) ? &m1 : &m2;
	return	m1.e00 * pm2->e00 + m1.e01 * pm2->e01 + m1.e02 * pm2->e02 +
		m1.e10 * pm2->e10 + m1.e11 * pm2->e11 + m1.e12 * pm2->e12 +
		m1.e20 * pm2->e20 + m1.e21 * pm2->e21 + m1.e22 * pm2->e22;
}

/* symmetrize a matrix --------------------------------------------------------
* symmetrize a given matrix
* args   : mat3      m        IO  input/output matrix to be symmetrized
* return : none
*-----------------------------------------------------------------------------*/
void symmetry(mat3& m)
{
	m.e01 = m.e10 = (m.e01 + m.e10) * 0.5;
	m.e02 = m.e20 = (m.e02 + m.e20) * 0.5;
	m.e12 = m.e21 = (m.e12 + m.e21) * 0.5;
}

/* matrix-vector multiplication -----------------------------------------------
* multiply matrix by a vector
* args   : const vect3& v      I   vector to multiply
* return : resulting vector
*-----------------------------------------------------------------------------*/
mat3 operator*(double f, const mat3& m)
{
	return mat3(m.e00 * f, m.e01 * f, m.e02 * f, m.e10 * f, m.e11 * f, m.e12 * f, m.e20 * f, m.e21 * f, m.e22 * f);
}

/* scalar multiplication (from left) ------------------------------------------
* multiply matrix by a scalar (scalar * matrix)
* args   : double     f        I   scale factor
*          const mat3& m      I   matrix to scale
* return : scaled matrix
*-----------------------------------------------------------------------------*/
vect3 mat3::operator*(const vect3& v) const
{
	return vect3(e00 * v.i + e01 * v.j + e02 * v.k, e10 * v.i + e11 * v.j + e12 * v.k, e20 * v.i + e21 * v.j + e22 * v.k);
}

/* calculate determinant of a matrix ------------------------------------------
* calculate the determinant of a matrix
* args   : mat3      m        I   input matrix
* return : determinant of the matrix
*-----------------------------------------------------------------------------*/
double det(const mat3& m)
{
	return m.e00 * (m.e11 * m.e22 - m.e12 * m.e21) - m.e01 * (m.e10 * m.e22 - m.e12 * m.e20) + m.e02 * (m.e10 * m.e21 - m.e11 * m.e20);
}

/* calculate trace of a matrix ------------------------------------------------
* calculate the trace of a matrix
* args   : mat3      m        I   input matrix
* return : trace of the matrix
*-----------------------------------------------------------------------------*/
double trace(const mat3& m)
{
	return (m.e00 + m.e11 + m.e22);
}

/* calculate k-th power of a matrix -------------------------------------------
* calculate the k-th power of a matrix
* args   : mat3      m        I   input matrix
*          int       k        I   power to raise the matrix
* return : matrix raised to the k-th power
*-----------------------------------------------------------------------------*/
mat3 pow(const mat3& m, int k)
{
	mat3 mm = m;
	for (int i = 1; i < k; i++)	mm = mm * m;
	return mm;
}

/* convert reversed Euler angles to Direction Cosine Matrix (DCM) --------------
* convert a set of reversed Euler angles to a Direction Cosine Matrix (DCM)
* args   : vect3      attr     I   reversed Euler angles {yaw, pitch, roll} (radians)
* return : Direction Cosine Matrix (DCM)
*-----------------------------------------------------------------------------*/
mat3 ar2mat(const vect3& attr)  // reversed Euler angles to DCM
{
	double	si = sin(attr.i), ci = cos(attr.i),
		sj = sin(attr.j), cj = cos(attr.j),
		sk = sin(attr.k), ck = cos(attr.k);
	mat3 Cnb;
	Cnb.e00 = cj * ck;	Cnb.e01 = si * sj * ck - ci * sk;	Cnb.e02 = ci * sj * ck + si * sk;
	Cnb.e10 = cj * sk;	Cnb.e11 = si * sj * sk + ci * ck;	Cnb.e12 = ci * sj * sk - si * ck;
	Cnb.e20 = -sj;		Cnb.e21 = si * cj;			Cnb.e22 = ci * cj;
	return Cnb;
}

/* convert reversed Euler angles to quaternion ---------------------------------
* convert a set of reversed Euler angles to a quaternion
* args   : vect3      attr     I   reversed Euler angles {yaw, pitch, roll} (radians)
* return : quaternion
*-----------------------------------------------------------------------------*/
quat ar2qua(const vect3& attr)
{
	mat3 Cnb = ar2mat(attr);
	return m2qua(Cnb);
}

/* convert Direction Cosine Matrix (DCM) to reversed Euler angles --------------
* convert a Direction Cosine Matrix (DCM) to reversed Euler angles (3-2-1 sequence)
* args   : mat3       Cnb      I   Direction Cosine Matrix (DCM)
* return : reversed Euler angles {yaw, pitch, roll} (radians)
*-----------------------------------------------------------------------------*/
vect3 m2attr(const mat3& Cnb)
{
	vect3 attr;
	attr.i = atan2Ex(Cnb.e21, Cnb.e22);
	attr.j = asinEx(-Cnb.e20);
	attr.k = atan2Ex(Cnb.e10, Cnb.e00);
	return attr;
}

/* convert quaternion to reversed Euler angles ---------------------------------
* convert a quaternion to reversed Euler angles (3-2-1 sequence)
* args   : quat       qnb      I   quaternion
* return : reversed Euler angles {yaw, pitch, roll} (radians)
*-----------------------------------------------------------------------------*/
vect3 q2attr(const quat& qnb)
{
	return m2attr(q2mat(qnb));
}


/* convert Direction Cosine Matrix (DCM) to rotation vector -------------------
* convert a Direction Cosine Matrix (DCM) to a rotation vector
* args   : mat3      Cnb      I   Direction Cosine Matrix (DCM)
* return : rotation vector
*-----------------------------------------------------------------------------*/
vect3 m2rv(const mat3& Cnb)
{
	double phi = acos((trace(Cnb) - 1.0) / 2), afa;
	afa = (-1.0e-10 < phi && phi < 1.0e-10) ? 1.0 / 2 : phi / (2 * sin(phi));
	return vect3(Cnb.e21 - Cnb.e12, Cnb.e02 - Cnb.e20, Cnb.e10 - Cnb.e01) * afa;
}

/* calculate adjoint of a matrix ----------------------------------------------
* calculate the adjoint of a 3x3 matrix
* args   : mat3      m        I   input matrix
* return : adjoint of the matrix
*-----------------------------------------------------------------------------*/
mat3 adj(const mat3& m)
{
	mat3 mtmp;
	mtmp.e00 = (m.e11 * m.e22 - m.e12 * m.e21);
	mtmp.e10 = -(m.e10 * m.e22 - m.e12 * m.e20);
	mtmp.e20 = (m.e10 * m.e21 - m.e11 * m.e20);
	mtmp.e01 = -(m.e01 * m.e22 - m.e02 * m.e21);
	mtmp.e11 = (m.e00 * m.e22 - m.e02 * m.e20);
	mtmp.e21 = -(m.e00 * m.e21 - m.e01 * m.e20);
	mtmp.e02 = (m.e01 * m.e12 - m.e02 * m.e11);
	mtmp.e12 = -(m.e00 * m.e12 - m.e02 * m.e10);
	mtmp.e22 = (m.e00 * m.e11 - m.e01 * m.e10);
	return mtmp;
}

/* calculate inverse of a matrix ----------------------------------------------
* calculate the inverse of a 3x3 matrix
* args   : mat3      m        I   input matrix
* return : inverse of the matrix
*-----------------------------------------------------------------------------*/
mat3 inv(const mat3& m)
{
	mat3 adjm = adj(m);
	double detm = m.e00 * adjm.e00 + m.e01 * adjm.e10 + m.e02 * adjm.e20;
	return adjm * (1.0 / detm);
}

/* extract diagonal of a matrix -----------------------------------------------
* extract the diagonal elements of a matrix as a vector
* args   : mat3      m        I   input matrix
* return : vector containing the diagonal elements of the matrix
*-----------------------------------------------------------------------------*/
vect3 diag(const mat3& m)
{
	return vect3(m.e00, m.e11, m.e22);
}

/* create diagonal matrix from a vector ---------------------------------------
* create a diagonal matrix from a vector
* args   : vect3     v        I   input vector
* return : diagonal matrix
*-----------------------------------------------------------------------------*/
mat3 diag(const vect3& v)
{
	return mat3(v.i, 0, 0, 0, v.j, 0, 0, 0, v.k);
}

/* calculate askew-symmetric matrix -------------------------------------------
* calculate the askew-symmetric matrix of a matrix
* args   : mat3      m        I   input matrix
*          int       I        I   index for askew operation
* return : askew-symmetric matrix
*-----------------------------------------------------------------------------*/
mat3 askew(const mat3& m, int I)
{
	mat3 m1;
	m1.e01 = (m.e01 - m.e10) / 2;  m1.e02 = (m.e02 - m.e20) / 2;  m1.e12 = (m.e12 - m.e21) / 2;
	m1.e10 = -m1.e01; m1.e20 = -m1.e02; m1.e21 = -m1.e12;
	if (I == 0)  m1.e00 = m1.e11 = m1.e22 = 0.0;
	else if (I == 1)  m1.e00 = m1.e11 = m1.e22 = 1.0;
	else { m1.e00 = m.e00, m1.e11 = m.e11, m1.e22 = m.e22; }
	return m1;
}

/* calculate matrix multiplication with transpose -----------------------------
* calculate the product of a matrix and the transpose of another matrix
* args   : mat3      m1       I   first input matrix
*          mat3      m2       I   second input matrix
* return : product of m1 and the transpose of m2
*-----------------------------------------------------------------------------*/
mat3 MMT(const mat3& m1, const mat3& m2)
{
	mat3 mtmp; const mat3* pm2;
	pm2 = (&m2 == &I33) ? &m1 : &m2;
	mtmp.e00 = m1.e00 * pm2->e00 + m1.e01 * pm2->e01 + m1.e02 * pm2->e02;
	mtmp.e01 = m1.e00 * pm2->e10 + m1.e01 * pm2->e11 + m1.e02 * pm2->e12;
	mtmp.e02 = m1.e00 * pm2->e20 + m1.e01 * pm2->e21 + m1.e02 * pm2->e22;
	mtmp.e11 = m1.e10 * pm2->e10 + m1.e11 * pm2->e11 + m1.e12 * pm2->e12;
	mtmp.e12 = m1.e10 * pm2->e20 + m1.e11 * pm2->e21 + m1.e12 * pm2->e22;
	mtmp.e22 = m1.e20 * pm2->e20 + m1.e21 * pm2->e21 + m1.e22 * pm2->e22;
	mtmp.e10 = mtmp.e01;
	mtmp.e20 = mtmp.e02;
	mtmp.e21 = mtmp.e12;
	return mtmp;
}

/* calculate norm of a matrix -------------------------------------------------
* calculate the norm of a matrix
* args   : mat3      m        I   input matrix
* return : norm of the matrix
*-----------------------------------------------------------------------------*/
double norm(const mat3& m)
{
	return sqrt(trMMT(m));
}


//***************************  class mat  *********************************/
/* Default constructor for mat class -------------------------------------------
* Initializes an empty matrix object with default values.
*-----------------------------------------------------------------------------*/
mat::mat(void)
{

}

/* Constructor for mat class with specified size -------------------------------
* Initializes a matrix with the given number of rows and columns.
* args   : int row0         I   number of rows
*          int clm0         I   number of columns
*-----------------------------------------------------------------------------*/
mat::mat(int row0, int clm0)
{
	row = row0; clm = clm0; rc = row * clm;
}

/* Constructor for mat class with specified size and fill value ----------------
* Initializes a matrix with the given number of rows and columns, and fills all elements with the specified value.
* args   : int row0         I   number of rows
*          int clm0         I   number of columns
*          double f         I   fill value for all elements
*-----------------------------------------------------------------------------*/
mat::mat(int row0, int clm0, double f)
{
	row = row0; clm = clm0; rc = row * clm;
	for (double* pd = dd, *pEnd = &dd[rc]; pd < pEnd; pd++)  *pd = f;
}

/* Constructor for mat class with variadic fill values -------------------------
* Initializes a matrix with the given number of rows and columns, and fills elements with the provided values.
* args   : int row0         I   number of rows
*          int clm0         I   number of columns
*          double f         I   first fill value
*          double f1, ...   I   additional fill values
*-----------------------------------------------------------------------------*/
mat::mat(int row0, int clm0, double f, double f1, ...)
{
	row = row0; clm = clm0; rc = row * clm;
	va_list vl;
	va_start(vl, f);
	for (int i = 0; i < rc; i++)
	{
		if (f > 2 * INF) break;  dd[i] = f;  f = va_arg(vl, double);
	}
	va_end(vl);
}

/* Constructor for mat class from array ----------------------------------------
* Initializes a matrix with the given number of rows and columns, copying data from the provided array.
* args   : int row0         I   number of rows
*          int clm0         I   number of columns
*          const double *pf I   pointer to array of values (size row0*clm0)
*-----------------------------------------------------------------------------*/
mat::mat(int row0, int clm0, const double* pf)
{
	row = row0; clm = clm0; rc = row * clm;
	memcpy(dd, pf, rc * sizeof(double));
}

/* Clear all elements of the matrix --------------------------------------------
* Sets all elements of the matrix to zero.
*-----------------------------------------------------------------------------*/
void mat::Clear(void)
{
	for (double* p = dd, *pEnd = &dd[rc]; p < pEnd; p++)	*p = 0.0;
}

/* Matrix multiplication operator ----------------------------------------------
* Multiplies this matrix by another matrix and returns the result.
* args   : const mat &m0    I   right-hand side matrix
* return : mat                   result of multiplication
*-----------------------------------------------------------------------------*/
mat mat::operator*(const mat& m0) const
{
	assert(this->clm == m0.row);
	mat mtmp(this->row, m0.clm);
	int m = this->row, k = this->clm, n = m0.clm;
	double* p = mtmp.dd; const double* p1i = this->dd, * p2 = m0.dd;
	for (register int i = 0; i < m; i++, p1i += k)
	{
		for (register int j = 0; j < n; j++)
		{
			double f = 0.0; const double* p1is = p1i, * p1isEnd = &p1i[k], * p2sj = &p2[j];
			for (; p1is < p1isEnd; p1is++, p2sj += n)
				f += (*p1is) * (*p2sj);
			*p++ = f;
		}
	}
	return mtmp;
}

/* Matrix-vector multiplication operator ---------------------------------------
* Multiplies this matrix by a vector and returns the result.
* args   : const vect &v    I   right-hand side vector
* return : vect                  result of multiplication
*-----------------------------------------------------------------------------*/
vect mat::operator*(const vect& v) const
{
	assert(this->clm == v.row);
	vect vtmp(this->row);
	double* p = vtmp.dd, * pEnd = &vtmp.dd[vtmp.row]; const double* p1ij = this->dd, * p2End = &v.dd[v.row];
	for (; p < pEnd; p++)
	{
		double f = 0.0; const double* p2j = v.dd;
		for (; p2j < p2End; p1ij++, p2j++)	f += (*p1ij) * (*p2j);
		*p = f;
	}
	return vtmp;
}

/* Matrix addition operator ----------------------------------------------------
* Adds this matrix to another matrix and returns the result.
* args   : const mat &m0    I   right-hand side matrix
* return : mat                   result of addition
*-----------------------------------------------------------------------------*/
mat mat::operator+(const mat& m0) const
{
	assert(row == m0.row && clm == m0.clm);
	mat mtmp(row, clm);
	double* p = mtmp.dd, * pEnd = &mtmp.dd[rc]; const double* p1 = this->dd, * p2 = m0.dd;
	while (p < pEnd)
	{
		*p++ = (*p1++) + (*p2++);
	}
	return mtmp;
}

/* Matrix and vector addition assignment operator ------------------------------
* Adds a vector to the matrix (element-wise) and assigns the result to this matrix.
* args   : const vect &v    I   vector to add
* return : mat&                  reference to this matrix
*-----------------------------------------------------------------------------*/
mat& mat::operator+=(const vect& v)
{
	assert(row == v.row || clm == v.clm);
	int row1 = row + 1;
	double* p = dd, * pEnd = &dd[rc];
	for (const double* p1 = v.dd; p < pEnd; p += row1, p1++)	*p += *p1;
	return *this;
}

/* Matrix subtraction operator -------------------------------------------------
* Subtracts another matrix from this matrix and returns the result.
* args   : const mat &m0    I   right-hand side matrix
* return : mat                   result of subtraction
*-----------------------------------------------------------------------------*/
mat mat::operator-(const mat& m0) const
{
	assert(row == m0.row && clm == m0.clm);
	mat mtmp(row, clm);
	double* p = mtmp.dd, * pEnd = &mtmp.dd[rc]; const double* p1 = this->dd, * p2 = m0.dd;
	while (p < pEnd)
	{
		*p++ = (*p1++) - (*p2++);
	}
	return mtmp;
}

/* Matrix-scalar multiplication operator ---------------------------------------
* Multiplies all elements of the matrix by a scalar and returns the result.
* args   : double f         I   scalar value
* return : mat                   result of multiplication
*-----------------------------------------------------------------------------*/
mat mat::operator*(double f) const
{
	mat mtmp(row, clm);
	double* p = mtmp.dd, * pEnd = &mtmp.dd[rc]; const double* p1 = this->dd;
	while (p < pEnd)
	{
		*p++ = (*p1++) * f;
	}
	return mtmp;
}

/* Matrix assignment operator (scalar) -----------------------------------------
* Assigns a scalar value to all elements of the matrix.
* args   : double f         I   scalar value
* return : mat&                  reference to this matrix
*-----------------------------------------------------------------------------*/
mat& mat::operator=(double f)
{
	for (double* p = dd, *pEnd = &dd[rc]; p < pEnd; p++) { *p = f; }
	return *this;
}

/* Matrix addition assignment operator -----------------------------------------
* Adds another matrix to this matrix (element-wise) and assigns the result.
* args   : const mat &m0    I   matrix to add
* return : mat&                  reference to this matrix
*-----------------------------------------------------------------------------*/
mat& mat::operator+=(const mat& m0)
{
	assert(row == m0.row && clm == m0.clm);
	double* p = dd, * pEnd = &dd[rc]; const double* p1 = m0.dd;
	while (p < pEnd)
	{
		*p++ += *p1++;
	}
	return *this;
}

/* Matrix subtraction assignment operator --------------------------------------
* Subtracts another matrix from this matrix (element-wise) and assigns the result.
* args   : const mat &m0    I   matrix to subtract
* return : mat&                  reference to this matrix
*-----------------------------------------------------------------------------*/
mat& mat::operator-=(const mat& m0)
{
	assert(row == m0.row && clm == m0.clm);
	double* p = dd, * pEnd = &dd[rc]; const double* p1 = m0.dd;
	while (p < pEnd)
	{
		*p++ -= *p1++;
	}
	return *this;
}

/* Matrix-scalar multiplication assignment operator ----------------------------
* Multiplies all elements of the matrix by a scalar and assigns the result.
* args   : double f         I   scalar value
* return : mat&                  reference to this matrix
*-----------------------------------------------------------------------------*/
mat& mat::operator*=(double f)
{
	double* p = dd, * pEnd = &dd[rc];
	while (p < pEnd)
	{
		*p++ *= f;
	}
	return *this;
}

/* Matrix increment operator ---------------------------------------------------
* Increments the diagonal elements of the matrix by 1.
* return : mat&                  reference to this matrix
*-----------------------------------------------------------------------------*/
mat& mat::operator++()
{
	int row1 = row + 1;
	for (double* p = dd, *pEnd = &dd[rc]; p < pEnd; p += row1)	*p += 1.0;
	return *this;
}

/* Matrix transpose operator ---------------------------------------------------
* Returns the transpose of the given matrix.
* args   : const mat &m0    I   matrix to transpose
* return : mat                   transposed matrix
*-----------------------------------------------------------------------------*/
mat operator~(const mat& m0)
{
	mat mtmp(m0.clm, m0.row);
	const double* pm = m0.dd;
	for (register int i = 0; i < m0.row; i++)
	{
		for (register int j = i; j < m0.rc; j += m0.row) mtmp.dd[j] = *pm++;
	}
	return mtmp;
}

/* Make matrix symmetric -------------------------------------------------------
* Makes a square matrix symmetric by averaging corresponding off-diagonal elements.
* args   : mat &m           IO  matrix to be symmetrized
* return : none
*-----------------------------------------------------------------------------*/
void symmetry(mat& m)
{
	assert(m.row == m.clm);
	double* prow0 = &m.dd[1], * prowEnd = &m.dd[m.clm], * pclm0 = &m.dd[m.clm], * pEnd = &m.dd[m.rc];
	for (int clm1 = m.clm + 1; prow0 < pEnd; prow0 += clm1, pclm0 += clm1, prowEnd += m.clm)
	{
		for (double* prow = prow0, *pclm = pclm0; prow < prowEnd; prow++, pclm += m.clm)
			*prow = *pclm = (*prow + *pclm) * 0.5;
	}
}

/* Trace of a square matrix ----------------------------------------------------
* Computes the trace (sum of diagonal elements) of a square matrix.
* args   : const mat &m     I   input matrix (must be square)
* return : double                trace of the matrix
*-----------------------------------------------------------------------------*/
double trace(const mat& m)
{
	assert(m.row == m.clm);
	int row1 = m.row + 1;
	double s = 0.0;
	for (const double* p = m.dd, *pEnd = &m.dd[m.rc]; p < pEnd; p += row1)  s += *p;
	return s;
}

/* Element-wise multiplication of two matrices ---------------------------------
* Computes the element-wise (Hadamard) product of two matrices of the same size.
* args   : const mat &m1    I   first matrix
*          const mat &m2    I   second matrix
* return : mat                   result of element-wise multiplication
*-----------------------------------------------------------------------------*/
mat dotmul(const mat& m1, const mat& m2)
{
	assert(m1.row == m2.row && m1.clm == m2.clm);
	mat res(m1.row, m1.clm);
	double* p = res.dd;
	for (const double* p1 = m1.dd, *p2 = m2.dd, *pEnd = &m1.dd[m1.rc]; p1 < pEnd; p1++, p2++, p++) { *p = (*p1) * (*p2); }
	return res;
}

/* Matrix element access operator ----------------------------------------------
* Provides access to the element at row r and column c.
* args   : int r            I   row index
*          int c            I   column index
* return : double&               reference to the element
*-----------------------------------------------------------------------------*/
double& mat::operator()(int r, int c)
{
	if (c < 0) c = r;
	return this->dd[r * this->clm + c];
}

/* Set a row of the matrix with variadic values --------------------------------
* Sets the elements of the specified row with the provided values.
* args   : int i            I   row index
*          double f         I   first value
*          ...              I   additional values
* return : none
*-----------------------------------------------------------------------------*/
void mat::SetRow(int i, double f, ...)
{
	va_list vl;
	va_start(vl, f);
	for (double* p = &dd[i * clm], *pEnd = p + clm; p < pEnd; p++)
	{
		*p = f;  f = va_arg(vl, double);
	}
	va_end(vl);
	return;
}

/* Set a row of the matrix with a vector ---------------------------------------
* Sets the elements of the specified row with the values from a vector.
* args   : int i            I   row index
*          const vect &v    I   vector to copy from
* return : none
*-----------------------------------------------------------------------------*/
void mat::SetRow(int i, const vect& v)
{
	assert(clm == v.clm);
	const double* p = v.dd;
	for (double* p1 = &dd[i * clm], *pEnd = p1 + clm; p1 < pEnd; p++, p1++) *p1 = *p;
	return;
}

/* Set a column of the matrix with an array ------------------------------------
* Sets the elements of the specified column with the provided array values.
* args   : int j            I   column index
*          double f[]       I   array of values
*          int len          I   number of elements to set
* return : none
*-----------------------------------------------------------------------------*/
void mat::SetClm(int j, double f[], int len)
{
	int i = 0;
	for (double* p = &dd[j], *pEnd = &p[rc]; p < pEnd && i < len; p += clm, i++)
	{
		*p = f[i];
	}
	return;
}

/* Set a column of the matrix with a vector ------------------------------------
* Sets the elements of the specified column with the values from a vector.
* args   : int j            I   column index
*          const vect &v    I   vector to copy from
* return : none
*-----------------------------------------------------------------------------*/
void mat::SetClm(int j, const vect& v)
{
	assert(row == v.row);
	const double* p = v.dd;
	for (double* p1 = &dd[j], *pEnd = &dd[rc]; p1 < pEnd; p++, p1 += clm) *p1 = *p;
	return;
}

/* Set a 3-element column vector at a specific position ------------------------
* Sets a 3-element vector at the specified row and column in the matrix.
* args   : int i            I   starting row index
*          int j            I   column index
*          const vect3 &v   I   3-element vector to set
* return : none
*-----------------------------------------------------------------------------*/
void mat::SetClmVect3(int i, int j, const vect3& v)
{
	double* p = &dd[i * clm + j];
	*p = v.i; p += clm;
	*p = v.j; p += clm;
	*p = v.k;
}

/* Set a 3x2 block in the matrix with two 3-element vectors --------------------
* Sets a 3x2 block in the matrix at the specified position with two 3-element vectors.
* args   : int i            I   starting row index
*          int j            I   starting column index
*          const vect3 &v   I   first 3-element vector
*          const vect3 &v1  I   second 3-element vector
* return : none
*-----------------------------------------------------------------------------*/
void mat::SetClmVect3(int i, int j, const vect3& v, const vect3& v1)
{
	double* p = &dd[i * clm + j];
	*p = v.i; *(p + 1) = v1.i; p += clm;
	*p = v.j; *(p + 1) = v1.j; p += clm;
	*p = v.k; *(p + 1) = v1.k;
}

/* Set a 3x3 block in the matrix with three 3-element vectors ------------------
* Sets a 3x3 block in the matrix at the specified position with three 3-element vectors.
* args   : int i            I   starting row index
*          int j            I   starting column index
*          const vect3 &v   I   first 3-element vector
*          const vect3 &v1  I   second 3-element vector
*          const vect3 &v2  I   third 3-element vector
* return : none
*-----------------------------------------------------------------------------*/
void mat::SetClmVect3(int i, int j, const vect3& v, const vect3& v1, const vect3& v2)
{
	double* p = &dd[i * clm + j];
	*p = v.i; *(p + 1) = v1.i; *(p + 2) = v2.i; p += clm;
	*p = v.j; *(p + 1) = v1.j; *(p + 2) = v2.j; p += clm;
	*p = v.k; *(p + 1) = v1.k; *(p + 2) = v2.k;
	//	SetMat3(i,j, mat3(v, v1, v2));
}

/* Set a 3-element row vector at a specific position ---------------------------
* Sets a 3-element vector at the specified row and column in the matrix.
* args   : int i            I   row index
*          int j            I   starting column index
*          const vect3 &v   I   3-element vector to set
* return : none
*-----------------------------------------------------------------------------*/
void mat::SetRowVect3(int i, int j, const vect3& v)
{
	*(vect3*)&dd[i * clm + j] = v;
}

/* Set a 3x2 block in the matrix with two 3-element vectors (row-wise) ---------
* Sets a 3x2 block in the matrix at the specified position with two 3-element vectors (row-wise).
* args   : int i            I   row index
*          int j            I   starting column index
*          const vect3 &v   I   first 3-element vector
*          const vect3 &v1  I   second 3-element vector
* return : none
*-----------------------------------------------------------------------------*/
void mat::SetRowVect3(int i, int j, const vect3& v, const vect3& v1)
{
	vect3* p = (vect3*)&dd[i * clm + j];
	*p++ = v;  *p = v1;
}

/* Set a 3x3 block in the matrix with three 3-element vectors (row-wise) -------
* Sets a 3x3 block in the matrix at the specified position with three 3-element vectors (row-wise).
* args   : int i            I   row index
*          int j            I   starting column index
*          const vect3 &v   I   first 3-element vector
*          const vect3 &v1  I   second 3-element vector
*          const vect3 &v2  I   third 3-element vector
* return : none
*-----------------------------------------------------------------------------*/
void mat::SetRowVect3(int i, int j, const vect3& v, const vect3& v1, const vect3& v2)
{
	vect3* p = (vect3*)&dd[i * clm + j];
	*p++ = v;  *p++ = v1;  *p = v2;
}

/* Get a 3-element row vector from the matrix ----------------------------------
* Retrieves a 3-element vector from the specified row and column in the matrix.
* args   : int i            I   row index
*          int j            I   column index
* return : vect3                 3-element vector
*-----------------------------------------------------------------------------*/
vect3 mat::GetRowVect3(int i, int j) const
{
	return *(vect3*)&dd[i * clm + j];
}

/* Get a 3-element column vector from the matrix -------------------------------
* Retrieves a 3-element vector from the specified row and column in the matrix (column-wise).
* args   : int i            I   row index
*          int j            I   column index
* return : vect3                 3-element vector
*-----------------------------------------------------------------------------*/
vect3 mat::GetClmVect3(int i, int j) const
{
	vect3 v;
	const double* p = &dd[i * clm + j];
	v.i = *p; p += clm;
	v.j = *p; p += clm;
	v.k = *p;
	return v;
}

/* Set diagonal elements of a 3x3 block with a 3-element vector ----------------
* Sets the diagonal elements of a 3x3 block in the matrix with the values from a 3-element vector.
* args   : int i            I   starting row index
*          int j            I   starting column index
*          const vect3 &v   I   3-element vector
* return : none
*-----------------------------------------------------------------------------*/
void mat::SetDiagVect3(int i, int j, const vect3& v)
{
	double* p = &dd[i * clm + j];
	*p = v.i;  p += clm + 1;
	*p = v.j;  p += clm + 1;
	*p = v.k;
}

/* Get diagonal elements of a 3x3 block as a 3-element vector ------------------
* Retrieves the diagonal elements of a 3x3 block in the matrix as a 3-element vector.
* args   : int i            I   starting row index
*          int j            I   starting column index (-1 means use i)
* return : vect3                 3-element vector
*-----------------------------------------------------------------------------*/
vect3 mat::GetDiagVect3(int i, int j) const
{
	if (j == -1) j = i;
	vect3 v;
	const double* p = &dd[i * clm + j];
	v.i = *p;  p += clm + 1;
	v.j = *p;  p += clm + 1;
	v.k = *p;
	return v;
}

/* Set a 3x3 skew-symmetric block in the matrix from a 3-element vector --------
* Sets a 3x3 skew-symmetric block in the matrix at the specified position using a 3-element vector.
* args   : int i            I   starting row index
*          int j            I   starting column index
*          const vect3 &v   I   3-element vector
* return : none
*-----------------------------------------------------------------------------*/
void mat::SetAskew(int i, int j, const vect3& v)
{
	double* p = &dd[i * clm + j];
	p[0] = 0.0; p[1] = -v.k; p[2] = v.j;  p += clm;
	p[0] = v.k; p[1] = 0.0; p[2] = -v.i;  p += clm;
	p[0] = -v.j; p[1] = v.i; p[2] = 0.0;
}

/* Set a 3x3 block in the matrix with a mat3 object ----------------------------
* Sets a 3x3 block in the matrix at the specified position with the values from a mat3 object.
* args   : int i            I   starting row index
*          int j            I   starting column index
*          const mat3 &m    I   3x3 matrix to set
* return : none
*-----------------------------------------------------------------------------*/
void mat::SetMat3(int i, int j, const mat3& m)
{
	double* p = &dd[i * clm + j];
	*(vect3*)p = *(vect3*)&m.e00;  p += clm;
	*(vect3*)p = *(vect3*)&m.e10;  p += clm;
	*(vect3*)p = *(vect3*)&m.e20;
}

/* Set a 3x6 block in the matrix with two mat3 objects -------------------------
* Sets a 3x6 block in the matrix at the specified position with two mat3 objects.
* args   : int i            I   starting row index
*          int j            I   starting column index
*          const mat3 &m    I   first 3x3 matrix
*          const mat3 &m1   I   second 3x3 matrix
* return : none
*-----------------------------------------------------------------------------*/
void mat::SetMat3(int i, int j, const mat3& m, const mat3& m1)
{
	double* p = &dd[i * clm + j];
	*(vect3*)p = *(vect3*)&m.e00;  *(vect3*)(p + 3) = *(vect3*)&m1.e00;  p += clm;
	*(vect3*)p = *(vect3*)&m.e10;  *(vect3*)(p + 3) = *(vect3*)&m1.e10;  p += clm;
	*(vect3*)p = *(vect3*)&m.e20;  *(vect3*)(p + 3) = *(vect3*)&m1.e20;
}

/* Set a 3x9 block in the matrix with three mat3 objects -----------------------
* Sets a 3x9 block in the matrix at the specified position with three mat3 objects.
* args   : int i            I   starting row index
*          int j            I   starting column index
*          const mat3 &m    I   first 3x3 matrix
*          const mat3 &m1   I   second 3x3 matrix
*          const mat3 &m2   I   third 3x3 matrix
* return : none
*-----------------------------------------------------------------------------*/
void mat::SetMat3(int i, int j, const mat3& m, const mat3& m1, const mat3& m2)
{
	double* p = &dd[i * clm + j];
	*(vect3*)p = *(vect3*)&m.e00;  *(vect3*)(p + 3) = *(vect3*)&m1.e00;  *(vect3*)(p + 6) = *(vect3*)&m2.e00;  p += clm;
	*(vect3*)p = *(vect3*)&m.e10;  *(vect3*)(p + 3) = *(vect3*)&m1.e10;  *(vect3*)(p + 6) = *(vect3*)&m2.e10;  p += clm;
	*(vect3*)p = *(vect3*)&m.e20;  *(vect3*)(p + 3) = *(vect3*)&m1.e20;  *(vect3*)(p + 6) = *(vect3*)&m2.e20;
}

/* Get a 3x3 block from the matrix as a mat3 object ----------------------------
* Retrieves a 3x3 block from the matrix at the specified position as a mat3 object.
* args   : int i            I   starting row index
*          int j            I   starting column index (-1 means use i)
* return : mat3                  3x3 matrix
*-----------------------------------------------------------------------------*/
mat3 mat::GetMat3(int i, int j) const
{
	if (j == -1) j = i;
	mat3 m;
	const double* p = &dd[i * clm + j];
	*(vect3*)&m.e00 = *(vect3*)p;  p += clm;
	*(vect3*)&m.e10 = *(vect3*)p;  p += clm;
	*(vect3*)&m.e20 = *(vect3*)p;
	return m;
}

/* Add a mat3 object to a 3x3 block in the matrix ------------------------------
* Adds a mat3 object to a 3x3 block in the matrix at the specified position.
* args   : int i            I   starting row index
*          int j            I   starting column index
*          const mat3 &m    I   3x3 matrix to add
* return : none
*-----------------------------------------------------------------------------*/
void mat::SubAddMat3(int i, int j, const mat3& m)
{
	double* p = &dd[i * clm + j];
	*(vect3*)p += *(vect3*)&m.e00;  p += clm;
	*(vect3*)p += *(vect3*)&m.e10;  p += clm;
	*(vect3*)p += *(vect3*)&m.e20;
}

/* Get a row of the matrix as a vect object ------------------------------------
* Retrieves the specified row of the matrix as a vect object.
* args   : int i            I   row index
* return : vect                  vector containing the row elements
*-----------------------------------------------------------------------------*/
vect mat::GetRow(int i) const
{
	vect v(1, clm);
	const double* p1 = &dd[i * clm], * pEnd = p1 + clm;
	for (double* p = v.dd; p1 < pEnd; p++, p1++) *p = *p1;
	return v;
}

/* Get a column of the matrix as a vect object ---------------------------------
* Retrieves the specified column of the matrix as a vect object.
* args   : int j            I   column index
* return : vect                  vector containing the column elements
*-----------------------------------------------------------------------------*/
vect mat::GetClm(int j) const
{
	vect v(row, 1);
	const double* p1 = &dd[j], * pEnd = &dd[rc];
	for (double* p = v.dd; p1 < pEnd; p++, p1 += clm) *p = *p1;
	return v;
}

/* Zero out a row of the matrix ------------------------------------------------
* Sets all elements of the specified row to zero.
* args   : int i            I   row index
* return : none
*-----------------------------------------------------------------------------*/
void mat::ZeroRow(int i)
{
	for (double* p = &dd[i * clm], *pEnd = p + clm; p < pEnd; p++) *p = 0.0;
	return;
}

/* Zero out a column of the matrix ---------------------------------------------
* Sets all elements of the specified column to zero.
* args   : int j            I   column index
* return : none
*-----------------------------------------------------------------------------*/
void mat::ZeroClm(int j)
{
	for (double* p = &dd[j], *pEnd = &dd[rc]; p < pEnd; p += clm) *p = 0.0;
	return;
}

/* Set diagonal elements of the matrix from an array ---------------------------
* Sets the diagonal elements of the matrix with the provided array values.
* args   : double f[]       I   array of values
*          int len          I   number of elements to set
* return : none
*-----------------------------------------------------------------------------*/
void mat::SetDiag(double f[], int len)
{
	*this = mat(this->row, this->clm, 0.0);

	double* p = dd, * pEnd = &dd[rc];
	for (int row1 = row + 1, i = 0; p < pEnd && i < len; p += row1, i++)
	{
		if (f[i] > 2 * INF) break;  *p = f[i];
	}
}

/* Set diagonal elements of the matrix from an array (squared values) ----------
* Sets the diagonal elements of the matrix with the squares of the provided array values.
* args   : double f[]       I   array of values
*          int len          I   number of elements to set
* return : none
*-----------------------------------------------------------------------------*/
void mat::SetDiag2(double f[], int len)
{
	*this = mat(this->row, this->clm, 0.0);

	double* p = dd, * pEnd = &dd[rc];
	for (int row1 = row + 1, i = 0; p < pEnd && i < len; p += row1, i++)
	{
		if (f[i] > 2 * INF) break;  *p = f[i] * f[i];
	}
}

/* calculate 1-norm of a matrix -----------------------------------------------
* calculate the 1-norm (maximum column sum) of a matrix
* args   : mat        m        I   input matrix
* return : 1-norm of the matrix
*-----------------------------------------------------------------------------*/
double norm1(const mat& m)
{
	return norm1(&m.dd[0], m.rc);
}

/* calculate infinity norm of a matrix ----------------------------------------
* calculate the infinity norm (maximum row sum) of a matrix
* args   : mat        m        I   input matrix
* return : infinity norm of the matrix
*-----------------------------------------------------------------------------*/
double normInf(const mat& m)
{
	return normInf(&m.dd[0], m.rc);
}

/* extract diagonal of a matrix -----------------------------------------------
* extract the diagonal elements of a matrix as a vector
* args   : mat        m        I   input matrix
* return : vector containing the diagonal elements of the matrix
*-----------------------------------------------------------------------------*/
vect diag(const mat& m)
{
	int row1 = m.row + 1;
	vect vtmp(m.row, 1);
	double* p = vtmp.dd, * pEnd = &vtmp.dd[vtmp.row];
	for (const double* p1 = m.dd; p < pEnd; p++, p1 += row1) *p = *p1;
	return vtmp;
}

/* create identity matrix -----------------------------------------------------
* create an identity matrix of specified size
* args   : int        n        I   size of the identity matrix
* return : identity matrix
*-----------------------------------------------------------------------------*/
mat eye(int n)
{
	mat m(n, n, 0.0);
	double* p = m.dd, * pEnd = &m.dd[m.rc];
	for (n = n + 1; p < pEnd; p += n)  *p = 1.0;
	return m;
}

/* calculate inverse of a 4x4 matrix ------------------------------------------
* calculate the inverse of a 4x4 matrix
* args   : mat        m        I   input matrix
* return : inverse of the matrix
*-----------------------------------------------------------------------------*/
mat inv4(const mat& m)
{
	assert(m.clm == m.row && m.clm == 4);
	mat3 A11(m.dd[0], m.dd[1], m.dd[2], m.dd[4], m.dd[5], m.dd[6], m.dd[8], m.dd[9], m.dd[10]), iA11;
	vect3 A12(m.dd[3], m.dd[7], m.dd[11]), A21(m.dd[12], m.dd[13], m.dd[14]);
	double A22 = m.dd[15], iA22;
	iA11 = inv(A11);  iA22 = 1.0 / (A22 - dot(A21 * iA11, A12));
	mat M(4, 4);
	M.SetMat3(0, 0, iA11 + iA11 * A12 * iA22 * A21 * iA11);  // by using matrix-inversion-lemma
	M.SetClmVect3(0, 3, -iA11 * A12 * iA22);
	M.SetRowVect3(3, 0, -iA22 * A21 * iA11);
	M.dd[15] = iA22;
	return M;
}

/* multiply row of a matrix by another matrix ---------------------------------
* multiply a row of a matrix by another matrix
* args   : mat        m        IO  input/output matrix
*          mat        m0       I   first input matrix
*          mat        m1       I   second input matrix
*          int        r        I   row index
*          int        fast     I   flag for fast computation (default: 0)
* return : none
*-----------------------------------------------------------------------------*/
void RowMul(mat& m, const mat& m0, const mat& m1, int r, int fast)
{
	assert(m0.clm == m1.row);
	int rc0 = r * m0.clm; fast = (r >= fast);
	double* p = &m.dd[rc0], * pEnd = p + m0.clm; const double* p0 = &m0.dd[rc0], * p0End = p0 + m0.clm, * p1j = m1.dd;
	for (; p < pEnd; p++, p1j++)
	{
		if (fast) { *p = p1j[r * m1.row]; continue; }
		double f = 0.0; const double* p0j = p0, * p1jk = p1j;
		for (; p0j < p0End; p0j++, p1jk += m1.clm) f += (*p0j) * (*p1jk);
		*p = f;
	}
}

/* multiply row of a matrix by transpose of another matrix --------------------
* multiply a row of a matrix by the transpose of another matrix
* args   : mat        m        IO  input/output matrix
*          mat        m0       I   first input matrix
*          mat        m1       I   second input matrix
*          int        r        I   row index
*          int        fast     I   flag for fast computation (default: 0)
* return : none
*-----------------------------------------------------------------------------*/
void RowMulT(mat& m, const mat& m0, const mat& m1, int r, int fast)
{
	assert(m0.clm == m1.clm);
	int rc0 = r * m0.clm, ifast = 0;
	double* p = &m.dd[rc0], * pEnd = p + m0.clm; const double* p0 = &m0.dd[rc0], * p0End = p0 + m0.clm, * p1jk = m1.dd;
	for (; p < pEnd; p++, ifast++)
	{
		if (ifast >= fast) { *p = p0[ifast]; p1jk += m1.clm; continue; }
		double f = 0.0; const double* p0j = p0;
		for (; p0j < p0End; p0j++, p1jk++) f += (*p0j) * (*p1jk);
		*p = f;
	}
}

/* create diagonal matrix from a vector ---------------------------------------
* create a diagonal matrix from a vector
* args   : vect       v        I   input vector
* return : diagonal matrix
*-----------------------------------------------------------------------------*/
mat diag(const vect& v)
{
	int rc = v.row > v.clm ? v.row : v.clm, rc1 = rc + 1;
	mat mtmp(rc, rc, 0.0);
	double* p = mtmp.dd;
	for (const double* p1 = v.dd, *p1End = &v.dd[rc]; p1 < p1End; p += rc1, p1++) *p = *p1;
	return mtmp;
}

/* scale matrix by diagonal vector --------------------------------------------
* scale a matrix by a diagonal vector and a scalar factor
* args   : vect       V        I   diagonal vector
*          mat        M        IO  input/output matrix
*          double     afa      I   scalar factor
* return : none
*-----------------------------------------------------------------------------*/
void DVMDVafa(const vect& V, mat& M, double afa)
{
	assert(V.rc == M.row && M.row == M.clm);
	int i = 0;
	const double* pv = V.dd;
	for (double vi = *pv, viafa = vi * afa; i < M.clm; i++, pv++, vi = *pv, viafa = vi * afa)
	{
		for (double* prow = &M.dd[i * M.clm], *prowEnd = prow + M.clm, *pclm = &M.dd[i]; prow < prowEnd; prow++, pclm += M.row)
		{
			*prow *= vi;
			*pclm *= viafa;
		}
	}
}

//***************************  class vect  *********************************/
/* default constructor --------------------------------------------------------
* initialize vector with zero elements and size 0x0
* args   : none
* return : none
*-----------------------------------------------------------------------------*/
vect::vect(void)
{
}
/* size constructor -----------------------------------------------------------
* initialize vector with given row and column size, all elements set to zero
* args   : int        row0     I   number of rows
*          int        clm0     I   number of columns (default: 1)
* return : none
*-----------------------------------------------------------------------------*/
vect::vect(int row0, int clm0)
{
	if (clm0 == 1) { row = row0; clm = 1; }
	else { row = 1;    clm = clm0; }
	rc = row * clm;
}
/* fill constructor -----------------------------------------------------------
* initialize vector with given size, all elements set to the same value
* args   : int        row0     I   number of rows
*          double     f        I   value to fill
* return : none
*-----------------------------------------------------------------------------*/
vect::vect(int row0, double f)
{
	row = row0; clm = 1; rc = row * clm;
	for (register int i = 0; i < row; i++) dd[i] = f;
}
/* array constructor ----------------------------------------------------------
* initialize vector with given size and values from array
* args   : int        row0     I   number of rows
*          const double* pf    I   pointer to array of values
* return : none
*-----------------------------------------------------------------------------*/
vect::vect(int row0, const double* pf)
{
	row = row0; clm = 1; rc = row * clm;
	memcpy(dd, pf, row * sizeof(double));
}
/* variadic fill constructor --------------------------------------------------
* initialize vector with given size and variadic list of values
* args   : int        row0     I   number of rows
*          double     f        I   first value
*          double     f1,...   I   additional values
* return : none
*-----------------------------------------------------------------------------*/
vect::vect(int row0, double f, double f1, ...)
{
	row = row0; clm = 1; rc = row * clm;
	assert(row <= MMD && clm <= MMD);
	dd[0] = f;
	va_list vl;
	va_start(vl, f1);
	for (register int i = 1, rc = row > clm ? row : clm; i < rc; i++)
	{
		if (f > 2 * INF) break;  dd[i] = f1;  f1 = va_arg(vl, double);
	}
	va_end(vl);
}
/* vect3 constructor ----------------------------------------------------------
* initialize vector from a vect3 object (3 elements)
* args   : const vect3& v      I   vect3 object
* return : none
*-----------------------------------------------------------------------------*/
vect::vect(const vect3& v)
{
	row = 3; clm = 1; rc = row * clm;
	dd[0] = v.i; dd[1] = v.j; dd[2] = v.k;
}
/* two vect3 constructor ------------------------------------------------------
* initialize vector from two vect3 objects (6 elements)
* args   : const vect3& v1     I   first vect3 object
*          const vect3  v2     I   second vect3 object
* return : none
*-----------------------------------------------------------------------------*/
vect::vect(const vect3& v1, const vect3 v2)
{
	row = 6; clm = 1; rc = row * clm;
	dd[0] = v1.i; dd[1] = v1.j; dd[2] = v1.k;
	dd[3] = v2.i; dd[4] = v2.j; dd[5] = v2.k;
}
/* vector transposition -------------------------------------------------------
* transpose the vector (row <-> column)
* args   : const vect& v       I   vector to transpose
* return : transposed vector
*-----------------------------------------------------------------------------*/
vect operator~(const vect& v)
{
	vect vtmp = v;
	vtmp.row = v.clm; vtmp.clm = v.row;
	return vtmp;
}

//    const mat vect::operator (const mat &m) const
//    {
//        assert(clm==m.row);
//        vect vtmp(row,clm);
//        double *p=vtmp.dd; const double *p1End=&dd[clm];
//        for(int j=0; j<clm; p++,j++)
//        {
//            double f=0.0; const double *p1j=dd, *p2jk=&m.dd[j];
//            for(; p1j<p1End; p1j++,p2jk+=m.clm)	 f += (*p1j) * (*p2jk);
//            *p = f;
//        }
//        return vtmp;
//    }

/* vector scaling -------------------------------------------------------------
* multiply all elements by a scalar
* args   : double     f        I   scale factor
* return : scaled vector
*-----------------------------------------------------------------------------*/
mat vect::operator*(const vect& v) const
{
	assert(clm == v.row);
	mat mtmp(row, v.clm);
	if (row == 1 && v.clm == 1)  // (1x1) = (1xn)*(nx1)
	{
		double f = 0.0;
		for (register int i = 0; i < clm; i++)  f += dd[i] * v.dd[i];
		mtmp.dd[0] = f;
	}
	else    // (nxn) = (nx1)*(1xn)
	{
		double* p = mtmp.dd;
		for (const double* p1 = &dd[0], *p1End = &dd[rc], *p2End = &v.dd[rc]; p1 < p1End; p1++)
		{
			for (const double* p2 = &v.dd[0]; p2 < p2End; p2++)  *p++ = *p1 * *p2;
		}
	}
	return mtmp;
}
/* vector addition ------------------------------------------------------------
* add two vectors element-wise
* args   : const vect& v       I   vector to add
* return : result of addition
*-----------------------------------------------------------------------------*/
vect vect::operator+(const vect& v) const
{
	assert(row == v.row && clm == v.clm);
	const double* p2 = v.dd, * p1 = dd, * p1End = &dd[rc];
	vect vtmp(row, clm);
	for (double* p = vtmp.dd; p1 < p1End; p++, p1++, p2++) { *p = *p1 + *p2; }
	return vtmp;
}
/* vector subtraction ---------------------------------------------------------
* subtract two vectors element-wise
* args   : const vect& v       I   vector to subtract
* return : result of subtraction
*-----------------------------------------------------------------------------*/
vect vect::operator-(const vect& v) const
{
	assert(row == v.row && clm == v.clm);
	const double* p2 = v.dd, * p1 = dd, * p1End = &dd[rc];
	vect vtmp(row, clm);
	for (double* p = vtmp.dd; p1 < p1End; p++, p1++, p2++) { *p = *p1 - *p2; }
	return vtmp;
}
/* vector scaling -------------------------------------------------------------
* multiply all elements by a scalar
* args   : double     f        I   scale factor
* return : scaled vector
*-----------------------------------------------------------------------------*/
vect vect::operator*(double f) const
{
	vect vtmp(row, clm);
	const double* p1 = dd, * p1End = &dd[rc];
	for (double* p = vtmp.dd; p1 < p1End; p++, p1++) { *p = *p1 * f; }
	return vtmp;
}
/* assign all elements to a value ---------------------------------------------
* set all elements to the same value
* args   : double     f        I   value to assign
* return : reference to this vector
*-----------------------------------------------------------------------------*/
vect& vect::operator=(double f)
{
	for (double* p = dd, *pEnd = &dd[rc]; p < pEnd; p++) { *p = f; }
	return *this;
}
/* assign from array ----------------------------------------------------------
* assign vector elements from array
* args   : const double* pf    I   pointer to array
* return : reference to this vector
*-----------------------------------------------------------------------------*/
vect& vect::operator=(const double* pf)
{
	for (double* p = dd, *pEnd = &dd[rc]; p < pEnd; p++, pf++) { *p = *pf; }
	return *this;
}
/* assign from mat3 -----------------------------------------------------------
* assign vector elements from a 3x3 matrix (flattened)
* args   : const mat3& m       I   matrix to assign from
* return : reference to this vector
*-----------------------------------------------------------------------------*/
vect& vect::operator=(const mat3& m)
{
	row = 9; clm = 1; rc = 9;
	memcpy(dd, &m.e00, 9 * sizeof(double));
	return *this;
}

/* vector addition assignment -------------------------------------------------
* add another vector to this vector (element-wise)
* args   : const vect& v       I   vector to add
* return : reference to this vector
*-----------------------------------------------------------------------------*/
vect& vect::operator+=(const vect& v)
{
	assert(row == v.row && clm == v.clm);
	const double* p1 = v.dd;
	for (double* p = dd, *pEnd = &dd[rc]; p < pEnd; p++, p1++) { *p += *p1; }
	return *this;
}
/* vector subtraction assignment ----------------------------------------------
* subtract another vector from this vector (element-wise)
* args   : const vect& v       I   vector to subtract
* return : reference to this vector
*-----------------------------------------------------------------------------*/
vect& vect::operator-=(const vect& v)
{
	assert(row == v.row && clm == v.clm);
	const double* p1 = v.dd;
	for (double* p = dd, *pEnd = &dd[rc]; p < pEnd; p++, p1++) { *p -= *p1; }
	return *this;
}
/* vector scaling assignment --------------------------------------------------
* scale this vector by a scalar
* args   : double     f        I   scale factor
* return : reference to this vector
*-----------------------------------------------------------------------------*/
vect& vect::operator*=(double f)
{
	for (double* p = dd, *pEnd = &dd[rc]; p < pEnd; p++) { *p *= f; }
	return *this;
}
/* calculate dot product of two vectors ---------------------------------------
* calculate the dot product of two vectors
* args   : vect       v1       I   first input vector
*          vect       v2       I   second input vector
* return : dot product of the two vectors
*-----------------------------------------------------------------------------*/
double dot(const vect& v1, const vect& v2)
{
	assert(v1.row == v2.row && v1.clm == v2.clm);
	double res = 0.0;
	for (const double* p1 = v1.dd, *p2 = v2.dd, *pEnd = &v1.dd[v1.rc]; p1 < pEnd; p1++, p2++) { res += (*p1) * (*p2); }
	return res;
}
/* calculate element-wise product of two vectors ------------------------------
* calculate the element-wise product of two vectors
* args   : vect       v1       I   first input vector
*          vect       v2       I   second input vector
* return : vector with element-wise product of the two vectors
*-----------------------------------------------------------------------------*/
vect dotmul(const vect& v1, const vect& v2)
{
	assert(v1.row == v2.row && v1.clm == v2.clm);
	vect res(v1.row, v1.clm);
	double* p = res.dd;
	for (const double* p1 = v1.dd, *p2 = v2.dd, *pEnd = &v1.dd[v1.rc]; p1 < pEnd; p1++, p2++, p++) { *p = (*p1) * (*p2); }
	return res;
}

/* calculate power of vector elements -----------------------------------------
* calculate the k-th power for each element of a vector
* args   : vect       v        I   input vector
*          int        k        I   power to raise each element
* return : vector with each element raised to the k-th power
*-----------------------------------------------------------------------------*/
vect pow(const vect& v, int k)
{
	vect pp = v;
	double* p, * pEnd = &pp.dd[pp.rc];
	for (register int i = 1; i < k; i++)
	{
		p = pp.dd;
		for (const double* p1 = v.dd; p < pEnd; p++, p1++)
			*p *= *p1;
	}
	return pp;
}
/* calculate absolute value of vector elements ---------------------------------
* calculate the absolute value for each element of a vector
* args   : vect       v        I   input vector
* return : vector with absolute values of each element
*-----------------------------------------------------------------------------*/
vect abs(const vect& v)
{
	vect res(v.row, v.clm);
	const double* p = v.dd, * pEnd = &v.dd[v.rc];
	for (double* p1 = res.dd; p < pEnd; p++, p1++) { *p1 = *p > 0 ? *p : -*p; }
	return res;
}
/* calculate vector norm ------------------------------------------------------
* calculate the Euclidean norm of a vector
* args   : vect       v        I   input vector
* return : Euclidean norm of the vector
*-----------------------------------------------------------------------------*/
double norm(const vect& v)
{
	return norm(&v.dd[0], v.rc);
}

double norm1(const vect& v)
{
	return norm1(&v.dd[0], v.rc);
}
/* calculate vector infinity norm ---------------------------------------------
* calculate the infinity norm (maximum absolute value) of a vector
* args   : vect       v        I   input vector
* return : infinity norm of the vector
*-----------------------------------------------------------------------------*/
double normInf(const vect& v)
{
	return normInf(&v.dd[0], v.rc);
}
/* element access -------------------------------------------------------------
* access vector element by index
* args   : int        r        I   element index
* return : reference to element
*-----------------------------------------------------------------------------*/
double& vect::operator()(int r)
{
	return this->dd[r];
}
/* set elements from array ----------------------------------------------------
* set vector elements from array
* args   : double*    f        I   pointer to array
*          int        size     I   number of elements to set
* return : none
*-----------------------------------------------------------------------------*/
void vect::Set(double* f, int size)
{
	assert(rc <= MMD);
	int j = 0;
	for (int i = 0; i < rc && j < size; i++, j++)
	{
		if (f[j] > 2 * INF) break;  dd[i] = f[j];
	}
}
/* set elements from array (alternative) --------------------------------------
* set vector elements from array (alternative method)
* args   : double*    f        I   pointer to array
*          int        size     I   number of elements to set
* return : none
*-----------------------------------------------------------------------------*/
void vect::Set2(double* f, int size)
{
	assert(rc <= MMD);
	int j = 0;

	for (int i = 0; i < rc && j < size; i++, j++)
	{
		if (f[j] > 2 * INF) break;  dd[i] = f[j] * f[j];
	}
}
/* set vect3 at position ------------------------------------------------------
* set a vect3 at specified position in the vector
* args   : int        i        I   start index
*          const vect3& v      I   vect3 to set
* return : none
*-----------------------------------------------------------------------------*/
void vect::SetVect3(int i, const vect3& v)
{
	*(vect3*)&dd[i] = v;
}
/* set vect3 at position (pow2) -----------------------------------------------
* set a vect3 at specified position in the vector (pow2 version)
* args   : int        i        I   start index
*          const vect3& v      I   vect3 to set
* return : none
*-----------------------------------------------------------------------------*/
void vect::Set2Vect3(int i, const vect3& v)
{
	dd[i++] = v.i * v.i; dd[i++] = v.j * v.j; dd[i] = v.k * v.k;
}
/* set element(s) by bit mask -------------------------------------------------
* set element(s) to value by bit mask
* args   : unsigned int bit    I   bit mask
*          double     f        I   value to set
* return : none
*-----------------------------------------------------------------------------*/
void vect::SetBit(unsigned int bit, double f)
{
	for (register int i = 0; i < rc; i++)		// assert(rc<32)
		if (bit & (0x01 << i)) dd[i] = f;
}
/* set vect3 by bit mask ------------------------------------------------------
* set vect3 to elements by bit mask
* args   : unsigned int bit    I   bit mask
*          const vect3& v      I   vect3 to set
* return : none
*-----------------------------------------------------------------------------*/
void vect::SetBit(unsigned int bit, const vect3& v)
{
	const double* p = &v.i;
	for (register int i = 0; i < rc; i++)		// assert(rc<32)
		if (bit & (0x01 << i)) { dd[i] = *p++;  if (p > &v.k) p = &v.i; }
}
/* get vect3 from position ----------------------------------------------------
* get a vect3 from specified position in the vector
* args   : int        i        I   start index
* return : vect3 at position i
*-----------------------------------------------------------------------------*/
vect3 vect::GetVect3(int i) const
{
	return *(vect3*)&dd[i];
}
/* sort vector elements -------------------------------------------------------
* sort the elements of a vector in ascending order
* args   : vect       v        I   input vector
* return : sorted vector
*-----------------------------------------------------------------------------*/
vect sort(const vect& v)
{
	vect vtmp = v;
	double* pi = vtmp.dd, * pj, * pend = &vtmp.dd[vtmp.rc];
	for (; pi < pend; pi++)
		for (pj = pi + 1; pj < pend; pj++)
			if (*pi < *pj) swapt(*pi, *pj, double);
	return vtmp;
}

/* constructor ----------------------------------------------------------------
* initialize earth model with given equatorial radius and flattening factor
* args   : double     a        I   equatorial radius (default: RE)
*          double     f        I   flattening factor (default: f0_earth)
* return : none
*-----------------------------------------------------------------------------*/
earth::earth(double a, double f)
{
	this->a = a;	this->f = f; this->wie = wie0;
	b = (1 - f) * a;
	gn = O31;  pgn = 0;
	Update(O31, O31);
}

/* initialize earth parameters ------------------------------------------------
* initialize earth parameters with given equatorial radius and flattening factor
* args   : double     a        I   equatorial radius (default: RE)
*          double     f        I   flattening factor (default: f0_earth)
* return : none
*-----------------------------------------------------------------------------*/
void earth::Init(double a, double f) {
	this->a = a;	this->f = f; this->wie = wie0;
	b = (1 - f) * a;
	gn = O31;  pgn = 0;
	Update(O31, O31);
}

/* update earth parameters ----------------------------------------------------
* update earth parameters based on current position and velocity
* args   : const vect3& pos   I   position vector (latitude, longitude, height)
*          const vect3& vn    I   velocity vector
* return : none
*-----------------------------------------------------------------------------*/
void earth::Update(const vect3& pos, const vect3& vn)
{
	this->pos = pos;  this->vn = vn;
	sl = sin(pos.i), cl = cos(pos.i), tl = sl / cl;
	double sq = 1 - e2 * sl * sl, sq2 = sqrt(sq);
	RMh = a * (1 - e2) / sq / sq2 + pos.k;	f_RMh = 1.0 / RMh;
	RNh = a / sq2 + pos.k;    clRNh = cl * RNh;  f_RNh = 1.0 / RNh; f_clRNh = 1.0 / clRNh;
	wnie.i = 0.0, wnie.j = wie * cl, wnie.k = wie * sl;
	wnen.i = -vn.j * f_RMh, wnen.j = vn.i * f_RNh, wnen.k = wnen.j * tl;
	wnin = wnie + wnen;
	sl2 = sl * sl, sl4 = sl2 * sl2;
	gn.k = -(G0 * (1 + 5.27094e-3 * sl2 + 2.32718e-5 * sl4) - 3.086e-6 * pos.k);
	gcc = pgn ? *pgn : gn;
	gcc -= (wnie + wnin) * vn;
}

/* convert velocity to position increment --------------------------------------
* convert velocity vector to position increment over a time step
* args   : const vect3& vn    I   velocity vector
*          float      ts      I   time step (default: 1.0)
* return : position increment vector
*-----------------------------------------------------------------------------*/
vect3 earth::vn2dpos(const vect3& vn, float ts) const
{
	return vect3(vn.j * f_RMh, vn.i * f_clRNh, vn.k) * ts;
}

/* convert position to local navigation frame ---------------------------------
* convert a position vector to the local navigation frame
* args   : vect3      pos      I   position vector in geodetic coordinates
* return : transformation matrix from ECEF to local navigation frame
*-----------------------------------------------------------------------------*/
mat3 pos2Cen(const vect3& pos)
{
	double si = sin(pos.i), ci = cos(pos.i), sj = sin(pos.j), cj = cos(pos.j);
	return mat3(-sj, -si * cj, ci * cj,
		cj, -si * sj, ci * sj,
		0, ci, si);
}

/* convert ECEF X/Y/Z to latitude/longitude/height ----------------------------
* convert a point in ECEF coordinates to geodetic coordinates
* args   : vect3      xyz      I   ECEF coordinates {X, Y, Z} (meters)
* return : geodetic coordinates {latitude, longitude, height} (radians, radians, meters)
*-----------------------------------------------------------------------------*/
vect3 xyz2blh(const vect3& xyz)
{
	double s = normXY(xyz);
	double theta = atan2(xyz.k * RE, RP);
	double s3 = sin(theta);
	double c3 = cos(theta);
	s3 = s3 * s3 * s3;
	c3 = c3 * c3 * c3;

	vect3 O31(0, 0, 0);
	if (s < (6378137.0 * 1.0 * DEG))  return O31;

	double L = atan2(xyz.j, xyz.i);
	double B = atan2(xyz.k + double(ep2 * RP * s3), s - e2 * RE * c3);
	double sB = sin(B);
	double cB = cos(B);
	double N = RE / sqrt(1 - e2 * sB * sB);

	return vect3(B, L, s / cB - N);
}

/* convert latitude/longitude/height to ECEF X/Y/Z ----------------------------
* convert a point in geodetic coordinates to ECEF coordinates
* args   : vect3      blh      I   geodetic coordinates {latitude, longitude, height} (radians, radians, meters)
* return : ECEF coordinates {X, Y, Z} (meters)
*-----------------------------------------------------------------------------*/
vect3 blh2xyz(const vect3& blh)
{
	double sB = sin(blh.i), cB = cos(blh.i), sL = sin(blh.j), cL = cos(blh.j),
		N = RE / ::sqrt(1 - e2 * sB * sB);
	return vect3((N + blh.k) * cB * cL, (N + blh.k) * cB * sL, (N * (1 - e2) + blh.k) * sB);
}

/* convert ECEF velocity to ENU velocity --------------------------------------
* convert a velocity vector in ECEF coordinates to ENU coordinates
* args   : vect3      Vxyz     I   velocity in ECEF coordinates {Vx, Vy, Vz} (m/s)
*          vect3      pos      I   geodetic position {latitude, longitude, height} (radians, radians, meters)
* return : velocity in ENU coordinates {Ve, Vn, Vu} (m/s)
*-----------------------------------------------------------------------------*/
vect3 Vxyz2enu(const vect3& Vxyz, const vect3& pos)
{
	return Vxyz * pos2Cen(pos);
}


/* initialize the maxmin statistics -------------------------------------------
* initialize all statistics and counters
* args   : int    cnt00   I   initial count (default: 100)
*          int    pre00   I   previous count (default: 0)
*          float  f       I   initial value (default: 0.0f)
* return : none
*-----------------------------------------------------------------------------*/
void maxmin::Init(int cnt00, int pre00, float f)
{
	maxCur = f, minCur = -f, maxpreCur = f, minpreCur = -f;
	maxRes = f, minRes = -f;
	cntCur = cnt0 = cnt00;
	cntpreCur = (pre00 <= 0 || pre00 >= cnt00) ? cnt00 / 2 : cnt0 - pre00;
	flag = 0;
}

/* update statistics with a new value -----------------------------------------
* update max, min, mean, and counters with a new value
* args   : float  f       I   new value to update statistics
* return : int            O   status or updated count
*-----------------------------------------------------------------------------*/
int maxmin::Update(float f)
{
	flag = 0;
	if (maxCur < f) maxCur = f; else if (minCur > f) minCur = f;
	if (maxpreCur < f) maxpreCur = f; else if (minpreCur > f) minpreCur = f;
	sumRes += f;
	if (--cntCur <= 0) {
		maxRes = maxCur; minRes = minCur; maxCur = minCur = f;  meanRes = sumRes / cnt0; cntCur = cnt0; flag = 1;
		sumRes = 0.0;
	}
	if (--cntpreCur <= 0) {
		maxpreCur = minpreCur = f;  cntpreCur = cnt0; flag = -1;
	}
	return flag;
}

/* initialize the maxminn statistics ------------------------------------------
* initialize all maxmin objects and counters for multiple variables
* args   : int    n0      I   number of variables (default: 3)
*          int    cnt00   I   initial count for each variable (default: 100)
*          int    pre00   I   previous count for each variable (default: 0)
*          float  f       I   initial value for each variable (default: 0.0f)
* return : none
*-----------------------------------------------------------------------------*/
void maxminn::Init(int n0, int cnt00, int pre00, float f)
{
	n = n0;
	maxmin mm0;
	mm0.Init(cnt00, pre00, f);
	for (int i = 0; i < n; i++)	mm[i] = mm0;
	flag = 0;
}

/* update statistics with an array of values ----------------------------------
* update all maxmin objects with new values from an array
* args   : double* f      I   pointer to array of new values
*          int     size   I   number of values in the array
* return : int            O   status or updated count
*-----------------------------------------------------------------------------*/
int maxminn::Update(double* f, int size)
{
	maxmin* pm = &mm[0];
	int j = 0;
	for (int i = 0; i < n && j < size; i++, pm++, j++)
	{
		pm->Update(f[j]);
	}
	return flag = mm[0].flag;
}

/* update statistics with a vect3 ---------------------------------------------
* update all maxmin objects with new values from a vect3
* args   : const vect3& v I   vect3 of new values
* return : int            O   status or updated count
*-----------------------------------------------------------------------------*/
int maxminn::Update(const vect3& v1)
{
	mm[0].Update((float)v1.i); mm[1].Update((float)v1.j); mm[2].Update((float)v1.k);
	return flag = mm[0].flag;
}

/* update statistics with two vect3 -------------------------------------------
* update all maxmin objects with new values from two vect3
* args   : const vect3& v1 I   first vect3 of new values
*          const vect3& v2 I   second vect3 of new values
* return : int             O   status or updated count
*-----------------------------------------------------------------------------*/
int maxminn::Update(const vect3& v1, const vect3& v2)
{
	mm[0].Update((float)v1.i); mm[1].Update((float)v1.j); mm[2].Update((float)v1.k);
	mm[3].Update((float)v2.i); mm[4].Update((float)v2.j); mm[5].Update((float)v2.k);
	return flag = mm[0].flag;
}


/* constructor ----------------------------------------------------------------
* initialize the statistics tracker with maximum size and initial value
* args   : int    imax0   I   maximum number of data points (default: 10)
*          float  data0   I   initial value (default: 0.0)
* return : none
*-----------------------------------------------------------------------------*/
STA_VAR::STA_VAR(int imax0, float data0) {
	imax = min(imax0, VARMAX);
	for (ipush = 0; ipush < imax; ipush++)	array[ipush] = data0;
	ipush = 0;
	mean = data0; var = 0.0;
}

/* update statistics with a new data point ------------------------------------
* update mean and variance with a new data point
* args   : float  data    I   new data point
*          bool   isvar   I   whether to update variance (default: 1)
* return : double         O   updated mean or variance
*-----------------------------------------------------------------------------*/
double STA_VAR::Update(float data, bool isvar) {
	array[ipush] = data;
	if (++ipush == imax) ipush = 0;
	float* p0, * p1;
	for (mean = 0.0, p0 = &array[0], p1 = &array[imax]; p0 < p1; p0++)  mean += *p0;
	mean /= imax;
	if (isvar)
	{
		for (var = 0.0, p0 = &array[0], p1 = &array[imax]; p0 < p1; p0++) { double vi = *p0 - mean; var += vi * vi; }
		var /= imax - 1;
	}
	return var;
}

/* constructor ----------------------------------------------------------------
* initialize the Welford variance calculator with number of elements
* args   : int    n_ele   I   number of elements in each data vector
* return : none
*-----------------------------------------------------------------------------*/
STA_VARS::STA_VARS(int n_ele) {
	this->n_ele = n_ele;
	num = 0;
	mean = vect(n_ele, 0);
	mean_2 = vect(n_ele, 0);
}

/* update statistics with a new data vector -----------------------------------
* update mean and variance with a new data vector
* args   : vect   data    I   new data vector
* return : vect           O   updated mean vector
*-----------------------------------------------------------------------------*/
vect STA_VARS::update(const vect data) {
	num++;
	vect delta = data - mean;
	mean += delta * (1.0 / num);
	vect delta_2 = data - mean;
	mean_2 += dotmul(delta, delta_2);
	return mean_2 * (num - 1);
}

/* default constructor --------------------------------------------------------
* initialize STA_AVPI object with default parameters
* args   : none
* return : none
*-----------------------------------------------------------------------------*/
STA_AVPI::STA_AVPI(void)
{
}

/* initialize the buffer and state --------------------------------------------
* initialize attitude, velocity, position buffers and parameters
* args   : const vect3 &att0   I   initial attitude
*          const vect3 &vn0    I   initial velocity
*          const vect3 &pos0   I   initial position
*          double     ts       I   time step (seconds)
*          int        num      I   number of samples to store (default: 0)
* return : none
*-----------------------------------------------------------------------------*/
void STA_AVPI::Init(const vect3& att0, const vect3& vn0, const vect3& pos0, double ts, int num)
{
	this->ts = ts;
	ipush = 0;  avpinum = num;
	for (int i = 0; i < AVPINUM; i++) { atti[i] = att0, vni[i] = vn0; posi[i] = pos0; }
	att = att0, vn = vn0, pos = pos0;
}

/* push new attitude, velocity, position sample -------------------------------
* push a new set of attitude, velocity, and position into the buffer
* args   : const vect3 &attk   I   attitude sample
*          const vect3 &vnk    I   velocity sample
*          const vect3 &posk   I   position sample
* return : none
*-----------------------------------------------------------------------------*/
void STA_AVPI::Push(const vect3& attk, const vect3& vnk, const vect3& posk)
{
	if (++ipush >= avpinum) ipush = 0;
	atti[ipush] = attk; vni[ipush] = vnk; posi[ipush] = posk;
}

/* interpolate state at a past time -------------------------------------------
* interpolate attitude, velocity, and position at a specified time in the past
* args   : double     tpast    I   time in the past to interpolate (seconds)
*          int        avp      I   flag for which states to interpolate (default: 0x7)
* return : status or index of interpolation
*-----------------------------------------------------------------------------*/
int STA_AVPI::Interp(double tpast, int avp)
{
	int res = 1, k, k1, k2;
	if (tpast < -avpinum * ts) tpast = -avpinum * ts; else if (tpast > 0) tpast = 0;
	k = (int)(-tpast / ts);
	if ((k2 = ipush - k) < 0) k2 += avpinum;
	if ((k1 = k2 - 1) < 0)  k1 += avpinum;
	double tleft = -tpast - k * ts;
	if (tleft > 0.99 * ts) {
		if (avp & 0x1) att = atti[k1]; if (avp & 0x2) vn = vni[k1]; if (avp & 0x4) pos = posi[k1];
	}
	else if (tleft < 0.01 * ts) {
		if (avp & 0x1) att = atti[k2]; if (avp & 0x2) vn = vni[k2]; if (avp & 0x4) pos = posi[k2];
	}
	else {
		double b = tleft / ts, a = 1 - b;
		if (avp & 0x1) { att = b * atti[k1] + a * atti[k2]; if (normInf(att - atti[k1]) > 10.0 * DEG) res = 0; }
		if (avp & 0x2) { vn = b * vni[k1] + a * vni[k2]; }
		if (avp & 0x4) { pos = b * posi[k1] + a * posi[k2]; if (fabs(pos.j - posi[k1].j) > 1.0 * DEG) res = 0; }
	}
	return res;
}

static double conefactors[5][4] = { { 2. / 3 },
{ 9. / 20,    27. / 20 },
{ 54. / 105,  92. / 105,  214. / 105 },
{ 250. / 504, 525. / 504, 650. / 504, 1375. / 504 }
};

/* default constructor --------------------------------------------------------
* initialize IMU object with default parameters
* args   : none
* return : none
*-----------------------------------------------------------------------------*/
IMU::IMU(void) {
	nSamples = 1;
	Reset();
	phim = dvbm = wmm = vmm = swmm = svmm = wm_1 = vm_1 = O31;
	pSf = NULL;
	pTempArray = NULL;
	pKga = pgSens = pgSens2 = pgSensX = NULL;
	pKa2 = NULL;
	prfu = NULL;
	plv = NULL;
	pCba = NULL;
	tk = tGA = 0.0;
	Kg = Ka = I33;
	eb = db = lvx = lvy = lvz = Ka2 = Q11 = Q12 = Q13 = Q21 = Q22 = Q23 = Q31 = Q32 = Q33 = O31;
	SSx = SSy = SSz = mat3(0.0);
}

/* set RFU string -------------------------------------------------------------
* set the RFU (Right-Forward-Up) string
* args   : const char *rfu0    I   RFU string
* return : none
*-----------------------------------------------------------------------------*/
void IMU::SetRFU(const char* rfu0) {
	for (int i = 0; i < 3; i++) rfu[i] = rfu0[i];
	prfu = rfu;
}

/* set scale factor nonlinearity ----------------------------------------------
* set gyroscope and accelerometer scale factor nonlinearity
* args   : const vect3 &Sfg0   I   gyroscope scale factor nonlinearity
*          const vect3 &Sfa0   I   accelerometer scale factor nonlinearity
* return : none
*-----------------------------------------------------------------------------*/
void IMU::SetSf(const vect3& Sfg0, const vect3& Sfa0) {
	Sfg = Sfg0;
	Sfa = Sfa0;
	pSf = &Sfg;
}

/* set temperature array ------------------------------------------------------
* set pointer to temperature array and type
* args   : double *tempArray0  I   pointer to temperature array
*          int    type         I   type of temperature compensation (default: 1)
* return : none
*-----------------------------------------------------------------------------*/
void IMU::SetTemp(double* tempArray0, int type) {
	pTempArray = tempArray0;
	iTemp = 0;
	double* p = &Kg.e00, * p1 = tempArray0;
	for (int k = 0; k < 37; k++, p++, p1 += 5) *p = *p1;
	pKga = &Kg;
	if (type > 1) pKa2 = &Ka2;
	if (type > 2) plv = &lvx;
}

/* set scale factor and bias matrices -----------------------------------------
* set gyroscope and accelerometer scale factor and bias
* args   : const mat3 &Kg0     I   gyroscope scale factor matrix
*          const vect3 eb0     I   gyroscope bias
*          const mat3 &Ka0     I   accelerometer scale factor matrix
*          const vect3 &db0    I   accelerometer bias
* return : none
*-----------------------------------------------------------------------------*/
void IMU::SetKga(const mat3& Kg0, const vect3 eb0, const mat3& Ka0,
	const vect3& db0) {
	Kg = Kg0;
	eb = eb0;
	Ka = Ka0;
	db = db0;
	pKga = &Kg;
}

/* set gyroscope g-sensitivity matrices ---------------------------------------
* set gyroscope g-sensitivity matrices
* args   : const mat3 &gSens0  I   primary g-sensitivity matrix
*          const mat3 &gSens20 I   secondary g-sensitivity matrix
*          const mat3 &gSensX0 I   cross g-sensitivity matrix
* return : none
*-----------------------------------------------------------------------------*/
void IMU::SetgSens(const mat3& gSens0, const mat3& gSens20, const mat3& gSensX0) {
	gSens = gSens0;
	pgSens = &gSens;
	mat3 O33 = mat3(0.0);
	if (&gSens20 != &O33) {
		gSens2 = gSens20;
		pgSens2 = &gSens2;
	}
	if (&gSensX0 != &O33) {
		gSensX = gSensX0;
		pgSensX = &gSensX;
	}
}

/* set accelerometer quadratic nonlinearity -----------------------------------
* set accelerometer quadratic nonlinearity coefficient
* args   : const vect3 &Ka20   I   quadratic nonlinearity coefficient
* return : none
*-----------------------------------------------------------------------------*/
void IMU::SetKa2(const vect3& Ka20) {
	Ka2 = Ka20;
	pKa2 = &Ka2;
}

/* set lever arm and time delay -----------------------------------------------
* set lever arm vectors and time delay between gyro and accelerometer
* args   : const vect3 &lvx0   I   lever arm in X direction
*          const vect3 &lvy0   I   lever arm in Y direction
*          const vect3 &lvz0   I   lever arm in Z direction (default: O31)
*          double tGA0         I   time delay (default: 0.0)
* return : none
*-----------------------------------------------------------------------------*/
void IMU::SetLvtGA(const vect3& lvx0, const vect3& lvy0, const vect3& lvz0, double tGA0) {
	lvx = lvx0;
	lvy = lvy0;
	lvz = lvz0;
	tGA = tGA0;
	plv = &lvx;
}

/* set body-to-antenna rotation matrix ----------------------------------------
* set the body-to-antenna rotation matrix
* args   : const mat3 &Cba0    I   body-to-antenna rotation matrix
* return : none
*-----------------------------------------------------------------------------*/
void IMU::SetCba(const mat3& Cba0) {
	Cba = Cba0;
	pCba = &Cba;
	mat3 U = inv(~Cba);
	vect3 V1 = Cba.GetClm(0), V2 = Cba.GetClm(1), V3 = Cba.GetClm(2);
	Q11 = U.e00 * V1, Q12 = U.e01 * V2, Q13 = U.e02 * V3,
		Q21 = U.e10 * V1, Q22 = U.e11 * V2, Q23 = U.e12 * V3,
		Q31 = U.e20 * V1, Q32 = U.e21 * V2, Q33 = U.e22 * V3;
	SetLvtGA(lvx, lvy, lvz, tGA);  // just open plv
}

/* reset IMU state ------------------------------------------------------------
* reset all IMU parameters and buffers to default values
* args   : none
* return : none
*-----------------------------------------------------------------------------*/
void IMU::Reset(void) {
	preFirst = onePlusPre = preWb = true;
	swmm = svmm = wm_1 = vm_1 = O31;
}

/* update IMU state with new measurements -------------------------------------
* update IMU state with new angular rate and velocity measurements
* args   : const vect3 *pwm    I   pointer to angular rate measurements
*          const vect3 *pvm    I   pointer to velocity measurements
*          int    nSamples     I   number of samples
*          double ts           I   time step
* return : updated time
*-----------------------------------------------------------------------------*/
double IMU::Update(const vect3* pwm, const vect3* pvm, int nSamples, double ts) {
	int i;
	double* pcf = conefactors[nSamples - 2];
	vect3 cm(0.0), sm(0.0);
	wmm = O31, vmm = O31;

	assert(nSamples > 0 && nSamples < 6);
	this->nSamples = nSamples;
	nts = nSamples * ts;
	_nts = 1.0 / nts;
	tk += nts;
	if (nSamples == 1 && onePlusPre)  // one-plus-previous sample
	{
		if (preFirst) {
			wm_1 = pwm[0];
			vm_1 = pvm[0];
			preFirst = false;
		}
		cm = 1.0 / 12 * wm_1;
		sm = 1.0 / 12 * vm_1;
	}
	for (i = 0; i < nSamples - 1; i++) {
		cm += pcf[i] * pwm[i];
		sm += pcf[i] * pvm[i];
		wmm += pwm[i];
		vmm += pvm[i];
	}
	wm_1 = pwm[i];
	vm_1 = pvm[i];
	wmm += pwm[i];
	swmm += wmm;
	vmm += pvm[i];
	svmm += vmm;
	phim = wmm + cm * pwm[i];
	dvbm = vmm + 1.0 / 2 * wmm * vmm + (cm * pvm[i] + sm * pwm[i]);
	if (pSf) {
		phim = dotmul(phim, Sfg);
		dvbm = dotmul(dvbm, Sfa);
	}
	if (pTempArray) {
		//            double *p=&pTempArray[iTemp*5];
		//            *(&Kg.e00+iTemp) = p[0] + polyval(&p[1], 3, Temp);
		//            if(++iTemp==37) iTemp=0;  // (&tGA-&Kg.e00)==37
	}
	if (pKga) {
		phim = Kg * phim - eb * nts;   // Kg ~= Ka ~= I33
		dvbm = Ka * dvbm - db * nts;
	}
	if (pgSens) {
		phim.i -= gSens.e00 * dvbm.i + gSens.e01 * dvbm.j + gSens.e02 * dvbm.k;   // gSens.eij in (rad/s)/(m/ss)
		phim.j -= gSens.e10 * dvbm.i + gSens.e11 * dvbm.j + gSens.e12 * dvbm.k;
		phim.k -= gSens.e20 * dvbm.i + gSens.e21 * dvbm.j + gSens.e22 * dvbm.k;
	}
	if (pgSens2) {
		double fx2_Ts = dvbm.i * dvbm.i * _nts, fy2_Ts = dvbm.j * dvbm.j * _nts, fz2_Ts = dvbm.k * dvbm.k * _nts;
		phim.i -=
			gSens2.e00 * fx2_Ts + gSens2.e01 * fy2_Ts + gSens2.e02 * fz2_Ts;   // gSens2.eij in (rad/s)/(m/ss)^2
		phim.j -= gSens2.e10 * fx2_Ts + gSens2.e11 * fy2_Ts + gSens2.e12 * fz2_Ts;
		phim.k -= gSens2.e20 * fx2_Ts + gSens2.e21 * fy2_Ts + gSens2.e22 * fz2_Ts;
	}
	if (pgSensX) {
		double fxy_Ts = dvbm.i * dvbm.j * _nts, fyz_Ts = dvbm.j * dvbm.k * _nts, fzx_Ts = dvbm.k * dvbm.i * _nts;
		phim.i -=
			gSensX.e00 * fxy_Ts + gSensX.e01 * fyz_Ts + gSensX.e02 * fzx_Ts;   // gSensX.eij in (rad/s)/(m/ss)^2
		phim.j -= gSensX.e10 * fxy_Ts + gSensX.e11 * fyz_Ts + gSensX.e12 * fzx_Ts;
		phim.k -= gSensX.e20 * fxy_Ts + gSensX.e21 * fyz_Ts + gSensX.e22 * fzx_Ts;
	}
	if (pKa2) {
		dvbm.i -= Ka2.i * dvbm.i * dvbm.i * _nts;   // Ka2.i in (m/ss)/(m/ss)^2
		dvbm.j -= Ka2.j * dvbm.j * dvbm.j * _nts;
		dvbm.k -= Ka2.k * dvbm.k * dvbm.k * _nts;
	}
	if (plv) {
		if (preWb) { wb_1 = phim * _nts, preWb = false; }
		vect3 wb = phim * _nts, dwb = (wb - wb_1) * _nts;
		wb_1 = wb;
		mat3 W = askew(dwb) + pow(askew(wb), 2);
		vect3 fL;
		if (pCba) {
			SSx = mat3(Q11 * W, Q21 * W, Q31 * W);
			SSy = mat3(Q12 * W, Q22 * W, Q32 * W);
			SSz = mat3(Q13 * W, Q23 * W, Q33 * W);
			fL = SSx * lvx + SSy * lvy + SSz * lvz;
		}
		else
			fL = vect3(dot(*(vect3*)&W.e00, lvx), dot(*(vect3*)&W.e10, lvy),
				dot(*(vect3*)&W.e20, lvz));
		dvbm -= fL * nts + tGA * (wb * dvbm);
	}
	if (prfu) IMURFU(&phim, &dvbm, 1, prfu);
	return tk;
}

/* apply RFU transformation to angular rates ---------------------------------
* apply RFU (Right-Forward-Up) transformation to angular rate measurements
* args   : vect3 *pwm          IO  pointer to angular rate measurements
*          int    nSamples     I   number of samples
*          const char *str     I   RFU string
* return : none
*-----------------------------------------------------------------------------*/
void IMU::IMURFU(vect3* pwm, int nSamples, const char* str) {
	if (str[0] == 'X') return;
	for (int n = 0; n < nSamples; n++) {
		vect3 tmpwm;
		double* pw = (double*)&pwm[n].i;
		for (int i = 0; i < 3; i++, pw++) {
			switch (str[i]) {
			case 'R':
				tmpwm.i = *pw;
				break;
			case 'L':
				tmpwm.i = -*pw;
				break;
			case 'F':
				tmpwm.j = *pw;
				break;
			case 'B':
				tmpwm.j = -*pw;
				break;
			case 'U':
				tmpwm.k = *pw;
				break;
			case 'D':
				tmpwm.k = -*pw;
				break;
			}
		}
		pwm[n] = tmpwm;
	}
}

/* apply RFU transformation to angular rates and velocities -------------------
* apply RFU (Right-Forward-Up) transformation to angular rate and velocity measurements
* args   : vect3 *pwm          IO  pointer to angular rate measurements
*          vect3 *pvm          IO  pointer to velocity measurements
*          int    nSamples     I   number of samples
*          const char *str     I   RFU string
* return : none
*-----------------------------------------------------------------------------*/
void IMU::IMURFU(vect3* pwm, vect3* pvm, int nSamples, const char* str) {
	if (str[0] == 'X') return;
	IMURFU(pwm, nSamples, str);
	IMURFU(pvm, nSamples, str);
}

/* static IMU alignment -------------------------------------------------------
* perform static alignment using mean angular rate and velocity
* args   : vect3 &wm           O   mean angular rate
*          vect3 &vm           O   mean velocity
*          vect3 &att0         O   initial attitude
*          vect3 &pos0         O   initial position
*          double ts           I   time step
* return : none
*-----------------------------------------------------------------------------*/
void IMUStatic(vect3& wm, vect3& vm, vect3& att0, vect3& pos0, double ts) {
	earth eth;
	eth.Update(pos0, O31);
	mat3 Cbn = ~a2mat(att0);
	wm = Cbn * eth.wnie * ts;
	vm = -Cbn * eth.gn * ts;
}

/* lever arm compensation for angular rates -----------------------------------
* lever arm compensation for angular rates using body-frame and navigation-frame angular rates
* args   : mat3 &wfb           I   body-frame angular rate matrix
*          mat3 &wfn           I   navigation-frame angular rate matrix
* return : compensated angular rate matrix
*-----------------------------------------------------------------------------*/
mat lsclbt(mat& wfb, mat& wfn) {
	mat A(wfb.row, 4, 0.0), Kga_edb(4, 3, 0.0);
	vect Y(wfb.row), X;
	for (int k = 0; k < 3; k++) {
		for (int i = 0; i < wfb.row; i++) {
			A.SetRow(i, vect(4, wfb(i, 0), wfb(i, 1), wfb(i, 2), -1.0));
			Y(i) = wfn(i, k);
		}
		X = inv4((~A) * A) * ((~A) * Y);
		Kga_edb(k, 0) = X(0), Kga_edb(k, 1) = X(1), Kga_edb(k, 2) = X(2), Kga_edb(3, k) = X(3);
	}
	return Kga_edb;
}

/* constructor (yaw, position, time) ------------------------------------------
* initialize INS with yaw angle, position, and initial time
* args   : double&    yaw0     I   initial yaw angle (radians)
*          vect3      pos0     I   initial position (default: O31)
*          double     tk0      I   initial time (default: 0.0)
* return : none
*-----------------------------------------------------------------------------*/
INS::INS(double& yaw0, const vect3& pos0, double tk0) {
	Init(a2qua(vect3(0, 0, yaw0)), O31, pos0, tk0);
}

/* constructor (attitude, velocity, position, time) ---------------------------
* initialize INS with attitude, velocity, position, and initial time
* args   : vect3      att0     I   initial attitude {roll, pitch, yaw}
*          vect3      vn0      I   initial velocity (default: O31)
*          vect3      pos0     I   initial position (default: O31)
*          double     tk0      I   initial time (default: 0.0)
* return : none
*-----------------------------------------------------------------------------*/
INS::INS(const vect3& att0, const vect3& vn0, const vect3& pos0, double tk0) {
	Init(a2qua(att0), vn0, pos0, tk0);
}

/* constructor (quaternion, velocity, position, time) -------------------------
* initialize INS with quaternion, velocity, position, and initial time
* args   : quat       qnb0     I   initial attitude quaternion (default: qI)
*          vect3      vn0      I   initial velocity (default: O31)
*          vect3      pos0     I   initial position (default: O31)
*          double     tk0      I   initial time (default: 0.0)
* return : none
*-----------------------------------------------------------------------------*/
INS::INS(const quat& qnb0, const vect3& vn0, const vect3& pos0, double tk0) {
	Init(qnb0, vn0, pos0, tk0);
}

/* initialize INS state -------------------------------------------------------
* initialize INS state using quaternion, velocity, and position
* args   : quat       qnb0     I   initial attitude quaternion (default: qI)
*          vect3      vn0      I   initial velocity (default: O31)
*          vect3      pos0     I   initial position (default: O31)
*          double     tk0      I   initial time (default: 0.0)
* return : none
*-----------------------------------------------------------------------------*/
void INS::Init(const quat& qnb0, const vect3& vn0, const vect3& pos0, double tk0) {
	tk = tk0;
	ts = nts = 1.0;
	dist = -EPS;
	afabar = 0.1;
	qnb = qnb0;
	vn = vn0, pos = pos0;
	Kg = Ka = I33;
	eb = db = Ka2 = O31;
	Maa = Mav = Map = Mva = Mvv = Mvp = Mpv = Mpp = mat3(0.0);
	SetTauGA(vect3(INF), vect3(INF));
	vect3 wib(0.0), fb = (~qnb) * vect3(0, 0, G0);
	lvr = an = anbar = webbar = O31;
	isOpenloop = isMemsgrade = isNocompasseffect = isOutlever = 0;
	Cbn = I33;
	Update(&wib, &fb, 1, 1.0);
	imu.preFirst = 1;
	tk = tk0;
	ts = nts = 1.0;
	qnb = qnb0;
	vn = vn0, pos = pos0;
	mmwb = mmfb = maxmin();
	mvn = mvnmax = mvnmin = vn;
	mvni = O31;
	mvnt = mvnT = 0.0;
	mvnk = 0;
	etm();
	lever();
	lever2();
	Extrap();
	mmwb.Init();
	mmfb.Init();
}

/* set Markov process time constants ------------------------------------------
* set Markov process correlation times for gyro and accelerometer
* args   : vect3      tauG     I   gyro correlation time
*          vect3      tauA     I   accelerometer correlation time
* return : none
*-----------------------------------------------------------------------------*/
void INS::SetTauGA(const vect3& tauG, const vect3& tauA) {
	if (tauG.i > EPS) {
		tauGyro = tauG;
		_betaGyro.i = tauG.i > INFp5 ? 0.0 : -1.0 / tauG.i;   // Gyro&Acc inverse correlation time for AR(1) model
		_betaGyro.j = tauG.j > INFp5 ? 0.0 : -1.0 / tauG.j;
		_betaGyro.k = tauG.k > INFp5 ? 0.0 : -1.0 / tauG.k;
	}
	if (tauA.i > EPS) {
		tauAcc = tauA;
		_betaAcc.i = tauA.i > INFp5 ? 0.0 : -1.0 / tauA.i;
		_betaAcc.j = tauA.j > INFp5 ? 0.0 : -1.0 / tauA.j;
		_betaAcc.k = tauA.k > INFp5 ? 0.0 : -1.0 / tauA.k;
	}
}

/* SINS update using Gyro & Acc samples ---------------------------------------
* update INS state using arrays of gyro and accelerometer samples
* args   : vect3*     pwm      I   array of angular rate increments
*          vect3*     pvm      I   array of velocity increments
*          int        nSamples I   number of samples
*          double     ts       I   time step
* return : none
*-----------------------------------------------------------------------------*/
void INS::Update(const vect3* pwm, const vect3* pvm, int nSamples, double ts) {
	if (isMemsgrade) {
		this->ts = ts;
		nts = nSamples * ts;
		tk += nts;
		double nts2 = nts / 2;
		imu.Update(pwm, pvm, nSamples, ts);
		imu.phim = Kg * imu.phim - eb * nts;
		imu.dvbm = Ka * imu.dvbm - db * nts;  // IMU calibration
		//		CVect3 vn01 = vn+an*nts2, pos01 = pos+eth.vn2dpos(vn01,nts2);
		if (!isOpenloop) eth.Update(pos, O31);
		wib = imu.phim / nts;
		fb = imu.dvbm / nts;
		web = wib;
		webbar = (1 - afabar) * webbar + afabar * web;
		wnb = wib;
		fn = qnb * fb;
		an = fn + eth.gcc;
		anbar = (1 - afabar) * anbar + afabar * an;
		vect3 vn1 = vn + an * nts;
		pos = pos + eth.vn2dpos(vn + vn1, nts2);
		vn = vn1;
		qnb = qnb * rv2q(imu.phim);
		Cnb = q2mat(qnb);
		att = m2att(Cnb);
		Cbn = ~Cnb;
		vb = Cbn * vn;
	}
	else {
		this->ts = ts;
		nts = nSamples * ts;
		tk += nts;
		double nts2 = nts / 2;
		imu.Update(pwm, pvm, nSamples, ts);
		imu.phim = Kg * imu.phim - eb * nts;
		imu.dvbm = Ka * imu.dvbm - db * nts;  // IMU calibration
		vect3 vn01 = vn + an * nts2, pos01 = pos + eth.vn2dpos(vn01, nts2);
		if (!isOpenloop) eth.Update(pos01, vn01);
		wib = imu.phim / nts;
		fb = imu.dvbm / nts;
		web = wib - Cbn * eth.wnie;
		webbar = (1 - afabar) * webbar + afabar * web;
		//		wnb = wib - (~(qnb*rv2q(imu.phim/2)))*eth.wnin;
		wnb = wib - Cbn * eth.wnin;
		fn = qnb * fb;
		an = rv2q(-eth.wnin * nts2) * fn + eth.gcc;
		anbar = (1 - afabar) * anbar + afabar * an;
		vect3 vn1 = vn + an * nts;
		pos = pos + eth.vn2dpos(vn + vn1, nts2);
		vn = vn1;
		qnb = rv2q(-eth.wnin * nts) * qnb * rv2q(imu.phim);
		Cnb = q2mat(qnb);
		att = m2att(Cnb);
		Cbn = ~Cnb;
		vb = Cbn * vn;
	}
	assert(pos.i < 85.0 * DEG && pos.i > -85.0 * DEG);
	if (vn.i > velMax) vn.i = velMax; else if (vn.i < -velMax) vn.i = -velMax;
	if (vn.j > velMax) vn.j = velMax; else if (vn.j < -velMax) vn.j = -velMax;
	if (vn.k > velMax) vn.k = velMax; else if (vn.k < -velMax) vn.k = -velMax;
	if (pos.i > latMax) pos.i = latMax; else if (pos.i < -latMax) pos.i = -latMax;
	if (pos.j > PI) pos.j -= _2PI; else if (pos.j < -PI) pos.j += _2PI;
	if (pos.k > hgtMax) pos.k = hgtMax; else if (pos.k < hgtMin) pos.k = hgtMin;
	if (mvnT > EPS) {   // calculate max/min/mean-vn within mvnT
		if (mvnk == 0) {
			mvnmax = mvnmin = vn;
			mvni = O31;
			mvnt = 0.0;
			mvnCnb0 = Cnb;
		}
		else {
			if (vn.i > mvnmax.i) mvnmax.i = vn.i; else if (vn.i < mvnmin.i) mvnmin.i = vn.i;
			if (vn.j > mvnmax.j) mvnmax.j = vn.j; else if (vn.j < mvnmin.j) mvnmin.j = vn.j;
			if (vn.k > mvnmax.k) mvnmax.k = vn.k; else if (vn.k < mvnmin.k) mvnmin.k = vn.k;
		}
		mvni += vn;
		mvnt += nts;
		mvnk++;
		if (mvnt >= mvnT) {
			mvn = mvni * (1.0 / mvnk);
			mvnk = 0;   // OK if mvnk==0
		}
	}
	if (dist >= 0.0)
		dist += sqrt(vn.i * vn.i + vn.j * vn.j + vn.k * vn.k) * nts;  // travel distance ^2
	mmwb.Update((float)norm(imu.wmm / nts));
	mmfb.Update((float)(norm(imu.vmm / nts) + eth.gn.k));
}

/* SINS fast extrapolation using 1 Gyro & Acc sample --------------------------
* perform fast extrapolation of INS state using one gyro and accelerometer sample
* args   : vect3      wm       I   angular rate increment (default: O31)
*          vect3      vm       I   velocity increment (default: O31)
*          double     ts       I   time step (default: 0.0)
* return : none
*-----------------------------------------------------------------------------*/
void INS::Extrap(const vect3& wm, const vect3& vm, double ts) {
	if (ts < 1.0e-6)  // reset
	{
		qnbE = qnb, vnE = vn, posE = pos, attE = att;
	}
	else {
		vnE = vnE + qnbE * vm + eth.gcc * ts;
		posE = posE + eth.vn2dpos(vnE, ts);
		qnbE = qnbE * rv2q(wm);
		attE = q2att(qnbE);
	}
}

/* SINS fast extrapolation using previous Gyro & Acc sample -------------------
* perform fast extrapolation of INS state using previous gyro and accelerometer sample
* args   : double     extts    I   extrapolation time step
* return : none
*-----------------------------------------------------------------------------*/
void INS::Extrap(double extts) {
	double k = extts / nts;
	vnE = vn + qnb * imu.dvbm * k + eth.gcc * extts;
	posE = pos + eth.vn2dpos(vn, extts);
	attE = q2att(qnb * rv2q(imu.phim * k));
}

/* lever arm compensation -----------------------------------------------------
* apply lever arm compensation to position and velocity
* args   : vect3      dL       I   lever arm vector (default: O31)
*          vect3*     ppos     O   pointer to output position (optional)
*          vect3*     pvn      O   pointer to output velocity (optional)
* return : none
*-----------------------------------------------------------------------------*/
void INS::lever(const vect3& dL, vect3* ppos, vect3* pvn) {
	if (&dL != &O31) lvr = dL;
	//	Mpv = CMat3(0,eth.f_RMh,0, eth.f_clRNh,0,0, 0,0,1);
	Mpv.e01 = eth.f_RMh, Mpv.e10 = eth.f_clRNh, Mpv.e22 = 1.0;
	CW = Cnb * askew(web), MpvCnb = Mpv * Cnb;
	if (ppos == NULL) {
		posL = pos + MpvCnb * lvr;
		if (pvn == NULL) vnL = vn + CW * lvr;
	}
	else {
		*ppos = pos + MpvCnb * lvr;
		if (pvn != NULL) *pvn = vn + CW * lvr;
	}
}

/* lever arm compensation (alternative) ---------------------------------------
* apply lever arm compensation to position and velocity (alternative method)
* args   : vect3      dL2      I   lever arm vector (default: O31)
*          vect3*     ppos2    O   pointer to output position (optional)
*          vect3*     pvn2     O   pointer to output velocity (optional)
* return : none
*-----------------------------------------------------------------------------*/
void INS::lever2(const vect3& dL2, vect3* ppos2, vect3* pvn2) {
	if (&dL2 != &O31) lvr = dL2;
	//	Mpv = CMat3(0,eth.f_RMh,0, eth.f_clRNh,0,0, 0,0,1);
	Mpv.e01 = eth.f_RMh, Mpv.e10 = eth.f_clRNh, Mpv.e22 = 1.0;
	CW = Cnb * askew(web), MpvCnb = Mpv * Cnb;
	if (ppos2 == NULL) {
		;
		//*ppos2 = *ppos2 + MpvCnb*lvr;
		//if(pvn2==NULL)  *pvn2 = *pvn2+ CW*lvr;
	}
	else {
		*ppos2 = *ppos2 + MpvCnb * lvr;
		if (pvn2 != NULL) *pvn2 = *pvn2 + CW * lvr;
	}
}

/* SINS error transform matrix coefficients -----------------------------------
* compute error transform matrix coefficients for SINS
* args   : none
* return : none
*-----------------------------------------------------------------------------*/
void INS::etm(void) {
	if (isMemsgrade) {
		Mva = askew(fn);
		Mpv = mat3(0, eth.f_RMh, 0, eth.f_clRNh, 0, 0, 0, 0, 1);
	}
	else {
		double tl = eth.tl, secl = 1.0 / eth.cl, secl2 = secl * secl,
			wN = eth.wnie.j, wU = eth.wnie.k, vE = vn.i, vN = vn.j;
		double f_RMh = eth.f_RMh, f_RNh = eth.f_RNh, f_clRNh = eth.f_clRNh,
			f_RMh2 = f_RMh * f_RMh, f_RNh2 = f_RNh * f_RNh;
		mat3 Avn = askew(vn),
			Mp1(0, 0, 0, -wU, 0, 0, wN, 0, 0),
			Mp2(0, 0, vN * f_RMh2, 0, 0, -vE * f_RNh2, vE * secl2 * f_RNh, 0, -vE * tl * f_RNh2);
		if (isNocompasseffect) Maa = mat3(0.0);
		else Maa = askew(-eth.wnin);
		Mav = mat3(0, -f_RMh, 0, f_RNh, 0, 0, tl * f_RNh, 0, 0);
		Map = Mp1 + Mp2;
		Mva = askew(fn);
		Mvv = Avn * Mav - askew(eth.wnie + eth.wnin);
		Mvp = Avn * (Mp1 + Map);
		double scl = eth.sl * eth.cl;
		Mvp.e20 = Mvp.e20 - G0 * (5.27094e-3 * 2 * scl + 2.32718e-5 * 4 * eth.sl2 * scl);
		Mvp.e22 = Mvp.e22 + 3.086e-6;
		Mpv = mat3(0, f_RMh, 0, f_clRNh, 0, 0, 0, 0, 1);
		Mpp = mat3(0, 0, -vN * f_RMh2, vE * tl * f_clRNh, 0, -vE * secl * f_RNh2, 0, 0, 0);
	}
}

/* add error to INS state -----------------------------------------------------
* add error vectors to attitude, velocity, and position
* args   : vect3      phi      I   attitude error
*          vect3      dvn      I   velocity error (default: O31)
*          vect3      dpos     I   position error (default: O31)
* return : none
*-----------------------------------------------------------------------------*/
void INS::AddErr(const vect3& phi, const vect3& dvn, const vect3& dpos) {
	qnb -= -phi;
	vn += dvn;
	pos += vect3(dpos.i * eth.f_RMh, dpos.j * eth.f_clRNh, dpos.k);  // NOTE: dpos in meter
}

/* add error to INS state (scalar attitude error) -----------------------------
* add scalar attitude error and error vectors to velocity and position
* args   : double     phiU     I   scalar attitude error
*          vect3      dvn      I   velocity error (default: O31)
*          vect3      dpos     I   position error (default: O31)
* return : none
*-----------------------------------------------------------------------------*/
void INS::AddErr(double phiU, const vect3& dvn, const vect3& dpos) {
	AddErr(vect3(0, 0, phiU), dvn, dpos);
}

//void INS::DebugStop(double t1, int absT, int ext) {
//	static double dst0 = -INF;
//	if (dst0 < -INFp5) dst0 = tk;
//	double t = tk - dst0 * absT;  // absT=0 for absolute time
//	if (t1 < t && t < t1 + 3 * nts) {
//		if (ext == 0)
//			t1 = tk - dst0 * absT;  // please place breakpoint here
//		else
//			exit(0);  // for exit
//	}
//}


void IMURFUTest(const double* pwm, const char* str_in, const char* str_out, double* wmout)
{
	double tmpwm[3] = { 99, 99, 99 };
	const double* pw = pwm;
	int i;
	for (i = 0; i < 3; i++, pw++) {
		switch (str_in[i]) {
		case 'R':
			tmpwm[0] = *pw;
			break;
		case 'L':
			tmpwm[0] = -*pw;
			break;
		case 'F':
			tmpwm[1] = *pw;
			break;
		case 'B':
			tmpwm[1] = -*pw;
			break;
		case 'U':
			tmpwm[2] = *pw;
			break;
		case 'D':
			tmpwm[2] = -*pw;
			break;
		}
	}
	for (i = 0; i < 3; i++) {
		switch (str_out[i]) {
		case 'R':
			wmout[i] = tmpwm[0];
			break;
		case 'L':
			wmout[i] = -tmpwm[0];
			break;
		case 'F':
			wmout[i] = tmpwm[1];
			break;
		case 'B':
			wmout[i] = -tmpwm[1];
			break;
		case 'U':
			wmout[i] = tmpwm[2];
			break;
		case 'D':
			wmout[i] = -tmpwm[2];
			break;
		}
	}
	return;
}


/* initialize odometry state --------------------------------------------------
* initialize all odometry parameters and buffers
* args   : none
* return : initialization status
*-----------------------------------------------------------------------------*/
int odo::Init()
{
	SetDistance(0.0);
	odmeasT = 1.0;
	lvOD = vnOD = O31;
	ODKappa(vect3(0, 1, 0));
	odmeasOK = badODCnt = 0;
	return 1;
}

/* update odometry with new distance measurement ------------------------------
* update odometry state with a new distance increment
* args   : double     dS      I   distance increment
* return : update status
*-----------------------------------------------------------------------------*/
int odo::Update(double dS) {
	if (dS < -10.0 || dS > 10.0) {
		dS = dS0;
		badODCnt++;
		return odmeasOK = 0;
	}
	else { badODCnt = 0; };
	dS0 = dS;  // if bad OD input
	if (odmeast < EPS || badODCnt > 5) {   // re-initialize
		IVno = IVni = Ifn = O31;
		IMv = odMlever = mat3(0.0);
	}
	dS *= Kod;
	distance += dS;
	if (++distK0 == distN) { distK0 = 0; }
	distT01 = distance - distances[distK0];
	distances[distK0] = distance;  // distT01=dist increment within [T0,T1]
	odVel = distT01 / (distN * (*nts));
	vect3 dSn =
		*Cnb * (vect3(Cbo.e01 * dS, Cbo.e11 * dS, Cbo.e21 * dS) - *phim * lvOD);  // dSn = Cnb*Cbo*dSb
	odmeast += *nts;
	if (odmeast >= odmeasT) {  // take measurement every (odmeasT)(s)
		odMphi = -askew(odmeast / 2.0 * Ifn + IVno);
		odMphi.e02 *= Hkv2y, odMphi.e12 *= Hkv2y;
		odMvn = odmeast * I33;
		odMkappa = mat3(IMv.e02, -IMv.e01, -IMv.e00,
			IMv.e12, -IMv.e11, -IMv.e10,
			IMv.e22, -IMv.e21, -IMv.e20);
		//odMlever = sins.Cnb*askew(-odmeast*sins.web);
		odZk = IVni - IVno;
		odmeast = 0.0;
		return odmeasOK = 1;
	}
	else {
		IVno += dSn;
		IVni += (*vn) * (*nts);
		Ifn += *fn * *nts;
		IMv += dS * *Cnb;
		odMlever += *Cnb * askew(*web * (-*nts));
		return odmeasOK = 0;
	}
}

/* reset odometry state -------------------------------------------------------
* reset all odometry parameters and buffers to default values
* args   : none
* return : reset status
*-----------------------------------------------------------------------------*/
int odo::Reset()
{
	odmeast = 0.0;  odmeasT = 1.0;  dS0 = 0.0;  Hkv2y = 1.0;
	lvOD = vnOD = O31;  ODKappa(vect3(0, 1, 0));
	odmeasOK = badODCnt = 0;

	return 1;
}

/* calculate odometry kappa ---------------------------------------------------
* calculate odometry kappa (curvature) from input vector
* args   : const vect3 &kpp   I   input vector
* return : kappa vector
*-----------------------------------------------------------------------------*/
vect3 odo::ODKappa(const vect3& kpp)
{
	vect3 res;
	if (&kpp == &O31) {		// get
		res = m2att(Cbo);  res.j = Kod;  // Kappa = [dPitch; Kod; dYaw]
	}
	else {					// set
		Kod = kpp.j;
		Cbo = a2mat(vect3(kpp.i, 0.0, kpp.k));
		res = O31;
	}
	return res;
}

/* set total odometry distance ------------------------------------------------
* set the total odometry distance
* args   : double     dist    I   distance to set
* return : none
*-----------------------------------------------------------------------------*/
void odo::SetDistance(double dist)
{
	distance = dist;
	for (int k = 0; k < distN; k++) { distances[k] = distance; }
	distK0 = 0; distT01 = 0.0; odVel = 0.0;
}

/* default constructor --------------------------------------------------------
* initialize IIR filter with zero coefficients and state
* args   : none
* return : none
*-----------------------------------------------------------------------------*/
IIR::IIR(void)
{
}

/* parameterized constructor --------------------------------------------------
* initialize IIR filter with given coefficients and order
* args   : double*    b0      I   numerator coefficients array
*          double*    a0      I   denominator coefficients array
*          int        n0      I   filter order
* return : none
*-----------------------------------------------------------------------------*/
IIR::IIR(double* b0, double* a0, int n0)
{
	assert(n0 < IIRnMax);
	for (int i = 0; i < n0; i++) { b[i] = b0[i] / a0[0]; a[i] = a0[i]; x[i] = y[i] = 0.0; }
	n = n0;
}

/* update filter with new input -----------------------------------------------
* process a new input sample and update filter state
* args   : double     x0      I   new input sample
* return : double             O   filtered output
*-----------------------------------------------------------------------------*/
double IIR::Update(double x0)
{
	//	a(1)*y(n) = b(1)*x(n) + b(2)*x(n-1) + ... + b(nb+1)*x(n-nb)
	//                        - a(2)*y(n-1) - ... - a(na+1)*y(n-na)
	double y0 = 0.0;
	for (register int i = n - 1; i > 0; i--)
	{
		x[i] = x[i - 1]; y[i] = y[i - 1];
		y0 += b[i] * x[i] - a[i] * y[i];
	}
	x[0] = x0;
	y0 += b[0] * x0;
	y[0] = y0;
	return y0;
}

/* default constructor --------------------------------------------------------
* initialize 3D IIR filter with default parameters
* args   : none
* return : none
*-----------------------------------------------------------------------------*/
IIRV3::IIRV3(void)
{
}

/* parameterized constructor --------------------------------------------------
* initialize 3D IIR filter with given coefficients for each component
* args   : double*    b0      I   numerator coefficients for iir0
*          double*    a0      I   denominator coefficients for iir0
*          int        n0      I   order for iir0
*          double*    b1      I   numerator coefficients for iir1 (optional)
*          double*    a1      I   denominator coefficients for iir1 (optional)
*          int        n1      I   order for iir1 (optional)
*          double*    b2      I   numerator coefficients for iir2 (optional)
*          double*    a2      I   denominator coefficients for iir2 (optional)
*          int        n2      I   order for iir2 (optional)
* return : none
*-----------------------------------------------------------------------------*/
IIRV3::IIRV3(double* b0, double* a0, int n0, double* b1, double* a1, int n1,
	double* b2, double* a2, int n2)
{
	iir0 = IIR(b0, a0, n0);
	if (n1 == 0)	iir1 = iir0;	// iir1 the same as iir0
	else		iir1 = IIR(b1, a1, n1);
	if (n2 == 0)	iir2 = iir0;	// iir2 the same as iir0
	else		iir2 = IIR(b2, a2, n2);
}

/* update filter with new input vector ----------------------------------------
* process a new input vector and update filter state
* args   : const vect3& x     I   new input vector
* return : vect3              O   filtered output vector
*-----------------------------------------------------------------------------*/
vect3 IIRV3::Update(const vect3& x)
{
	return y = vect3(iir0.Update(x.i), iir1.Update(x.j), iir2.Update(x.k));
}

/* default constructor --------------------------------------------------------
* initialize ZIHR detector with default parameters
* args   : none
* return : none
*-----------------------------------------------------------------------------*/
ZIHR::ZIHR(void)
{
}

/* initialize ZIHR detector ---------------------------------------------------
* initialize ZIHR detector with specified parameters
* args   : double     maxw0    I   maximum allowable angular rate
*          double     ts0      I   time step (seconds)
*          double     T0       I   detection window length (seconds, default: 10.0)
*          int        cntNP0   I   initial count (default: 10)
* return : none
*-----------------------------------------------------------------------------*/
void ZIHR::Init(double maxw0, double ts0, double T0, int cntNP0)
{
	ts = ts0;  T = T0;	maxw = maxw0;	cntNP = cntNP0;
	meanw = meanwpre = meanwprepre = tstop = 0.0;
	Reset();
}

/* reset ZIHR detector --------------------------------------------------------
* reset all ZIHR parameters and counters to default values
* args   : none
* return : none
*-----------------------------------------------------------------------------*/
void ZIHR::Reset(void)
{
	t = val = 0.0;
	cntNi = cntPi = big = 0;
}
/**
At the start, tstop is incremented by a time step ts.
If tstop is less than 3 seconds, the function returns -6, signaling that not enough time has passed for meaningful processing.
Otherwise, the function checks if the new measurement wz is outside the allowed range defined by maxw.
If wz is too negative, a counter cntNi is incremented;
 if this counter exceeds a threshold cntNP, the mean angular velocities are reset, the timer is reset, and the function returns -3.
 If the threshold is not exceeded, it returns -1.
 Similarly, if wz is too positive, a counter cntPi is incremented and similar logic applies, returning -4 or -2 as appropriate.
 Each time an out-of-range value is detected, a big counter is incremented.

If wz is within the allowed range, both counters are reset and big is decremented.
The big counter is then clamped to a minimum of zero.
 If big exceeds 10, the mean values are reset and the function returns -5, indicating too many large deviations.

If the result code so far is not a strong error (i.e., retres >= -2), the function accumulates the angular velocity and time.
When the accumulated time t exceeds a threshold T, it updates the rolling mean values (meanw, meanwpre, meanwprepre),
resets the accumulators, and checks if the previous mean values are outside a small epsilon (EPS).
 Depending on these checks, it sets the return code to 1, 2, or 3, indicating different levels of stability or change in the mean angular velocity.

Overall, this function implements a robust mechanism for monitoring angular velocity, detecting outliers,
and maintaining a rolling mean, with various return codes to indicate the current state or any detected issues.
 */
 /* update ZIHR detector with new angular rate ---------------------------------
 * update ZIHR detector with a new angular rate measurement
 * args   : double     wz       I   new angular rate measurement
 * return : detection result/status
 *-----------------------------------------------------------------------------*/
int ZIHR::Update(double wz)   // wz in rad/s
{
	retres = 0;
	if ((tstop += ts) < 3.0) {
		retres = -6;
	}
	else {
		if (wz < -maxw) {
			if (++cntNi > cntNP) { meanw = meanwpre = meanwprepre = tstop = 0.0; Reset(); retres = -3; }
			else retres = -1;
			big++;
		}
		else if (wz > maxw) {
			if (++cntPi > cntNP) { meanw = meanwpre = meanwprepre = tstop = 0.0; Reset(); retres = -4; }
			else retres = -2;
			big++;
		}
		else {
			cntNi = cntPi = 0;
			big--;
		}
		if (big < 0) {
			big = 0;
		}
		else if (big > 10) {
			meanw = meanwpre = meanwprepre = 0.0; Reset(); retres = -5;
		}
		if (retres >= -2) {
			val += wz;
			t += ts;
			if (t > T) {
				meanwprepre = meanwpre;
				meanwpre = meanw;
				meanw = val * ts / T;  // meanw in rad/s
				Reset();
				if (meanwpre<-EPS || meanwpre>EPS) {
					if (meanwprepre<-EPS || meanwprepre>EPS) retres = 3;
					else retres = 2;
				}
				else {
					retres = 1;
				}
			}
		}
	}
	return retres;
}

KalmanFilter::KalmanFilter(void)
{
}

KalmanFilter::KalmanFilter(int nq0, int nr0)
{
	if (!(nq0 <= MAX_MAT_DIM && nr0 <= MAX_MAT_DIM)) { /*printf("\tMMD too small!\n");*/ }
	Init(nq0, nr0);
}

/* initialize Kalman filter ---------------------------------------------------
* initialize the Kalman filter with process and measurement noise dimensions
* args   : int        nq0      I   dimension of process noise
*          int        nr0      I   dimension of measurement noise
* return : none
*-----------------------------------------------------------------------------*/
void KalmanFilter::Init(int nq0, int nr0)
{
	kftk = 0.0;
	nq = nq0; nr = nr0;
	Ft = Pk = mat(nq, nq, 0.0);
	Hk = mat(nr, nq, 0.0);  Fading = mat(nr, nq, 1.0); zfdafa = 0.1f;
	Qt = Pmin = Xk = vect(nq, 0.0);  Xmax = Pmax = vect(nq, INF);  Pset = vect(nq, -INF);
	Zk = Zk_1 = vect(nr, 0.0);  Rt = Rt0 = vect(nr, INF); Rset = vect(nr, -INF); rts = vect(nr, 1.0);  Zfd = vect(nr, 0.0); Zfd0 = Zmm0 = Zmax = vect(nr, INF);
	innoPre = vect(nr, 0.0); innoDiffMax = vect(nr, INF);
	RtTau = Rmax = vect(nr, INF); measstop = measlost = Rmin = Rb = Rstop = vect(nr, 0.0); Rbeta = vect(nr, 1.0);
	SetRmaxcount(5);
	innoMax = vect(nr, INF);
	SetInnoMaxcount(5);
	FBTau = FBMax = FBOne = FBOne1 = vect(nq, INF); FBXk = FBTotal = vect(nq, 0.0);
	kfcount = measflag = measflaglog = 0;  SetMeasMask(nr0, 3);
	Zmm.Init(nr0, 10);
}

/* set measurement noise bounds and time constant ----------------------------
* set the minimum and maximum measurement noise, bias, and time constant
* args   : double     rmin     I   minimum measurement noise (default: 0.1)
*          double     rmax     I   maximum measurement noise (default: INF)
*          double     b        I   bias (default: INF)
*          double     tau      I   time constant (default: INF)
* return : none
*-----------------------------------------------------------------------------*/
void KalmanFilter::SetRmmbt(double rmin, double rmax, double b, double tau)
{
	if (tau < INF / 2)  RtTau = tau;
	if (b < INF / 2)  Rb = b;
	if (rmax < INF / 2)  Rmax = Rt * (rmax * rmax);
	Rmin = Rt * (rmin * rmin);
}

/* set maximum count for measurement noise ------------------------------------
* set the maximum count for measurement noise adaptation
* args   : int        cnt      I   maximum count (default: 5)
* return : none
*-----------------------------------------------------------------------------*/
void KalmanFilter::SetRmaxcount(int cnt)
{
	for (int i = 0; i < nr; i++) { Rmaxcount[i] = 0, Rmaxcount0[i] = cnt; }
}

/* set maximum count for innovation -------------------------------------------
* set the maximum count for innovation adaptation
* args   : int        cnt      I   maximum count (default: 5)
* return : none
*-----------------------------------------------------------------------------*/
void KalmanFilter::SetInnoMaxcount(int cnt)
{
	for (int i = 0; i < nr; i++) { innoMaxcount[i] = 0, innoMaxcount0 = cnt; }
}

/* set measurement bounds ----------------------------------------------------
* set the maximum and minimum bounds for a specific measurement
* args   : int        zi       I   measurement index
*          int        pi       I   state index
*          double     zmm0     I   initial measurement bound
*          int        cnt      I   count for adaptation (default: 10)
* return : none
*-----------------------------------------------------------------------------*/
void KalmanFilter::SetZmm(int zi, int pi, double zmm0, int cnt)
{
	if (cnt > 0) Zmm.mm[zi].Init(cnt);
	Zmmpk[zi] = pi;  Zmm0.dd[zi] = zmm0;
}

/* set velocity measurement bounds --------------------------------------------
* set the maximum and minimum bounds for velocity measurements
* args   : vect3      zmm0     I   initial velocity bounds
*          int        cnt      I   count for adaptation (default: 10)
* return : none
*-----------------------------------------------------------------------------*/
void KalmanFilter::SetZmmVn(const vect3& zmm0, int cnt)
{
	SetZmm(0, 3, zmm0.i, cnt);	SetZmm(1, 4, zmm0.j, cnt);	SetZmm(2, 5, zmm0.k, cnt);
}

/* set position measurement bounds --------------------------------------------
* set the maximum and minimum bounds for position measurements
* args   : vect3      zmm0     I   initial position bounds
*          int        cnt      I   count for adaptation (default: 10)
* return : none
*-----------------------------------------------------------------------------*/
void KalmanFilter::SetZmmPos(const vect3& zmm0, int cnt)
{
	SetZmm(3, 6, zmm0.i, cnt);	SetZmm(4, 7, zmm0.j, cnt);	SetZmm(5, 8, zmm0.k, cnt);
}

/* time update ----------------------------------------------------------------
* perform time update for the Kalman filter
* args   : double     kfts0    I   time step
*          int        fback    I   feedback flag (default: 1)
* return : none
*-----------------------------------------------------------------------------*/
void KalmanFilter::TimeUpdate(double kfts0, int fback)
{
	mat Fk;
	kftk += kfts0;  kfcount++;
	SetFt(nq);
	Fk = ++(Ft * kfts0);  // Fk = I+Ft*ts
	Xk = Fk * Xk;
	Pk = Fk * Pk * (~Fk);  Pk += Qt * kfts0;
	if (fback)  Feedback(nq, kfts0);
	for (int i = 0; i < nr; i++) {
		measlost.dd[i] += kfts0;
		if (measstop.dd[i] > 0.0f) measstop.dd[i] -= kfts0;
	}
}

/* set measurement flag -------------------------------------------------------
* set the measurement flag for a specific type
* args   : unsigned int flag   I   measurement flag
*          int        type     I   type of measurement (default: 1)
* return : none
*-----------------------------------------------------------------------------*/
void KalmanFilter::SetMeasFlag(unsigned int flag, int type)
{
	if (type == 1)
		measflag = (flag == 0) ? 0 : (measflag | flag);  // add the flag bits to 1
	else if (type == 0)
		measflag &= ~flag;  // set the flag bits to 0
}

/* set measurement mask -------------------------------------------------------
* set the measurement mask for a specific type
* args   : unsigned int mask   I   measurement mask
*          int        type     I   type of measurement (default: 1)
* return : none
*-----------------------------------------------------------------------------*/
void KalmanFilter::SetMeasMask(unsigned int mask, int type)
{
	int m;
	if (type == 1) measmask = mask;		// set mask 1
	else if (type == 0) measmask &= ~mask;	// delete mask to 0
	else if (type == 2) measmask |= mask;	// add mask 1
	else if (type == 3) {					// set mask-LSB 1
		for (m = 0; mask > 0; mask--)  m |= 1 << (mask - 1);
		SetMeasMask(m);
	}
}

/* set measurement stop condition ---------------------------------------------
* set the stop condition for a specific measurement
* args   : unsigned int meas   I   measurement index
*          double     stop     I   stop threshold (default: 10.0)
* return : none
*-----------------------------------------------------------------------------*/
void KalmanFilter::SetMeasStop(unsigned int meas, double stop)
{
	for (register int i = 0; i < measstop.rc; i++) {	// assert(rc<32)
		if (meas & (0x01 << i) && (measstop.dd[i] < stop || stop <= 0.0)) measstop.dd[i] = stop;
	}
	//	measstop.SetBit(meas, stop);
}

/* set adaptive stop condition ------------------------------------------------
* set the adaptive stop condition for a specific measurement
* args   : unsigned int meas   I   measurement index
*          double     stop     I   stop threshold (default: 10.0)
* return : none
*-----------------------------------------------------------------------------*/
void KalmanFilter::SetRadptStop(unsigned int meas, double stop)
{
	Rstop.SetBit(meas, stop);
}

/* measurement update ---------------------------------------------------------
* perform measurement update for the Kalman filter
* args   : double     fading   I   fading factor (default: 1.0)
* return : int                 O   update status
*-----------------------------------------------------------------------------*/
int KalmanFilter::MeasUpdate(double fading)
{
	vect Pxz, Kk, Hi;
	SetMeas();
	for (int i = 0; i < nr; i++)
	{
		if (((measflag&measmask)&(0x01 << i)) && measstop.dd[i] < EPS)
		{
			Hi = Hk.GetRow(i);
			Pxz = Pk * (~Hi);
			double Pz0 = (Hi * Pxz)(0, 0), r = Zk(i) - (Hi * Xk)(0, 0);
			if (Rb.dd[i] > EPS)
				RAdaptive(i, r, Pz0);
			if (Zfd.dd[i] < INFp5)
				RPkFading(i);
			double Pzz = Pz0 + Rt.dd[i] / rts.dd[i];
			Kk = Pxz * (1.0 / Pzz);
			Xk += Kk * r;
			Pk -= Kk * (~Pxz);
			measlost.dd[i] = 0.0;
		}
	}
	if (fading > 1.0) Pk *= fading;
	XPConstrain();
	symmetry(Pk);
	int measres = measflag & measmask;
	measflaglog |= measres;
	SetMeasFlag(0);
	return measres;
}

/* Rt adaptive ----------------------------------------------------------------
* adaptively adjust Rt based on measurement
* args   : int        i        I   index of the state
*          double     r        I   measurement noise
*          double     Pr       I   process noise
* return : int                 O   adaptation status
*-----------------------------------------------------------------------------*/
int KalmanFilter::RAdaptive(int i, double r, double Pr)
{
	double rr = r * r - Pr;
	if (Rb.dd[i] > 1.0) {  // s^2=Rb.dd[i], for heavy-tailed noise
		Rt.dd[i] = rr / Rb.dd[i];  // rho^2/s^2;
		if (Rt.dd[i] < Rmin.dd[i]) Rt.dd[i] = Rmin.dd[i];
		return 1; //adptOK=1;
	}
	if (rr < Rmin.dd[i])	rr = Rmin.dd[i];
	if (rr > Rmax.dd[i]) { Rt.dd[i] = Rmax.dd[i]; Rmaxcount[i]++; }
	else { Rt.dd[i] = (1.0 - Rbeta.dd[i]) * Rt.dd[i] + Rbeta.dd[i] * rr; Rmaxcount[i] = 0; }
	Rbeta.dd[i] = Rbeta.dd[i] / (Rbeta.dd[i] + Rb.dd[i]);   // beta = beta / (beta+b)
	int adptOK = (Rmaxcount[i] == 0 || Rmaxcount[i] > Rmaxcount0[i]) ? 1 : 0;
	return adptOK;
}

/* multiple fading ------------------------------------------------------------
* apply multiple fading to the Kalman filter
* args   : int        i        I   index of the state
* return : none
*-----------------------------------------------------------------------------*/
void KalmanFilter::RPkFading(int i)
{
	Zfd.dd[i] = Zfd.dd[i] * (1.0f - zfdafa) + Zk.dd[i] * zfdafa;
	if (Zfd.dd[i] > Zfd0.dd[i] || Zfd.dd[i] < -Zfd0.dd[i])
		DVMDVafa(Fading.GetRow(i), Pk, 1);
}

/* Z_max_min for Pk setting ---------------------------------------------------
* set the maximum and minimum bounds for Pk
* args   : int        i        I   index of the state
* return : none
*-----------------------------------------------------------------------------*/
void KalmanFilter::ZmmPkSet(int i)
{
	maxmin* pmm = &Zmm.mm[i];
	pmm->Update((float)Zk.dd[i]);
	if (pmm->flag) {  // if conitnously big abs(Zk[i]), enlarge Pk[i,i]
		if (pmm->minRes > Zmm0.dd[i] || pmm->maxRes < -Zmm0.dd[i])
			Pset.dd[Zmmpk[i]] = pmm->meanRes * pmm->meanRes;
	}
}

/* constrain state and covariance ---------------------------------------------
* constrain the state vector and covariance matrix within specified bounds
* args   : none
* return : none
*-----------------------------------------------------------------------------*/
void KalmanFilter::XPConstrain(void)
{
	int i = 0, nq1 = nq + 1;
	for (double* px = Xk.dd, *pxmax = Xmax.dd, *p = Pk.dd, *pmin = Pmin.dd, *pminEnd = &Pmin.dd[nq], *pmax = Pmax.dd, *pset = Pset.dd;
		pmin < pminEnd; px++, pxmax++, p += nq1, pmin++, pmax++, pset++)
	{
		if (*px > *pxmax)		// Xk constrain
		{
			*px = *pxmax;
		}
		else if (*px < -*pxmax)
		{
			*px = -*pxmax;
		}
		if (*p < *pmin)		// Pk constrain
		{
			*p = *pmin;
		}
		else if (*p > *pmax)
		{
			double sqf = sqrt(*pmax / (*p)) * 0.9;
			for (double* prow = &Pk.dd[i * Pk.clm], *prowEnd = prow + nq, *pclm = &Pk.dd[i]; prow < prowEnd; prow++, pclm += nq)
			{
				*prow *= sqf;
				*pclm = *prow;
			}
			Pk.dd[i * Pk.clm + i] *= sqf;  //20200303
			break;
		}
		if (*pset > 0.0)	// Pk set
		{
			if (*p < *pset)
			{
				*p = *pset;
			}
			else if (*p > *pset)
			{
				double sqf = sqrt(*pset / (*p));
				for (double* prow = &Pk.dd[i * Pk.clm], *prowEnd = prow + nq, *pclm = &Pk.dd[i]; prow < prowEnd; prow++, pclm += nq)
				{
					*prow *= sqf;
					*pclm *= sqf;
				}
			}
			*pset = -1.0;
		}
		i++;
	}
}

/* check covariance bounds ----------------------------------------------------
* check if the covariance matrix is within specified bounds
* args   : none
* return : none
*-----------------------------------------------------------------------------*/
void KalmanFilter::PmaxPminCheck(void)
{
	for (double* p = Pk.dd, *pmax = Pmax.dd, *pmin = Pmin.dd, *pminEnd = &Pmin.dd[nq]; pmin < pminEnd; p += nq + 1, pmax++, pmin++)
	{
		if (*p > *pmax) *pmax = *p * 10.0;
		if (*p < EPS)	 *pmin = 0.0;  else if (*p < *pmin)  *pmin = *p / 2.0;
	}
}

/* state feedback -------------------------------------------------------------
* apply feedback to the system state based on Kalman filter results
* args   : int        nnq      I   size of the state vector
*          double     fbts     I   feedback time step
* return : none
*-----------------------------------------------------------------------------*/
void KalmanFilter::Feedback(int nnq, double fbts)
{
	double* pTau = FBTau.dd, * pTotal = FBTotal.dd, * pMax = FBMax.dd, * pOne = FBOne.dd, * pOne1 = FBOne1.dd, * pXk = FBXk.dd, * p = Xk.dd;
	for (int i = 0; i < nq; i++, pTau++, pTotal++, pMax++, pOne++, pOne1++, pXk++, p++)
	{
		if (*pTau < INFp5)
		{
			if (*p > *pOne1 || *p < -*pOne1) {  // max feedback/one step
				*pXk = *p;
			}
			else {
				double afa = fbts < *pTau ? fbts / (*pTau) : 1.0;
				*pXk = *p * afa;
				if (*pXk > *pOne) *pXk = *pOne; else if (*pXk < -*pOne) *pXk = -*pOne;  // min feedback/one step
			}
			if (*pMax < INFp5)
			{
				if (*pTotal + *pXk > *pMax)			*pXk = *pMax - *pTotal;
				else if (*pTotal + *pXk < -*pMax)	    *pXk = -*pMax - *pTotal;
			}
			*p -= *pXk;
			*pTotal += *pXk;
		}
		else
		{
			*pXk = 0.0;
		}
	}
}
/* Rt fading ------------------------------------------------------------------
* increase Rt if no measurement is available
* args   : int        i        I   index of the state
*          double     fdts     I   fading time step
* return : none
*-----------------------------------------------------------------------------*/
void KalmanFilter::RtFading(int i, double fdts)
{
	double Taui = RtTau.dd[i], Rti = Rt.dd[i], Rmaxi = Rmax.dd[i];
	if (measlost.dd[i] > 3.0 && Taui < INFp5 && Rb.dd[i]>0.0 && Rti < Rmaxi)
	{
		double afa = fdts < Taui ? fdts / Taui : 1.0;
		Rti += 2 * sqrt(Rmaxi * Rti) * afa;
		Rt.dd[i] = Rti;
	}
}

/* fusion of two states and covariances ---------------------------------------
* perform fusion of two states and their covariances
* args   : double*    x1       IO  state vector 1
*          double*    p1       IO  covariance matrix 1
*          const double* x2    I   state vector 2
*          const double* p2    I   covariance matrix 2
*          int        n        I   dimension of the state vectors
*          double*    xf       O   fused state vector (optional)
*          double*    pf       O   fused covariance matrix (optional)
* return : none
*-----------------------------------------------------------------------------*/
void fusion(double* x1, double* p1, const double* x2, const double* p2, int n, double* xf, double* pf)
{
	if (xf == NULL) { xf = x1, pf = p1; }
	double* x10, * xf0;
	vect3 att1;
	if (n < 100) {  // n<100 for att(1:3), n>100 for no_att(1:3)
		x10 = x1, xf0 = xf;
		att1 = *(vect3*)x1;
		*(vect3*)x2 = qq2phi(a2qua(*(vect3*)x2), a2qua(att1));
		*(vect3*)x1 = O31;
	}
	int j;
	for (j = (n > 100) ? 100 : 0; j < n; j++, x1++, p1++, x2++, p2++, xf++, pf++)
	{
		double p1p2 = *p1 + *p2;
		*xf = (*p1 * *x2 + *p2 * *x1) / p1p2;
		*pf = *p1 * *p2 / p1p2;
	}
	if (j < 100) {
		*(vect3*)xf0 = q2att(a2qua(att1) + *(vect3*)xf0);
		if (xf0 != x10) *(vect3*)x10 = att1;
	}
}

/* fusion of two vect3 states and covariances ---------------------------------
* perform fusion of two vect3 states and their covariances
* args   : vect3&     x1       IO  state vector 1
*          vect3&     p1       IO  covariance matrix 1
*          const vect3 x2      I   state vector 2
*          const vect3 p2      I   covariance matrix 2
* return : none
*-----------------------------------------------------------------------------*/
void fusion(vect3& x1, vect3& p1, const vect3 x2, const vect3 p2)
{
	fusion(&x1.i, &p1.i, &x2.i, &p2.i, 103);
}

/* fusion of two vect3 states and covariances with output ---------------------
* perform fusion of two vect3 states and their covariances, with output
* args   : vect3&     x1       IO  state vector 1
*          vect3&     p1       IO  covariance matrix 1
*          const vect3 x2      I   state vector 2
*          const vect3 p2      I   covariance matrix 2
*          vect3&     xf       O   fused state vector
*          vect3&     pf       O   fused covariance matrix
* return : none
*-----------------------------------------------------------------------------*/
void fusion(vect3& x1, vect3& p1, const vect3 x2, const vect3 p2,
	vect3& xf, vect3& pf)
{
	fusion(&x1.i, &p1.i, &x2.i, &p2.i, 103, &xf.i, &pf.i);
}

EKFTDKF::EKFTDKF(void)
{
}

EKFTDKF::EKFTDKF(int nq0, int nr0)
{
	KalmanFilter::Init(nq0, nr0);
	Kkp2y = Kkv2y = 1.0;
}

/* initialize the Kalman system ---------------------------------------------------
* initialize the INSKalman system with initial state
* return : none
*-----------------------------------------------------------------------------*/
void EKFTDKF::Init()
{

	Fk = eye(nq);  Pk1 = mat(nq, nq, 0.0);
	Pxz = Qk = Kk = tmeas = vect(nr, 0.0);
	meantdts = 1.0; tdts = 0.0;
	maxStep = 2 * (nq + nr) + 3;
	TDReset();
	curOutStep = 0; maxOutStep = 1;
	timcnt0 = timcnt1 = 0, timcntmax = 100;  burden = 0.0;
	cststt = nq;  for (int k = 0; k < nq; k++) hi1[k] = -1;
}

/* reset the Time-Distributed Kalman filter -----------------------------------
* reset the Time-Distributed Kalman filter
* args   : none
* return : none
*-----------------------------------------------------------------------------*/
void EKFTDKF::TDReset(void)
{
	iter = -2;
	ifn = 0;	meanfn = O31;
	SetMeasFlag(0);
}

/* calculate innovation for a specific row ------------------------------------
* calculate the innovation value for a specific row in the Kalman filter
* args   : int        row      I   row index
* return : innovation value
*-----------------------------------------------------------------------------*/
double EKFTDKF::Innovationi(int row)
{
	double hx = 0.0;
	for (double* ph = &Hk.dd[row * nq], *px = &Xk.dd[0], *pxEnd = &Xk.dd[Xk.rc]; px < pxEnd; ph++, px++) { hx += (*ph) * (*px); }
	return (Zk.dd[row] - hx);
}
/* TDUpdate function for segmented (fragmented) Kalman filtering ---------------
* Performs a segmented (fragmented) Kalman filter update over a specified number of steps.
* This function implements a time-distributed or sequential update for the Extended Kalman Filter (EKF),
* breaking the full update into multiple smaller steps to improve numerical stability and real-time performance.
*
* args   : int nStep        I   number of update steps to perform (if <=0 or >=maxStep, uses maxStep)
* return : int              O   measurement result flag (bitmask of updated measurements)
*
* Method:
*   - The function uses an internal iteration counter (iter) to step through the segmented update process.
*   - At iter == -2, it sets up the system matrices and measurements.
*   - At iter == -1, it discretizes the system matrix Ft to Fk, updates the state Xk, and process noise Qk.
*   - For 0 <= iter < nq, it computes Fk*Pk row by row.
*   - For nq <= iter < 2*nq, it computes Fk*Pk*Fk' + Qk row by row.
*   - For 2*nq <= iter < 2*(nq+nr), it performs sequential measurement updates for each measurement dimension,
*     including innovation calculation, adaptive measurement noise adjustment, and Kalman gain computation.
*   - At iter == 2*(nq+nr), it applies state and covariance constraints and enforces symmetry.
*   - At iter >= 2*(nq+nr)+1, it resets the iteration counter for the next cycle.
*   - The function returns a bitmask indicating which measurements were successfully updated.
*-----------------------------------------------------------------------------*/
int EKFTDKF::TDUpdate(int nStep)
{
	measRes = 0;
	if (nStep <= 0 || nStep >= maxStep) { nStep = maxStep; }
	tdStep = nStep;
	for (int i = 0; i < nStep; i++)
	{
		if (iter == -2)			// -2: set measurements
		{
			SetFt(nq);
			SetMeas(); SetHk(nq);
		}
		else if (iter == -1)			// -1: discrete
		{
			//Fk = ++(Ft*tdts); // Fk = I+Ft*ts
			double* pFk, * pFt, * pEnd;
			for (pFk = Fk.dd, pFt = Ft.dd, pEnd = &Fk.dd[cststt * Fk.clm]; pFk < pEnd; pFk++, pFt++)  *pFk = *pFt * tdts;
			for (pFk = Fk.dd; pFk < pEnd; pFk += Fk.clm + 1)  *pFk += 1.0;
			//			Xk = Fk*Xk;
			pFk = Fk.dd, pEnd = &Xk.dd[Xk.rc];
			int jj;
			for (jj = 0; jj < cststt; jj++) {
				double f = 0.0;
				for (double* pX = Xk.dd; pX < pEnd; pFk++, pX++)  f += *pFk * *pX;
				Pxz.dd[jj] = f;  // Pxz just for store data
			}
			for (jj = 0; jj < cststt; jj++)  Xk.dd[jj] = Pxz.dd[jj];
			Qk = Qt * tdts;
			//RtFading(tdts);
			meantdts = tdts; tdts = 0.0;
		}
		else if (iter < nq)		// 0 -> (nq-1): Fk*Pk
		{
			int row = iter;
			RowMul(Pk1, Fk, Pk, row, cststt);
		}
		else if (iter < 2 * nq)		// nq -> (2*nq-1): Fk*Pk*Fk+Qk
		{
			int row = iter - nq;
			RowMulT(Pk, Pk1, Fk, row, cststt);
			Pk.dd[nq * row + row] += Qk.dd[row];
			//if(row==nq-1) {	Pk += Qk; }
		}
		else if (iter < 2 * (nq + nr))	// (2*nq) -> (2*(nq+nr)-1): sequential measurement updating
		{
			int row = (iter - 2 * Ft.row) / 2;
			int flag = (measflag & measmask) & (0x01 << row);
			if (flag)
			{
				if (iter % 2 == 0)
				{
					Hi = Hk.GetRow(row);  int hi = hi1[row];
					if (hi >= 0) {
						Pxz = Pk.GetClm(hi);
						Pz0 = Pxz.dd[hi];
						innovation = Zk.dd[row] - Xk.dd[hi];  Zk_1.dd[row] = Zk.dd[row];
					}
					else {
						Pxz = Pk * (~Hi);
						Pz0 = dot(Hi, Pxz);
						innovation = Zk.dd[row] - dot(Hi, Xk);  Zk_1.dd[row] = Zk.dd[row];
					}
					//trace(4, "row was %3d,innovation was %10.7f\n", row, innovation);
					double innoDiff = innovation - innoPre.dd[row];  innoPre.dd[row] = innovation;
					int* pinnoMaxcount = &innoMaxcount[row];
					if (*pinnoMaxcount > 0) (*pinnoMaxcount)--;
					if (innovation<-innoMax.dd[row] || innovation>innoMax.dd[row])
					{
						if (*pinnoMaxcount < 2 * innoMaxcount0) *pinnoMaxcount += 2;
					}
					if ((*pinnoMaxcount == 0 || *pinnoMaxcount > innoMaxcount0)
						&& (innovation > -Zmax.dd[row] && innovation < Zmax.dd[row])
						&& (innoDiff > -innoDiffMax.dd[row] && innoDiff < innoDiffMax.dd[row]))
					{
						adptOKi = 1;
						if (Rb.dd[row] > 0.0 && Rstop.dd[row] <= 0.0) {
							adptOKi = RAdaptive(row, innovation, Pz0);
						}
						if (Rset.dd[row] > 0.0) {
							Rt.dd[row] = Rset.dd[row];  Rset.dd[row] = -1.0;  adptOKi = 1;
						}
						double Pzz = Pz0 + Rt.dd[row] / rts.dd[row];
						Kk = Pxz * (1.0 / Pzz);
					}
					else
					{
						adptOKi = 0;
					}
				}
				else
				{
					measflag ^= flag;
					if (adptOKi && measstop.dd[row] < EPS)
					{
						measRes |= flag;
						Pk -= Kk * (~Pxz);
						//if(flag&030) Kk.dd[2]*=Kkp2y;  // disable pos2yaw
						//else if(flag&003) Kk.dd[2]*=Kkv2y;
						Xk += Kk * innovation;
						measlost.dd[row] = 0.0;
					}
					if (Zfd0.dd[row] < INFp5)
					{
						RPkFading(row);
					}
					if (Zmm0.dd[row] < INFp5)
					{
						ZmmPkSet(row);
					}
				}
			}
			else
			{
				nStep++;
			}
			if (iter % 2 == 0 && Rstop.dd[row] < EPS)
				RtFading(row, meantdts);
		}
		else if (iter == 2 * (nq + nr))	// 2*(nq+nr): Xk,Pk constrain & symmetry
		{
			XPConstrain();
			symmetry(Pk);
		}
		else if (iter >= 2 * (nq + nr) + 1)	// 2*(nq+nr)+1: Miscellanous
		{
			//Miscellanous();
			iter = -3;
		}
		iter++;
	}
	//SecretAttitude();

	measflaglog |= measRes;
	return measRes;
}

/* perform measurement update -------------------------------------------------
* perform a measurement update in the Kalman filter
* args   : vect       Hi       I   measurement matrix
*          double     Ri       I   measurement noise covariance
*          double     Zi       I   measurement value
* return : none
*-----------------------------------------------------------------------------*/
void EKFTDKF::MeasUpdate(const vect& Hi, double Ri, double Zi)
{
	if (iter >= 0 && iter < 2 * nq) return;  // !
	Pxz = Pk * (~Hi);
	Pz0 = (Hi * Pxz)(0, 0);
	innovation = Zi - (Hi * Xk)(0, 0);
	double Pzz = Pz0 + Ri;
	Kk = Pxz * (1.0 / Pzz);
	Xk += Kk * innovation;
	Pk -= Kk * (~Pxz);
}

/* set vertical channel covariance ---------------------------------------------
* set the covariance for the vertical channel in the Kalman filter
* args   : double     sph      I   covariance for position
*          double     spv      I   covariance for velocity (default: 0.0)
*          double     spd      I   covariance for distance (default: 0.0)
* return : none
*-----------------------------------------------------------------------------*/
void EKFTDKF::PSetVertCh(double sph, double spv, double spd)
{
	sph = sph * sph;  spv = spv * spv;  spd = spd * spd;
	if (sph > Pk.dd[nq * 8 + 8])   Pset.dd[8] = sph;
	if (spv > Pk.dd[nq * 5 + 5])   Pset.dd[5] = spv;
	if (spd > Pk.dd[nq * 14 + 14]) Pset.dd[14] = spd;
}

/* set calculation burden ------------------------------------------------------
* set the calculation burden for the Kalman filter
* args   : unsigned int timcnt I   time count
*          int        itype    I   type of calculation
* return : calculation burden
*-----------------------------------------------------------------------------*/
double EKFTDKF::SetCalcuBurden(unsigned int timcnt, int itype)
{
	double res = 0.0;
	if (itype == -1) {
		timcntmax = timcnt;
	}
	else if (itype == 0) {
		timcnt0 = timcnt;
	}
	else {
		timcnt1 = timcnt;
		if (timcntmax < timcnt1) timcntmax = timcnt1;
		if (timcnt1 < timcnt0) timcnt1 += timcntmax;
		res = (double)(timcnt1 - timcnt0) / timcntmax;
		if (res > 0.99) res = 0.99;
		burden = burden < res ? res : burden - 0.01;
	}
	return (iter + 100) + res;
}

SINSGNSSLOOSE::SINSGNSSLOOSE(void)
{
}

SINSGNSSLOOSE::SINSGNSSLOOSE(int nq0, int nr0, double ts, int yawHkRow0) :EKFTDKF(nq0, nr0)
{
	posGNSSdelay = vnGNSSdelay = yawGNSSdelay = dtGNSSdelay = dyawGNSS = -0.0f;
	kfts = ts;	gnssLost = &measlost.dd[3];
	lvGNSS = O31;
	Hk(0, 3) = Hk(1, 4) = Hk(2, 5) = 1.0;		// dvn
	Hk(3, 6) = Hk(4, 7) = Hk(5, 8) = 1.0;		// dpos
	yawHkRow = yawHkRow0;
	if (yawHkRow >= 6) Hk(yawHkRow, 2) = 1.0;	// dyaw
	SetMeasMask(077);
	SetMeasStop(0xffffffff, 1.0);
}

/* initialize the INS system ---------------------------------------------------
* initialize the INS system with initial state
* args   : INS        sins0    I   initial INS state
*          int        grade    I   grade level (default: -1)
* return : none
*-----------------------------------------------------------------------------*/
void SINSGNSSLOOSE::Init(const INS& sins0, int grade)
{
	sins = sins0;  kftk = sins.tk;
	EKFTDKF::Init();
	sins.lever(-lvGNSS, &sins.pos);   // sins0.pos is GNSS pos
	avpi.Init(sins.att, sins.vnL, sins.posL, kfts, 0);
}

/* set process noise matrix ---------------------------------------------------
* set the process noise matrix for the Kalman filter
* args   : int        nnq      I   size of the process noise matrix
* return : none
*-----------------------------------------------------------------------------*/
void SINSGNSSLOOSE::SetFt(int nnq)
{
	sins.etm();
	//	Ft.SetMat3(0,0,sins.Maa), Ft.SetMat3(0,3,sins.Mav), Ft.SetMat3(0,6,sins.Map), Ft.SetMat3(0,9,-sins.Cnb);
	//	Ft.SetMat3(3,0,sins.Mva), Ft.SetMat3(3,3,sins.Mvv), Ft.SetMat3(3,6,sins.Mvp), Ft.SetMat3(3,12,sins.Cnb);
	//						NULL, Ft.SetMat3(6,3,sins.Mpv), Ft.SetMat3(6,6,sins.Mpp);
	Ft.SetMat3(0, 0, sins.Maa, sins.Mav, sins.Map), Ft.SetMat3(0, 9, -sins.Cnb);
	Ft.SetMat3(3, 0, sins.Mva, sins.Mvv, sins.Mvp), Ft.SetMat3(3, 12, sins.Cnb);
	Ft.SetMat3(6, 3, sins.Mpv, sins.Mpp);
	Ft.SetDiagVect3(9, 9, sins._betaGyro);
	Ft.SetDiagVect3(12, 12, sins._betaAcc);  // 0-14 phi,dvn,dpos,eb,db
}

/* set measurement matrix -----------------------------------------------------
* set the measurement matrix for the Kalman filter
* args   : int        nnq      I   size of the measurement matrix
* return : none
*-----------------------------------------------------------------------------*/
void SINSGNSSLOOSE::SetHk(int nnq)
{

}

/* apply feedback to the system state -----------------------------------------
* apply feedback to the system state based on Kalman filter results
* args   : int        nnq      I   size of the state vector
*          double     fbts     I   feedback time step
* return : none
*-----------------------------------------------------------------------------*/
void SINSGNSSLOOSE::Feedback(int nnq, double fbts)
{
	KalmanFilter::Feedback(nq, fbts);
	for (int i = 0; i < avpi.avpinum; i++) {
		avpi.vni[i] -= *(vect3*)&FBXk.dd[3];
		avpi.posi[i] -= *(vect3*)&FBXk.dd[6];
	}
	sins.qnb -= *(vect3*)&FBXk.dd[0];
	sins.vn -= *(vect3*)&FBXk.dd[3];
	sins.pos -= *(vect3*)&FBXk.dd[6];
	sins.eb += *(vect3*)&FBXk.dd[9];
	sins.db += *(vect3*)&FBXk.dd[12];  // 0-14 phi,dvn,dpos,eb,db
}

/* set GNSS measurements ------------------------------------------------------
* set GNSS position, velocity, and yaw measurements
* args   : vect3      posgnss  I   GNSS position {x, y, z} (default: O31)
*          vect3      vngnss   I   GNSS velocity {vx, vy, vz} (default: O31)
*          double     yawgnss  I   GNSS yaw angle (default: 0.0)
* return : none
*-----------------------------------------------------------------------------*/
void SINSGNSSLOOSE::SetMeasGNSS(const vect3& posgnss, const vect3& vngnss, double yawgnss)
{
	if (!IsZeros(posgnss) && avpi.Interp(posGNSSdelay + dtGNSSdelay, 0x4))
	{
		*(vect3*)&Zk.dd[3] = avpi.pos - posgnss;
		//if (IsZeros(Zk.dd[3], 1.0))
			SetMeasFlag(00070);
	}
	if (!IsZeros(vngnss) && avpi.Interp(vnGNSSdelay + dtGNSSdelay, 0x2))
	{
		*(vect3*)&Zk.dd[0] = avpi.vn - vngnss;
		//if (IsZeros(Zk.dd[0], 0.5))
			SetMeasFlag(00007);
	}
	if (!IsZero(yawgnss) && avpi.Interp(yawGNSSdelay + dtGNSSdelay, 0x1))
	{
		Zk.dd[yawHkRow] = -diffYaw(avpi.att.k, yawgnss);
		//if (IsZero(Zk.dd[yawHkRow], 10.0 * DEG))
			SetMeasFlag(01 << yawHkRow);
	}
}

/* measure GNSS zero-velocity stop --------------------------------------------
* measure GNSS zero-velocity stop based on a threshold
* args   : vect3      dvnth    O   velocity difference threshold
*          double     stop     I   stop threshold (default: 5.0)
* return : none
*-----------------------------------------------------------------------------*/
void SINSGNSSLOOSE::MeasGNSSZvStop(vect3& dvnth, double stop)  // Zv/dvn threshold value
{
	int meas = 001;
	double* z = &Zk.dd[0], * v = &dvnth.i;
	for (int i = 0; i < 3; i++, z++, v++) {
		if (*z > *v || *z < -(*v)) { SetMeasStop(meas, stop); meas <<= 1; }
	}
}

/* measure GNSS zero-position stop --------------------------------------------
* measure GNSS zero-position stop based on a threshold
* args   : vect3      dposth   O   position difference threshold
*          double     stop     I   stop threshold (default: 5.0)
* return : none
*-----------------------------------------------------------------------------*/
void SINSGNSSLOOSE::MeasGNSSZpStop(vect3& dposth, double stop)  // Zk/dpos threshold value
{
	int meas = 010;
	double* z = &Zk.dd[3], * p = &dposth.i;
	for (int i = 0; i < 3; i++, z++, p++) {
		if (*z > *p || *z < -(*p)) { SetMeasStop(meas, stop); meas <<= 1; }
	}
}

/* measure GNSS zero-position to X-axis ---------------------------------------
* measure GNSS zero-position and project it to the X-axis
* args   : vect3      dposth   O   position difference threshold
* return : none
*-----------------------------------------------------------------------------*/
void SINSGNSSLOOSE::MeasGNSSZp2X(vect3& dposth)  // Zk/dpos threshold value
{
	double* z = &Zk.dd[3], * p = &dposth.i, * ps = &Pset.dd[6];
	for (int i = 0; i < 3; i++, z++, p++, ps++) {
		if (*z > *p || *z < -(*p)) { *ps = 100.0 * (*z) * (*z); }
	}
}

/* update the system state ----------------------------------------------------
* update the system state using sensor measurements
* args   : vect3*     pwm      I   angular velocity measurements
*          vect3*     pvm      I   linear velocity measurements
*          int        nSamples I   number of samples
*          double     ts       I   time sep
*          int        nSteps   I   number of steps (default: 5)
* return : update status
*-----------------------------------------------------------------------------*/
int SINSGNSSLOOSE::Update(const vect3* pwm, const vect3* pvm, int nn, double ts, int nSteps)
{
	sins.Update(pwm, pvm, nn, ts);
	if (++curOutStep >= maxOutStep) { RTOutput(), curOutStep = 0; }
	Feedback(nq, sins.nts);

	for (int j = 0; j < nr; j++) {
		measlost.dd[j] += sins.nts;
		if (Rstop.dd[j] > 0.0) Rstop.dd[j] -= sins.nts;
		if (measstop.dd[j] > 0.0) measstop.dd[j] -= sins.nts;
	}
	tdts += sins.nts; kftk = sins.tk;  kfcount++;
	meanfn = meanfn + sins.fn; ifn++;

	if (iter == -2) {
		vect3 fn = sins.fn, an = sins.an;
		sins.fn = meanfn * (1.0 / ifn); meanfn = O31; ifn = 0;
		sins.an = sins.fn + sins.eth.gcc;
		sins.fn = fn; sins.an = an;
	}


	int res = TDUpdate(nSteps);
	sins.lever(lvGNSS);
	avpi.Push(sins.att, sins.vnL, sins.posL);
	return res;
}

/* set Markov process for gyroscope -------------------------------------------
* set the Markov process parameters for the gyroscope
* args   : vect3      tauG     I   correlation time for gyroscope
*          vect3      sRG      I   noise standard deviation for gyroscope
*          int        stateeb  I   state index for gyroscope bias (default: 9)
* return : none
*-----------------------------------------------------------------------------*/
void SINSGNSSLOOSE::MarkovGyro(const vect3& tauG, const vect3& sRG, int stateeb)
{
	sins.SetTauGA(tauG, O31);
	*(vect3*)&Qt.dd[stateeb] = MKQt(sRG, sins.tauGyro);
}

/* set Markov process for accelerometer ----------------------------------------
* set the Markov process parameters for the accelerometer
* args   : vect3      tauA     I   correlation time for accelerometer
*          vect3      sRA      I   noise standard deviation for accelerometer
*          int        statedb  I   state index for accelerometer bias (default: 12)
* return : none
*-----------------------------------------------------------------------------*/
void SINSGNSSLOOSE::MarkovAcc(const vect3& tauA, const vect3& sRA, int statedb)
{
	sins.SetTauGA(O31, tauA);
	*(vect3*)&Qt.dd[statedb] = MKQt(sRA, sins.tauAcc);
}

/* set yaw angle ---------------------------------------------------------------
* set the yaw angle in the Kalman filter state
* args   : double     yaw      I   yaw angle (radians)
*          int        statephi I   state index for attitude (default: 0)
*          int        statedvn I   state index for velocity (default: 3)
* return : none
*-----------------------------------------------------------------------------*/
void SINSGNSSLOOSE::SetYaw(double yaw, int statephi, int statedvn)
{
	quat qnn = a2qua(0, 0, diffYaw(yaw, sins.att.k));
	sins.qnb = qnn * sins.qnb;  sins.Cnb = q2mat(sins.qnb);  sins.Cbn = ~sins.Cnb;  sins.att = m2att(sins.Cnb);
	sins.vn = qnn * sins.vn;
	*(vect3*)&Xk.dd[statephi] = qnn * *(vect3*)&Xk.dd[statephi];
	*(vect3*)&Xk.dd[statedvn] = qnn * *(vect3*)&Xk.dd[statedvn];
	Pk = diag(diag(Pk));
	/*	CMat3 Cnn=q2mat(qnn);
	CMat Cnn15(15,15,0.0);
	Cnn15.SetMat3(0,0, Cnn); Cnn15.SetMat3(3,3, Cnn);Cnn15.SetMat3(6,6, rcijk(Cnn,102));
	Cnn15.SetMat3(9,9, Cnn); Cnn15.SetMat3(12,12, Cnn);
	Pk = (Cnn15)*Pk*(~Cnn15);*/
	meanfn = O31;
	TDReset();
}

SINSGNSSBorad::SINSGNSSBorad(void)
{
}

SINSGNSSBorad::SINSGNSSBorad(int nq0, int nr0, double ts, int yawHkRow0) :SINSGNSSLOOSE(nq0, nr0, ts, yawHkRow0)
{
	od.Init();
	od_nhc.Init();
	// 0-14: phi,dvn,dpos,eb,db;
	// 15-17: Kappa;

	// 18:gnss_yaw deafult disable
	//Hk(15, 18) = -1.0;

	gnssLostdist = gnssLostnofixdist = 0.0;
	distance_trj = 0.0;
	odLost = &measlost.dd[6];
	// Hk(0:5,:) ...		// 0-5: SINS/GNSS-dvn,dpos
	Hk.SetMat3(6, 3, I33);  // 6-8: SINS/OD-dvn
	Hk.SetMat3(9, 3, I33);  // 9-11: ZUPT
	Hk.SetMat3(12, 3, I33); // 12-14: NHC
	Hk(15, 2) = 1.0;		// 15: SINS/GNSS-dyaw
	Hk(16, 11) = 1.0;		// 16: WzHold (ZIHR)
	//SetMeasMask(0377777);
}

/* initialize the INS system ---------------------------------------------------
* initialize the INS system with initial state
* args   : INS        sins0    I   initial INS state
*          int        grade    I   grade level (default: -1)
* return : none
*-----------------------------------------------------------------------------*/
void SINSGNSSBorad::Init(const INS& sins0, int grade)
{
	SINSGNSSLOOSE::Init(sins0, grade);
	config_od = 1;

	sins.lever(od.lvOD, &od.posOD, &od.vnOD);
	od.odmeasT = 1.0;
	od.phim = &sins.imu.phim;
	od.vn = &sins.vn;
	od.fn = &sins.fn;
	od.web = &sins.web;
	od.nts = &sins.nts;
	od.Cnb = &sins.Cnb;
	od.odmeast = 0;
	od.Hkv2y = 1.0;

	sins.lever(od_nhc.lvOD, &od_nhc.posOD, &od_nhc.vnOD);
	od_nhc.odmeasT = 1.0;
	od_nhc.phim = &sins.imu.phim;
	od_nhc.vn = &sins.vn;
	od_nhc.fn = &sins.fn;
	od_nhc.web = &sins.web;
	od_nhc.nts = &sins.nts;
	od_nhc.Cnb = &sins.Cnb;
	od_nhc.odmeast = 0;
	od_nhc.Hkv2y = 1.0;

	wzhd.Init(1500.0 * DPH, kfts, 10.0);

	Rb.SetBit(0100077, 0.5);
	*(vect3*)&Zmax.dd[0] = vect3(2.0, 2.0, 0.5);
	FBTau = 1.0;
	pos_pre = O31;
	SetGNSSFixMode(0);
}

/* set process noise matrix ---------------------------------------------------
* set the process noise matrix for the Kalman filter
* args   : int        nnq      I   size of the process noise matrix
* return : none
*-----------------------------------------------------------------------------*/
void SINSGNSSBorad::SetFt(int nnq)
{
	SINSGNSSLOOSE::SetFt(15);
}

/* set measurement matrix -----------------------------------------------------
* set the measurement matrix for the Kalman filter
* args   : int        nnq      I   size of the measurement matrix
* return : none
*-----------------------------------------------------------------------------*/
void SINSGNSSBorad::SetHk(int nnq)
{

}

/* update the system state ----------------------------------------------------
* update the system state using sensor measurements
* args   : vect3*     pwm      I   angular velocity measurements
*          vect3*     pvm      I   linear velocity measurements
*          double     dS       I   distance measurement
*          int        nSamples I   number of samples
*          double     ts       I   time step
*          int        nSteps   I   number of steps (default: 6)
* return : update status
*-----------------------------------------------------------------------------*/
int SINSGNSSBorad::Update(const vect3* pwm, const vect3* pvm, double dS,
	int nn, double ts, int nSteps, bool isBack)
{
	int res = SINSGNSSLOOSE::Update(pwm, pvm, nn, ts, nSteps);
	sins.lever(od.lvOD, &od.posOD, &od.vnOD);
	sins.lever(od_nhc.lvOD, &od_nhc.posOD, &od_nhc.vnOD);

	if (config_od) {
		if (fabs(dS) < 66.00) {
			if(isBack) od.Update(-dS * ts);
			else od.Update(dS * ts);
		}
		//else if(sqrt(norm(Pk.GetDiagVect3(3))) < 2e-2) od.Update(norm(sins.vn));
		if (od.odmeasOK) {
			Hk.SetMat3(6, 0, od.odMphi, od.odMvn);
			Hk.SetMat3(6, 15, od.odMkappa);
			*(vect3*)&Zk.dd[6] = od.odZk;  // SINS/OD-dvn
			SetMeasFlag(0700);
		}
	}
	else {
		vect3 pk_vn = Pk.GetDiagVect3(3);
		if (norm(pk_vn) < 0.001) {
			dS = norm(sins.vn);
			if (isBack) od_nhc.Update(-dS * ts);
			else od_nhc.Update(dS * ts);
			if (od_nhc.odmeasOK) {
				Hk.SetMat3(12, 0, od_nhc.odMphi, od_nhc.odMvn);
				Hk.SetMat3(12, 15, od_nhc.odMkappa);
				*(vect3*)&Zk.dd[12] = od_nhc.odZk;  // SINS/OD-dvn
				SetMeasFlag(070000);
			}
			if (dS < 0.09) {
				//sins.pos = pos_pre;
			}
		}
		else {
			pos_pre = sins.pos;
		}
	}
	dS = norm(sins.vn) * ts;
	distance_trj += dS;
	gnssLostdist += dS;
	gnssLostnofixdist += dS;

	ZIHRtest();
	return res;
}

/* apply feedback to the system state -----------------------------------------
* apply feedback to the system state based on Kalman filter results
* args   : int        nnq      I   size of the state vector
*          double     fbts     I   feedback time step
* return : none
*-----------------------------------------------------------------------------*/
void SINSGNSSBorad::Feedback(int nnq, double fbts)
{
	SINSGNSSLOOSE::Feedback(15, fbts);
	dyawGNSS += FBXk.dd[18];

	if (config_od) {
		od.Cbo = od.Cbo * a2mat(vect3(FBXk.dd[15], 0.0, FBXk.dd[17]));
		od.Kod *= 1 - FBXk.dd[16];
	}
	else {
		od_nhc.Cbo = od_nhc.Cbo * a2mat(vect3(FBXk.dd[15], 0.0, FBXk.dd[17]));
		od_nhc.Kod *= 1 - FBXk.dd[16];
	}
}

/* test zero-velocity update (ZUPT) -------------------------------------------
* perform a zero-velocity update test
* args   : none
* return : none
*-----------------------------------------------------------------------------*/
void SINSGNSSBorad::ZUPTtest(int gnss_lost)
{
	if (sins.mmwb.flag == 1) {
		double dwb = sins.mmwb.maxRes - sins.mmwb.minRes,
			dfb = sins.mmfb.maxRes - sins.mmfb.minRes;
		if (gnss_lost) {
			if (dfb < 60 * MG && sins.mmfb.maxRes < 120 * MG) {
				*(vect3*)&Zk.dd[9] = sins.vn - vn_pre;
				//*(vect3*)&Zk.dd[9] = O31;
				sins.vn = vn_pre;
				SetMeasFlag(0007000);
			}
		}
		else {
			if (dwb < 0.3 * DPS && sins.mmwb.maxRes < 1.0 * DPS && dfb < 100 * MG && sins.mmfb.maxRes < 200 * MG) {
				*(vect3*)&Zk.dd[9] = sins.vn;
				SetMeasFlag(0007000);
			}
			vn_pre = sins.vn;
		}
		//vn_pre = sins.vn;
	}
}

/* test zero-integrated heading rate (ZIHR) -----------------------------------
* perform a zero-integrated heading rate test
* args   : none
* return : none
*-----------------------------------------------------------------------------*/
void SINSGNSSBorad::ZIHRtest(void)
{
	wzhd.Update(sins.imu.phim.k / sins.nts);
	if (wzhd.retres == 3 && sins.fn.k > 9.5) {
		//Zk.dd[16] = wzhd.meanwpre-sins.eth.wnie.k;
		Zk.dd[16] = wzhd.meanw - sins.eth.wnie.k;
		SetMeasFlag(0200000);
		SetMeasStop(0100000, wzhd.T);
		Xk.dd[2] -= Zk.dd[16] * wzhd.T;    //	sins.qnb -= CVect3(0,0,-Zk.dd[16]*wzhd.T);
	}
}

/* set GNSS fix mode ----------------------------------------------------------
* set the GNSS fix mode
* args   : int        mode     I   GNSS fix mode (default: 0)
* return : none
*-----------------------------------------------------------------------------*/
void SINSGNSSBorad::SetGNSSFixMode(int mode)
{
	if (mode == 0) { // init or non-GNSS
		fixLost = fixLast = nofixLost = nofixLast = gnssLast = 0;
	}
	else if (mode == 1) { // fixed mode
		fixLost = 0;  fixLast++;  nofixLost++;  nofixLast = 0;  gnssLast++;
		if (fixLast > 3) { gnssLostdist = 0.0; gnssLostnofixdist = 0.0; }
	}
	else if (mode == 2) { // non-fixed mode
		fixLost++;  fixLast = 0;  nofixLost = 0;  nofixLast++;  gnssLast++;
		if (nofixLast > 5) { gnssLostdist = 0.0; }
	}
}

#ifdef MEMORY
/* Default constructor for RingMemory ------------------------------------------
* Initializes the ring buffer pointers to NULL.
*-----------------------------------------------------------------------------*/
RingMemory::RingMemory(void)
{
	pMemStart0 = NULL;
}

/* Constructor for RingMemory with allocation ----------------------------------
* Allocates memory for the ring buffer with the given number of records and record length.
* args   : long recordNum   I   number of records
*          int recordLen0   I   length of each record (bytes)
*-----------------------------------------------------------------------------*/
RingMemory::RingMemory(long recordNum, int recordLen0)
{
	uint8_t* pb = (uint8_t*)malloc(recordNum * recordLen0);
	*this = RingMemory(pb, recordNum * recordLen0, recordLen0);
	pMemStart0 = pMemStart;
}

/* Constructor for RingMemory with external memory -----------------------------
* Initializes the ring buffer using external memory.
* args   : uint8_t *pMem    I   pointer to memory buffer
*          long memLen0     I   total memory length (bytes)
*          int recordLen0   I   length of each record (bytes)
*-----------------------------------------------------------------------------*/
RingMemory::RingMemory(uint8_t* pMem, long memLen0, int recordLen0)
{
	assert(recordLen0 <= MAX_RECORD_BYTES);
	pMemStart0 = NULL;
	pMemStart = pMemPush = pMemPop = pMem;
	pMemEnd = pMemStart + memLen0;
	pushLen = popLen = recordLen = recordLen0;
	memLen = memLen0;
	dataLen = 0;
}

/* Destructor for RingMemory ---------------------------------------------------
* Frees allocated memory if owned by the buffer.
*-----------------------------------------------------------------------------*/
RingMemory::~RingMemory()
{
	if (pMemStart0) { free(pMemStart0); pMemStart0 = NULL; }
}

/* Pop a record from the ring buffer -------------------------------------------
* Pops one record from the buffer into the provided pointer.
* args   : uint8_t *p       O   pointer to output buffer (can be NULL)
* return : number of bytes popped
*-----------------------------------------------------------------------------*/
uint8_t RingMemory::pop(uint8_t* p)
{
	if (dataLen == 0) return 0;
	popLen = recordLen == 0 ? *pMemPop : recordLen;
	if (p == (uint8_t*)NULL) p = popBuf;
	uint8_t i;
	for (i = 0; i < popLen; i++, dataLen--)
	{
		*p++ = *pMemPop++;
		if (pMemPop >= pMemEnd)  pMemPop = pMemStart;
	}
	return i;
}

/* Get pointer to a specific frame in the buffer -------------------------------
* Returns a pointer to the specified frame in the buffer.
* args   : int iframe       I   frame index
* return : pointer to the frame
*-----------------------------------------------------------------------------*/
uint8_t* RingMemory::get(int iframe)
{
	return &pMemStart[popLen * iframe];
}

/* Set data for a specific frame in the buffer ---------------------------------
* Sets the data for the specified frame in the buffer.
* args   : int iframe       I   frame index
*          const uint8_t *p I   pointer to input data
* return : pointer to the frame
*-----------------------------------------------------------------------------*/
uint8_t* RingMemory::set(int iframe, const uint8_t* p)
{
	uint8_t* p0 = &pMemStart[pushLen * iframe];
	uint8_t i;
	for (i = 0; i < pushLen; i++)
	{
		*p0++ = *p++;
		if (p0 >= pMemEnd)  p0 = pMemStart;
	}
	return &pMemStart[pushLen * iframe];
}

/* Push a record into the ring buffer ------------------------------------------
* Pushes one record into the buffer. Overwrites oldest data if buffer is full.
* args   : const uint8_t *p I   pointer to input data (can be NULL)
* return : 1 if normal, 0 if overwrite occurred
*-----------------------------------------------------------------------------*/
uint8_t RingMemory::push(const uint8_t* p)
{
	uint8_t res = 1;
	if (p == (uint8_t*)NULL) p = pushBuf;
	pushLen = recordLen == 0 ? *p : recordLen;
	assert(pushLen <= MAX_RECORD_BYTES);
	for (uint8_t i = 0; i < pushLen; i++, dataLen++)
	{
		*pMemPush++ = *p++;
		if (pMemPush >= pMemEnd)  pMemPush = pMemStart;
		if (pMemPush == pMemPop) { res = 0; pop(); }
	}
	return res;
}

/* Smooth class constructor ----------------------------------------------------
* Initializes the smoothing filter with specified vector dimension and window size.
* args   : int clm          I   vector dimension
*          int row          I   smoothing window size
*-----------------------------------------------------------------------------*/
SmoothAVP::SmoothAVP(int clm, int row)
{
	assert(clm <= MAX_MAT_DIM);
	irow = 0; maxrow = row;
	pmem = new RingMemory(row + 2, clm * sizeof(double));
	sum = mean = tmp = vect(clm, 0.0);
}

/* Smooth class destructor -----------------------------------------------------
* Releases the memory used by the smoothing filter.
*-----------------------------------------------------------------------------*/
SmoothAVP::~SmoothAVP()
{
	delete pmem;
}

/* Update the smoothing filter with new data -----------------------------------
* Updates the smoothing filter with a new data vector and computes the mean.
* args   : const double *p  I   pointer to new data vector
*          double *pmean    O   pointer to output mean vector (can be NULL)
* return : mean vector after update
*-----------------------------------------------------------------------------*/
vect SmoothAVP::Update(const double* p, double* pmean)
{
	pmem->push((unsigned char*)p);
	sum += vect(tmp.rc, p);
	if (irow < maxrow) {
		irow++;
		tmp = 0.0;
	}
	else {
		pmem->pop((unsigned char*)tmp.dd);
	}
	sum -= tmp;
	mean = sum * (1.0 / irow);
	if (pmean) memcpy(pmean, mean.dd, mean.rc * sizeof(double));


	return mean;
}
#endif

#ifdef FILEIO
char CFileRdWt::dirIn[256] = { 0 }, CFileRdWt::dirOut[256] = { 0 };

/* set input directory --------------------------------------------------------
* set the input directory path
* args   : const char* dirI   I   input directory path
* return : none
*-----------------------------------------------------------------------------*/
void CFileRdWt::DirI(const char* dirI)  // set dirIN
{
	int len = strlen(dirI);
	memcpy(dirIn, dirI, len);
	if (dirIn[len - 1] != '\\') { dirIn[len] = '\\'; dirIn[len + 1] = '\0'; }
	if (access(dirIn, 0) == -1) dirIn[0] = '\0';
}

/* set output directory -------------------------------------------------------
* set the output directory path
* args   : const char* dirO   I   output directory path
* return : none
*-----------------------------------------------------------------------------*/
void CFileRdWt::DirO(const char* dirO)  // set dirOUT
{
	int len = strlen(dirO);
	memcpy(dirOut, dirO, len);
	if (dirOut[len - 1] != '\\') { dirOut[len] = '\\'; dirOut[len + 1] = '\0'; }
	if (access(dirOut, 0) == -1) dirOut[0] = '\0';
}

/* set input and output directories -------------------------------------------
* set both input and output directory paths
* args   : const char* dirI   I   input directory path
*          const char* dirO   I   output directory path (optional)
* return : none
*-----------------------------------------------------------------------------*/
void CFileRdWt::Dir(const char* dirI, const char* dirO)  // set dirIN&OUT
{
	DirI(dirI);
	DirO(dirO ? dirO : dirI);
}

/* default constructor --------------------------------------------------------
* initialize file reader/writer object with default values
* args   : none
* return : none
*-----------------------------------------------------------------------------*/
CFileRdWt::CFileRdWt()
{
	rwf = 0;
}

/* constructor with file name and columns -------------------------------------
* initialize with file name and number of columns
* args   : const char* fname0 I   file name
*          int columns0       I   number of columns (default: 0)
* return : none
*-----------------------------------------------------------------------------*/
CFileRdWt::CFileRdWt(const char* fname0, int columns0)
{
	rwf = 0;
	Init(fname0, columns0);
	memset(buff, 0, sizeof(buff));
}

/* destructor -----------------------------------------------------------------
* close file and release resources
* args   : none
* return : none
*-----------------------------------------------------------------------------*/
CFileRdWt::~CFileRdWt()
{
	if (rwf) { fclose(rwf); rwf = (FILE*)NULL; }
}

/* initialize with file name and columns --------------------------------------
* open file and set number of columns
* args   : const char* fname0 I   file name
*          int columns0       I   number of columns (default: 0)
* return : none
*-----------------------------------------------------------------------------*/
void CFileRdWt::Init(const char* fname0, int columns0)
{
	fname[0] = '\0';
	int findc = 0, len0 = strlen(fname0);
	for (int i = 0; i < len0; i++) { if (fname0[i] == '\\') { findc = 1; break; } }
	columns = columns0;
	if (columns == 0)		// file write
	{
		if (dirOut[0] != 0 && findc == 0) { strcat(fname, dirOut); }
	}
	else				// file read
	{
		if (dirIn[0] != 0 && findc == 0) { strcat(fname, dirIn); }
	}
	strcat(fname, fname0);
	if (rwf) this->~CFileRdWt();
	if (columns == 0)				// bin file write
	{
		rwf = fopen(fname, "wb");
	}
	else if (columns < 0)			// bin file read
	{
		rwf = fopen(fname, "rb");
	}
	else if (columns > 0)			// txt file read
	{
		rwf = fopen(fname, "rt");
		if (!rwf) return;
		//		fpos_t pos;
		long pos;
		while (1)  // skip txt-file comments
		{
			pos = ftell(rwf);
			fgets(line, sizeof(line), rwf);
			if (feof(rwf)) break;
			int allSpace = 1, allDigital = 1;
			for (register int i = 0; i < sizeof(line); i++)
			{
				char c = line[i];
				if (c == '\n') break;
				if (c != ' ') allSpace = 0;
				if (!(c == ' ' || c == ',' || c == ';' || c == ':' || c == '+' || c == '-' || c == '.' || c == '\t'
					|| c == 'e' || c == 'E' || c == 'd' || c == 'D' || ('9' >= c && c >= '0')))
					allDigital = 0;
			}
			if (!allSpace && allDigital) break;
		}
		//		fsetpos(rwf, &pos);
		fseek(rwf, pos, SEEK_SET);
		this->columns = columns;
		for (int i = 0; i < columns; i++)
		{
			sstr[4 * i + 0] = '%', sstr[4 * i + 1] = 'l', sstr[4 * i + 2] = 'f', sstr[4 * i + 3] = ' ', sstr[4 * i + 4] = '\0';
		}
	}
	else
	{
		rwf = 0;
	}
	linek = 0;
	totsize = 0;
	svpos[0] = svpos[1] = svpos[2] = ftell(rwf);  items[0] = items[1] = items[2] = 0;
}

/* load data from file (double) -----------------------------------------------
* load specified number of lines from file as double data
* args   : int lines          I   number of lines to load (default: 1)
*          bool txtDelComma   I   whether to delete commas in text (default: 1)
* return : number of lines loaded
*-----------------------------------------------------------------------------*/
int CFileRdWt::load(int lines, bool txtDelComma)
{
	if (columns < 0)			// bin file read
	{
		if (lines > 1)	fseek(rwf, (lines - 1) * (-columns) * sizeof(double), SEEK_CUR);
		fread(buff, -columns, sizeof(double), rwf);
		if (lines == 0) fseek(rwf, columns * sizeof(double), SEEK_CUR);
	}
	else					// txt file read
	{
		for (int i = 0; i < lines; i++)  fgets(line, sizeof(line), rwf);
		if (txtDelComma)
		{
			for (char* pc = line, *pend = line + sizeof(line); pc < pend; pc++)
			{
				if (*pc == ',' || *pc == ';' || *pc == ':' || *pc == '\t:') *pc = ' ';
				else if (*pc == '\0') break;
			}
		}
		if (columns < 10)
			sscanf(line, sstr,
				&buff[0], &buff[1], &buff[2], &buff[3], &buff[4], &buff[5], &buff[6], &buff[7], &buff[8], &buff[9]
			);
		else if (columns < 20)
			sscanf(line, sstr,
				&buff[0], &buff[1], &buff[2], &buff[3], &buff[4], &buff[5], &buff[6], &buff[7], &buff[8], &buff[9],
				&buff[10], &buff[11], &buff[12], &buff[13], &buff[14], &buff[15], &buff[16], &buff[17], &buff[18], &buff[19]
			);
		else if (columns < 40)
			sscanf(line, sstr,
				&buff[0], &buff[1], &buff[2], &buff[3], &buff[4], &buff[5], &buff[6], &buff[7], &buff[8], &buff[9],
				&buff[10], &buff[11], &buff[12], &buff[13], &buff[14], &buff[15], &buff[16], &buff[17], &buff[18], &buff[19],
				&buff[20], &buff[21], &buff[22], &buff[23], &buff[24], &buff[25], &buff[26], &buff[27], &buff[28], &buff[29],
				&buff[30], &buff[31], &buff[32], &buff[33], &buff[34], &buff[35], &buff[36], &buff[37], &buff[38], &buff[39]
			);
	}
	linek += lines;
	if (feof(rwf))  return 0;
	else return 1;
}

/* load data from file (float) ------------------------------------------------
* load specified number of lines from file as float data
* args   : int lines          I   number of lines to load (default: 1)
* return : number of lines loaded
*-----------------------------------------------------------------------------*/
int CFileRdWt::loadf32(int lines)	// float32 bin file read
{
	if (lines > 1)
		fseek(rwf, (lines - 1) * (-columns) * sizeof(float), SEEK_CUR);
	fread(buff32, -columns, sizeof(float), rwf);
	for (register int i = 0; i < -columns; i++) buff[i] = buff32[i];
	linek += lines;
	if (feof(rwf))  return 0;
	else return 1;
}

/* load binary data from file -------------------------------------------------
* load binary data into buffer
* args   : uint8_t* buf       O   buffer to load data into
*          long bufsize       I   size of buffer
* return : number of bytes loaded
*-----------------------------------------------------------------------------*/
long CFileRdWt::load(uint8_t* buf, long bufsize)  // load bin file
{
	long cur = ftell(rwf);
	fseek(rwf, 0L, SEEK_END);
	long rest = ftell(rwf) - cur;
	fseek(rwf, cur, SEEK_SET);
	if (bufsize > rest) bufsize = rest;
	fread(buf, bufsize, 1, rwf);
	return bufsize;
}

/* seek in binary file --------------------------------------------------------
* seek to a specific line in binary file
* args   : int lines          I   number of lines to seek
*          int mod            I   seek mode (default: SEEK_CUR)
* return : none
*-----------------------------------------------------------------------------*/
void CFileRdWt::bwseek(int lines, int mod)  // double bin-file backward-seek lines
{
	fseek(rwf, lines * (-columns) * sizeof(double), mod);
	linek -= lines;
}

/* get file size --------------------------------------------------------------
* get the size of the file
* args   : int opt            I   option flag (default: 1)
* return : file size in bytes
*-----------------------------------------------------------------------------*/
long CFileRdWt::filesize(int opt)
{
	long cur = ftell(rwf);
	if (totsize == 0)
	{
		fseek(rwf, 0L, SEEK_END);
		totsize = ftell(rwf);			// get total_size
		fseek(rwf, cur, SEEK_SET);
	}
	remsize = totsize - cur;
	return opt ? remsize : totsize;  // opt==1 for remain_size, ==0 for total_size
}

/* get a line from file -------------------------------------------------------
* read a line from the file into buffer
* args   : none
* return : number of items read or status
*-----------------------------------------------------------------------------*/
int CFileRdWt::getl(void)	// txt file get a line
{
	fgets(line, sizeof(line), rwf);
	return strlen(line);
}

/* save current file position -------------------------------------------------
* save the current file position for later restoration
* args   : int i              I   index for saving position (default: 0)
* return : saved file position
*-----------------------------------------------------------------------------*/
long CFileRdWt::savepos(int i)
{
	if (i < 0 || i>2) i = 0;
	items[i] = linek;
	return (svpos[i] = ftell(rwf));
}

/* restore file position ------------------------------------------------------
* restore file position to a previously saved position
* args   : int i              I   index of saved position (default: 0)
* return : status (0: success, -1: failure)
*-----------------------------------------------------------------------------*/
int CFileRdWt::restorepos(int i)
{
	if (i < 0 || i>2) i = 0;
	linek = items[i];
	return fseek(rwf, svpos[i], SEEK_SET);
}

/* wait for value in column ---------------------------------------------------
* wait until a value appears in the specified column within a tolerance
* args   : int columnk        I   column index
*          double val         I   value to wait for (default: 0.0)
*          double eps         I   tolerance (default: EPS)
* return : true if value found, false otherwise
*-----------------------------------------------------------------------------*/
bool CFileRdWt::waitfor(int columnk, double val, double eps)
{
	double wf = buff[columnk] - val;
	while (-eps < wf && wf < eps) {
		load(1);
		if (feof(rwf)) return 0;
		wf = buff[columnk] - val;
	}
	return 1;
}

#ifdef MEMORY
CFileRdWt& CFileRdWt::operator<<(const RingMemory& m)
{
	fwrite(m.pMemStart0, m.memLen, 1, rwf);
	return *this;
}
#endif

static FILE* fp_sim;
static char* format_sim = "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d";
static uint8_t fix_type = 0;
static int8_t flag_file = 0;
int sim_open(char* file_name) {
	fp_sim = fopen(file_name, "rb");
	return 0;
}
int sim_close() {
	fclose(fp_sim);
	return 0;
}

int load_data(sim32bin_t* data_sim) {
	flag_file = fread(data_sim, sizeof(double), 32, fp_sim);
	return feof(fp_sim);
}
#endif

#ifdef TRACE
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include <errno.h>
static FILE* fp_trace = NULL;     /* file pointer of trace */
static char file_trace[1024];   /* trace file */
static int level_trace = 0;       /* level of trace */
static uint32_t tick_trace = 0;   /* tick time at traceopen (ms) */

void traceopen(const char* file)
{
	if ((fp_trace = fopen(file, "wb"))) {
		printf("log open -->\n");
	}
	else {
		printf("log not open -->\n");
	}
	//if (!(fp_trace = fopen(file, "wb"))) fp_trace = stderr;
	//strcpy(file_trace, file);
}

void traceclose(void)
{
	if (fp_trace) fclose(fp_trace);
	fp_trace = NULL;
	file_trace[0] = '\0';
}

void tracelevel(int level)
{
	level_trace = level;
}

int gettracelevel(void)
{
	return level_trace;
}

void trace(int level, const char* format, ...)
{
	va_list ap;

	/* print error message to stderr */
	if (level <= 1) {
		//        va_start(ap, format); vfprintf(stderr, format, ap); va_end(ap);
	}
	if (!fp_trace || level > level_trace) return;
	va_start(ap, format);
	vfprintf(fp_trace, format, ap);
	va_end(ap);
	fflush(fp_trace);
}
#else
void traceopen(const char* file) {}
void traceclose(void) {}
void tracelevel(int level) {}
void trace(int level, const char* format, ...) {}
void tracet(int level, const char* format, ...) {}
#endif
#ifdef CONFIGSIM
#include "ctype.h"
/* discard space characters at tail ------------------------------------------*/
static void chop(char* str)
{
	char* p;
	if ((p = strchr(str, '#'))) *p = '\0'; /* comment */
	for (p = str + strlen(str) - 1; p >= str && !isgraph((int)*p); p--) *p = '\0';
}
/* enum to string ------------------------------------------------------------*/
static int enum2str(char* s, const char* comment, int val)
{
	char str[32], * p, * q, com[1024];
	int n;

	strcpy(com, comment);
	n = sprintf(str, "%d:", val);
	if (!(p = strstr(com, str))) {
		return sprintf(s, "%d", val);
	}
	if (!(q = strchr(p + n, ',')) && !(q = strchr(p + n, ')'))) {
		strcpy(s, p + n);
		return (int)strlen(p + n);
	}
	strncpy(s, p + n, q - p - n); s[q - p - n] = '\0';
	return (int)(q - p - n);
}
/* string to enum ------------------------------------------------------------*/
static int str2enum(const char* str, const char* comment, int* val)
{
	const char* p;
	char s[32];

	for (p = comment;; p++) {
		if (!(p = strstr(p, str))) break;
		if (*(p - 1) != ':') continue;
		for (p -= 2; '0' <= *p && *p <= '9'; p--);
		return sscanf(p + 1, "%d", val) == 1;
	}
	sprintf(s, "%30.30s:", str);
	if ((p = strstr(comment, s))) { /* number */
		return sscanf(p, "%d", val) == 1;
	}
	return 0;
}

/* search option ---------------------------------------------------------------
* search option record
* args   : char   *name     I  option name
*          opt_t  *opts     I  options table
*                              (terminated with table[i].name="")
* return : option record (NULL: not found)
*-----------------------------------------------------------------------------*/
CONFIG_t* searchopt(const char* name, const CONFIG_t* opts)
{
	int i;

	for (i = 0; *opts[i].name; i++) {
		if (strstr(opts[i].name, name)) return (CONFIG_t*)(opts + i);
	}
	return NULL;
}
/* string to option value ------------------------------------------------------
* convert string to option value
* args   : opt_t  *opt      O  option
*          char   *str      I  option value string
* return : status (1:ok,0:error)
*-----------------------------------------------------------------------------*/
int str2opt(CONFIG_t* opt, const char* str)
{
	switch (opt->format) {
	case 0: *(int*)opt->var = atoi(str); break;
	case 1: *(double*)opt->var = atof(str); break;
	case 2: strcpy((char*)opt->var, str);  break;
	case 3: return str2enum(str, opt->comment, (int*)opt->var);
	default: return 0;
	}
	return 1;
}
/* option value to string ------------------------------------------------------
* convert option value to string
* args   : opt_t  *opt      I  option
*          char   *str      O  option value string
* return : length of output string
*-----------------------------------------------------------------------------*/
int config2str(const CONFIG_t* opt, char* str)
{
	char* p = str;
	switch (opt->format) {
	case 0: p += sprintf(p, "%d", *(int*)opt->var); break;
	case 1: p += sprintf(p, "%.15g", *(double*)opt->var); break;
	case 2: p += sprintf(p, "%s", (char*)opt->var); break;
	case 3: p += enum2str(p, opt->comment, *(int*)opt->var); break;
	}
	return (int)(p - str);
}
/* option to string -------------------------------------------------------------
* convert option to string (keyword=value # comment)
* args   : opt_t  *opt      I  option
*          char   *buff     O  option string
* return : length of output string
*-----------------------------------------------------------------------------*/
int config2buf(const CONFIG_t* opt, char* buff)
{
	char* p = buff;
	int n;
	p += sprintf(p, "%-18s =", opt->name);
	p += config2str(opt, p);
	if (*opt->comment) {
		if ((n = (int)(buff + 30 - p)) > 0) p += sprintf(p, "%*s", n, "");
		p += sprintf(p, " # (%s)", opt->comment);
	}
	return (int)(p - buff);
}
/* load options ----------------------------------------------------------------
* load options from file
* args   : char   *file     I  options file
*          opt_t  *opts     IO options table
*                              (terminated with table[i].name="")
* return : status (1:ok,0:error)
*-----------------------------------------------------------------------------*/
int loadconfigs(const char* file, CONFIG_t* opts)
{
	FILE* fp;
	CONFIG_t* opt;
	char buff[2048], * p;
	int n = 0;

	if (!(fp = fopen(file, "r"))) {
		return 0;
	}
	while (fgets(buff, sizeof(buff), fp)) {
		n++;
		chop(buff);

		if (buff[0] == '\0') continue;

		if (!(p = strstr(buff, "="))) {
			fprintf(stderr, "invalid option %s (%s:%d)\n", buff, file, n);
			continue;
		}
		*p++ = '\0';
		chop(buff);
		if (!(opt = searchopt(buff, opts))) continue;

		if (!str2opt(opt, p)) {
			fprintf(stderr, "invalid option value %s (%s:%d)\n", buff, file, n);
			continue;
		}
	}
	fclose(fp);

	return 1;
}

/* save options to file --------------------------------------------------------
* save options to file
* args   : char   *file     I  options file
*          char   *mode     I  write mode ("w":overwrite,"a":append);
*          char   *comment  I  header comment (NULL: no comment)
*          opt_t  *opts     I  options table
*                              (terminated with table[i].name="")
* return : status (1:ok,0:error)
*-----------------------------------------------------------------------------*/
int saveconfigs(const char* file, const char* mode, const char* comment,
	const CONFIG_t* opts)
{
	FILE* fp;
	char buff[1024];
	int i;

	if (!(fp = fopen(file, mode))) {
		return 0;
	}
	if (comment) fprintf(fp, "# %s\n\n", comment);

	for (i = 0; *opts[i].name; i++) {
		config2buf(opts + i, buff);
		fprintf(fp, "%s\n", buff);
	}
	fclose(fp);
	return 1;
}
#endif

/**
 * @brief Constructor for the KFAPP class.
 *
 * Initializes the KFAPP object with the specified navigation type and sets up
 * the Kalman filter parameters.
 *
 * @param NavType The navigation type to initialize the KFAPP object.
 */
KFAPP::KFAPP(int NavType) :SINSGNSSBorad(19, 17, TS, 15) {
	nav_type = NavType;
	time_run = 0.0;
	wmm_bias_align = O31;
	vmm_bias_align = O31;
#ifdef LOW_PASS_IIR
	//double a[3] = { 1.0,  -1.561018075800718,  0.641351538057563 };
	//double b[3] = { 0.020083365564211,  0.040166731128423,  0.020083365564211 };
	//double a[3] = {1.0,-0.369527377351241,0.195815712655833};
	//double b[3] = { 0.206572083826148,0.413144167652296,0.206572083826148 };
	double a[3] = { 1.0,  -1.142980502539901,  0.412801598096189 };
	double b[3] = { 0.067455273889072,  0.134910547778144,  0.067455273889072 };

	IIR_wmm = IIRV3(b, a, 3);
	IIR_vmm = IIRV3(b, a, 3);
#endif
}

/**
 * @brief Performs coast alignment using GNSS antenna or trajectory data.
 *
 * This function estimates the bias using IMU data in a static state and determines
 * whether to use the GNSS antenna heading or trajectory heading for coast alignment.
 *
 * @param ptr_UpPara Pointer to the `KFAPP_Update` structure containing the update parameters,
 *                   including IMU and GNSS data.
 * @return An integer indicating the type of coast alignment used:
 *         - `ALIGN_ANT` (1): Alignment using GNSS antenna heading.
 *         - `ALIGN_TRJ` (2): Alignment using trajectory heading.
 *         - `0`: Alignment not performed due to insufficient data.
 */
int KFAPP::coast_align(KFAPP_Update* ptr_UpPara) {
	/* estmate the bias with imu in static statue */
	get_static_bias(vect3(ptr_UpPara->imu.gyr_x, ptr_UpPara->imu.gyr_y, ptr_UpPara->imu.gyr_z),
		vect3(ptr_UpPara->imu.acc_x, ptr_UpPara->imu.acc_y, ptr_UpPara->imu.acc_z));

#ifdef LOW_PASS_IIR
	vect3 wmm(ptr_UpPara->imu.gyr_x, ptr_UpPara->imu.gyr_y, ptr_UpPara->imu.gyr_z);
	vect3 vmm(ptr_UpPara->imu.acc_x, ptr_UpPara->imu.acc_y, ptr_UpPara->imu.acc_z);

	wmm = IIR_wmm.Update(wmm);
	vmm = IIR_vmm.Update(vmm);
#endif

	int  res = 0;
	if (ptr_UpPara->gnss.fix_type < 1) return res;
	bool flag_trj = 0;
	bool flag_ant = 0;

	/** process for trj */
	float trj = vn2att(ptr_UpPara->gnss.vel_east, ptr_UpPara->gnss.vel_north);
	float std_trj = 0.0;
	if (fabs(trj) > 1e-6) {
		std_trj = trj_angle.Update(trj, 1);
		flag_trj = 1;
	}

	/** process for ant */
	float std_ant = 0.0;
	float ant = ptr_UpPara->gnss.heading;

	if (fabs(ant) > 1e-6 && ptr_UpPara->gnss.accu_heading < YAW_ALIGN && 4 == ptr_UpPara->gnss.fix_type && ptr_UpPara->gnss.accu_heading > 0.01 ) {
		std_ant = ant_angle.Update(ant, 1);
		flag_ant = 1;
	}

	/** which angle will be used?*/
	if (flag_ant && std_ant < YAW_STD * DEG) {
		yaw_coast_align = ant_angle.mean;
		res = ALIGN_ANT;
		flag_align = ALIGN_ANT;
		//trace(2,"ant init att was %0.3f deg\n",yaw_coast_align/DEG);
	}
	else if (flag_trj && std_trj < YAW_STD * DEG) {
		yaw_coast_align = trj_angle.mean;
		res = ALIGN_TRJ;
		flag_align = ALIGN_TRJ;
		//trace(2,"trj init att was %0.3f deg\n",yaw_coast_align/DEG);
	}
	return res;
}

/**
 * @brief Initializes the KFAPP object with configuration and update parameters.
 *
 * This function sets up the Kalman filter parameters, initializes the SINS system,
 * and configures the GNSS and odometer settings.
 *
 * @param ptr_ConPara Pointer to the `KFAPP_ConfigPara` structure containing configuration parameters.
 * @param ptr_UpPara Pointer to the `KFAPP_Update` structure containing update parameters.
 * @return An integer indicating the success of the initialization (1 for success).
 */
int KFAPP::para_init(KFAPP_ConfigPara* ptr_ConPara, KFAPP_Update* ptr_UpPara) {
	SINSGNSSBorad::Init(sins);
	/* if wmm < 1200 dph, assign the gyro was sattic & online the wzhd */
	wzhd.Init(0.3 * DPS, TS, 5.0);
	flag_resver = 0;
	/* init the para in the appconfigpara*/
	{
		{
			double s[] = { fXXZ(Pk_att_h_max,Pk_att_v_max),fXXZ(Pk_vel_h_max,Pk_vel_v_max),fXXZ(Pk_pos_h_max,Pk_pos_v_max),
				fXXZ(Pk_eb_h_max,Pk_eb_v_max), fXXZ(Pk_db_h_max,Pk_db_v_max),
				Pk_od_pitch_max, Pk_od_scale_max, Pk_od_yaw_max,
				Pk_gnss_yaw_max
			};
			Pmax.Set2(s, nq);
		}
		{
			double s[] = { fXXZ(Pk_att_h_min,Pk_att_v_min),fXXZ(Pk_vel_h_min,Pk_vel_v_min),fXXZ(Pk_pos_h_min,Pk_pos_v_min),
				fXXZ(Pk_eb_h_min,Pk_eb_v_min),  fXXZ(Pk_db_h_min,Pk_db_v_min),
				Pk_od_pitch_min, Pk_od_scale_min, Pk_od_yaw_min,
				Pk_gnss_yaw_min
			};
			Pmin.Set2(s, nq);
		}
		{
			double s[] = { fXXZ(Pk_att_h,Pk_att_v),fXXZ(Pk_vel_h,Pk_vel_v),fXXZ(Pk_pos_h,Pk_pos_v),
				fXXZ(Pk_eb_h,Pk_eb_v),  fXXZ(Pk_db_h,Pk_db_v),
				Pk_od_pitch, Pk_od_scale, Pk_od_yaw,
				Pk_gnss_yaw
			};
			Pk.SetDiag2(s, nq);
		}
		{
			double s[] = { fXXZ(Qt_gyr_h,Qt_gyr_v),  fXXZ(Qt_acc_h,Qt_acc_v), fXXX(Qt_pos),
				fXXX(Qt_eb),fXXX(Qt_db),
				fXXX(Qt_Other),
				Qt_Other
			};
			Qt.Set2(s, nq);
		}

		{
			double s[] = { fXXZ(Xk_att_h_max,Xk_att_v_max),fXXZ(Xk_vel_h_max,Xk_vel_v_max),fXXZ(Xk_pos_h_max,Xk_pos_v_max),
				fXXZ(Xk_eb_h_max,Xk_eb_v_max),  fXXZ(Xk_db_h_max,Xk_db_v_max),
				Xk_od_pitch_max, Xk_od_scale_max, Xk_od_yaw_max,
				Xk_gnss_yaw_max
			};
			Xmax.Set(s, nq);
		}

		{
			double s[] = { fXXZ(Rt_Vn_gnss_h,Rt_Vn_gnss_v),fdLLH(Rt_Pos_gnss_h,Rt_Pos_gnss_v),
				fXXZ(Rt_vn_od_h,Rt_vn_od_v),
				fXXX(Rt_zupt),fXXX(Rt_nhc),
				Rt_yaw_gnss,
				Rt_zihr
			};
			Rt.Set2(s, nr);
		}

		{
			double s[] = { fXXZ(FB_att_h,FB_att_v), fXXZ(FB_vel_h,FB_vel_v), fXXZ(FB_pos_h,FB_pos_v),
				fXXZ(FB_eb_h,FB_eb_v), fXXZ(FB_db_h,FB_db_v),
				FB_od_pitch, FB_od_scale, FB_od_yaw,
				FB_gnss_yaw
			};
			FBOne1.Set(s, nq);
		}

		Rmax = Rt * 300;
		Rmin = Rt * 0.001;
		Rb.SetBit(0000077, RB_Init);
		//Rb.SetBit(0000044, INF);
		Rb.SetBit(0100000, 0.0);
		Rb.SetBit(0000700, 0.5);
		Rb.SetBit(0007000, 0.0);
		Rb.SetBit(0070000, 0.0);
		Rmin.dd[15] = Rt.dd[15] * 0.001;
		Rmax.dd[15] = Rt.dd[15] * 10.0;
		RtTau.SetBit(007, 100.0);
		SetRadptStop(0xffffffff, 10.000);
	}

	{
		double s[] = { fXXX(FB_att_TAU),fXXX(FB_vel_TAU),fXXX(FB_pos_TAU),
			fXXX(FB_eb_TAU),fXXX(FB_db_TAU),
			fXXX(FB_od_TAU),
			FB_gnss_yaw_TAU
		};
		FBTau.Set(s, nq);
	}
#ifdef DSPRELTIME
	/* yaw ant config*/
	yawGNSSType = ptr_ConPara->ant_mode;
#else
	imu_cnt_ca = ptr_ConPara->imu_cnt;
	//yawGNSSType = Ant_Mode_LR_L;
	//yawGNSSType = Ant_Mode_LR_R;
	//yawGNSSType = Ant_Mode_FB_F;
	yawGNSSType = Ant_Mode_FB_B;
#endif
	dyawGNSS = ptr_ConPara->conf_GNSSAgle * DEG;

	if (ALIGN_ANT == flag_align) {
#ifdef DSPRELTIME
		yaw_coast_align = GNSS2Ali(yaw_coast_align, 2);
#else
		yaw_coast_align = GNSS2Ali(yaw_coast_align, 0);
#endif
		yaw_coast_align = diffYaw(yaw_coast_align, -dyawGNSS);
	}

	vect3 att(0, 0, yaw_coast_align);
	vect3 vn(ptr_UpPara->gnss.vel_east, ptr_UpPara->gnss.vel_north, -ptr_UpPara->gnss.vel_down);
	vect3 pos(ptr_UpPara->gnss.lat, ptr_UpPara->gnss.lon, ptr_UpPara->gnss.height_ellipsoid);
	lvGNSS = vect3(ptr_ConPara->conf_lvGNSS_i, ptr_ConPara->conf_lvGNSS_j, ptr_ConPara->conf_lvGNSS_k);
	sins.Init(a2qua(att), vn, pos, ptr_UpPara->imu.secs);
	sins.lever(lvGNSS * (-1.0), &sins.pos, &sins.vn);
	sins.lever(lvGNSS);
	pos_pre = sins.pos;
	avpi.Init(sins.att, sins.vnL, sins.posL, TS, AVPINUM);

	if (cnt_align) {
		sins.eb.i = wmm_bias_align.i / cnt_align;
		sins.eb.j = wmm_bias_align.j / cnt_align;
		sins.eb.k = wmm_bias_align.k / cnt_align;

		sins.db.i = vmm_bias_align.i / cnt_align;
		sins.db.j = vmm_bias_align.j / cnt_align;
		sins.db.k = vmm_bias_align.k / cnt_align;
	}

	vnGNSSdelay  = DT_VEL;
	posGNSSdelay = DT_POS;
	yawGNSSdelay = DT_YAW;
	dtGNSSdelay  = DT_GNSS;

	vnGNSS = O31;
	posGNSS = O31;
	yawGNSS = 0;
	flag_rms_gnss = 0;
	fixtype_gnss = 0;

#ifdef DSPRELTIME
	config_od = ptr_ConPara->conf_od;
#else
	config_od = OD_ENABLE;
#endif

	if (NAV_TYPE == nav_type) {
		od.ODKappa(vect3(ptr_ConPara->conf_odKappa_i * DEG, ptr_ConPara->conf_odKappa_j, ptr_ConPara->conf_odKappa_k * DEG));
		od.lvOD = vect3(ptr_ConPara->conf_lvOD_i, ptr_ConPara->conf_lvOD_j, ptr_ConPara->conf_lvOD_k);
	}
	else {
		od.ODKappa(vect3(0.0, 1.0, 0.0));
		od.lvOD = vect3(0.0);
	}

	od_nhc.ODKappa(vect3(0.0, 1.0, 0.0));
	od_nhc.lvOD = vect3(0.0);


	SetMeasMask(MASK_MEAS, 1);

	//MeasGNSSZvStop(vect3(0.2, 0.2, 0.5), 1.0);
	//MeasGNSSZpStop(vect3(0.5 / RE, 0.5 / RE, 0.8), 1.0);

	Hk(yawHkRow, 18) = 0.0;

//	if (CAL_TYPE == nav_type) {
////		Hk(yawHkRow, 18) = -1.0;
//	}
//	else if (NAV_TYPE == nav_type) {
//		Hk(yawHkRow, 18) = 0.0;
//	}

	return 1;
}

/**
 * @brief Updates the KFAPP object with new IMU and GNSS data.
 *
 * This function processes the IMU and GNSS data, updates the Kalman filter state,
 * and applies measurement corrections.
 *
 * @param ptr_UpPara Pointer to the `KFAPP_Update` structure containing the update parameters.
 * @return An integer indicating the success of the update (1 for success).
 *
 *
 */
int  KFAPP::Update(KFAPP_Update* ptr_UpPara) {
	
	vect3 wmm(ptr_UpPara->imu.gyr_x, ptr_UpPara->imu.gyr_y, ptr_UpPara->imu.gyr_z);
	vect3 vmm(ptr_UpPara->imu.acc_x, ptr_UpPara->imu.acc_y, ptr_UpPara->imu.acc_z);
#ifdef LOW_PASS_IIR
	wmm = IIR_wmm.Update(wmm);
	vmm = IIR_vmm.Update(vmm);
#endif
	double dS = ptr_UpPara->odo.mean;
	SINSGNSSBorad::Update(&wmm, &vmm, dS, 1, TS, N_STEP);
	sins.lever(lvGNSS);

	if (0 < ptr_UpPara->gnss.fix_type) {
		vnGNSS = vect3(ptr_UpPara->gnss.vel_east, ptr_UpPara->gnss.vel_north, -ptr_UpPara->gnss.vel_down);
		posGNSS = vect3(ptr_UpPara->gnss.lat, ptr_UpPara->gnss.lon, ptr_UpPara->gnss.height_ellipsoid);
		accu_posGNSS = vect3(ptr_UpPara->gnss.accu_lat, ptr_UpPara->gnss.accu_lon, ptr_UpPara->gnss.accu_hegit);
		fixtype_gnss = ptr_UpPara->gnss.fix_type;
		pos_dop_GNSS = ptr_UpPara->gnss.pos_dop;
		
		//vn_pre = vnGNSS;
		if (ptr_UpPara->gnss.accu_heading < YAW_ALIGN && ptr_UpPara->gnss.accu_heading > 0.01 && !IsZero(ptr_UpPara->gnss.heading)) {
#ifdef DSPRELTIME
			ptr_UpPara->gnss.heading = GNSS2Ali(ptr_UpPara->gnss.heading, 2);
#else
			ptr_UpPara->gnss.heading = GNSS2Ali(ptr_UpPara->gnss.heading, 0);
#endif
			yawGNSS = diffYaw(ptr_UpPara->gnss.heading, -dyawGNSS);
		}
		}

#ifdef DSPRELTIME
	if (0 < ptr_UpPara->gnss.fix_type) {
		SetMeasGNSS(posGNSS, vnGNSS, yawGNSS);
		check_GNSS_quality();
		accu_posGNSS = O31;
		fixtype_gnss = 0;
		pos_dop_GNSS = 0;
		posGNSS = O31;
		yawGNSS = 0;
		vnGNSS = O31;
	}
#else 
	if (imu_cnt_ca == ptr_UpPara->imu.cnt) {
		SetMeasGNSS(posGNSS, vnGNSS, yawGNSS);
		//SetMeasGNSS(posGNSS, vnGNSS, yawGNSS);
		//check_GNSS_quality();
		accu_posGNSS = O31;
		fixtype_gnss = 0;
		pos_dop_GNSS = 0;
		posGNSS = O31;
		yawGNSS = 0;
		vnGNSS = O31;
	}


#endif

	ZUPTtest(0);

#ifdef LOST_ASUME
	/*if GNSS was lost, assume heading was a line*/
	if ((0 == (flag_resver & 01)) && fabs(*gnssLost - 3.0) < 0.01) {
		wzhd.Init(0.5 * DPS, TS, 3.0);
		flag_resver |= 001;
	}
	else if ((1 == (flag_resver & 01)) && (*gnssLost) < 1) {
		wzhd.Init(0.3 * DPS, TS, 5.0);
		flag_resver &= ~001;
	}

	if (*gnssLost > 20.0) {
		ZUPTtest(1);
	}
#endif 

	time_run += TS;
	if ((0 == (flag_resver & 04)) && time_run > 180) {
		Rb.SetBit(0000033, RB_Norm);
		Rb.SetBit(0000044, RB_Norm);
		flag_resver |= 004;
	}
	return 1;
}

/**
 * @brief Converts GNSS heading to alignment heading based on the specified type.
 *
 * This function adjusts the GNSS heading to match the alignment heading based on
 * the antenna mode and type.
 *
 * @param gnss_Ang The GNSS heading angle in degrees or radians.
 * @param type The type of conversion to apply:
 *             - Bit 1: Indicates whether the input angle is in degrees.
 *             - Bit 2: Indicates whether the output angle should be normalized to [-180, 180].
 * @return The converted alignment heading angle.
 */
double KFAPP::GNSS2Ali(double gnss_Ang, int type) {
	double res = 0;
	if (type & 1) gnss_Ang *= DEG;
	switch (yawGNSSType) {
	case Ant_Mode_One:
		/* do nothing */
		res = 0;
		break;
	case Ant_Mode_LR_L:
		res = diffYaw(gnss_Ang, PI_2);
		break;
	case Ant_Mode_LR_R:
		res = diffYaw(gnss_Ang, -PI_2);
		break;
	case Ant_Mode_FB_F:
		res = diffYaw(gnss_Ang, PI);
		break;
	case Ant_Mode_FB_B:
		res = diffYaw(gnss_Ang, 0);
		break;
	case Ant_Mode_Ang:
		/* wait further process */

		break;
	}
	if (type & 2) res = C360CC180(res);
	return res;
}

/**
 * @brief Estimates the static bias of the IMU in a stationary state.
 *
 * This function calculates the gyroscope and accelerometer biases when the IMU
 * is stationary and updates the bias accumulators.
 *
 * @param wmm The gyroscope measurements (angular velocity) as a 3D vector.
 * @param vmm The accelerometer measurements (linear acceleration) as a 3D vector.
 * @return An integer indicating the success of the bias estimation (1 for success).
 */
int KFAPP::get_static_bias(vect3 wmm, vect3 vmm) {
	wmm *= 100.0;
	vmm *= 100.0; vmm.k -= G0;
	if (IsZeros(wmm, 0.1 * DPS) && IsZeros(vmm, 0.1)) {
		cnt_align++;
		wmm_bias_align += wmm;
		vmm_bias_align += vmm;
	}
	return 1;
}
#ifdef GNSS_CHECK_UM482
/**
 * @brief Checks the quality of GNSS data based on fix type, position RMS, and DOP.
 *
 * This function evaluates the GNSS data quality and adjusts measurement settings
 * based on the fix type, position root mean square (RMS) errors, and dilution of precision (DOP).
 *
 * @param fix_type The GNSS fix type (e.g., single, float, fixed).
 * @param pos_rms The position RMS errors as a 3D vector (latitude, longitude, height).
 * @param dop The dilution of precision value.
 * @return An integer indicating the quality check result (0 for success).
 *
 * note:
 *           for the rtk solution    20250417
 *           for the UM982 sol       20250418
 *           fix logical bug         20250427
 *           adopt spp solution      20250508
 *
 */
int KFAPP::check_GNSS_quality() {
	int res = 0;
	/* --------------------------------------------------------- */
	/* Classification of GNSS Solution Types:
		0: No Solution
		1: Single Solution
		2: Float Solution
		4: Fixed Solution
	*/
	int fix_lever = 0;
	if (4 == fixtype_gnss || 64 == fixtype_gnss) { SetGNSSFixMode(1); fix_lever = 4; }
	else if (1 == fixtype_gnss) { SetGNSSFixMode(2); fix_lever = 1; }
	else { SetGNSSFixMode(2); fix_lever = 2; }

	/* disable all GNSS meas*/
	SetMeasFlag(070, 0);
	/* if gnss in fix = 4,dop < 0.1,rms < 0.1, accp the gnss meas, Rset.dd[4] = -1.0;*/
	if (4 == fix_lever && pos_dop_GNSS < 1.0) {
		SetMeasFlag(077);
		if (accu_posGNSS.i < 0.1) { flag_rms_gnss &= ~0007; }
		if (accu_posGNSS.j < 0.1) { flag_rms_gnss &= ~0070; }
		if (accu_posGNSS.k < 0.1) { flag_rms_gnss &= ~0700; }
	}
	/* else if gnss in fix = 4, 0.1 =< dop < 0.3, that's mean there was abnormal pos, change the Rt*/
	else if (4 == fix_lever && pos_dop_GNSS < 3.0) {
		SetMeasFlag(077);
		/* check the lat*/
		if (!(flag_rms_gnss & 01) && accu_posGNSS.i < 0.01) { flag_rms_gnss &= ~0007; flag_rms_gnss |= 001; Rt.dd[3] = Rt.dd[3] * 2.0; }
		else if (accu_posGNSS.i >= 0.01 && accu_posGNSS.i < 0.1) {}
		else if (!(flag_rms_gnss & 02) && accu_posGNSS.i >= 0.1 && accu_posGNSS.i < 0.8) { flag_rms_gnss &= ~0007; flag_rms_gnss |= 002; Rt.dd[3] = Rt.dd[3] * 1.1; }
		else if (!(flag_rms_gnss & 04) && accu_posGNSS.i >= 0.8 && accu_posGNSS.i < 5.0) { flag_rms_gnss &= ~0007; flag_rms_gnss |= 004; Rt.dd[3] = Rt.dd[3] * 1.3; }
		else if (!(flag_rms_gnss & 01) && accu_posGNSS.i >= 5.0) { flag_rms_gnss &= ~0007; flag_rms_gnss |= 001;  Rt.dd[3] = Rt.dd[3] * 2.0; }

		/* check the lon*/
		if (!(flag_rms_gnss & 010) && accu_posGNSS.j < 0.01) { flag_rms_gnss &= ~0070; flag_rms_gnss |= 010; Rt.dd[4] = Rt.dd[4] * 2.0; }
		else if (accu_posGNSS.j >= 0.01 && accu_posGNSS.j < 0.1) {}
		else if (!(flag_rms_gnss & 020) && accu_posGNSS.j >= 0.1 && accu_posGNSS.j < 0.8) { flag_rms_gnss &= ~0070; flag_rms_gnss |= 020; Rt.dd[4] = Rt.dd[4] * 1.1; }
		else if (!(flag_rms_gnss & 040) && accu_posGNSS.i >= 0.8 && accu_posGNSS.j < 5.0) { flag_rms_gnss &= ~0070; flag_rms_gnss |= 040; Rt.dd[4] = Rt.dd[4] * 1.3; }
		else if (!(flag_rms_gnss & 010) && accu_posGNSS.j >= 5.0) { flag_rms_gnss &= ~0070; flag_rms_gnss |= 010; Rt.dd[4] = Rt.dd[4] * 2.0; }

		/* check the hgt*/
		if (!(flag_rms_gnss & 0100) && accu_posGNSS.k < 0.01) { flag_rms_gnss &= ~0700; flag_rms_gnss |= 0100; Rt.dd[5] = Rt.dd[5] * 2.0; }
		else if (accu_posGNSS.k >= 0.01 && accu_posGNSS.k < 0.1) {}
		else if (!(flag_rms_gnss & 0200) && accu_posGNSS.k >= 0.1 && accu_posGNSS.k < 0.8) { flag_rms_gnss &= ~0700; flag_rms_gnss |= 0200; Rt.dd[5] = Rt.dd[5] * 1.1; }
		else if (!(flag_rms_gnss & 0400) && accu_posGNSS.k >= 0.8 && accu_posGNSS.k < 5.0) { flag_rms_gnss &= ~0700; flag_rms_gnss |= 0400; Rt.dd[5] = Rt.dd[5] * 1.3; }
		else if (!(flag_rms_gnss & 0100) && accu_posGNSS.k >= 5.0) { flag_rms_gnss &= ~0700; flag_rms_gnss |= 0100; Rt.dd[5] = Rt.dd[5] * 2.0; }
	}
	else if (1 == fix_lever && pos_dop_GNSS < 10) {
		/* check the lat*/
		if (!(flag_rms_gnss & 01) && accu_posGNSS.i < 0.01) { flag_rms_gnss &= ~0007; }
		else if (accu_posGNSS.i >= 0.01 && accu_posGNSS.i < 1.0) { flag_rms_gnss &= ~0007; SetMeasFlag(012); }
		else if (!(flag_rms_gnss & 02) && accu_posGNSS.i >= 1.0 && accu_posGNSS.i < 15.0) { flag_rms_gnss &= ~0007; }
		else if (!(flag_rms_gnss & 04) && accu_posGNSS.i >= 15.0 && accu_posGNSS.i < 30.0) { flag_rms_gnss &= ~0007; }
		else if (!(flag_rms_gnss & 01) && accu_posGNSS.i >= 30.0) { flag_rms_gnss &= ~0007; }

		/* check the lon*/
		if (!(flag_rms_gnss & 010) && accu_posGNSS.j < 0.01) { flag_rms_gnss &= ~0070; }
		else if (accu_posGNSS.j >= 0.01 && accu_posGNSS.j < 1.0) { flag_rms_gnss &= ~0070; SetMeasFlag(021); }
		else if (!(flag_rms_gnss & 020) && accu_posGNSS.j >= 1.0 && accu_posGNSS.j < 15.0) { flag_rms_gnss &= ~0070; }
		else if (!(flag_rms_gnss & 040) && accu_posGNSS.j >= 15.0 && accu_posGNSS.j < 30.0) { flag_rms_gnss &= ~0070; }
		else if (!(flag_rms_gnss & 010) && accu_posGNSS.j >= 30) { flag_rms_gnss &= ~0070; }

		/* check the hgt*/
		if (!(flag_rms_gnss & 0100) && accu_posGNSS.k < 0.01) { flag_rms_gnss &= ~0007; }
		else if (accu_posGNSS.k >= 0.01 && accu_posGNSS.k < 1.0) { flag_rms_gnss &= ~0700; SetMeasFlag(044); }
		else if (!(flag_rms_gnss & 0200) && accu_posGNSS.k >= 1.0 && accu_posGNSS.k < 30.0) { flag_rms_gnss &= ~0700; flag_rms_gnss |= 0200; Rt.dd[5] = Rt.dd[5] * 3.0; SetMeasFlag(044); }
		else if (!(flag_rms_gnss & 0400) && accu_posGNSS.k >= 30.0 && accu_posGNSS.k < 100.0) { flag_rms_gnss &= ~0700; }
		else if (!(flag_rms_gnss & 0100) && accu_posGNSS.k >= 100.0) { flag_rms_gnss &= ~0700; }
	}
	else if (fix_lever == 2 && pos_dop_GNSS < 3.0) {
		/* check the lat*/
		if (accu_posGNSS.i < 0.01) {}
		else if (accu_posGNSS.i >= 0.01 && accu_posGNSS.i < 0.1) { SetMeasFlag(012); flag_rms_gnss &= ~0007; }
		else if (!(flag_rms_gnss & 002) && accu_posGNSS.i >= 0.1 && accu_posGNSS.i < 1.0) { SetMeasFlag(012); flag_rms_gnss &= ~0007; flag_rms_gnss |= 002; Rt.dd[3] = Rt.dd[3] * 3.0; }
		else if (!(flag_rms_gnss & 004) && accu_posGNSS.i >= 1.0 && accu_posGNSS.i < 10.0) { flag_rms_gnss &= ~0007; }
		else if (!(flag_rms_gnss & 001) && accu_posGNSS.i >= 10.0) {  }

		/* check the lon*/
		if (accu_posGNSS.j < 0.01) {}
		else if (accu_posGNSS.j < 0.1 && accu_posGNSS.j >= 0.01) { SetMeasFlag(021); flag_rms_gnss &= ~0070; }
		else if (!(flag_rms_gnss & 0020) && accu_posGNSS.j > 0.8 && accu_posGNSS.j <= 1.0) { SetMeasFlag(021); flag_rms_gnss &= ~0070; flag_rms_gnss |= 020; Rt.dd[4] = Rt.dd[4] * 3.0; }
		else if (!(flag_rms_gnss & 0040) && accu_posGNSS.j < 10.0 && accu_posGNSS.j >= 1.0) { flag_rms_gnss &= ~0070; }
		else if (!(flag_rms_gnss & 0010) && accu_posGNSS.j >= 10.0) {  }

		/* check the hgt*/
		if (accu_posGNSS.k < 0.01) {}
		else if (accu_posGNSS.k < 0.1 && accu_posGNSS.k >= 0.01) { SetMeasFlag(044); flag_rms_gnss &= ~0700;}
		else if (!(flag_rms_gnss & 0200) && accu_posGNSS.k < 0.8 && accu_posGNSS.k <= 1.0) { SetMeasFlag(044); flag_rms_gnss &= ~0700; flag_rms_gnss |= 0200; Rt.dd[5] = Rt.dd[5] * 3.0; }
		else if (!(flag_rms_gnss & 0400) && accu_posGNSS.k < 10.0 && accu_posGNSS.k >= 1.0) { SetMeasFlag(044); flag_rms_gnss &= ~0700; flag_rms_gnss |= 0400; Rt.dd[5] = Rt.dd[5] * 10.0; }
		else if (!(flag_rms_gnss & 0100) && accu_posGNSS.k >= 10.0) { SetMeasFlag(044); flag_rms_gnss &= ~0700; flag_rms_gnss |= 0100; Rt.dd[5] = Rt.dd[5] * 100.0; }
	}

	return res;
}
#endif 

#ifdef GNSS_CHECK_UM982
/**
 * @brief Checks the quality of GNSS data based on fix type, position RMS, and DOP.
 *
 * This function evaluates the GNSS data quality and adjusts measurement settings
 * based on the fix type, position root mean square (RMS) errors, and dilution of precision (DOP).
 *
 * @param fix_type The GNSS fix type (e.g., single, float, fixed).
 * @param pos_rms The position RMS errors as a 3D vector (latitude, longitude, height).
 * @param dop The dilution of precision value.
 * @return An integer indicating the quality check result (0 for success).
 *                   
 * flag_rms_gnss   0 7   7   7
 *                   |   |   |
 *         bit      6:9 3:5 0:2
 *                   |   |   |
 *                  hgt  lon lat
 *  00:good
 *  01:can't use
 *  02:useable, but not good
 *  04:bad 
 * 
 * note:
 *           for the rtk solution    20250417
 *           for the UM982 sol       20250418
 *           fix logical bug         20250427
 *           adopt spp solution      20250508
 *           
 *			 
 *
 */
int KFAPP::check_GNSS_quality() {
	int res = 0;
	/* --------------------------------------------------------- */
	/* Classification of GNSS Solution Types:
		0: No Solution
		1: Single Solution
		2: Float Solution
		4: Fixed Solution
	*/
	int fix_lever = 0;
	if (4 == fixtype_gnss || 64 == fixtype_gnss) { SetGNSSFixMode(1); fix_lever = 4; }
	else if (1 == fixtype_gnss) { SetGNSSFixMode(2); fix_lever = 1; }
	else { SetGNSSFixMode(2); fix_lever = 2; }

	/* disable all GNSS meas*/
	SetMeasFlag(070, 0);
	/* if gnss in fix = 4,dop < 0.1,rms < 0.1, accp the gnss meas, Rset.dd[4] = -1.0;*/
	if (4 == fix_lever && pos_dop_GNSS < 1.0) {
		SetMeasFlag(077);
		if (accu_posGNSS.i < 0.1) { flag_rms_gnss &= ~0007; }
		if (accu_posGNSS.j < 0.1) { flag_rms_gnss &= ~0070; }
		if (accu_posGNSS.k < 0.1) { flag_rms_gnss &= ~0700; }
	}
	/* else if gnss in fix = 4, 0.1 =< dop < 0.3, that's mean there was abnormal pos, change the Rt*/
	else if (4 == fix_lever && pos_dop_GNSS < 3.0) {
		SetMeasFlag(077);
		/* check the lat*/
		if (!(flag_rms_gnss & 01) && accu_posGNSS.i < 0.01) { flag_rms_gnss &= ~0007; flag_rms_gnss |= 001; Rt.dd[3] = Rt.dd[3] * 2.0; }
		else if (accu_posGNSS.i >= 0.01 && accu_posGNSS.i < 0.1) {}
		else if (!(flag_rms_gnss & 02) && accu_posGNSS.i >= 0.1 && accu_posGNSS.i < 0.8) { flag_rms_gnss &= ~0007; flag_rms_gnss |= 002; Rt.dd[3] = Rt.dd[3] * 1.1; }
		else if (!(flag_rms_gnss & 04) && accu_posGNSS.i >= 0.8 && accu_posGNSS.i < 5.0) { flag_rms_gnss &= ~0007; flag_rms_gnss |= 004; Rt.dd[3] = Rt.dd[3] * 1.3; }
		else if (!(flag_rms_gnss & 01) && accu_posGNSS.i >= 5.0) { flag_rms_gnss &= ~0007; flag_rms_gnss |= 001;  Rt.dd[3] = Rt.dd[3] * 2.0; }

		/* check the lon*/
		if (!(flag_rms_gnss & 010) && accu_posGNSS.j < 0.01) { flag_rms_gnss &= ~0070; flag_rms_gnss |= 010; Rt.dd[4] = Rt.dd[4] * 2.0; }
		else if (accu_posGNSS.j >= 0.01 && accu_posGNSS.j < 0.1) {}
		else if (!(flag_rms_gnss & 020) && accu_posGNSS.j >= 0.1 && accu_posGNSS.j < 0.8) { flag_rms_gnss &= ~0070; flag_rms_gnss |= 020; Rt.dd[4] = Rt.dd[4] * 1.1; }
		else if (!(flag_rms_gnss & 040) && accu_posGNSS.i >= 0.8 && accu_posGNSS.j < 5.0) { flag_rms_gnss &= ~0070; flag_rms_gnss |= 040; Rt.dd[4] = Rt.dd[4] * 1.3; }
		else if (!(flag_rms_gnss & 010) && accu_posGNSS.j >= 5.0) { flag_rms_gnss &= ~0070; flag_rms_gnss |= 010; Rt.dd[4] = Rt.dd[4] * 2.0; }

		/* check the hgt*/
		if (!(flag_rms_gnss & 0100) && accu_posGNSS.k < 0.01) { flag_rms_gnss &= ~0700; flag_rms_gnss |= 0100; Rt.dd[5] = Rt.dd[5] * 2.0; }
		else if (accu_posGNSS.k >= 0.01 && accu_posGNSS.k < 0.1) {}
		else if (!(flag_rms_gnss & 0200) && accu_posGNSS.k >= 0.1 && accu_posGNSS.k < 0.8) { flag_rms_gnss &= ~0700; flag_rms_gnss |= 0200; Rt.dd[5] = Rt.dd[5] * 1.1; }
		else if (!(flag_rms_gnss & 0400) && accu_posGNSS.k >= 0.8 && accu_posGNSS.k < 5.0) { flag_rms_gnss &= ~0700; flag_rms_gnss |= 0400; Rt.dd[5] = Rt.dd[5] * 1.3; }
		else if (!(flag_rms_gnss & 0100) && accu_posGNSS.k >= 5.0) { flag_rms_gnss &= ~0700; flag_rms_gnss |= 0100; Rt.dd[5] = Rt.dd[5] * 2.0; }
	}
	else if (1 == fix_lever && pos_dop_GNSS < 340) {
		/* check the lat*/
		if (!(flag_rms_gnss & 01) && accu_posGNSS.i < 0.01) { flag_rms_gnss &= ~0007; }
		else if (accu_posGNSS.i >= 0.01 && accu_posGNSS.i < 5.0) { flag_rms_gnss &= ~0007; SetMeasFlag(012); }
		else if (!(flag_rms_gnss & 02) && accu_posGNSS.i >= 5.0 && accu_posGNSS.i < 15.0) { flag_rms_gnss &= ~0007; }
		else if (!(flag_rms_gnss & 04) && accu_posGNSS.i >= 15.0 && accu_posGNSS.i < 30.0) { flag_rms_gnss &= ~0007; }
		else if (!(flag_rms_gnss & 01) && accu_posGNSS.i >= 30.0) { flag_rms_gnss &= ~0007; }

		/* check the lon*/
		if (!(flag_rms_gnss & 010) && accu_posGNSS.j < 0.01) { flag_rms_gnss &= ~0070; }
		else if (accu_posGNSS.j >= 0.01 && accu_posGNSS.j < 5.0) { flag_rms_gnss &= ~0070; SetMeasFlag(021); }
		else if (!(flag_rms_gnss & 020) && accu_posGNSS.j >= 5.0 && accu_posGNSS.j < 15.0) { flag_rms_gnss &= ~0070; }
		else if (!(flag_rms_gnss & 040) && accu_posGNSS.j >= 15.0 && accu_posGNSS.j < 30.0) { flag_rms_gnss &= ~0070; }
		else if (!(flag_rms_gnss & 010) && accu_posGNSS.j >= 30) { flag_rms_gnss &= ~0070; }

		/* check the hgt*/
		if (!(flag_rms_gnss & 0100) && accu_posGNSS.k < 0.01) { flag_rms_gnss &= ~0007; }
		else if (accu_posGNSS.k >= 0.01 && accu_posGNSS.k < 10.0) { flag_rms_gnss &= ~0700; SetMeasFlag(044); }
		else if (!(flag_rms_gnss & 0200) && accu_posGNSS.k >= 10.0 && accu_posGNSS.k < 30.0) { flag_rms_gnss &= ~0700; flag_rms_gnss |= 0200; Rt.dd[5] = Rt.dd[5] * 3.0; SetMeasFlag(044); }
		else if (!(flag_rms_gnss & 0400) && accu_posGNSS.k >= 30.0 && accu_posGNSS.k < 100.0) { flag_rms_gnss &= ~0700; }
		else if (!(flag_rms_gnss & 0100) && accu_posGNSS.k >= 100.0) { flag_rms_gnss &= ~0700; }
	}
	else if (fix_lever == 2 && pos_dop_GNSS < 340) {
		/* check the lat*/
		if (accu_posGNSS.i < 0.01) {}
		else if (accu_posGNSS.i >= 0.01 && accu_posGNSS.i < 0.1) { SetMeasFlag(012); }
		else if (!(flag_rms_gnss & 002) && accu_posGNSS.i >= 0.1 && accu_posGNSS.i < 0.8) {}
		else if (!(flag_rms_gnss & 004) && accu_posGNSS.i >= 0.8 && accu_posGNSS.i < 10.0) {}
		else if (!(flag_rms_gnss & 001) && accu_posGNSS.i >= 10.0) {}

		/* check the lon*/
		if (accu_posGNSS.j < 0.01) {}
		else if (accu_posGNSS.j < 0.1 && accu_posGNSS.j >= 0.01) { SetMeasFlag(021); }
		else if (!(flag_rms_gnss & 0020) && accu_posGNSS.j < 0.8 && accu_posGNSS.j >= 0.1) {}
		else if (!(flag_rms_gnss & 0040) && accu_posGNSS.j < 10.0 && accu_posGNSS.j >= 0.8) {}
		else if (!(flag_rms_gnss & 0010) && accu_posGNSS.j >= 10.0) {}

		/* check the hgt*/
		if (accu_posGNSS.k < 0.01) {}
		else if (accu_posGNSS.k < 0.1 && accu_posGNSS.k >= 0.01) { SetMeasFlag(044); }
		else if (!(flag_rms_gnss & 0200) && accu_posGNSS.k < 0.8 && accu_posGNSS.k >= 0.1) {}
		else if (!(flag_rms_gnss & 0400) && accu_posGNSS.k < 10.0 && accu_posGNSS.k >= 0.8) {}
		else if (!(flag_rms_gnss & 0100) && accu_posGNSS.k >= 10.0) {}
	}

	return 0;
}
#endif 

#ifdef GNSS_CHECK_LG69T
/* LG69T feature*/
int KFAPP::check_GNSS_quality() {
	int res = 0;
	/* --------------------------------------------------------- */
	/* Classification of GNSS Solution Types:
		0: No Solution
		1: Single Solution
		2: Float Solution
		4: Fixed Solution
	*/
	int fix_lever = 0;
	if (1 == fixtype_gnss || 64 == fixtype_gnss) { SetGNSSFixMode(1); fix_lever = 4; }
	else if (5 == fixtype_gnss) { SetGNSSFixMode(2); fix_lever = 1; }
	else { SetGNSSFixMode(2); fix_lever = 2; }

	/* disable all GNSS meas*/
	SetMeasFlag(077, 0);
	/* if gnss in fix = 4,dop < 0.1,rms < 0.1, accp the gnss meas, Rset.dd[4] = -1.0;*/
	if (pos_dop_GNSS < 10.0 && pos_dop_GNSS > 0.01) {
		if (accu_posGNSS.i < 100) { flag_rms_gnss &= ~0007;	SetMeasFlag(012); }
		if (accu_posGNSS.j < 100) { flag_rms_gnss &= ~0070; SetMeasFlag(021); }
		if (accu_posGNSS.k < 100) { flag_rms_gnss &= ~0700; SetMeasFlag(044); }
	}

	if ((*gnssLost) > 10) {
		Rt.dd[0] = Rmin.dd[0] * 1.1;
		Rt.dd[1] = Rmin.dd[1] * 1.1;
		Rt.dd[2] = Rmin.dd[2] * 1.1;

		Rt.dd[3] = Rmin.dd[3] * 1.1;
		Rt.dd[4] = Rmin.dd[4] * 1.1;
		Rt.dd[5] = Rmin.dd[5] * 1.1;

		//*(vect3*)&Zk.dd[0] = O31;
		//*(vect3*)&Zk.dd[3] = O31;
	}
	return 0;
}
#endif // GNSS_CHECK_LG69T

#ifdef GNSS_CHECK_UBOX
/* LG69T feature*/
int KFAPP::check_GNSS_quality() {
	int res = 0;
	/* --------------------------------------------------------- */
	/* Classification of GNSS Solution Types:
		0: No Solution
		1: Single Solution
		2: Float Solution
		4: Fixed Solution
	*/
	int fix_lever = 0;
	if (1 == fixtype_gnss || 64 == fixtype_gnss) { SetGNSSFixMode(1); fix_lever = 4; }
	else if (5 == fixtype_gnss) { SetGNSSFixMode(2); fix_lever = 1; }
	else { SetGNSSFixMode(2); fix_lever = 2; }

	/* disable all GNSS meas*/
	SetMeasFlag(070, 0);
	/* if gnss in fix = 4,dop < 0.1,rms < 0.1, accp the gnss meas, Rset.dd[4] = -1.0;*/
	if (pos_dop_GNSS < 10.0 && pos_dop_GNSS > 0.01) {
		if (accu_posGNSS.i < 100) { flag_rms_gnss &= ~0007;	SetMeasFlag(012); }
		if (accu_posGNSS.j < 100) { flag_rms_gnss &= ~0070; SetMeasFlag(021); }
		if (accu_posGNSS.k < 100) { flag_rms_gnss &= ~0700; SetMeasFlag(044); }
	}

	if ((*gnssLost) > 10) {
		Rt.dd[0] = Rmin.dd[0] * 1.1;
		Rt.dd[1] = Rmin.dd[1] * 1.1;
		Rt.dd[2] = Rmin.dd[2] * 1.1;

		Rt.dd[3] = Rmin.dd[3] * 1.1;
		Rt.dd[4] = Rmin.dd[4] * 1.1;
		Rt.dd[5] = Rmin.dd[5] * 1.1;

		//*(vect3*)&Zk.dd[0] = O31;
		//*(vect3*)&Zk.dd[3] = O31;
	}
	return 0;
}
#endif // GNSS_CHECK_LG69T


void KFAPP::SetMeas(void) {

}

