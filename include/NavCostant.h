/**************************************************************
 * @file    NavCostant.h
 * @brief   PPOI_Nav Core Constants and Type Definitions Header
 *
 * This header file defines the core constants, macro definitions, structure types, and main classes (such as vect3)
 * for the PPOI_Nav navigation and integrated positioning system.
 * It includes Earth parameters, navigation system identifiers, observation and state-related constants,
 * filtering and error model parameters, satellite system macros, observation code types, positioning modes, solution statuses, etc.
 * These definitions provide unified foundational parameters and type support for modules such as navigation solution, integrated positioning, and data processing.
 *
 * @author   yejin
 * @date     2024-08-06
 * @version  1.0
 * @copyright Copyright (c) 2020-2025 PPOI_Nav Project
 *                          PSINS: A C++ Inertial Navigation Library, https://psins.org.cn/
 *
 * @mainpage PPOI_Nav Constants and Type Definitions
 * @section intro_sec Introduction
 * This file provides low-level constants, macro definitions, structures, and class types for the PPOI_Nav navigation and integrated positioning system.
 * It is suitable for navigation solution, integrated positioning, observation data processing, and related scenarios.
 *
 * @section macro_sec Macros and Parameter Overview
 * - Earth and Physical Constants: RE_WGS84, FE_WGS84, PI, CLIGHT, etc.
 * - Satellite Systems and Observation Codes: SYS_GPS, CODE_L1C, etc.
 * - State and Observation Related: VARMAX, MAXSAT, MAXOBS, etc.
 * - Positioning Modes and Solution Status: PMODE_SINGLE, SOLQ_FIX, etc.
 * - Structure and Class Definitions: vect3, gtime_t, FXPT, etc.
 *
 * @section history_sec Revision History
 * - 2024-08-06: Initial version, organized main constants, macros, and types.
 *
 * @section reference_sec References
 * - [1] PPOI_Nav Project Documentation and Design Specifications
 * - [2] PSINS: A C++ Inertial Navigation Library, https://psins.org.cn/
 * - [3] RTKLIB: An Open Source Program Package for GNSS Positioning, http://www.rtklib.com/
 *
 * @section usage_sec Usage
 * Include this header file directly to obtain the core constants, macro definitions, structures, and class types of the PPOI_Nav system.
 * It provides foundational parameter and type support for modules such as navigation solution and integrated positioning.
 *
 * @defgroup nav_const Core Constants and Macros
 * @defgroup nav_struct Structure and Class Types
 **************************************************************/

#ifdef NavCostant
#else
#define NavCostant

// #include "../stdint.h"
#include <stdint.h>

#define  CONFIGSIM

//#define DSPRELTIME
#ifndef  DSPRELTIME
	#define  TRACE
	#define  FILEIO
#endif


class vect3;                  /*< the define of vector 3x1*/


#define VARMAX      5        /**< Maximum number of data points. */

#define MAX_MAT_DIM     19                        /*the max dim of vect*/
#define MAX_MAT_DIM_2   MAX_MAT_DIM*MAX_MAT_DIM   /*the max dim of matrix*/

#define ALIGN_ANT  1    /* using the ant for coast align*/
#define ALIGN_TRJ  2    /* using the trj for coast align*/

#define MAXLEAPS    64                  /* max number of leap seconds table */
#define RE_WGS84    6378137.0           /* earth semimajor axis (WGS84) (m) */
#define FE_WGS84    (1.0/298.257223563) /* earth flattening (WGS84) */

#define PI      3.1415926535897932      /* pi */
#define PI_2    (PI/2.0)                /**< PI divided by 2 */
#define PI_4    (PI/4.0)                /**< PI divided by 4 */
#define _2PI    (2.0*PI)                /**< Twice the value of PI */
#define DEG     (PI/180.0)              /**< Conversion factor from degrees to radians */
#define MIN     (DEG/60.0)              /**< Conversion factor from minutes to radians */
#define SEC     (MIN/60.0)              /**< Conversion factor from seconds to radians */
#define HUR     3600.0                  /**< Number of seconds in an hour (hur) */
#define SHUR    60.0                    /**< Square root of the number of seconds in an hour */
#define DPS     (DEG/1.0)               /**< Conversion factor from degrees per second to radians per second */
#define DPH     (DEG/HUR)               /**< Conversion factor from degrees per hour to radians per second */
#define DPSH    (DEG/SHUR)              /**< Conversion factor from degrees per square root of hour to radians per second */
#define G0      9.7803267714            /**< Standard acceleration due to gravity (G0) */
#define MG      (G0/1.0e3)              /**< Conversion factor from meters per second squared to milli-g */
#define UG      (G0/1.0e6)              /**< Conversion factor from meters per second squared to micro-g */
#define UGPSHZ  (UG/1)                  /**< Conversion factor from micro-g to micro-g per square root of hertz */
#define RE      6378137.0               /**< Earth's equatorial radius (RE) */
#define PPM     1.0e-6                  /**< Parts per million (PPM) */


#define f0_earth (1.0/298.257)           /**< Flattening factor (f) */
#define wie0     7.2921151467e-5

#define EPS		2.220446049e-16     /* the min value in this space*/
#define INF		3.402823466e+30     /* INF */
#define INFp5	INF*0.5             /* INF*0.5 */

#define assert(b)  {};                            /*safe action*/
#define swapt(a,b,tmp) {tmp c = a; a = b;b = c;}  /*swapt the a and b*/
#define asinEx(x)		asin(range(x, -1.0, 1.0)) /*get value the sin of x*/
#define acosEx(x)		acos(range(x, -1.0, 1.0)) /*get value the cos of x*/
#define min(x,y)        ( (x)<=(y)?(x):(y) )

#define velMax  400.0
#define hgtMin  -RE * 0.01
#define hgtMax  RE * 0.01
#define latMax  85.0 * DEG

#define fXYZU(X,Y,Z,U)	1.0*(X)*(U),1.0*(Y)*(U),1.0*(Z)*(U)
#define fXXZU(X,Z,U)	fXYZU(X,X,Z,U)
#define fXYZ(X,Y,Z)		fXYZU(X,Y,Z,1.0)
#define fXXZ(X,Z)		fXYZ(X,X,Z)
#define fXXX(X)			fXYZ(X,X,X)
#define fdLLH(LL,H)		fXXZ((LL)/RE,(H))
#define fdPOS(LLH)		fdLLH(LLH,LLH)

#define CC180C360(yaw)  ( (yaw)>0.0 ? (_2PI-(yaw)) : -(yaw) )   // counter-clockwise +-180deg -> clockwise 0~360deg for yaw
#define C360CC180(yaw)  ( (yaw)>=PI ? (_2PI-(yaw)) : -(yaw) )   // clockwise 0~360deg -> counter-clockwise +-180deg for yaw
#define pow2(x)			((x)*(x))
#define Ant_Mode_One       1
#define Ant_Mode_LR_L      2
#define Ant_Mode_LR_R      3
#define Ant_Mode_FB_F      4
#define Ant_Mode_FB_B      5
#define Ant_Mode_Ang       6

#define THR_NHC_w   0.8*DPS
#define THR_ZIHR_w  1.0*DPS
#define THR_ZIHR_t  1.0


class vect3
{
public:
	double i, j, k;

	/* default constructor ---------------------------------------------------------
    * initialize vect3 with all components set to zero
    * args   : none
    * return : none
    *-----------------------------------------------------------------------------*/
    vect3(void);

    /* parameterized constructor --------------------------------------------------
    * initialize vect3 with all components set to the same value
    * args   : double     xyz      I   value for all components
    * return : none
    *-----------------------------------------------------------------------------*/
    vect3(double xyz);

    /* parameterized constructor --------------------------------------------------
    * initialize vect3 with specified components
    * args   : double     xx       I   x component
    *          double     yy       I   y component
    *          double     zz       I   z component
    * return : none
    *-----------------------------------------------------------------------------*/
    vect3(double xx, double yy, double zz);

    /* array constructor (double) -------------------------------------------------
    * initialize vect3 from a double array
    * args   : const double *pdata I   pointer to array of 3 doubles
    * return : none
    *-----------------------------------------------------------------------------*/
    vect3(const double *pdata);

    /* array constructor (float) --------------------------------------------------
    * initialize vect3 from a float array
    * args   : const float *pdata  I   pointer to array of 3 floats
    * return : none
    *-----------------------------------------------------------------------------*/
    vect3(const float *pdata);

    /* assignment operator (double) -----------------------------------------------
    * assign all components to the same double value
    * args   : double     f        I   value to assign
    * return : reference to this vect3
    *-----------------------------------------------------------------------------*/
    vect3& operator=(double f);

    /* assignment operator (array) ------------------------------------------------
    * assign components from a double array
    * args   : const double *pf    I   pointer to array of 3 doubles
    * return : reference to this vect3
    *-----------------------------------------------------------------------------*/
    vect3& operator=(const double *pf);

    /* check if all elements are zeros --------------------------------------------
    * check if all components of vect3 are zero within a threshold
    * args   : const vect3 &v      I   vect3 to check
    *          double     eps      I   threshold
    * return : 1 if all elements are zero, 0 otherwise
    *-----------------------------------------------------------------------------*/
    int IsZeros(const vect3 &v, double eps);

    /* check if x and y elements are zeros ----------------------------------------
    * check if x and y components of vect3 are zero within a threshold
    * args   : const vect3 &v      I   vect3 to check
    *          double     eps      I   threshold
    * return : true if x and y are zero, false otherwise
    *-----------------------------------------------------------------------------*/
    bool IsZeroXY(const vect3 &v, double eps);

    /* check if any element is NaN ------------------------------------------------
    * check if any component of vect3 is NaN
    * args   : const vect3 &v      I   vect3 to check
    * return : true if any element is NaN, false otherwise
    *-----------------------------------------------------------------------------*/
    bool IsNaN(const vect3 &v);

    /* vector addition ------------------------------------------------------------
    * add two vect3 objects
    * args   : const vect3 &v      I   vect3 to add
    * return : result of addition (vect3)
    *-----------------------------------------------------------------------------*/
    vect3 operator+(const vect3 &v) const;

    /* vector subtraction ---------------------------------------------------------
    * subtract two vect3 objects
    * args   : const vect3 &v      I   vect3 to subtract
    * return : result of subtraction (vect3)
    *-----------------------------------------------------------------------------*/
    vect3 operator-(const vect3 &v) const;

    /* vector cross multiplication ------------------------------------------------
    * cross product of two vect3 objects
    * args   : const vect3 &v      I   vect3 to cross
    * return : result of cross product (vect3)
    *-----------------------------------------------------------------------------*/
    vect3 operator*(const vect3 &v) const;

    /* row-vector multiply matrix -------------------------------------------------
    * multiply vect3 (as row vector) by matrix
    * args   : const mat &m        I   matrix to multiply
    * return : result of multiplication (vect3)
    *-----------------------------------------------------------------------------*/
    vect3 operator*(const mat &m) const;

    /* vector multiply scale ------------------------------------------------------
    * multiply vect3 by a scalar
    * args   : double     f        I   scalar value
    * return : result of multiplication (vect3)
    *-----------------------------------------------------------------------------*/
    vect3 operator*(double f) const;

    /* vector multiply mat3 -------------------------------------------------------
    * multiply vect3 by a 3x3 matrix
    * args   : const mat3 &m       I   3x3 matrix to multiply
    * return : result of multiplication (vect3)
    *-----------------------------------------------------------------------------*/
    vect3 operator*(const mat3 &m) const;

    /* vector divide scale --------------------------------------------------------
    * divide vect3 by a scalar
    * args   : double     f        I   scalar value
    * return : result of division (vect3)
    *-----------------------------------------------------------------------------*/
    vect3 operator/(double f) const;

    /* vector divide element by element -------------------------------------------
    * element-wise division of two vect3 objects
    * args   : const vect3 &v      I   vect3 divisor
    * return : result of division (vect3)
    *-----------------------------------------------------------------------------*/
    vect3 operator/(const vect3 &v) const;

    /* vector addition assignment -------------------------------------------------
    * add vect3 to this vect3 (in place)
    * args   : const vect3 &v      I   vect3 to add
    * return : reference to this vect3
    *-----------------------------------------------------------------------------*/
    vect3& operator+=(const vect3 &v);

    /* vector subtraction assignment ----------------------------------------------
    * subtract vect3 from this vect3 (in place)
    * args   : const vect3 &v      I   vect3 to subtract
    * return : reference to this vect3
    *-----------------------------------------------------------------------------*/
    vect3& operator-=(const vect3 &v);

    /* vector multiply scale assignment -------------------------------------------
    * multiply this vect3 by a scalar (in place)
    * args   : double     f        I   scalar value
    * return : reference to this vect3
    *-----------------------------------------------------------------------------*/
    vect3& operator*=(double f);

    /* vector divide scale assignment ---------------------------------------------
    * divide this vect3 by a scalar (in place)
    * args   : double     f        I   scalar value
    * return : reference to this vect3
    *-----------------------------------------------------------------------------*/
    vect3& operator/=(double f);

    /* vector divide element by element assignment --------------------------------
    * element-wise division of this vect3 by another (in place)
    * args   : const vect3 &v      I   vect3 divisor
    * return : reference to this vect3
    *-----------------------------------------------------------------------------*/
    vect3& operator/=(const vect3 &v);

    /* scale multiply vector (friend) ---------------------------------------------
    * multiply scalar by vect3
    * args   : double     f        I   scalar value
    *          const vect3 &v      I   vect3 to multiply
    * return : result of multiplication (vect3)
    *-----------------------------------------------------------------------------*/
    friend vect3 operator*(double f, const vect3 &v);

    /* minus (friend) -------------------------------------------------------------
    * negate vect3 (unary minus)
    * args   : const vect3 &v      I   vect3 to negate
    * return : result of negation (vect3)
    *-----------------------------------------------------------------------------*/
    friend vect3 operator-(const vect3 &v);
};

typedef struct {        
    long time;         
    float sec;         
} gtime_t;

typedef struct {
	double  lon;
	double  lat;
	float   height_ellipsoid;

	uint16_t  num_sv;
	uint16_t  fix_type;
	float     pos_dop;

	uint16_t  week;
	double    secs;

	float  vel_ground;
	float  heading_motion;
	float  vel_north;
	float  vel_east;
	float  vel_down;

	float accu_lon;
	float accu_lat;
	float accu_hegit;

	float  accu_heading;
	float  heading;      
	float  baseline;
	float  age;
}gnss_info;

typedef struct {
	uint16_t week;
	double secs;

	float acc_x;
	float acc_y;
	float acc_z;

	float gyr_x;
	float gyr_y;
	float gyr_z;

	uint16_t cnt;
}imu_info;

typedef struct {
	uint16_t week;
	double secs;

	float right_front;
	float right_back;
	float left_front;
	float left_back;

	float mean;
}odo_info;


#endif // NavCostant