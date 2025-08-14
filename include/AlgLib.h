/**
 * @file AlgLib.h
 * @brief Alignment and Navigation Library - Core Data Structures and Algorithms
 * @author LEADOR PPOI Team - Yejin 
 * @date 2025
 * @version 1.0
 * @copyright Copyright (c) 2020-2025 PPOI_Nav Project
 *                          yejin  yejinbenjohn@qq.com
 *                          liuhuichao 
 *
 * @mainpage PPOI_Nav Alignment and Navigation Library
 * @section intro_sec Introduction
 * This header defines the core mathematical and navigation-related classes and functions
 * used in the PPOI_Nav navigation and sensor fusion system. It includes matrix, vector,
 * quaternion, filtering, statistics, and navigation state classes, as well as utility
 * functions for coordinate transformations and sensor modeling.
 *
 * @section class_sec Classes Overview
 * - mat3: 3x3 matrix class, supporting rotation and linear algebra operations.
 * - quat: Quaternion class, supporting rotation representation and conversion with Euler angles/rotation vectors.
 * - vect/vect3: Vector classes, supporting general vector operations.
 * - mat: General matrix class, supporting matrix algebra and submatrix operations.
 * - earth: Earth model and navigation-related parameters.
 * - maxmin/maxminn: Single-variable/multi-variable statistical max/min and mean tracking.
 * - STA_VAR/STA_VARS: Mean and variance statistics, supporting Welford's algorithm.
 * - STA_AVPI: Circular buffer and interpolation for attitude/velocity/position.
 * - IMU: Inertial sensor modeling and data structure.
 * - INS: Inertial navigation system state and update logic.
 * - IIR/IIRV3: 1D and 3D IIR filters.
 * - odo: Odometer data and processing.
 * - ZIHR: Zero-Integrated Heading Rate detection.
 * - KalmanFilter: General Kalman filter base class.
 * - EKFTDKF: Extended/Distributed Kalman filter.
 * - SINSGNSSLOOSE/SINSGNSSBorad/SINSGNSSPOST: Loosely coupled SINS/GNSS integration, with odometer and post-processing support.
 * - RingMemory: Fixed-length circular buffer.
 * - Smooth: Sliding window mean filter.
 *
 * @section history_sec History
 * - 2020-2021: Initial version by PPOI_Nav Team
 * - 2022-2023: Major refactoring and documentation improvements
 * - 2024-2025: Feature extensions, bug fixes, and performance optimizations
 *
 * @section reference_sec References
 * - [0] PSINS: A C++ Inertial Navigation Library, https://psins.org.cn/
 * - [1] Farrell, J. Aided Navigation: GPS with High Rate Sensors, 2008.
 * - [2] Titterton, D. H., & Weston, J. L. Strapdown Inertial Navigation Technology, 2004.
 * - [3] RTKLIB: An Open Source Program Package for GNSS Positioning, http://www.rtklib.com/
 * - [4] Groves, P. D. Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems, 2013.
 *
 * @section usage_sec Usage
 * Include this header in your navigation or sensor fusion project to access the
 * mathematical and navigation primitives, as well as advanced filtering and alignment
 * algorithms. The library is suitable for embedded, real-time, and post-processing applications.
 *
 * @defgroup math_core Mathematical Foundations
 * @defgroup nav_core Navigation Core
 * @defgroup filter_core Filtering and Statistics
 * @defgroup util_core Utilities and Helpers
 * 
 * @date 20250607 Fixed documentation and improved English translation.
 */

#ifdef AlignLib_Header
#else
#define AlignLib_Header

#include <stdlib.h>
#include "NavCostant.h"
#include "AppConfigPara.h"

extern vect3 O31;
extern vect3 One31;
extern mat3 I33;
extern quat qI;

//midmsg_t midmsg_[MAXDIFOBS * 2];

/**
 * @class mat3
 * @brief 3x3 Matrix class for rotation and linear algebra operations.
 * @author PPOI_Nav Team
 * @date 2020-2025
 *
 * Provides constructors for various initialization methods, and overloaded operators
 * for matrix arithmetic, scaling, and vector multiplication. Includes utility methods
 * for row/column access, rotation, and rearrangement.
 */
class mat3
{
public:
    double e00, e01, e02, e10, e11, e12, e20, e21, e22;

    /* default constructor --------------------------------------------------------
    * initialize all elements of the matrix to zero
    * args   : none
    * return : none
    *-----------------------------------------------------------------------------*/
    mat3(void);

    /* diagonal initialization constructor ----------------------------------------
    * initialize the matrix as a diagonal matrix with the given value
    * args   : double     xyz      I   value for diagonal elements
    * return : none
    *-----------------------------------------------------------------------------*/
    mat3(double xyz);

    /* array initialization constructor (double) ----------------------------------
    * initialize the matrix from a double array (row-major order)
    * args   : const double* pxyz  I   pointer to 9 double values
    * return : none
    *-----------------------------------------------------------------------------*/
    mat3(const double* pxyz);

    /* array initialization constructor (float) -----------------------------------
    * initialize the matrix from a float array (row-major order)
    * args   : const float* pxyz   I   pointer to 9 float values
    * return : none
    *-----------------------------------------------------------------------------*/
    mat3(const float* pxyz);

    /* diagonal elements constructor ----------------------------------------------
    * initialize the matrix as a diagonal matrix with specified diagonal values
    * args   : double     xx       I   value for (0,0)
    *          double     yy       I   value for (1,1)
    *          double     zz       I   value for (2,2)
    * return : none
    *-----------------------------------------------------------------------------*/
    mat3(double xx, double yy, double zz);

    /* full elements constructor --------------------------------------------------
    * initialize the matrix with all 9 elements specified (row-major order)
    * args   : double     xx,xy,xz I   first row elements
    *          double     yx,yy,yz I   second row elements
    *          double     zx,zy,zz I   third row elements
    * return : none
    *-----------------------------------------------------------------------------*/
    mat3(double xx, double xy, double xz,
        double yx, double yy, double yz,
        double zx, double zy, double zz);

    /* vector rows/columns constructor --------------------------------------------
    * initialize the matrix from three vectors as rows or columns
    * args   : vect3      v0       I   first vector
    *          vect3      v1       I   second vector
    *          vect3      v2       I   third vector
    *          bool       isrow    I   true: use as rows; false: use as columns (default: true)
    * return : none
    *-----------------------------------------------------------------------------*/
    mat3(const vect3& v0, const vect3& v1,
        const vect3& v2, bool isrow = 1);               /*< M = [v0; v1; v2] */

    /* matrix addition ------------------------------------------------------------
    * add two matrices element-wise
    * args   : const mat3& m       I   matrix to add
    * return : result of addition
    *-----------------------------------------------------------------------------*/
    mat3 operator+(const mat3& m) const;

    /* matrix subtraction ---------------------------------------------------------
    * subtract two matrices element-wise
    * args   : const mat3& m       I   matrix to subtract
    * return : result of subtraction
    *-----------------------------------------------------------------------------*/
    mat3 operator-(const mat3& m) const;

    /* matrix multiplication ------------------------------------------------------
    * multiply two matrices
    * args   : const mat3& m       I   matrix to multiply
    * return : result of multiplication
    *-----------------------------------------------------------------------------*/
    mat3 operator*(const mat3& m) const;

    /* matrix scaling -------------------------------------------------------------
    * multiply all elements by a scalar
    * args   : double     f        I   scale factor
    * return : scaled matrix
    *-----------------------------------------------------------------------------*/
    mat3 operator*(double f) const;

    /* matrix-vector multiplication -----------------------------------------------
    * multiply matrix by a vector
    * args   : const vect3& v      I   vector to multiply
    * return : resulting vector
    *-----------------------------------------------------------------------------*/
    vect3 operator*(const vect3& v) const;

    /* matrix addition assignment -------------------------------------------------
    * add another matrix to this matrix (element-wise)
    * args   : const mat3& m       I   matrix to add
    * return : reference to this matrix
    *-----------------------------------------------------------------------------*/
    mat3& operator+=(const mat3& m);

    /* matrix + diag(vector) ------------------------------------------------------
    * add a vector to the diagonal elements of the matrix
    * args   : const vect3& v      I   vector to add to diagonal
    * return : resulting matrix
    *-----------------------------------------------------------------------------*/
    mat3 operator+(const vect3& v) const;

    /* matrix + diag(vector) assignment -------------------------------------------
    * add a vector to the diagonal elements of this matrix
    * args   : const vect3& v      I   vector to add to diagonal
    * return : reference to this matrix
    *-----------------------------------------------------------------------------*/
    mat3& operator+=(const vect3& v);

    /* set row from vector --------------------------------------------------------
    * set the i-th row of the matrix from a vector
    * args   : int        i        I   row index (0~2)
    *          vect3      v        I   vector to set
    * return : none
    *-----------------------------------------------------------------------------*/
    void SetRow(int i, const vect3& v);

    /* set column from vector -----------------------------------------------------
    * set the i-th column of the matrix from a vector
    * args   : int        i        I   column index (0~2)
    *          vect3      v        I   vector to set
    * return : none
    *-----------------------------------------------------------------------------*/
    void SetClm(int i, const vect3& v);

    /* get row as vector ----------------------------------------------------------
    * get the i-th row of the matrix as a vector
    * args   : int        i        I   row index (0~2)
    * return : row vector
    *-----------------------------------------------------------------------------*/
    vect3 GetRow(int i) const;

    /* get column as vector -------------------------------------------------------
    * get the i-th column of the matrix as a vector
    * args   : int        i        I   column index (0~2)
    * return : column vector
    *-----------------------------------------------------------------------------*/
    vect3 GetClm(int i) const;

    /* rotation by axis -----------------------------------------------------------
    * generate a rotation matrix for rotation about x/y/z axis by angle
    * args   : double     angle    I   rotation angle (radians)
    *          char       axis     I   axis ('x','y','z')
    * return : rotation matrix
    *-----------------------------------------------------------------------------*/
    mat3 Rot(double angle, char axis);

    /* re-arrange row/column by index ---------------------------------------------
    * re-arrange the matrix rows/columns according to ijk index
    * args   : const mat3& m      I   input matrix
    *          int        ijk     I   index for re-arrangement
    * return : rearranged matrix
    *-----------------------------------------------------------------------------*/
    mat3 rcijk(const mat3& m, int ijk);

    /* unary minus (negation) -----------------------------------------------------
    * negate all elements of the matrix
    * args   : const mat3& m      I   matrix to negate
    * return : negated matrix
    *-----------------------------------------------------------------------------*/
    friend mat3 operator-(const mat3& m);

    /* matrix transposition -------------------------------------------------------
    * transpose the matrix
    * args   : const mat3& m      I   matrix to transpose
    * return : transposed matrix
    *-----------------------------------------------------------------------------*/
    friend mat3 operator~(const mat3& m);

    /* scalar multiplication (from left) ------------------------------------------
    * multiply matrix by a scalar (scalar * matrix)
    * args   : double     f        I   scale factor
    *          const mat3& m      I   matrix to scale
    * return : scaled matrix
    *-----------------------------------------------------------------------------*/
    friend mat3 operator*(double f, const mat3& m);
};

/**
 * @class quat
 * @brief Quaternion class for representing rotations.
 * @author PPOI_Nav Team
 * @date 2020-2025
 *
 * Supports quaternion arithmetic, conversion to/from Euler angles and rotation vectors,
 * and utility functions for normalization and conjugation.
 */
class quat
{
public:
    double q0, q1, q2, q3;

    /* default constructor --------------------------------------------------------
    * initialize quaternion to (1, 0, 0, 0)
    * args   : none
    * return : none
    *-----------------------------------------------------------------------------*/
    quat(void);

    /* parameterized constructor --------------------------------------------------
    * initialize quaternion with specified components
    * args   : double     qq0      I   scalar part
    *          double     qq1      I   first vector part (default: 0.0)
    *          double     qq2      I   second vector part (default: 0.0)
    *          double     qq3      I   third vector part (default: 0.0)
    * return : none
    *-----------------------------------------------------------------------------*/
    quat(double qq0, double qq1 = 0.0, double qq2 = 0.0, double qq3 = 0.0);

    /* quaternion from scalar and vector ------------------------------------------
    * initialize quaternion from scalar and 3D vector
    * args   : double     qq0      I   scalar part
    *          vect3      qqv      I   vector part
    * return : none
    *-----------------------------------------------------------------------------*/
    quat(double qq0, const vect3& qqv);

    /* quaternion from array ------------------------------------------------------
    * initialize quaternion from array of 4 doubles
    * args   : const double* pdata I   pointer to 4 double values
    * return : none
    *-----------------------------------------------------------------------------*/
    quat(const double* pdata);

    /* add misalignment vector ----------------------------------------------------
    * add a misalignment vector to the quaternion (true quaternion add misalign angles)
    * args   : const vect3& phi    I   misalignment vector
    * return : resulting quaternion
    *-----------------------------------------------------------------------------*/
    quat operator+(const vect3& phi) const;

    /* subtract misalignment vector -----------------------------------------------
    * subtract a misalignment vector from the quaternion (calculated quaternion delete misalign angles)
    * args   : const vect3& phi    I   misalignment vector
    * return : resulting quaternion
    *-----------------------------------------------------------------------------*/
    quat operator-(const vect3& phi) const;

    /* get misalignment angles ----------------------------------------------------
    * get misalignment angles from calculated quaternion and true quaternion
    * args   : const quat& quat    I   true quaternion
    * return : misalignment vector
    *-----------------------------------------------------------------------------*/
    vect3 operator-(const quat& quat) const;

    /* quaternion multiplication --------------------------------------------------
    * multiply two quaternions
    * args   : const quat& q       I   quaternion to multiply
    * return : resulting quaternion
    *-----------------------------------------------------------------------------*/
    quat operator*(const quat& q) const;

    /* quaternion-vector multiplication -------------------------------------------
    * rotate a vector by the quaternion
    * args   : const vect3& v      I   vector to rotate
    * return : rotated vector
    *-----------------------------------------------------------------------------*/
    vect3 operator*(const vect3& v) const;

    /* quaternion multiplication assignment ---------------------------------------
    * multiply this quaternion by another quaternion
    * args   : const quat& q       I   quaternion to multiply
    * return : reference to this quaternion
    *-----------------------------------------------------------------------------*/
    quat& operator*=(const quat& q);

    /* subtract misalignment vector assignment ------------------------------------
    * subtract a misalignment vector from this quaternion
    * args   : const vect3& phi    I   misalignment vector
    * return : reference to this quaternion
    *-----------------------------------------------------------------------------*/
    quat& operator-=(const vect3& phi);

    /* set yaw angle --------------------------------------------------------------
    * set the yaw (heading) angle of the quaternion, keeping roll and pitch unchanged
    * args   : double     yaw      I   yaw angle (radians, default: 0.0)
    * return : none
    *-----------------------------------------------------------------------------*/
    void SetYaw(double yaw = 0.0);

    /* normalize quaternion -------------------------------------------------------
    * normalize the quaternion to unit length
    * args   : quat*      q        O   pointer to output normalized quaternion (optional)
    * return : none
    *-----------------------------------------------------------------------------*/
    void normlize(quat* q);

    /* quaternion conjugate -------------------------------------------------------
    * compute the conjugate of the quaternion
    * args   : const quat& q       I   input quaternion
    * return : conjugated quaternion
    *-----------------------------------------------------------------------------*/
    friend quat operator~(const quat& q);
};

/**
 * @class vect
 * @brief General vector class for mathematical operations.
 * @author PPOI_Nav Team
 * @date 2020-2025
 *
 * Supports vector arithmetic, assignment, and conversion to/from other types.
 */
class vect
{
public:
    int row, clm, rc;
    double dd[MAX_MAT_DIM];

    /* default constructor --------------------------------------------------------
    * initialize vector with zero elements and size 0x0
    * args   : none
    * return : none
    *-----------------------------------------------------------------------------*/
    vect(void);

    /* size constructor -----------------------------------------------------------
    * initialize vector with given row and column size, all elements set to zero
    * args   : int        row0     I   number of rows
    *          int        clm0     I   number of columns (default: 1)
    * return : none
    *-----------------------------------------------------------------------------*/
    vect(int row0, int clm0 = 1);

    /* fill constructor -----------------------------------------------------------
    * initialize vector with given size, all elements set to the same value
    * args   : int        row0     I   number of rows
    *          double     f        I   value to fill
    * return : none
    *-----------------------------------------------------------------------------*/
    vect(int row0, double f);

    /* variadic fill constructor --------------------------------------------------
    * initialize vector with given size and variadic list of values
    * args   : int        row0     I   number of rows
    *          double     f        I   first value
    *          double     f1,...   I   additional values
    * return : none
    *-----------------------------------------------------------------------------*/
    vect(int row0, double f, double f1, ...);

    /* array constructor ----------------------------------------------------------
    * initialize vector with given size and values from array
    * args   : int        row0     I   number of rows
    *          const double* pf    I   pointer to array of values
    * return : none
    *-----------------------------------------------------------------------------*/
    vect(int row0, const double* pf);

    /* vect3 constructor ----------------------------------------------------------
    * initialize vector from a vect3 object (3 elements)
    * args   : const vect3& v      I   vect3 object
    * return : none
    *-----------------------------------------------------------------------------*/
    vect(const vect3& v);

    /* two vect3 constructor ------------------------------------------------------
    * initialize vector from two vect3 objects (6 elements)
    * args   : const vect3& v1     I   first vect3 object
    *          const vect3  v2     I   second vect3 object
    * return : none
    *-----------------------------------------------------------------------------*/
    vect(const vect3& v1, const vect3 v2);

    /* set elements from array ----------------------------------------------------
    * set vector elements from array
    * args   : double*    f        I   pointer to array
    *          int        size     I   number of elements to set
    * return : none
    *-----------------------------------------------------------------------------*/
    void Set(double f[], int size);

    /* set elements from array (alternative) --------------------------------------
    * set vector elements from array (alternative method)
    * args   : double*    f        I   pointer to array
    *          int        size     I   number of elements to set
    * return : none
    *-----------------------------------------------------------------------------*/
    void Set2(double f[], int size);

    /* set vect3 at position ------------------------------------------------------
    * set a vect3 at specified position in the vector
    * args   : int        i        I   start index
    *          const vect3& v      I   vect3 to set
    * return : none
    *-----------------------------------------------------------------------------*/
    void SetVect3(int i, const vect3& v);

    /* set vect3 at position (pow2) -----------------------------------------------
    * set a vect3 at specified position in the vector (pow2 version)
    * args   : int        i        I   start index
    *          const vect3& v      I   vect3 to set
    * return : none
    *-----------------------------------------------------------------------------*/
    void Set2Vect3(int i, const vect3& v);

    /* set element(s) by bit mask -------------------------------------------------
    * set element(s) to value by bit mask
    * args   : unsigned int bit    I   bit mask
    *          double     f        I   value to set
    * return : none
    *-----------------------------------------------------------------------------*/
    void SetBit(unsigned int bit, double f);

    /* set vect3 by bit mask ------------------------------------------------------
    * set vect3 to elements by bit mask
    * args   : unsigned int bit    I   bit mask
    *          const vect3& v      I   vect3 to set
    * return : none
    *-----------------------------------------------------------------------------*/
    void SetBit(unsigned int bit, const vect3& v);

    /* get vect3 from position ----------------------------------------------------
    * get a vect3 from specified position in the vector
    * args   : int        i        I   start index
    * return : vect3 at position i
    *-----------------------------------------------------------------------------*/
    vect3 GetVect3(int i) const;

    /* vector addition ------------------------------------------------------------
    * add two vectors element-wise
    * args   : const vect& v       I   vector to add
    * return : result of addition
    *-----------------------------------------------------------------------------*/
    vect operator+(const vect& v) const;

    /* vector subtraction ---------------------------------------------------------
    * subtract two vectors element-wise
    * args   : const vect& v       I   vector to subtract
    * return : result of subtraction
    *-----------------------------------------------------------------------------*/
    vect operator-(const vect& v) const;

    /* vector scaling -------------------------------------------------------------
    * multiply all elements by a scalar
    * args   : double     f        I   scale factor
    * return : scaled vector
    *-----------------------------------------------------------------------------*/
    vect operator*(double f) const;

    /* assign all elements to a value ---------------------------------------------
    * set all elements to the same value
    * args   : double     f        I   value to assign
    * return : reference to this vector
    *-----------------------------------------------------------------------------*/
    vect& operator=(double f);

    /* assign from array ----------------------------------------------------------
    * assign vector elements from array
    * args   : const double* pf    I   pointer to array
    * return : reference to this vector
    *-----------------------------------------------------------------------------*/
    vect& operator=(const double* pf);

    /* assign from mat3 -----------------------------------------------------------
    * assign vector elements from a 3x3 matrix (flattened)
    * args   : const mat3& m       I   matrix to assign from
    * return : reference to this vector
    *-----------------------------------------------------------------------------*/
    vect& operator=(const mat3& m);

    /* vector addition assignment -------------------------------------------------
    * add another vector to this vector (element-wise)
    * args   : const vect& v       I   vector to add
    * return : reference to this vector
    *-----------------------------------------------------------------------------*/
    vect& operator+=(const vect& v);

    /* vector subtraction assignment ----------------------------------------------
    * subtract another vector from this vector (element-wise)
    * args   : const vect& v       I   vector to subtract
    * return : reference to this vector
    *-----------------------------------------------------------------------------*/
    vect& operator-=(const vect& v);

    /* vector scaling assignment --------------------------------------------------
    * scale this vector by a scalar
    * args   : double     f        I   scale factor
    * return : reference to this vector
    *-----------------------------------------------------------------------------*/
    vect& operator*=(double f);

    /* vector-matrix multiplication -----------------------------------------------
    * multiply this vector by another vector (outer product)
    * args   : const vect& v       I   vector to multiply
    * return : resulting matrix
    *-----------------------------------------------------------------------------*/
    mat operator*(const vect& v) const;

    /* element access -------------------------------------------------------------
    * access vector element by index
    * args   : int        r        I   element index
    * return : reference to element
    *-----------------------------------------------------------------------------*/
    double& operator()(int r);

    /* vector transposition -------------------------------------------------------
    * transpose the vector (row <-> column)
    * args   : const vect& v       I   vector to transpose
    * return : transposed vector
    *-----------------------------------------------------------------------------*/
    friend vect operator~(const vect& v);
};

/**
 * @class mat
 * @brief General matrix class for mathematical operations.
 * @author PPOI_Nav Team
 * @date 2020-2025
 *
 * Supports matrix arithmetic, assignment, and utility functions for diagonal and submatrix operations.
 */
class mat
{
public:
    int row, clm, rc;
    double dd[MAX_MAT_DIM_2];

    /* default constructor --------------------------------------------------------
    * initialize matrix with zero elements and size 0x0
    * args   : none
    * return : none
    *-----------------------------------------------------------------------------*/
    mat(void);

    /* size constructor -----------------------------------------------------------
    * initialize matrix with given row and column size, all elements set to zero
    * args   : int        row0     I   number of rows
    *          int        clm0     I   number of columns
    * return : none
    *-----------------------------------------------------------------------------*/
    mat(int row0, int clm0);

    /* fill constructor -----------------------------------------------------------
    * initialize matrix with given size, all elements set to the same value
    * args   : int        row0     I   number of rows
    *          int        clm0     I   number of columns
    *          double     f        I   value to fill
    * return : none
    *-----------------------------------------------------------------------------*/
    mat(int row0, int clm0, double f);

    /* variadic fill constructor --------------------------------------------------
    * initialize matrix with given size and variadic list of values (row-major order)
    * args   : int        row0     I   number of rows
    *          int        clm0     I   number of columns
    *          double     f        I   first value
    *          double     f1,...   I   additional values
    * return : none
    *-----------------------------------------------------------------------------*/
    mat(int row0, int clm0, double f, double f1, ...);

    /* array constructor ----------------------------------------------------------
    * initialize matrix with given size and values from array (row-major order)
    * args   : int        row0     I   number of rows
    *          int        clm0     I   number of columns
    *          const double* pf    I   pointer to array of values
    * return : none
    *-----------------------------------------------------------------------------*/
    mat(int row0, int clm0, const double* pf);

    /* clear matrix ---------------------------------------------------------------
    * set all elements of the matrix to zero
    * args   : none
    * return : none
    *-----------------------------------------------------------------------------*/
    void Clear(void);

    /* set diagonal elements from array -------------------------------------------
    * set the diagonal elements of the matrix from an array
    * args   : double*    f        I   pointer to array of values
    *          int        len      I   number of diagonal elements to set
    * return : none
    *-----------------------------------------------------------------------------*/
    void SetDiag(double f[], int len);

    /* set diagonal elements from array (alternative) -----------------------------
    * set the diagonal elements of the matrix from an array (alternative method)
    * args   : double*    f        I   pointer to array of values
    *          int        len      I   number of diagonal elements to set
    * return : none
    *-----------------------------------------------------------------------------*/
    void SetDiag2(double f[], int len);

    /* matrix addition ------------------------------------------------------------
    * add two matrices element-wise
    * args   : const mat& m        I   matrix to add
    * return : result of addition
    *-----------------------------------------------------------------------------*/
    mat operator+(const mat& m) const;

    /* matrix subtraction ---------------------------------------------------------
    * subtract two matrices element-wise
    * args   : const mat& m        I   matrix to subtract
    * return : result of subtraction
    *-----------------------------------------------------------------------------*/
    mat operator-(const mat& m) const;

    /* matrix scaling -------------------------------------------------------------
    * multiply all elements by a scalar
    * args   : double     f        I   scale factor
    * return : scaled matrix
    *-----------------------------------------------------------------------------*/
    mat operator*(double f) const;

    /* matrix-vector multiplication -----------------------------------------------
    * multiply matrix by a vector
    * args   : const vect& v       I   vector to multiply
    * return : resulting vector
    *-----------------------------------------------------------------------------*/
    vect operator*(const vect& v) const;

    /* matrix multiplication ------------------------------------------------------
    * multiply two matrices
    * args   : const mat& m        I   matrix to multiply
    * return : result of multiplication
    *-----------------------------------------------------------------------------*/
    mat operator*(const mat& m) const;

    /* assign all elements to a value ---------------------------------------------
    * set all elements to the same value
    * args   : double     f        I   value to assign
    * return : reference to this matrix
    *-----------------------------------------------------------------------------*/
    mat& operator=(double f);

    /* matrix addition assignment -------------------------------------------------
    * add another matrix to this matrix (element-wise)
    * args   : const mat& m0       I   matrix to add
    * return : reference to this matrix
    *-----------------------------------------------------------------------------*/
    mat& operator+=(const mat& m0);

    /* matrix + diag(vector) assignment -------------------------------------------
    * add a vector to the diagonal elements of this matrix
    * args   : const vect& v       I   vector to add to diagonal
    * return : reference to this matrix
    *-----------------------------------------------------------------------------*/
    mat& operator+=(const vect& v);

    /* matrix subtraction assignment ----------------------------------------------
    * subtract another matrix from this matrix (element-wise)
    * args   : const mat& m0       I   matrix to subtract
    * return : reference to this matrix
    *-----------------------------------------------------------------------------*/
    mat& operator-=(const mat& m0);

    /* matrix scaling assignment --------------------------------------------------
    * scale this matrix by a scalar
    * args   : double     f        I   scale factor
    * return : reference to this matrix
    *-----------------------------------------------------------------------------*/
    mat& operator*=(double f);

    /* increment diagonal by 1.0 --------------------------------------------------
    * increment all diagonal elements by 1.0
    * args   : none
    * return : reference to this matrix
    *-----------------------------------------------------------------------------*/
    mat& operator++();

    /* element access -------------------------------------------------------------
    * access matrix element by row and column index
    * args   : int        r        I   row index
    *          int        c        I   column index (default: -1, for vector access)
    * return : reference to element
    *-----------------------------------------------------------------------------*/
    double& operator()(int r, int c = -1);

    /* set row to zero ------------------------------------------------------------
    * set all elements in the i-th row to zero
    * args   : int        i        I   row index
    * return : none
    *-----------------------------------------------------------------------------*/
    void ZeroRow(int i);

    /* set column to zero ---------------------------------------------------------
    * set all elements in the j-th column to zero
    * args   : int        j        I   column index
    * return : none
    *-----------------------------------------------------------------------------*/
    void ZeroClm(int j);

    /* set row from variadic values -----------------------------------------------
    * set the i-th row from a list of double values
    * args   : int        i        I   row index
    *          double     f,...    I   values to set
    * return : none
    *-----------------------------------------------------------------------------*/
    void SetRow(int i, double f, ...);

    /* set row from vector --------------------------------------------------------
    * set the i-th row from a vector
    * args   : int        i        I   row index
    *          const vect& v       I   vector to set
    * return : none
    *-----------------------------------------------------------------------------*/
    void SetRow(int i, const vect& v);

    /* set column from array ------------------------------------------------------
    * set the j-th column from an array of values
    * args   : int        j        I   column index
    *          double*    f        I   pointer to array of values
    *          int        len      I   number of elements to set
    * return : none
    *-----------------------------------------------------------------------------*/
    void SetClm(int j, double f[], int len);

    /* set column from vector -----------------------------------------------------
    * set the j-th column from a vector
    * args   : int        j        I   column index
    *          const vect& v       I   vector to set
    * return : none
    *-----------------------------------------------------------------------------*/
    void SetClm(int j, const vect& v);

    /* get row as vector ----------------------------------------------------------
    * get the i-th row of the matrix as a vector
    * args   : int        i        I   row index
    * return : row vector
    *-----------------------------------------------------------------------------*/
    vect GetRow(int i) const;

    /* get column as vector -------------------------------------------------------
    * get the j-th column of the matrix as a vector
    * args   : int        j        I   column index
    * return : column vector
    *-----------------------------------------------------------------------------*/
    vect GetClm(int j) const;

    /* set row and columns from vect3 ---------------------------------------------
    * set i-th row and j...(j+2)-th columns from vect3
    * args   : int        i        I   row index
    *          int        j        I   starting column index
    *          const vect3& v      I   vect3 to set
    * return : none
    *-----------------------------------------------------------------------------*/
    void SetRowVect3(int i, int j, const vect3& v);

    /* set row and columns from two vect3 -----------------------------------------
    * set i-th row and j...(j+5)-th columns from two vect3
    * args   : int        i        I   row index
    *          int        j        I   starting column index
    *          const vect3& v      I   first vect3 to set
    *          const vect3& v1     I   second vect3 to set
    * return : none
    *-----------------------------------------------------------------------------*/
    void SetRowVect3(int i, int j, const vect3& v, const vect3& v1);

    /* set row and columns from three vect3 ---------------------------------------
    * set i-th row and j...(j+8)-th columns from three vect3
    * args   : int        i        I   row index
    *          int        j        I   starting column index
    *          const vect3& v      I   first vect3 to set
    *          const vect3& v1     I   second vect3 to set
    *          const vect3& v2     I   third vect3 to set
    * return : none
    *-----------------------------------------------------------------------------*/
    void SetRowVect3(int i, int j, const vect3& v, const vect3& v1, const vect3& v2);

    /* set rows and column from vect3 ---------------------------------------------
    * set i...(i+2)-th rows and j-th column from vect3
    * args   : int        i        I   starting row index
    *          int        j        I   column index
    *          const vect3& v      I   vect3 to set
    * return : none
    *-----------------------------------------------------------------------------*/
    void SetClmVect3(int i, int j, const vect3& v);

    /* set rows and column from two vect3 -----------------------------------------
    * set i...(i+5)-th rows and j-th column from two vect3
    * args   : int        i        I   starting row index
    *          int        j        I   column index
    *          const vect3& v      I   first vect3 to set
    *          const vect3& v1     I   second vect3 to set
    * return : none
    *-----------------------------------------------------------------------------*/
    void SetClmVect3(int i, int j, const vect3& v, const vect3& v1);

    /* set rows and column from three vect3 ---------------------------------------
    * set i...(i+8)-th rows and j-th column from three vect3
    * args   : int        i        I   starting row index
    *          int        j        I   column index
    *          const vect3& v      I   first vect3 to set
    *          const vect3& v1     I   second vect3 to set
    *          const vect3& v2     I   third vect3 to set
    * return : none
    *-----------------------------------------------------------------------------*/
    void SetClmVect3(int i, int j, const vect3& v, const vect3& v1, const vect3& v2);

    /* get row and columns as vect3 -----------------------------------------------
    * get i-th row and j...(j+2)-th columns as vect3
    * args   : int        i        I   row index
    *          int        j        I   starting column index
    * return : vect3 from specified row and columns
    *-----------------------------------------------------------------------------*/
    vect3 GetRowVect3(int i, int j) const;

    /* get rows and column as vect3 -----------------------------------------------
    * get i...(i+2)-th rows and j-th column as vect3
    * args   : int        i        I   starting row index
    *          int        j        I   column index
    * return : vect3 from specified rows and column
    *-----------------------------------------------------------------------------*/
    vect3 GetClmVect3(int i, int j) const;

    /* set diagonal elements from vect3 -------------------------------------------
    * set diagonal elements m(i,j), m(i+1,j+1), m(i+2,j+2) from vect3
    * args   : int        i        I   starting row index
    *          int        j        I   starting column index
    *          const vect3& v      I   vect3 to set
    * return : none
    *-----------------------------------------------------------------------------*/
    void SetDiagVect3(int i, int j, const vect3& v);

    /* get diagonal elements as vect3 ---------------------------------------------
    * get diagonal elements m(i,j), m(i+1,j+1), m(i+2,j+2) as vect3
    * args   : int        i        I   starting row index
    *          int        j        I   starting column index (default: -1)
    * return : vect3 of diagonal elements
    *-----------------------------------------------------------------------------*/
    vect3 GetDiagVect3(int i, int j = -1) const;

    /* set askew-symmetric submatrix from vect3 -----------------------------------
    * set i...(i+2)-th rows and j...(j+2)-th columns from askew vect3
    * args   : int        i        I   starting row index
    *          int        j        I   starting column index
    *          const vect3& v      I   vect3 to set as askew-symmetric matrix
    * return : none
    *-----------------------------------------------------------------------------*/
    void SetAskew(int i, int j, const vect3& v);

    /* set submatrix from mat3 ----------------------------------------------------
    * set i...(i+2)-th rows and j...(j+2)-th columns from mat3
    * args   : int        i        I   starting row index
    *          int        j        I   starting column index
    *          const mat3& m       I   mat3 to set
    * return : none
    *-----------------------------------------------------------------------------*/
    void SetMat3(int i, int j, const mat3& m);

    /* set submatrix from two mat3 ------------------------------------------------
    * set i...(i+5)-th rows and j...(j+2)-th columns from two mat3
    * args   : int        i        I   starting row index
    *          int        j        I   starting column index
    *          const mat3& m       I   first mat3 to set
    *          const mat3& m1      I   second mat3 to set
    * return : none
    *-----------------------------------------------------------------------------*/
    void SetMat3(int i, int j, const mat3& m, const mat3& m1);

    /* set submatrix from three mat3 ----------------------------------------------
    * set i...(i+8)-th rows and j...(j+2)-th columns from three mat3
    * args   : int        i        I   starting row index
    *          int        j        I   starting column index
    *          const mat3& m       I   first mat3 to set
    *          const mat3& m1      I   second mat3 to set
    *          const mat3& m2      I   third mat3 to set
    * return : none
    *-----------------------------------------------------------------------------*/
    void SetMat3(int i, int j, const mat3& m, const mat3& m1, const mat3& m2);

    /* get submatrix as mat3 ------------------------------------------------------
    * get mat3 from i...(i+2)-th rows and j...(j+2)-th columns
    * args   : int        i        I   starting row index
    *          int        j        I   starting column index (default: -1)
    * return : mat3 from specified submatrix
    *-----------------------------------------------------------------------------*/
    mat3 GetMat3(int i, int j = -1) const;

    /* add mat3 to submatrix ------------------------------------------------------
    * add mat3 to i...(i+2)-th rows and j...(j+2)-th columns
    * args   : int        i        I   starting row index
    *          int        j        I   starting column index
    *          const mat3& m       I   mat3 to add
    * return : none
    *-----------------------------------------------------------------------------*/
    void SubAddMat3(int i, int j, const mat3& m);

    /* matrix transposition -------------------------------------------------------
    * transpose the matrix
    * args   : const mat& m        I   matrix to transpose
    * return : transposed matrix
    *-----------------------------------------------------------------------------*/
    friend mat operator~(const mat& m);
};

/**
 * @class earth
 * @ingroup nav_core
 * @brief Earth model and parameters for navigation calculations.
 *
 * Stores ellipsoid parameters, gravity, and rotation rates, and provides methods for
 * updating and transforming positions and velocities.
 */
class earth {
public:
    double a;        /*< Equatorial radius of the Earth (m) */
    double b;        /*< Polar radius of the Earth (m) */
    double f;        /*< Flattening factor of the Earth */
    double wie;      /*< Earth's angular velocity (rad/s) */
    double sl;       /*< Sine of latitude */
    double sl2;      /*< Square of sine of latitude */
    double sl4;      /*< Fourth power of sine of latitude */
    double cl;       /*< Cosine of latitude */
    double tl;       /*< Tangent of latitude */
    double RMh;      /*< Meridian radius of curvature (m) */
    double RNh;      /*< Transverse radius of curvature (m) */
    double clRNh;    /*< Cosine of latitude times transverse radius of curvature */
    double f_RMh;    /*< Flattening factor times meridian radius of curvature */
    double f_RNh;    /*< Flattening factor times transverse radius of curvature */
    double f_clRNh;  /*< Flattening factor times cosine of latitude times transverse radius of curvature */

    vect3 pos;       /*< Position vector */
    vect3 vn;        /*< Velocity vector */
    vect3 wnie;      /*< Earth rotation rate in the local frame */
    vect3 wnen;      /*< Earth rotation rate in the navigation frame */
    vect3 wnin;      /*< Earth rotation rate in the local frame */
    vect3 gn;        /*< Gravity vector in the local frame */
    vect3 gcc;       /*< Gravity vector in the navigation frame */
    vect3* pgn;      /*< Pointer to gravity vector */

    /* constructor ----------------------------------------------------------------
    * initialize earth model with given equatorial radius and flattening factor
    * args   : double     a        I   equatorial radius (default: RE)
    *          double     f        I   flattening factor (default: f0_earth)
    * return : none
    *-----------------------------------------------------------------------------*/
    earth(double a = RE, double f = f0_earth);

    /* initialize earth parameters ------------------------------------------------
    * initialize earth parameters with given equatorial radius and flattening factor
    * args   : double     a        I   equatorial radius (default: RE)
    *          double     f        I   flattening factor (default: f0_earth)
    * return : none
    *-----------------------------------------------------------------------------*/
    void Init(double a = RE, double f = f0_earth);

    /* update earth parameters ----------------------------------------------------
    * update earth parameters based on current position and velocity
    * args   : const vect3& pos   I   position vector (latitude, longitude, height)
    *          const vect3& vn    I   velocity vector
    * return : none
    *-----------------------------------------------------------------------------*/
    void Update(const vect3& pos, const vect3& vn);

    /* convert velocity to position increment --------------------------------------
    * convert velocity vector to position increment over a time step
    * args   : const vect3& vn    I   velocity vector
    *          float      ts      I   time step (default: 1.0)
    * return : position increment vector
    *-----------------------------------------------------------------------------*/
    vect3 vn2dpos(const vect3& vn, float ts = 1.0) const;
};

/**
 * @class maxmin
 * @ingroup filter_core
 * @brief Tracks max, min, mean, and statistics for a single variable.
 */
class maxmin {
public:
    float maxCur;    /*< Current maximum value. */
    float minCur;    /*< Current minimum value. */
    float maxpreCur; /*< Maximum value in the previous iteration. */
    float minpreCur; /*< Minimum value in the previous iteration. */
    float maxRes;    /*< Maximum result over all updates. */
    float minRes;    /*< Minimum result over all updates. */
    float meanRes;   /*< Mean result over all updates. */
    float sumRes;    /*< Sum of all values. */
    int cnt0;        /*< Initial count for statistics. */
    int cntCur;      /*< Current count of values in this iteration. */
    int cntpreCur;   /*< Count of values in the previous iteration. */
    int flag;        /*< Flag variable for status or mode. */

    /* initialize the maxmin statistics -------------------------------------------
    * initialize all statistics and counters
    * args   : int    cnt00   I   initial count (default: 100)
    *          int    pre00   I   previous count (default: 0)
    *          float  f       I   initial value (default: 0.0f)
    * return : none
    *-----------------------------------------------------------------------------*/
    void Init(int cnt00 = 100, int pre00 = 0, float f = 0.0f);

    /* update statistics with a new value -----------------------------------------
    * update max, min, mean, and counters with a new value
    * args   : float  f       I   new value to update statistics
    * return : int            O   status or updated count
    *-----------------------------------------------------------------------------*/
    int Update(float f);
};

/**
 * @class maxminn
 * @ingroup filter_core
 * @brief Tracks statistics for multiple variables using maxmin objects.
 */
class maxminn {
public:
    int n;                                        /*< Number of maxmin objects. */
    int flag;                                     /*< Flag variable. */
    maxmin mm[MAX_MAT_DIM];                       /*< Array of maxmin objects. */

    /* initialize the maxminn statistics ------------------------------------------
    * initialize all maxmin objects and counters for multiple variables
    * args   : int    n0      I   number of variables (default: 3)
    *          int    cnt00   I   initial count for each variable (default: 100)
    *          int    pre00   I   previous count for each variable (default: 0)
    *          float  f       I   initial value for each variable (default: 0.0f)
    * return : none
    *-----------------------------------------------------------------------------*/
    void Init(int n0 = 3, int cnt00 = 100, int pre00 = 0, float f = 0.0f);

    /* update statistics with an array of values ----------------------------------
    * update all maxmin objects with new values from an array
    * args   : double* f      I   pointer to array of new values
    *          int     size   I   number of values in the array
    * return : int            O   status or updated count
    *-----------------------------------------------------------------------------*/
    int Update(double* f, int size);

    /* update statistics with a vect3 ---------------------------------------------
    * update all maxmin objects with new values from a vect3
    * args   : const vect3& v I   vect3 of new values
    * return : int            O   status or updated count
    *-----------------------------------------------------------------------------*/
    int Update(const vect3& v);

    /* update statistics with two vect3 -------------------------------------------
    * update all maxmin objects with new values from two vect3
    * args   : const vect3& v1 I   first vect3 of new values
    *          const vect3& v2 I   second vect3 of new values
    * return : int             O   status or updated count
    *-----------------------------------------------------------------------------*/
    int Update(const vect3& v1, const vect3& v2);

    /* update statistics with three vect3 -----------------------------------------
    * update all maxmin objects with new values from three vect3
    * args   : const vect3& v1 I   first vect3 of new values
    *          const vect3& v2 I   second vect3 of new values
    *          const vect3& v3 I   third vect3 of new values
    * return : int             O   status or updated count
    *-----------------------------------------------------------------------------*/
    int Update(const vect3& v1, const vect3& v2, const vect3& v3);

    /* update statistics with four vect3 ------------------------------------------
    * update all maxmin objects with new values from four vect3
    * args   : const vect3& v1 I   first vect3 of new values
    *          const vect3& v2 I   second vect3 of new values
    *          const vect3& v3 I   third vect3 of new values
    *          const vect3& v4 I   fourth vect3 of new values
    * return : int             O   status or updated count
    *-----------------------------------------------------------------------------*/
    int Update(const vect3& v1, const vect3& v2, const vect3& v3, const vect3& v4);

    /* update statistics with five vect3 ------------------------------------------
    * update all maxmin objects with new values from five vect3
    * args   : const vect3& v1 I   first vect3 of new values
    *          const vect3& v2 I   second vect3 of new values
    *          const vect3& v3 I   third vect3 of new values
    *          const vect3& v4 I   fourth vect3 of new values
    *          const vect3& v5 I   fifth vect3 of new values
    * return : int             O   status or updated count
    *-----------------------------------------------------------------------------*/
    int Update(const vect3& v1, const vect3& v2, const vect3& v3, const vect3& v4, const vect3& v5);

    /* get result as float --------------------------------------------------------
    * get the minimum, mean, or maximum result for the i-th variable
    * args   : int    i              I   index of the variable
    *          int    minmeanmaxFlag I   flag (0: min, 1: mean, 2: max)
    * return : float                 O   requested result
    *-----------------------------------------------------------------------------*/
    float ResFloat(int i, int minmeanmaxFlag);
};

/**
 * @class STA_VAR
 * @ingroup filter_core
 * @brief Tracks mean and variance for a set of data points.
 */
class STA_VAR {
public:
    int ipush;                                  /*< Push index. */
    int imax;                                   /*< Maximum index. */
    float array[VARMAX];                        /*< Array to store data points. */
    float mean;                                 /*< Mean value. */
    float var;                                  /*< Variance value. */

    /* constructor ----------------------------------------------------------------
    * initialize the statistics tracker with maximum size and initial value
    * args   : int    imax0   I   maximum number of data points (default: 10)
    *          float  data0   I   initial value (default: 0.0)
    * return : none
    *-----------------------------------------------------------------------------*/
    STA_VAR(int imax0 = 10, float data0 = 0.0);

    /* update statistics with a new data point ------------------------------------
    * update mean and variance with a new data point
    * args   : float  data    I   new data point
    *          bool   isvar   I   whether to update variance (default: 1)
    * return : double         O   updated mean or variance
    *-----------------------------------------------------------------------------*/
    double Update(float data, bool isvar = 1);

    /* reset statistics -----------------------------------------------------------
    * reset all statistics and counters
    * args   : none
    * return : none
    *-----------------------------------------------------------------------------*/
    void Reset();
};

/**
 * @class STA_VARS
 * @ingroup filter_core
 * @brief Welford's algorithm for variance of vector data.
 */
class STA_VARS {
public:
    vect mean;                                 /*< Mean of the data. */
    vect mean_2;                               /*< Mean squared of the data. */
    long num;                                  /*< Number of data points. */
    int n_ele;                                 /*< Number of elements in each data vector. */

    /* constructor ----------------------------------------------------------------
    * initialize the Welford variance calculator with number of elements
    * args   : int    n_ele   I   number of elements in each data vector
    * return : none
    *-----------------------------------------------------------------------------*/
    STA_VARS(int n_ele);

    /* update statistics with a new data vector -----------------------------------
    * update mean and variance with a new data vector
    * args   : vect   data    I   new data vector
    * return : vect           O   updated mean vector
    *-----------------------------------------------------------------------------*/
    vect update(const vect data);

    /* reset statistics -----------------------------------------------------------
    * reset all statistics and counters
    * args   : none
    * return : none
    *-----------------------------------------------------------------------------*/
    void Reset();
};

/**
 * @class STA_AVPI
 * @ingroup filter_core
 * @brief Buffer for attitude, velocity, and position samples with interpolation.
 */
class STA_AVPI {
public:
    double ts;                        /*< Time step (seconds) */
    int ipush;                        /*< Current push index */
    int avpinum;                      /*< Number of stored attitude/velocity/position samples */
    vect3 atti[AVPINUM];              /*< Array of attitude samples */
    vect3 vni[AVPINUM];               /*< Array of velocity samples */
    vect3 posi[AVPINUM];              /*< Array of position samples */
    vect3 att;                        /*< Current attitude */
    vect3 vn;                         /*< Current velocity */
    vect3 pos;                        /*< Current position */

    /* default constructor --------------------------------------------------------
    * initialize STA_AVPI object with default parameters
    * args   : none
    * return : none
    *-----------------------------------------------------------------------------*/
    STA_AVPI(void);

    /* initialize the buffer and state --------------------------------------------
    * initialize attitude, velocity, position buffers and parameters
    * args   : const vect3 &att0   I   initial attitude
    *          const vect3 &vn0    I   initial velocity
    *          const vect3 &pos0   I   initial position
    *          double     ts       I   time step (seconds)
    *          int        num      I   number of samples to store (default: 0)
    * return : none
    *-----------------------------------------------------------------------------*/
    void Init(const vect3& att0, const vect3& vn0, const vect3& pos0, double ts, int num = 0);

    /* push new attitude, velocity, position sample -------------------------------
    * push a new set of attitude, velocity, and position into the buffer
    * args   : const vect3 &attk   I   attitude sample
    *          const vect3 &vnk    I   velocity sample
    *          const vect3 &posk   I   position sample
    * return : none
    *-----------------------------------------------------------------------------*/
    void Push(const vect3& attk, const vect3& vnk, const vect3& posk);

    /* interpolate state at a past time -------------------------------------------
    * interpolate attitude, velocity, and position at a specified time in the past
    * args   : double     tpast    I   time in the past to interpolate (seconds)
    *          int        avp      I   flag for which states to interpolate (default: 0x7)
    * return : status or index of interpolation
    *-----------------------------------------------------------------------------*/
    int Interp(double tpast, int avp = 0x7);
};

/**
 * @class IMU
 * @ingroup nav_core
 * @brief IMU sensor model and data structure.
 *
 * Contains calibration parameters, sensor state, and methods for updating and transforming IMU data.
 */
class IMU {
public:
    mat3 Kg;        /*< Gyroscope scale factor matrix */
    vect3 eb;       /*< Gyroscope bias vector */
    mat3 Ka;        /*< Accelerometer scale factor matrix */
    vect3 db;       /*< Accelerometer bias vector */
    vect3 Ka2;      /*< Accelerometer quadratic nonlinearity coefficient */
    vect3 lvx;      /*< Lever arm vector in X direction */
    vect3 lvy;      /*< Lever arm vector in Y direction */
    vect3 lvz;      /*< Lever arm vector in Z direction */
    double tGA;     /*< Time delay between gyro and accelerometer */
    mat3* pKga;     /*< Pointer to combined scale factor matrix */
    mat3* pgSens;   /*< Pointer to gyroscope g-sensitivity matrix */
    mat3 gSens;     /*< Gyroscope g-sensitivity matrix */
    mat3* pgSens2;  /*< Pointer to secondary gyroscope g-sensitivity matrix */
    mat3 gSens2;    /*< Secondary gyroscope g-sensitivity matrix */
    mat3* pgSensX;  /*< Pointer to cross gyroscope g-sensitivity matrix */
    mat3 gSensX;    /*< Cross gyroscope g-sensitivity matrix */
    vect3 Sfg;      /*< Gyroscope scale factor nonlinearity */
    vect3 Sfa;      /*< Accelerometer scale factor nonlinearity */
    vect3* pSf;     /*< Pointer to scale factor nonlinearity vector */
    vect3* pKa2;    /*< Pointer to quadratic nonlinearity coefficient */
    vect3* plv;     /*< Pointer to lever arm vector */
    vect3 wb_1;     /*< Previous angular rate (for lever arm compensation) */
    mat3 Cba;       /*< Body-to-antenna rotation matrix */
    mat3 SSx;       /*< Skew-symmetric matrix X */
    mat3 SSy;       /*< Skew-symmetric matrix Y */
    mat3 SSz;       /*< Skew-symmetric matrix Z */
    mat3* pCba;     /*< Pointer to body-to-antenna rotation matrix */
    vect3 Q11, Q12, Q13, Q21, Q22, Q23, Q31, Q32, Q33; /*< Process noise covariance blocks */
    char* prfu;     /*< Pointer to RFU (Right-Forward-Up) string */
    char rfu[3];    /*< RFU (Right-Forward-Up) string */
    int nSamples;   /*< Number of IMU samples in current update */
    int iTemp;      /*< Temperature index */
    double tk;      /*< Current time */
    double nts;     /*< Nominal time step */
    double _nts;    /*< Internal time step */
    double Temp;    /*< Current temperature */
    double* pTempArray; /*< Pointer to temperature array */
    bool preFirst;      /*< Flag: first update or not */
    bool onePlusPre;    /*< Flag: one plus previous sample */
    bool preWb;         /*< Flag: previous angular rate available */
    vect3 phim;         /*< Integrated angle increment */
    vect3 dvbm;         /*< Integrated velocity increment */
    vect3 wmm;          /*< Mean angular rate */
    vect3 vmm;          /*< Mean velocity */
    vect3 swmm;         /*< Sum of angular rate */
    vect3 svmm;         /*< Sum of velocity */
    vect3 wm_1;         /*< Previous angular rate */
    vect3 vm_1;         /*< Previous velocity */

    /* default constructor --------------------------------------------------------
    * initialize IMU object with default parameters
    * args   : none
    * return : none
    *-----------------------------------------------------------------------------*/
    IMU(void);

    /* reset IMU state ------------------------------------------------------------
    * reset all IMU parameters and buffers to default values
    * args   : none
    * return : none
    *-----------------------------------------------------------------------------*/
    void Reset(void);

    /* set scale factor nonlinearity ----------------------------------------------
    * set gyroscope and accelerometer scale factor nonlinearity
    * args   : const vect3 &Sfg0   I   gyroscope scale factor nonlinearity
    *          const vect3 &Sfa0   I   accelerometer scale factor nonlinearity
    * return : none
    *-----------------------------------------------------------------------------*/
    void SetSf(const vect3& Sfg0, const vect3& Sfa0);

    /* set temperature array ------------------------------------------------------
    * set pointer to temperature array and type
    * args   : double *tempArray0  I   pointer to temperature array
    *          int    type         I   type of temperature compensation (default: 1)
    * return : none
    *-----------------------------------------------------------------------------*/
    void SetTemp(double* tempArray0, int type = 1);

    /* set scale factor and bias matrices -----------------------------------------
    * set gyroscope and accelerometer scale factor and bias
    * args   : const mat3 &Kg0     I   gyroscope scale factor matrix
    *          const vect3 eb0     I   gyroscope bias
    *          const mat3 &Ka0     I   accelerometer scale factor matrix
    *          const vect3 &db0    I   accelerometer bias
    * return : none
    *-----------------------------------------------------------------------------*/
    void SetKga(const mat3& Kg0, const vect3 eb0, const mat3& Ka0, const vect3& db0);

    /* set gyroscope g-sensitivity matrices ---------------------------------------
    * set gyroscope g-sensitivity matrices
    * args   : const mat3 &gSens0  I   primary g-sensitivity matrix
    *          const mat3 &gSens20 I   secondary g-sensitivity matrix
    *          const mat3 &gSensX0 I   cross g-sensitivity matrix
    * return : none
    *-----------------------------------------------------------------------------*/
    void SetgSens(const mat3& gSens0, const mat3& gSens20, const mat3& gSensX0);

    /* set accelerometer quadratic nonlinearity -----------------------------------
    * set accelerometer quadratic nonlinearity coefficient
    * args   : const vect3 &Ka20   I   quadratic nonlinearity coefficient
    * return : none
    *-----------------------------------------------------------------------------*/
    void SetKa2(const vect3& Ka20);

    /* set lever arm and time delay -----------------------------------------------
    * set lever arm vectors and time delay between gyro and accelerometer
    * args   : const vect3 &lvx0   I   lever arm in X direction
    *          const vect3 &lvy0   I   lever arm in Y direction
    *          const vect3 &lvz0   I   lever arm in Z direction (default: O31)
    *          double tGA0         I   time delay (default: 0.0)
    * return : none
    *-----------------------------------------------------------------------------*/
    void SetLvtGA(const vect3& lvx0, const vect3& lvy0, const vect3& lvz0 = O31, double tGA0 = 0.0);

    /* set body-to-antenna rotation matrix ----------------------------------------
    * set the body-to-antenna rotation matrix
    * args   : const mat3 &Cba0    I   body-to-antenna rotation matrix
    * return : none
    *-----------------------------------------------------------------------------*/
    void SetCba(const mat3& Cba0);

    /* set RFU string -------------------------------------------------------------
    * set the RFU (Right-Forward-Up) string
    * args   : const char *rfu0    I   RFU string
    * return : none
    *-----------------------------------------------------------------------------*/
    void SetRFU(const char* rfu0);

    /* update IMU state with new measurements -------------------------------------
    * update IMU state with new angular rate and velocity measurements
    * args   : const vect3 *pwm    I   pointer to angular rate measurements
    *          const vect3 *pvm    I   pointer to velocity measurements
    *          int    nSamples     I   number of samples
    *          double ts           I   time step
    * return : updated time
    *-----------------------------------------------------------------------------*/
    double Update(const vect3* pwm, const vect3* pvm, int nSamples, double ts);

    /* apply RFU transformation to angular rates ---------------------------------
    * apply RFU (Right-Forward-Up) transformation to angular rate measurements
    * args   : vect3 *pwm          IO  pointer to angular rate measurements
    *          int    nSamples     I   number of samples
    *          const char *str     I   RFU string
    * return : none
    *-----------------------------------------------------------------------------*/
    void IMURFU(vect3* pwm, int nSamples, const char* str);

    /* apply RFU transformation to angular rates and velocities -------------------
    * apply RFU (Right-Forward-Up) transformation to angular rate and velocity measurements
    * args   : vect3 *pwm          IO  pointer to angular rate measurements
    *          vect3 *pvm          IO  pointer to velocity measurements
    *          int    nSamples     I   number of samples
    *          const char *str     I   RFU string
    * return : none
    *-----------------------------------------------------------------------------*/
    void IMURFU(vect3* pwm, vect3* pvm, int nSamples, const char* str);

    /* static IMU alignment -------------------------------------------------------
    * perform static alignment using mean angular rate and velocity
    * args   : vect3 &wm           O   mean angular rate
    *          vect3 &vm           O   mean velocity
    *          vect3 &att0         O   initial attitude
    *          vect3 &pos0         O   initial position
    *          double ts           I   time step
    * return : none
    *-----------------------------------------------------------------------------*/
    void IMUStatic(vect3& wm, vect3& vm, vect3& att0, vect3& pos0, double ts);

    /* lever arm compensation for angular rates -----------------------------------
    * lever arm compensation for angular rates using body-frame and navigation-frame angular rates
    * args   : mat3 &wfb           I   body-frame angular rate matrix
    *          mat3 &wfn           I   navigation-frame angular rate matrix
    * return : compensated angular rate matrix
    *-----------------------------------------------------------------------------*/
    mat3 lsclbt(mat3& wfb, mat3& wfn);
};

/**
 * @class INS
 * @ingroup nav_core
 * @brief Inertial Navigation System state and update logic.
 *
 * Stores navigation state, error models, and provides update and extrapolation methods.
 */
class INS {
public:
    float ts, nts;           /*< Time step (seconds), nominal time step (seconds) */
    double tk;               /*< Current time (seconds) */
    double mvnt, mvnT;       /*< Moving window time, total moving time */
    double dist;             /*< Distance traveled */
    double afabar;           /*< Mean specific force (for alignment) */
    int mvnk;                /*< Moving window sample count */
    earth eth;               /*< Earth model parameters */
    IMU imu;                 /*< IMU sensor model and data */
    quat qnb;                /*< Attitude quaternion (body to navigation) */
    mat3 Cnb, Cnb0, Cbn;     /*< Direction Cosine Matrices (body to nav, initial, nav to body) */
    mat3 mvnCnb0;            /*< DCM for moving window, initial */
    mat3 Kg, Ka;             /*< Gyro and accelerometer scale factor matrices */
    vect3 wib;               /*< Angular rate of body w.r.t inertial frame, in body frame */
    vect3 fb;                /*< Specific force in body frame */
    vect3 fn;                /*< Specific force in navigation frame */
    vect3 an;                /*< Acceleration in navigation frame */
    vect3 anbar;             /*< Mean acceleration in navigation frame */
    vect3 web;               /*< Angular rate of Earth in body frame */
    vect3 webbar;            /*< Mean angular rate of Earth in body frame */
    vect3 wnb;               /*< Angular rate of navigation frame w.r.t inertial, in body frame */
    vect3 att;               /*< Attitude angles {roll, pitch, yaw} (radians) */
    vect3 vn;                /*< Velocity in navigation frame */
    vect3 mvn;               /*< Mean velocity in navigation frame */
    vect3 mvni;              /*< Initial mean velocity */
    vect3 mvnmax;            /*< Maximum mean velocity */
    vect3 mvnmin;            /*< Minimum mean velocity */
    vect3 vb;                /*< Velocity in body frame */
    vect3 pos;               /*< Position {latitude, longitude, height} */
    vect3 eb;                /*< Gyro bias */
    vect3 db;                /*< Accelerometer bias */
    vect3 Ka2;               /*< Accelerometer quadratic nonlinearity coefficient */
    vect3 tauGyro;           /*< Gyro Markov correlation time */
    vect3 tauAcc;            /*< Accelerometer Markov correlation time */
    vect3 _betaGyro;         /*< Gyro Markov process noise */
    vect3 _betaAcc;          /*< Accelerometer Markov process noise */
    mat3 Maa, Mav, Map, Mva, Mvv, Mvp, Mpv, Mpp; /*< Error transform matrix coefficients (for etm) */
    vect3 lvr, vnL, posL;    /*< Lever arm vector, velocity and position at lever arm */
    mat3 CW, MpvCnb;         /*< Lever arm correction matrices */
    quat qnbE;               /*< Extrapolated attitude quaternion */
    vect3 attE, vnE, posE;   /*< Extrapolated attitude, velocity, position */
    maxmin mmwb, mmfb;       /*< Max/min statistics for angular rate and specific force */
    bool isOpenloop;         /*< Flag: open-loop mode */
    bool isMemsgrade;        /*< Flag: MEMS grade IMU */
    bool isNocompasseffect;  /*< Flag: ignore compass effect */
    bool isOutlever;         /*< Flag: output lever arm compensation */

    /* constructor (yaw, position, time) ------------------------------------------
    * initialize INS with yaw angle, position, and initial time
    * args   : double&    yaw0     I   initial yaw angle (radians)
    *          vect3      pos0     I   initial position (default: O31)
    *          double     tk0      I   initial time (default: 0.0)
    * return : none
    *-----------------------------------------------------------------------------*/
    INS(double& yaw0, const vect3& pos0 = O31, double tk0 = 0.0);

    /* constructor (attitude, velocity, position, time) ---------------------------
    * initialize INS with attitude, velocity, position, and initial time
    * args   : vect3      att0     I   initial attitude {roll, pitch, yaw}
    *          vect3      vn0      I   initial velocity (default: O31)
    *          vect3      pos0     I   initial position (default: O31)
    *          double     tk0      I   initial time (default: 0.0)
    * return : none
    *-----------------------------------------------------------------------------*/
    INS(const vect3& att0, const vect3& vn0 = O31, const vect3& pos0 = O31, double tk0 = 0.0);

    /* constructor (quaternion, velocity, position, time) -------------------------
    * initialize INS with quaternion, velocity, position, and initial time
    * args   : quat       qnb0     I   initial attitude quaternion (default: qI)
    *          vect3      vn0      I   initial velocity (default: O31)
    *          vect3      pos0     I   initial position (default: O31)
    *          double     tk0      I   initial time (default: 0.0)
    * return : none
    *-----------------------------------------------------------------------------*/
    INS(const quat& qnb0 = qI, const vect3& vn0 = O31, const vect3& pos0 = O31, double tk0 = 0.0);

    /* initialize INS state -------------------------------------------------------
    * initialize INS state using quaternion, velocity, and position
    * args   : quat       qnb0     I   initial attitude quaternion (default: qI)
    *          vect3      vn0      I   initial velocity (default: O31)
    *          vect3      pos0     I   initial position (default: O31)
    *          double     tk0      I   initial time (default: 0.0)
    * return : none
    *-----------------------------------------------------------------------------*/
    void Init(const quat& qnb0 = qI, const vect3& vn0 = O31, const vect3& pos0 = O31, double tk0 = 0.0);

    /* set Markov process time constants ------------------------------------------
    * set Markov process correlation times for gyro and accelerometer
    * args   : vect3      tauG     I   gyro correlation time
    *          vect3      tauA     I   accelerometer correlation time
    * return : none
    *-----------------------------------------------------------------------------*/
    void SetTauGA(const vect3& tauG, const vect3& tauA);

    /* SINS update using Gyro & Acc samples ---------------------------------------
    * update INS state using arrays of gyro and accelerometer samples
    * args   : vect3*     pwm      I   array of angular rate increments
    *          vect3*     pvm      I   array of velocity increments
    *          int        nSamples I   number of samples
    *          double     ts       I   time step
    * return : none
    *-----------------------------------------------------------------------------*/
    void Update(const vect3* pwm, const vect3* pvm, int nSamples, double ts);

    /* SINS fast extrapolation using 1 Gyro & Acc sample --------------------------
    * perform fast extrapolation of INS state using one gyro and accelerometer sample
    * args   : vect3      wm       I   angular rate increment (default: O31)
    *          vect3      vm       I   velocity increment (default: O31)
    *          double     ts       I   time step (default: 0.0)
    * return : none
    *-----------------------------------------------------------------------------*/
    void Extrap(const vect3& wm = O31, const vect3& vm = O31, double ts = 0.0);

    /* SINS fast extrapolation using previous Gyro & Acc sample -------------------
    * perform fast extrapolation of INS state using previous gyro and accelerometer sample
    * args   : double     extts    I   extrapolation time step
    * return : none
    *-----------------------------------------------------------------------------*/
    void Extrap(double extts);

    /* lever arm compensation -----------------------------------------------------
    * apply lever arm compensation to position and velocity
    * args   : vect3      dL       I   lever arm vector (default: O31)
    *          vect3*     ppos     O   pointer to output position (optional)
    *          vect3*     pvn      O   pointer to output velocity (optional)
    * return : none
    *-----------------------------------------------------------------------------*/
    void lever(const vect3& dL = O31, vect3* ppos = NULL, vect3* pvn = NULL);

    /* lever arm compensation (alternative) ---------------------------------------
    * apply lever arm compensation to position and velocity (alternative method)
    * args   : vect3      dL2      I   lever arm vector (default: O31)
    *          vect3*     ppos2    O   pointer to output position (optional)
    *          vect3*     pvn2     O   pointer to output velocity (optional)
    * return : none
    *-----------------------------------------------------------------------------*/
    void lever2(const vect3& dL2 = O31, vect3* ppos2 = NULL, vect3* pvn2 = NULL);

    /* SINS error transform matrix coefficients -----------------------------------
    * compute error transform matrix coefficients for SINS
    * args   : none
    * return : none
    *-----------------------------------------------------------------------------*/
    void etm(void);

    /* add error to INS state -----------------------------------------------------
    * add error vectors to attitude, velocity, and position
    * args   : vect3      phi      I   attitude error
    *          vect3      dvn      I   velocity error (default: O31)
    *          vect3      dpos     I   position error (default: O31)
    * return : none
    *-----------------------------------------------------------------------------*/
    void AddErr(const vect3& phi, const vect3& dvn = O31, const vect3& dpos = O31);

    /* add error to INS state (scalar attitude error) -----------------------------
    * add scalar attitude error and error vectors to velocity and position
    * args   : double     phiU     I   scalar attitude error
    *          vect3      dvn      I   velocity error (default: O31)
    *          vect3      dpos     I   position error (default: O31)
    * return : none
    *-----------------------------------------------------------------------------*/
    void AddErr(double phiU, const vect3& dvn = O31, const vect3& dpos = O31);

    /* debug stop -----------------------------------------------------------------
    * stop execution for debugging at a specified time
    * args   : double     t1       I   stop time
    *          int        absT     I   absolute time flag (default: 0)
    *          int        ext      I   extra flag (default: 0)
    * return : none
    *-----------------------------------------------------------------------------*/
    void DebugStop(double t1, int absT = 0, int ext = 0);
};


/**
 * @class IIR
 * @ingroup filter_core
 * @brief Infinite Impulse Response (IIR) filter for scalar data.
 */
class IIR
{
public:
#define IIRnMax 4
    int n;                  /*< Filter order (number of coefficients) */
    double b[IIRnMax];      /*< Numerator coefficients of the IIR filter */
    double a[IIRnMax];      /*< Denominator coefficients of the IIR filter */
    double x[IIRnMax];      /*< Input history (previous input samples) */
    double y[IIRnMax];      /*< Output history (previous output samples) */

    /* default constructor --------------------------------------------------------
    * initialize IIR filter with zero coefficients and state
    * args   : none
    * return : none
    *-----------------------------------------------------------------------------*/
    IIR(void);

    /* parameterized constructor --------------------------------------------------
    * initialize IIR filter with given coefficients and order
    * args   : double*    b0      I   numerator coefficients array
    *          double*    a0      I   denominator coefficients array
    *          int        n0      I   filter order
    * return : none
    *-----------------------------------------------------------------------------*/
    IIR(double* b0, double* a0, int n0);

    /* update filter with new input -----------------------------------------------
    * process a new input sample and update filter state
    * args   : double     x0      I   new input sample
    * return : double             O   filtered output
    *-----------------------------------------------------------------------------*/
    double Update(double x0);
};

/**
 * @class IIRV3
 * @ingroup filter_core
 * @brief 3D IIR filter for vector data.
 */
class IIRV3
{
public:
    IIR iir0;           /*< IIR filter for the first vector component */
    IIR iir1;           /*< IIR filter for the second vector component */
    IIR iir2;           /*< IIR filter for the third vector component */
    vect3 y;            /*< Filtered output vector */

    /* default constructor --------------------------------------------------------
    * initialize 3D IIR filter with default parameters
    * args   : none
    * return : none
    *-----------------------------------------------------------------------------*/
    IIRV3(void);

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
    IIRV3(double* b0, double* a0, int n0,
        double* b1 = (double*)NULL, double* a1 = (double*)NULL, int n1 = 0,
        double* b2 = (double*)NULL, double* a2 = (double*)NULL, int n2 = 0);

    /* update filter with new input vector ----------------------------------------
    * process a new input vector and update filter state
    * args   : const vect3& x     I   new input vector
    * return : vect3              O   filtered output vector
    *-----------------------------------------------------------------------------*/
    vect3 Update(const vect3& x);
};

/**
 * @class odo
 * @ingroup nav_core
 * @brief Odometry data and processing.
 */
class odo {
public:
#define distN 20
    vect3 IVno;        /*< Initial velocity (old) in navigation frame */
    vect3 IVni;        /*< Initial velocity (new) in navigation frame */
    vect3 Ifn;         /*< Integrated specific force in navigation frame */
    vect3 lvOD;        /*< Lever arm vector for odometry */
    vect3 vnOD;        /*< Velocity from odometry */
    vect3 posOD;       /*< Position from odometry */
    vect3 odZk;        /*< Odometry measurement innovation */
    vect3* phim;       /*< Pointer to angle increment array */
    vect3* vn;         /*< Pointer to velocity array */
    vect3* fn;         /*< Pointer to specific force array */
    vect3* web;        /*< Pointer to earth rotation rate array */

    mat3 Cbo;          /*< Rotation matrix from body-frame to OD-frame */
    mat3 IMv;          /*< Intermediate matrix for velocity update */
    mat3 odMphi;       /*< Odometry error transfer matrix for angle */
    mat3 odMvn;        /*< Odometry error transfer matrix for velocity */
    mat3 odMkappa;     /*< Odometry error transfer matrix for kappa */
    mat3 odMlever;     /*< Odometry error transfer matrix for lever arm */
    mat3* Cnb;         /*< Pointer to body-to-navigation rotation matrix */

    float Kod;         /*< Odometry scale factor */
    float odmeast;     /*< Odometry measurement time */
    float odmeasT;     /*< Odometry measurement total time */
    float dS0;         /*< Initial odometry distance */
    float distance;    /*< Total odometry distance */
    float distances[distN]; /*< Array of recent odometry distances */
    float distT01;     /*< Time interval for distance calculation */
    float odVel;       /*< Odometry velocity */
    float Hkv2y;       /*< Heading correction factor */
    int distK0;        /*< Distance index for distances array */
    int badODCnt;      /*< Counter for bad odometry measurements */
    float* nts;        /*< Pointer to time step array */

    bool odmeasOK;     /*< Flag indicating if odometry measurement is valid */

    /* initialize odometry state --------------------------------------------------
    * initialize all odometry parameters and buffers
    * args   : none
    * return : initialization status
    *-----------------------------------------------------------------------------*/
    int Init();

    /* update odometry with new distance measurement ------------------------------
    * update odometry state with a new distance increment
    * args   : double     dS      I   distance increment
    * return : update status
    *-----------------------------------------------------------------------------*/
    int Update(double dS);

    /* reset odometry state -------------------------------------------------------
    * reset all odometry parameters and buffers to default values
    * args   : none
    * return : reset status
    *-----------------------------------------------------------------------------*/
    int Reset(void);

    /* calculate odometry kappa ---------------------------------------------------
    * calculate odometry kappa (curvature) from input vector
    * args   : const vect3 &kpp   I   input vector
    * return : kappa vector
    *-----------------------------------------------------------------------------*/
    vect3 ODKappa(const vect3& kpp);

    /* set total odometry distance ------------------------------------------------
    * set the total odometry distance
    * args   : double     dist    I   distance to set
    * return : none
    *-----------------------------------------------------------------------------*/
    void SetDistance(double dist);
};

/**
 * @class ZIHR
 * @ingroup filter_core
 * @brief Zero-Integrated Heading Rate detector for motion state detection.
 */
class ZIHR {
public:
    double maxw;         /*< Maximum allowable angular rate */
    double ts;           /*< Time step (seconds) */
    double t;            /*< Current time (seconds) */
    double T;            /*< Detection window length (seconds) */
    double tstop;        /*< Time when stop is detected */
    double meanw;        /*< Mean angular rate */
    double meanwpre;     /*< Previous mean angular rate */
    double meanwprepre;  /*< Mean angular rate two steps ago */
    double val;          /*< Current value for detection */
    int cntNP;           /*< Negative/Positive count */
    int cntNi;           /*< Negative count */
    int cntPi;           /*< Positive count */
    int big;             /*< Flag for large value detection */
    int retres;          /*< Return result/status flag */

    /* default constructor --------------------------------------------------------
    * initialize ZIHR detector with default parameters
    * args   : none
    * return : none
    *-----------------------------------------------------------------------------*/
    ZIHR(void);

    /* initialize ZIHR detector ---------------------------------------------------
    * initialize ZIHR detector with specified parameters
    * args   : double     maxw0    I   maximum allowable angular rate
    *          double     ts0      I   time step (seconds)
    *          double     T0       I   detection window length (seconds, default: 10.0)
    *          int        cntNP0   I   initial count (default: 10)
    * return : none
    *-----------------------------------------------------------------------------*/
    void Init(double maxw0, double ts0, double T0 = 10.0, int cntNP0 = 10);

    /* reset ZIHR detector --------------------------------------------------------
    * reset all ZIHR parameters and counters to default values
    * args   : none
    * return : none
    *-----------------------------------------------------------------------------*/
    void Reset(void);

    /* update ZIHR detector with new angular rate ---------------------------------
    * update ZIHR detector with a new angular rate measurement
    * args   : double     wz       I   new angular rate measurement
    * return : detection result/status
    *-----------------------------------------------------------------------------*/
    int Update(double wz);
};

/**
 * @class KalmanFilter
 * @ingroup filter_core
 * @brief General Kalman filter base class for state estimation.
 */
class KalmanFilter {
public:
    double kftk;         /*< Kalman filter time (current epoch time) */
    double zfdafa;       /*< Auxiliary variable for filter (purpose-specific) */
    int nq;              /*< Dimension of process noise/state vector */
    int nr;              /*< Dimension of measurement noise/measurement vector */
    unsigned int kfcount;      /*< Kalman filter update count */
    unsigned int measflag;     /*< Measurement flag (bitmask for measurement types) */
    unsigned int measflaglog;  /*< Measurement flag log (history of measurement flags) */
    unsigned int measmask;     /*< Measurement mask (bitmask for valid measurements) */
    mat Ft;              /*< State transition matrix (discrete-time) */
    mat Pk;              /*< State covariance matrix */
    mat Hk;              /*< Measurement matrix */
    mat Fading;          /*< Fading matrix for adaptive filtering */
    vect Xk;             /*< State vector */
    vect Zk;             /*< Measurement vector */
    vect Zk_1;           /*< Previous measurement vector */
    vect Qt;             /*< Process noise covariance vector */
    vect Rt;             /*< Measurement noise covariance vector */
    vect Rt0;            /*< Initial measurement noise covariance vector */
    vect Rset;           /*< Set of measurement noise values */
    vect rts;            /*< Measurement noise scaling vector */
    vect RtTau;          /*< Measurement noise time constant vector */
    vect measstop;       /*< Measurement stop threshold vector */
    vect measlost;       /*< Measurement lost threshold vector */
    vect Xmax;           /*< Maximum allowed state values */
    vect Pmax;           /*< Maximum allowed covariance values */
    vect Pmin;           /*< Minimum allowed covariance values */
    vect Pset;           /*< Set of covariance values */
    vect Zfd;            /*< Measurement difference vector */
    vect Zfd0;           /*< Initial measurement difference vector */
    vect Zmm0;           /*< Initial measurement bounds vector */
    vect Zmax;           /*< Maximum measurement bounds vector */
    vect innoDiffMax;    /*< Maximum innovation difference vector */
    vect innoPre;        /*< Previous innovation vector */
    vect Rmax;           /*< Maximum measurement noise vector (adaptive) */
    vect Rmin;           /*< Minimum measurement noise vector (adaptive) */
    vect Rbeta;          /*< Measurement noise adaptation rate vector */
    vect Rb;             /*< Measurement noise bias vector */
    vect Rstop;          /*< Measurement noise stop threshold vector */
    vect innoMax;        /*< Maximum innovation outlier vector */
    vect FBTau;          /*< Feedback time constant vector */
    vect FBMax;          /*< Maximum feedback value vector */
    vect FBOne;          /*< Feedback unit vector */
    vect FBOne1;         /*< Secondary feedback unit vector */
    vect FBXk;           /*< Feedback state vector */
    vect FBTotal;        /*< Total feedback vector */

    int Rmaxcount[MAX_MAT_DIM];      /*< Counter for maximum measurement noise adaptation */
    int Rmaxcount0[MAX_MAT_DIM];     /*< Initial counter for maximum measurement noise adaptation */
    int innoMaxcount[MAX_MAT_DIM];   /*< Counter for maximum innovation adaptation */
    int innoMaxcount0;               /*< Initial counter for maximum innovation adaptation */
    int Zmmpk[MAX_MAT_DIM];          /*< Counter for measurement bounds adaptation */
    maxminn Zmm;                     /*< Max/min statistics for measurement innovation */

    KalmanFilter(void);
    KalmanFilter(int nq0, int nr0);
    /* initialize Kalman filter ---------------------------------------------------
    * initialize the Kalman filter with process and measurement noise dimensions
    * args   : int        nq0      I   dimension of process noise
    *          int        nr0      I   dimension of measurement noise
    * return : none
    *-----------------------------------------------------------------------------*/
    void Init(int nq0, int nr0);

    /* set measurement noise bounds and time constant ----------------------------
    * set the minimum and maximum measurement noise, bias, and time constant
    * args   : double     rmin     I   minimum measurement noise (default: 0.1)
    *          double     rmax     I   maximum measurement noise (default: INF)
    *          double     b        I   bias (default: INF)
    *          double     tau      I   time constant (default: INF)
    * return : none
    *-----------------------------------------------------------------------------*/
    void SetRmmbt(double rmin = 0.1f, double rmax = INF, double b = INF, double tau = INF);

    /* set maximum count for measurement noise ------------------------------------
    * set the maximum count for measurement noise adaptation
    * args   : int        cnt      I   maximum count (default: 5)
    * return : none
    *-----------------------------------------------------------------------------*/
    void SetRmaxcount(int cnt = 5);

    /* set maximum count for innovation -------------------------------------------
    * set the maximum count for innovation adaptation
    * args   : int        cnt      I   maximum count (default: 5)
    * return : none
    *-----------------------------------------------------------------------------*/
    void SetInnoMaxcount(int cnt = 5);

    /* set measurement bounds ----------------------------------------------------
    * set the maximum and minimum bounds for a specific measurement
    * args   : int        zi       I   measurement index
    *          int        pi       I   state index
    *          double     zmm0     I   initial measurement bound
    *          int        cnt      I   count for adaptation (default: 10)
    * return : none
    *-----------------------------------------------------------------------------*/
    void SetZmm(int zi, int pi, double zmm0, int cnt = 10);

    /* set velocity measurement bounds --------------------------------------------
    * set the maximum and minimum bounds for velocity measurements
    * args   : vect3      zmm0     I   initial velocity bounds
    *          int        cnt      I   count for adaptation (default: 10)
    * return : none
    *-----------------------------------------------------------------------------*/
    void SetZmmVn(const vect3& zmm0, int cnt = 10);

    /* set position measurement bounds --------------------------------------------
    * set the maximum and minimum bounds for position measurements
    * args   : vect3      zmm0     I   initial position bounds
    *          int        cnt      I   count for adaptation (default: 10)
    * return : none
    *-----------------------------------------------------------------------------*/
    void SetZmmPos(const vect3& zmm0, int cnt = 10);

    /* set process matrix ---------------------------------------------------------
    * set the process matrix for the Kalman filter
    * args   : int        nnq      I   size of the process noise matrix
    * return : none
    *-----------------------------------------------------------------------------*/
    virtual void SetFt(int nnq) = 0;

    /* set measurement matrix -----------------------------------------------------
    * set the measurement matrix for the Kalman filter
    * args   : int        nnq      I   size of the measurement matrix
    * return : none
    *-----------------------------------------------------------------------------*/
    virtual void SetHk(int nnq) = 0;

    /* set measurement ------------------------------------------------------------
    * set the measurement for the Kalman filter
    * args   : none
    * return : none
    *-----------------------------------------------------------------------------*/
    virtual void SetMeas(void) = 0;

    /* state feedback -------------------------------------------------------------
    * apply feedback to the system state based on Kalman filter results
    * args   : int        nnq      I   size of the state vector
    *          double     fbts     I   feedback time step
    * return : none
    *-----------------------------------------------------------------------------*/
    virtual void Feedback(int nnq, double fbts);

    /* Rt fading ------------------------------------------------------------------
    * increase Rt if no measurement is available
    * args   : int        i        I   index of the state
    *          double     fdts     I   fading time step
    * return : none
    *-----------------------------------------------------------------------------*/
    void RtFading(int i, double fdts);

    /* time update ----------------------------------------------------------------
    * perform time update for the Kalman filter
    * args   : double     kfts0    I   time step
    *          int        fback    I   feedback flag (default: 1)
    * return : none
    *-----------------------------------------------------------------------------*/
    void TimeUpdate(double kfts0, int fback = 1);

    /* measurement update ---------------------------------------------------------
    * perform measurement update for the Kalman filter
    * args   : double     fading   I   fading factor (default: 1.0)
    * return : int                 O   update status
    *-----------------------------------------------------------------------------*/
    int MeasUpdate(double fading = 1.0);

    /* Rt adaptive ----------------------------------------------------------------
    * adaptively adjust Rt based on measurement
    * args   : int        i        I   index of the state
    *          double     r        I   measurement noise
    *          double     Pr       I   process noise
    * return : int                 O   adaptation status
    *-----------------------------------------------------------------------------*/
    int RAdaptive(int i, double r, double Pr);

    /* multiple fading ------------------------------------------------------------
    * apply multiple fading to the Kalman filter
    * args   : int        i        I   index of the state
    * return : none
    *-----------------------------------------------------------------------------*/
    void RPkFading(int i);

    /* Z_max_min for Pk setting ---------------------------------------------------
    * set the maximum and minimum bounds for Pk
    * args   : int        i        I   index of the state
    * return : none
    *-----------------------------------------------------------------------------*/
    void ZmmPkSet(int i);

    /* set measurement flag -------------------------------------------------------
    * set the measurement flag for a specific type
    * args   : unsigned int flag   I   measurement flag
    *          int        type     I   type of measurement (default: 1)
    * return : none
    *-----------------------------------------------------------------------------*/
    void SetMeasFlag(unsigned int flag, int type = 1);

    /* set measurement mask -------------------------------------------------------
    * set the measurement mask for a specific type
    * args   : unsigned int mask   I   measurement mask
    *          int        type     I   type of measurement (default: 1)
    * return : none
    *-----------------------------------------------------------------------------*/
    void SetMeasMask(unsigned int mask, int type = 1);

    /* set measurement stop condition ---------------------------------------------
    * set the stop condition for a specific measurement
    * args   : unsigned int meas   I   measurement index
    *          double     stop     I   stop threshold (default: 10.0)
    * return : none
    *-----------------------------------------------------------------------------*/
    void SetMeasStop(unsigned int meas, double stop = 10.0f);

    /* set adaptive stop condition ------------------------------------------------
    * set the adaptive stop condition for a specific measurement
    * args   : unsigned int meas   I   measurement index
    *          double     stop     I   stop threshold (default: 10.0)
    * return : none
    *-----------------------------------------------------------------------------*/
    void SetRadptStop(unsigned int meas, double stop = 10.0f);

    /* constrain state and covariance ---------------------------------------------
    * constrain the state vector and covariance matrix within specified bounds
    * args   : none
    * return : none
    *-----------------------------------------------------------------------------*/
    void XPConstrain(void);

    /* check covariance bounds ----------------------------------------------------
    * check if the covariance matrix is within specified bounds
    * args   : none
    * return : none
    *-----------------------------------------------------------------------------*/
    void PmaxPminCheck(void);

    /* perform measurement update using UD decomposition --------------------------
    * perform a measurement update in the Kalman filter using UD decomposition
    * args   : double     U[]      IO  upper triangular matrix of covariance
    *          double     D[]      IO  diagonal matrix of covariance
    *          const double H[]    I   measurement matrix
    *          double     R        I   measurement noise covariance
    *          double     K[]      O   Kalman gain
    *          int        n        I   dimension of the state vector
    * return : none
    *-----------------------------------------------------------------------------*/
    friend void MeasUD(double U[], double D[], const double H[], double R, double K[], int n);

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
    friend void fusion(double* x1, double* p1, const double* x2, const double* p2, int n, double* xf, double* pf);

    /* fusion of two vect3 states and covariances ---------------------------------
    * perform fusion of two vect3 states and their covariances
    * args   : vect3&     x1       IO  state vector 1
    *          vect3&     p1       IO  covariance matrix 1
    *          const vect3 x2      I   state vector 2
    *          const vect3 p2      I   covariance matrix 2
    * return : none
    *-----------------------------------------------------------------------------*/
    friend void fusion(vect3& x1, vect3& p1, const vect3 x2, const vect3 p2);

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
    friend void fusion(vect3& x1, vect3& p1, const vect3 x2, const vect3 p2, vect3& xf, vect3& pf);

};

/**
 * @class EKFTDKF
 * @ingroup filter_core
 * @brief Extended Kalman Filter with Time-Distributed Kalman Filter logic.
 */
class EKFTDKF :public KalmanFilter
{
public:
    double meantdts;         /*< Mean time-distributed time step */
    double tdts;             /*< Time-distributed time step */
    double Pz0;              /*< Initial measurement covariance */
    double innovation;       /*< Innovation value (measurement residual) */
    double Kkp2y;            /*< Kalman gain for position to yaw */
    double Kkv2y;            /*< Kalman gain for velocity to yaw */
    int iter;                /*< Iteration counter */
    int ifn;                 /*< Index for function or state */
    int cststt;              /*< First constant state index */
    int hi1[MAX_MAT_DIM];    /*< Index array for measurement selection */
    int adptOKi;             /*< Adaptive OK flag or index */
    int measRes;             /*< Measurement result/status */
    int tdStep;              /*< Current time-distributed step */
    int maxStep;             /*< Maximum number of time-distributed steps */
    int curOutStep;          /*< Current output step */
    int maxOutStep;          /*< Maximum output step */
    mat Fk;                  /*< State transition matrix for current step */
    mat Pk1;                 /*< State covariance matrix for previous step */
    vect Pxz;                /*< Cross-covariance vector */
    vect Qk;                 /*< Process noise covariance vector for current step */
    vect Kk;                 /*< Kalman gain vector */
    vect Hi;                 /*< Measurement matrix row vector */
    vect tmeas;              /*< Measurement time vector */
    vect3 meanfn;            /*< Mean value of specific force or other vector */

    unsigned int timcnt0;    /*< Time counter 0 (for calculation-burden analysis) */
    unsigned int timcnt1;    /*< Time counter 1 (for calculation-burden analysis) */
    unsigned int timcntmax;  /*< Maximum time counter (for calculation-burden analysis) */
    double burden;           /*< Calculation burden indicator */

    EKFTDKF(void);
    EKFTDKF(int nq0, int nr0);

    /* initialize the Kalman system ---------------------------------------------------
    * initialize the INSKalman system with initial state
    * return : none
    *-----------------------------------------------------------------------------*/
    void Init();

    /* reset the Time-Distributed Kalman filter -----------------------------------
    * reset the Time-Distributed Kalman filter
    * args   : none
    * return : none
    *-----------------------------------------------------------------------------*/
    void TDReset(void);

    /* calculate innovation for a specific row ------------------------------------
    * calculate the innovation value for a specific row in the Kalman filter
    * args   : int        row      I   row index
    * return : innovation value
    *-----------------------------------------------------------------------------*/
    double Innovationi(int row);

    /* perform Time-Distributed Update --------------------------------------------
    * perform a Time-Distributed Update using sensor measurements
    * args   : vect3*     pwm      I   angular velocity measurements
    *          vect3*     pvm      I   linear velocity measurements
    *          int        nSamples I   number of samples
    *          double     ts       I   time step
    *          int        nStep    I   number of steps (default: 1)
    * return : update status
    *-----------------------------------------------------------------------------*/
    int TDUpdate(int nStep = 1);

    /* perform measurement update -------------------------------------------------
    * perform a measurement update in the Kalman filter
    * args   : vect       Hi       I   measurement matrix
    *          double     Ri       I   measurement noise covariance
    *          double     Zi       I   measurement value
    * return : none
    *-----------------------------------------------------------------------------*/
    void MeasUpdate(const vect& Hi, double Ri, double Zi);

    /* set vertical channel covariance ---------------------------------------------
    * set the covariance for the vertical channel in the Kalman filter
    * args   : double     sph      I   covariance for position
    *          double     spv      I   covariance for velocity (default: 0.0)
    *          double     spd      I   covariance for distance (default: 0.0)
    * return : none
    *-----------------------------------------------------------------------------*/
    void PSetVertCh(double sph, double spv = 0.0, double spd = 0.0);

    /* set calculation burden ------------------------------------------------------
    * set the calculation burden for the Kalman filter
    * args   : unsigned int timcnt I   time count
    *          int        itype    I   type of calculation
    * return : calculation burden
    *-----------------------------------------------------------------------------*/
    double SetCalcuBurden(unsigned int timcnt, int itype);

    /* real-time output ------------------------------------------------------------
    * perform real-time output before Kalman filter update
    * args   : none
    * return : none
    *-----------------------------------------------------------------------------*/
    virtual void RTOutput(void) {};

    /* miscellaneous operations ----------------------------------------------------
    * perform miscellaneous operations
    * args   : none
    * return : none
    *-----------------------------------------------------------------------------*/
    virtual void Miscellanous(void) {};

    /* secret attitude operations --------------------------------------------------
    * perform secret attitude-related operations
    * args   : none
    * return : none
    *-----------------------------------------------------------------------------*/
    virtual void SecretAttitude(void) {};
};

/**
 * @class SINSGNSSLOOSE
 * @ingroup nav_core
 * @brief Loosely coupled SINS/GNSS integration filter.
 */
class SINSGNSSLOOSE :public EKFTDKF
{
public:
    double kfts;                /*< Kalman filter time step (seconds) */
    double posGNSSdelay;        /*< GNSS position measurement delay (seconds) */
    double vnGNSSdelay;         /*< GNSS velocity measurement delay (seconds) */
    double yawGNSSdelay;        /*< GNSS yaw measurement delay (seconds) */
    double dtGNSSdelay;         /*< GNSS time delay (seconds) */
    double dyawGNSS;            /*< GNSS yaw difference (radians) */
    double* gnssLost;           /*< Pointer to GNSS lost status array */
    int yawHkRow;               /*< Row index for yaw in measurement matrix */
    int fixLast;                /*< Last fix status */
    int fixLost;                /*< Lost fix status */
    int nofixLost;              /*< Lost no-fix status */
    int nofixLast;              /*< Last no-fix status */
    int gnssLast;               /*< Last GNSS status */
    vect3 lvGNSS;               /*< GNSS lever arm vector */
    STA_AVPI avpi;              /*< Attitude/velocity/position buffer */
    INS sins;                   /*< Inertial navigation system state */

    SINSGNSSLOOSE(void);
    SINSGNSSLOOSE(int nq0, int nr0, double ts, int yawHkRow0 = 6);
    /* initialize the INS system ---------------------------------------------------
    * initialize the INS system with initial state
    * args   : INS        sins0    I   initial INS state
    *          int        grade    I   grade level (default: -1)
    * return : none
    *-----------------------------------------------------------------------------*/
    void Init(const INS& sins0, int grade = -1);

    /* set process noise matrix ---------------------------------------------------
    * set the process noise matrix for the Kalman filter
    * args   : int        nnq      I   size of the process noise matrix
    * return : none
    *-----------------------------------------------------------------------------*/
    virtual void SetFt(int nnq);

    /* set measurement matrix -----------------------------------------------------
    * set the measurement matrix for the Kalman filter
    * args   : int        nnq      I   size of the measurement matrix
    * return : none
    *-----------------------------------------------------------------------------*/
    virtual void SetHk(int nnq);

    /* apply feedback to the system state -----------------------------------------
    * apply feedback to the system state based on Kalman filter results
    * args   : int        nnq      I   size of the state vector
    *          double     fbts     I   feedback time step
    * return : none
    *-----------------------------------------------------------------------------*/
    virtual void Feedback(int nnq, double fbts);

    /* set GNSS measurements ------------------------------------------------------
    * set GNSS position, velocity, and yaw measurements
    * args   : vect3      posgnss  I   GNSS position {x, y, z} (default: O31)
    *          vect3      vngnss   I   GNSS velocity {vx, vy, vz} (default: O31)
    *          double     yawgnss  I   GNSS yaw angle (default: 0.0)
    * return : none
    *-----------------------------------------------------------------------------*/
    void SetMeasGNSS(const vect3& posgnss = O31, const vect3& vngnss = O31, double yawgnss = 0.0);

    /* measure GNSS zero-velocity stop --------------------------------------------
    * measure GNSS zero-velocity stop based on a threshold
    * args   : vect3      dvnth    O   velocity difference threshold
    *          double     stop     I   stop threshold (default: 5.0)
    * return : none
    *-----------------------------------------------------------------------------*/
    void MeasGNSSZvStop(vect3& dvnth, double stop = 5.0);

    /* measure GNSS zero-position stop --------------------------------------------
    * measure GNSS zero-position stop based on a threshold
    * args   : vect3      dposth   O   position difference threshold
    *          double     stop     I   stop threshold (default: 5.0)
    * return : none
    *-----------------------------------------------------------------------------*/
    void MeasGNSSZpStop(vect3& dposth, double stop = 5.0);

    /* measure GNSS zero-position to X-axis ---------------------------------------
    * measure GNSS zero-position and project it to the X-axis
    * args   : vect3      dposth   O   position difference threshold
    * return : none
    *-----------------------------------------------------------------------------*/
    void MeasGNSSZp2X(vect3& dposth);

    /* update the system state ----------------------------------------------------
    * update the system state using sensor measurements
    * args   : vect3*     pwm      I   angular velocity measurements
    *          vect3*     pvm      I   linear velocity measurements
    *          int        nSamples I   number of samples
    *          double     ts       I   time sep
    *          int        nSteps   I   number of steps (default: 5)
    * return : update status
    *-----------------------------------------------------------------------------*/
    int Update(const vect3* pwm, const vect3* pvm, int nSamples, double ts, int nSteps = 5);

    /* set Markov process for gyroscope -------------------------------------------
    * set the Markov process parameters for the gyroscope
    * args   : vect3      tauG     I   correlation time for gyroscope
    *          vect3      sRG      I   noise standard deviation for gyroscope
    *          int        stateeb  I   state index for gyroscope bias (default: 9)
    * return : none
    *-----------------------------------------------------------------------------*/
    void MarkovGyro(const vect3& tauG, const vect3& sRG, int stateeb = 9);

    /* set Markov process for accelerometer ----------------------------------------
    * set the Markov process parameters for the accelerometer
    * args   : vect3      tauA     I   correlation time for accelerometer
    *          vect3      sRA      I   noise standard deviation for accelerometer
    *          int        statedb  I   state index for accelerometer bias (default: 12)
    * return : none
    *-----------------------------------------------------------------------------*/
    void MarkovAcc(const vect3& tauA, const vect3& sRA, int statedb = 12);

    /* set yaw angle ---------------------------------------------------------------
    * set the yaw angle in the Kalman filter state
    * args   : double     yaw      I   yaw angle (radians)
    *          int        statephi I   state index for attitude (default: 0)
    *          int        statedvn I   state index for velocity (default: 3)
    * return : none
    *-----------------------------------------------------------------------------*/
    void SetYaw(double yaw, int statephi = 0, int statedvn = 3);
};

/**
 * @class SINSGNSSBorad
 * @ingroup nav_core
 * @brief SINS/GNSS integration with odometry and advanced constraints.
 */
class SINSGNSSBorad :public SINSGNSSLOOSE
{
public:
    int distK0;                  /*< Distance index for odometry distances array */
    int badODCnt;                /*< Counter for bad odometry measurements */
    int config_od;               /*< Odometry configuration flag or mode */
    double* odLost;              /*< Pointer to odometry lost status array */
    double gnssLostdist;         /*< Distance when GNSS is lost */
    double gnssLostnofixdist;    /*< Distance when GNSS has no fix */
    double distance_trj;         /*< Total trajectory distance */
    vect3 vn_pre;                /*< Previous velocity vector */
    vect3 pos_pre;
    ZIHR wzhd;                   /*< Zero-Integrated Heading Rate (ZIHR) detector */
    odo od;                      /*< Odometry data object */
    odo od_nhc;                  /*< Odometry data object for non-holonomic constraint */


    SINSGNSSBorad(void);
    SINSGNSSBorad(int nq0, int nr0, double ts, int yawHkRow0 = 15);
    /* initialize the INS system ---------------------------------------------------
    * initialize the INS system with initial state
    * args   : INS        sins0    I   initial INS state
    *          int        grade    I   grade level (default: -1)
    * return : none
    *-----------------------------------------------------------------------------*/
    void Init(const INS& sins0, int grade = -1);

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
    int Update(const vect3* pwm, const vect3* pvm, double dS, int nSamples, double ts, int nSteps = 6, bool isBack = 0);

    /* set process noise matrix ---------------------------------------------------
    * set the process noise matrix for the Kalman filter
    * args   : int        nnq      I   size of the process noise matrix
    * return : none
    *-----------------------------------------------------------------------------*/
    virtual void SetFt(int nnq);

    /* set measurement matrix -----------------------------------------------------
    * set the measurement matrix for the Kalman filter
    * args   : int        nnq      I   size of the measurement matrix
    * return : none
    *-----------------------------------------------------------------------------*/
    virtual void SetHk(int nnq);

    /* apply feedback to the system state -----------------------------------------
    * apply feedback to the system state based on Kalman filter results
    * args   : int        nnq      I   size of the state vector
    *          double     fbts     I   feedback time step
    * return : none
    *-----------------------------------------------------------------------------*/
    virtual void Feedback(int nnq, double fbts);

    /* test zero-velocity update (ZUPT) -------------------------------------------
    * perform a zero-velocity update test
    * args   : none
    * return : none
    *-----------------------------------------------------------------------------*/
    void ZUPTtest(int flag_lost);

    /* test zero-integrated heading rate (ZIHR) -----------------------------------
    * perform a zero-integrated heading rate test
    * args   : none
    * return : none
    *-----------------------------------------------------------------------------*/
    void ZIHRtest(void);

    /* set GNSS fix mode ----------------------------------------------------------
    * set the GNSS fix mode
    * args   : int        mode     I   GNSS fix mode (default: 0)
    * return : none
    *-----------------------------------------------------------------------------*/
    void SetGNSSFixMode(int mode = 0);
};

#ifdef MEMORY
#include <malloc.h>
/* RingMemory class implementation ---------------------------------------------
* Implements a circular buffer for storing fixed-size records.
* Provides constructors for default initialization, memory allocation,
* and using an external memory buffer. Supports push and pop operations
* with automatic wrap-around and overwrite of oldest data when full.
* Methods:
*   - RingMemory(void): Default constructor, initializes pointers to NULL.
*   - RingMemory(long recordNum, int recordLen0): Allocates memory for the buffer.
*   - RingMemory(uint8_t *pMem, long memLen0, int recordLen0): Uses external memory.
*   - ~RingMemory(): Destructor, frees allocated memory if owned.
*   - uint8_t pop(uint8_t *p): Pops one record from the buffer.
*   - uint8_t* get(int iframe): Gets pointer to the specified frame.
*   - uint8_t* set(int iframe, const uint8_t *p): Sets data for the specified frame.
*   - uint8_t push(const uint8_t *p): Pushes one record into the buffer.
*-----------------------------------------------------------------------------*/
#define MAX_RECORD_BYTES 1024
class RingMemory
{
public:
    uint8_t* pMemStart0;         /*< Pointer to the original start of the memory buffer */
    uint8_t* pMemStart;          /*< Pointer to the start of the usable memory buffer */
    uint8_t* pMemEnd;            /*< Pointer to the end of the memory buffer */
    int pushLen;                 /*< Length of data to push (bytes) */
    int popLen;                  /*< Length of data to pop (bytes) */
    int recordLen;               /*< Length of each record (bytes) */
    long memLen;                 /*< Total length of the memory buffer (bytes) */
    long dataLen;                /*< Current length of valid data in the buffer (bytes) */
    uint8_t* pMemPush;           /*< Pointer to the current push position in the buffer */
    uint8_t* pMemPop;            /*< Pointer to the current pop position in the buffer */
    uint8_t pushBuf[MAX_RECORD_BYTES]; /*< Temporary buffer for pushing data */
    uint8_t popBuf[MAX_RECORD_BYTES];  /*< Temporary buffer for popping data */

    /* default constructor --------------------------------------------------------
    * initialize pointers and buffer to NULL/zero
    * args   : none
    * return : none
    *-----------------------------------------------------------------------------*/
    RingMemory(void);

    /* allocate memory for the buffer ---------------------------------------------
    * allocate internal memory for a given number of records
    * args   : long       recordNum   I   number of records
    *          int        recordLen0  I   length of each record (bytes)
    * return : none
    *-----------------------------------------------------------------------------*/
    RingMemory(long recordNum, int recordLen0);

    /* use external memory buffer -------------------------------------------------
    * initialize with an external memory buffer
    * args   : uint8_t*   pMem        I   pointer to external memory
    *          long       memLen0     I   length of memory (bytes)
    *          int        recordLen0  I   length of each record (bytes, default: 0)
    * return : none
    *-----------------------------------------------------------------------------*/
    RingMemory(uint8_t* pMem, long memLen0, int recordLen0 = 0);

    /* destructor -----------------------------------------------------------------
    * free allocated memory if owned
    * args   : none
    * return : none
    *-----------------------------------------------------------------------------*/
    ~RingMemory();

    /* push one record into the buffer --------------------------------------------
    * push a record into the circular buffer, overwrite oldest if full
    * args   : const uint8_t* p   I   pointer to data (default: NULL)
    * return : status (1: success, 0: failure)
    *-----------------------------------------------------------------------------*/
    uint8_t push(const uint8_t* p = (const uint8_t*)NULL);

    /* pop one record from the buffer ---------------------------------------------
    * pop a record from the circular buffer
    * args   : uint8_t*   p       O   pointer to output buffer (default: NULL)
    * return : status (1: success, 0: failure)
    *-----------------------------------------------------------------------------*/
    uint8_t pop(uint8_t* p = (uint8_t*)NULL);

    /* get pointer to specified frame ---------------------------------------------
    * get pointer to the specified frame in the buffer
    * args   : int        iframe  I   frame index
    * return : pointer to frame data
    *-----------------------------------------------------------------------------*/
    uint8_t* get(int iframe);

    /* set data for specified frame -----------------------------------------------
    * set data for the specified frame in the buffer
    * args   : int        iframe  I   frame index
    *          const uint8_t* p   I   pointer to data
    * return : pointer to frame data
    *-----------------------------------------------------------------------------*/
    uint8_t* set(int iframe, const uint8_t* p);
};

/* Smooth class implementation ------------------------------------------------
* Implements a moving average (sliding window) filter for vector data.
* Uses a RingMemory buffer to store recent vectors and efficiently
* computes the mean as new data arrives.
* Methods:
*   - Smooth(int clm, int row): Constructor, initializes buffer and accumulators.
*   - ~Smooth(): Destructor, releases buffer memory.
*   - vect Update(const double *p, double *pmean): Updates the filter with new data,
*     computes and returns the current mean.
*-----------------------------------------------------------------------------*/
class SmoothAVP
{
public:
    RingMemory* pmem;    /*< Pointer to the ring memory buffer for storing recent vectors */
    vect sum;            /*< Sum of vectors in the current window */
    vect mean;           /*< Mean vector of the current window */
    vect tmp;            /*< Temporary vector for intermediate calculations */
    int irow;            /*< Current row index in the buffer */
    int maxrow;          /*< Maximum number of rows (window size) */
    /* constructor ---------------------------------------------------------------
    * initialize the moving average filter with buffer size
    * args   : int        clm     I   number of columns (vector dimension, default: MAX_MAT_DIM)
    *          int        row     I   window size (default: 100)
    * return : none
    *-----------------------------------------------------------------------------*/
    SmoothAVP(int clm = MAX_MAT_DIM, int row = 100);

    /* destructor -----------------------------------------------------------------
    * release buffer memory
    * args   : none
    * return : none
    *-----------------------------------------------------------------------------*/
    ~SmoothAVP();

    /* update the filter with new data --------------------------------------------
    * add new vector data, update sum and mean, return current mean
    * args   : const double* p    I   pointer to new data
    *          double*       pmean O   pointer to output mean (optional)
    * return : current mean vector
    *-----------------------------------------------------------------------------*/
    vect Update(const double* p, double* pmean = NULL);
};
#endif

#ifdef FILEIO
#include <stdio.h>
#include <io.h>
class CFileRdWt
{
public:
    static char dirIn[256];      /*< Input directory path */
    static char dirOut[256];     /*< Output directory path */
    FILE* rwf;                   /*< File pointer for reading/writing */
    char fname[256];             /*< File name */
    char line[512];              /*< Buffer for reading a line from file */
    char sstr[64 * 4];             /*< Temporary string buffer */
    double buff[64];             /*< Buffer for double data */
    float buff32[64];            /*< Buffer for float data */
    int columns;                 /*< Number of columns in the file */
    int linek;                   /*< Current line index */
    int items[3];                /*< Array for storing item counts or indices */
    long totsize;                /*< Total file size */
    long remsize;                /*< Remaining file size */
    long svpos[3];               /*< Saved file positions for restoring */

    /* set input directory --------------------------------------------------------
    * set the input directory path
    * args   : const char* dirI   I   input directory path
    * return : none
    *-----------------------------------------------------------------------------*/
    static void DirI(const char* dirI);

    /* set output directory -------------------------------------------------------
    * set the output directory path
    * args   : const char* dirO   I   output directory path
    * return : none
    *-----------------------------------------------------------------------------*/
    static void DirO(const char* dirO);

    /* set input and output directories -------------------------------------------
    * set both input and output directory paths
    * args   : const char* dirI   I   input directory path
    *          const char* dirO   I   output directory path (optional)
    * return : none
    *-----------------------------------------------------------------------------*/
    static void Dir(const char* dirI, const char* dirO = (const char*)NULL);

    /* default constructor --------------------------------------------------------
    * initialize file reader/writer object with default values
    * args   : none
    * return : none
    *-----------------------------------------------------------------------------*/
    CFileRdWt(void);

    /* constructor with file name and columns -------------------------------------
    * initialize with file name and number of columns
    * args   : const char* fname0 I   file name
    *          int columns0       I   number of columns (default: 0)
    * return : none
    *-----------------------------------------------------------------------------*/
    CFileRdWt(const char* fname0, int columns0 = 0);

    /* destructor -----------------------------------------------------------------
    * close file and release resources
    * args   : none
    * return : none
    *-----------------------------------------------------------------------------*/
    ~CFileRdWt();

    /* initialize with file name and columns --------------------------------------
    * open file and set number of columns
    * args   : const char* fname0 I   file name
    *          int columns0       I   number of columns (default: 0)
    * return : none
    *-----------------------------------------------------------------------------*/
    void Init(const char* fname0, int columns0 = 0);

    /* load data from file (double) -----------------------------------------------
    * load specified number of lines from file as double data
    * args   : int lines          I   number of lines to load (default: 1)
    *          bool txtDelComma   I   whether to delete commas in text (default: 1)
    * return : number of lines loaded
    *-----------------------------------------------------------------------------*/
    int load(int lines = 1, bool txtDelComma = 1);

    /* load data from file (float) ------------------------------------------------
    * load specified number of lines from file as float data
    * args   : int lines          I   number of lines to load (default: 1)
    * return : number of lines loaded
    *-----------------------------------------------------------------------------*/
    int loadf32(int lines = 1);

    /* load binary data from file -------------------------------------------------
    * load binary data into buffer
    * args   : uint8_t* buf       O   buffer to load data into
    *          long bufsize       I   size of buffer
    * return : number of bytes loaded
    *-----------------------------------------------------------------------------*/
    long load(uint8_t* buf, long bufsize);

    /* seek in binary file --------------------------------------------------------
    * seek to a specific line in binary file
    * args   : int lines          I   number of lines to seek
    *          int mod            I   seek mode (default: SEEK_CUR)
    * return : none
    *-----------------------------------------------------------------------------*/
    void bwseek(int lines, int mod = SEEK_CUR);

    /* get file size --------------------------------------------------------------
    * get the size of the file
    * args   : int opt            I   option flag (default: 1)
    * return : file size in bytes
    *-----------------------------------------------------------------------------*/
    long filesize(int opt = 1);

    /* get a line from file -------------------------------------------------------
    * read a line from the file into buffer
    * args   : none
    * return : number of items read or status
    *-----------------------------------------------------------------------------*/
    int getl(void);

    /* save current file position -------------------------------------------------
    * save the current file position for later restoration
    * args   : int i              I   index for saving position (default: 0)
    * return : saved file position
    *-----------------------------------------------------------------------------*/
    long savepos(int i = 0);

    /* restore file position ------------------------------------------------------
    * restore file position to a previously saved position
    * args   : int i              I   index of saved position (default: 0)
    * return : status (0: success, -1: failure)
    *-----------------------------------------------------------------------------*/
    int restorepos(int i = 0);

    /* wait for value in column ---------------------------------------------------
    * wait until a value appears in the specified column within a tolerance
    * args   : int columnk        I   column index
    *          double val         I   value to wait for (default: 0.0)
    *          double eps         I   tolerance (default: EPS)
    * return : true if value found, false otherwise
    *-----------------------------------------------------------------------------*/
    bool waitfor(int columnk, double val = 0.0, double eps = EPS);
#ifdef MEMORY
    CFileRdWt& operator<<(const RingMemory& m);
#endif
};

int sim_open(char* file_name);
/* file to data*/
int load_data(sim32bin_t* data_sim);

int sim_close();

#endif

/** trace log function */
/* open trace log file ---------------------------------------------------------
* open a trace log file for writing debug or trace information
* args   : const char* file   I   file name to open for trace logging
* return : none
*-----------------------------------------------------------------------------*/
void traceopen(const char* file);

/* close trace log file --------------------------------------------------------
* close the currently open trace log file
* args   : none
* return : none
*-----------------------------------------------------------------------------*/
void traceclose(void);

/* set trace log level ---------------------------------------------------------
* set the verbosity level for trace logging
* args   : int        level   I   trace level (higher means more verbose)
* return : none
*-----------------------------------------------------------------------------*/
void tracelevel(int level);

/* write trace log message -----------------------------------------------------
* write a formatted message to the trace log at the specified level
* args   : int        level   I   trace level
*          const char* format I   printf-style format string
*          ...                I   variable arguments
* return : none
*-----------------------------------------------------------------------------*/
void trace(int level, const char* format, ...);

/* write trace log message with timestamp --------------------------------------
* write a formatted message with timestamp to the trace log at the specified level
* args   : int        level   I   trace level
*          const char* format I   printf-style format string
*          ...                I   variable arguments
* return : none
*-----------------------------------------------------------------------------*/
void tracet(int level, const char* format, ...);

#ifdef CONFIGSIM
/* search configuration by name ------------------------------------------------
* search for a configuration entry by name in a configuration array
* args   : const char* name      I   configuration name to search for
*          const CONFIG_t* configs I   array of configuration entries
* return : pointer to found CONFIG_t entry, or NULL if not found
*-----------------------------------------------------------------------------*/
CONFIG_t* searchconfigs(const char* name, const CONFIG_t* configs);

/* convert string to configuration ---------------------------------------------
* parse a string and set the configuration entry accordingly
* args   : CONFIG_t* config     O   pointer to configuration entry to set
*          const char* str      I   string to parse
* return : status (0: success, -1: error)
*-----------------------------------------------------------------------------*/
int str2config(CONFIG_t* config, const char* str);

/* convert configuration to string ---------------------------------------------
* convert a configuration entry to a string representation
* args   : const CONFIG_t* config I   configuration entry to convert
*          char* str              O   output string buffer
* return : status (0: success, -1: error)
*-----------------------------------------------------------------------------*/
int config2str(const CONFIG_t* config, char* str);

/* convert configuration to buffer ---------------------------------------------
* convert a configuration entry to a buffer (for binary or compact storage)
* args   : const CONFIG_t* config I   configuration entry to convert
*          char* buff             O   output buffer
* return : status (0: success, -1: error)
*-----------------------------------------------------------------------------*/
int config2buf(const CONFIG_t* config, char* buff);

/* load configurations from file -----------------------------------------------
* load an array of configuration entries from a file
* args   : const char* file      I   file name to load from
*          CONFIG_t* configs     O   array to store loaded configurations
* return : number of configurations loaded, or -1 on error
*-----------------------------------------------------------------------------*/
int loadconfigs(const char* file, CONFIG_t* configs);

/* save configurations to file -------------------------------------------------
* save an array of configuration entries to a file
* args   : const char* file      I   file name to save to
*          const char* mode      I   file open mode ("w", "a", etc.)
*          const char* comment   I   comment to write at the top of the file
*          const CONFIG_t* configs I   array of configurations to save
* return : number of configurations saved, or -1 on error
*-----------------------------------------------------------------------------*/
int saveconfigs(const char* file, const char* mode, const char* comment, const CONFIG_t* configs);
#endif
/* sign function ----------------------------------------------------------------
* determine the sign of a given value
* args   : double     val      I   input value
*          double     eps      I   threshold for determining zero (default: EPS)
* return : 1 if val > eps
*         -1 if val < -eps
*          0 if -eps <= val <= eps
*-----------------------------------------------------------------------------*/
int sign(double val, double eps = EPS);             /*< v.i v.j v.k < EPS */
/* check if a vector is approximately zero -------------------------------------
* check if all elements of a vector are within a small threshold of zero
* args   : vect3      v        I   input vector
*          double     eps      I   threshold for determining zero (default: EPS)
* return : 1 if all elements of v are within [-eps, eps]
*          0 otherwise
*-----------------------------------------------------------------------------*/
int IsZeros(const vect3& v, double eps = EPS);      /*< v.i v.j v.k < EPS */
/* check if a value is approximately zero --------------------------------------
* check if the given value is within a small threshold of zero
* args   : double     val      I   input value
*          double     eps      I   threshold for determining zero (default: EPS)
* return : 1 if -eps <= val <= eps
*          0 otherwise
*-----------------------------------------------------------------------------*/
int IsZero(const double& val, double eps = EPS);    /*< v.i v.j v.k < EPS */
/* calculate yaw difference ----------------------------------------------------
* calculate the difference between two yaw angles
* args   : double     yaw      I   first yaw angle (rad)
*          double     yaw0     I   second yaw angle (rad)
* return : difference between yaw and yaw0 (rad)
*-----------------------------------------------------------------------------*/
double diffYaw(double yaw, double yaw0);

/* calculate vector absolute value --------------------------------------------
* calculate the absolute value for each element of a vector
* args   : vect3      v        I   input vector
* return : vector with absolute values of each element
*-----------------------------------------------------------------------------*/
vect3 abs(const vect3& v);

/* calculate maximum absolute value -------------------------------------------
* calculate the maximum absolute value for each element between two vectors
* args   : vect3      v1       I   first input vector
*          vect3      v2       I   second input vector
* return : vector with maximum absolute values for each element
*-----------------------------------------------------------------------------*/
vect3 maxabs(const vect3& v1, const vect3& v2);

/* calculate vector norm ------------------------------------------------------
* calculate the Euclidean norm of a vector
* args   : vect3      v        I   input vector
* return : Euclidean norm of the vector
*-----------------------------------------------------------------------------*/
double norm(const vect3& v);

/* calculate vector infinity norm ---------------------------------------------
* calculate the infinity norm (maximum absolute value) of a vector
* args   : vect3      v        I   input vector
* return : infinity norm of the vector
*-----------------------------------------------------------------------------*/
double normInf(const vect3& v);

/* calculate XY components norm -----------------------------------------------
* calculate the Euclidean norm of the X and Y components of a vector
* args   : vect3      v        I   input vector
* return : Euclidean norm of the X and Y components
*-----------------------------------------------------------------------------*/
double normXY(const vect3& v);

/* calculate XY components infinity norm --------------------------------------
* calculate the infinity norm of the X and Y components of a vector
* args   : vect3      v        I   input vector
* return : infinity norm of the X and Y components
*-----------------------------------------------------------------------------*/
double normXYInf(const vect3& v);

/* calculate square root of vector elements -----------------------------------
* calculate the square root for each element of a vector
* args   : vect3      v        I   input vector
* return : vector with square root of each element
*-----------------------------------------------------------------------------*/
vect3 sqrt(const vect3& v);

/* calculate power of vector elements -----------------------------------------
* calculate the k-th power for each element of a vector
* args   : vect3      v        I   input vector
*          int        k        I   power to raise each element
* return : vector with each element raised to the k-th power
*-----------------------------------------------------------------------------*/
vect3 pow(const vect3& v, int k);

/* calculate dot product of two vectors ---------------------------------------
* calculate the dot product of two vectors
* args   : vect3      v1       I   first input vector
*          vect3      v2       I   second input vector
* return : dot product of the two vectors
*-----------------------------------------------------------------------------*/
double dot(const vect3& v1, const vect3& v2);

/* calculate element-wise product of two vectors ------------------------------
* calculate the element-wise product of two vectors
* args   : vect3      v1       I   first input vector
*          vect3      v2       I   second input vector
* return : vector with element-wise product of the two vectors
*-----------------------------------------------------------------------------*/
vect3 dotmul(const vect3& v1, const vect3& v2);

/* convert Euler angles to Direction Cosine Matrix (DCM) -----------------------
* convert a set of Euler angles to a Direction Cosine Matrix (DCM)
* args   : vect3      att      I   Euler angles {roll, pitch, yaw} (radians)
* return : Direction Cosine Matrix (DCM)
*-----------------------------------------------------------------------------*/
mat3 a2mat(const vect3& att);

/* convert Direction Cosine Matrix (DCM) to Euler angles -----------------------
* convert a Direction Cosine Matrix (DCM) to a set of Euler angles
* args   : mat3       Cnb      I   Direction Cosine Matrix (DCM)
* return : Euler angles {roll, pitch, yaw} (radians)
*-----------------------------------------------------------------------------*/
vect3 m2att(const mat3& Cnb);

/* convert Direction Cosine Matrix (DCM) to reversed Euler angles --------------
* convert a Direction Cosine Matrix (DCM) to reversed Euler angles (3-2-1 sequence)
* args   : mat3       Cnb      I   Direction Cosine Matrix (DCM)
* return : reversed Euler angles {yaw, pitch, roll} (radians)
*-----------------------------------------------------------------------------*/
vect3 m2attr(const mat3& Cnb);

/* convert quaternion to reversed Euler angles ---------------------------------
* convert a quaternion to reversed Euler angles (3-2-1 sequence)
* args   : quat       qnb      I   quaternion
* return : reversed Euler angles {yaw, pitch, roll} (radians)
*-----------------------------------------------------------------------------*/
vect3 q2attr(const quat& qnb);

/* convert Euler angles to quaternion ------------------------------------------
* convert a set of Euler angles to a quaternion
* args   : double     pitch    I   pitch angle (radians)
*          double     roll     I   roll angle (radians)
*          double     yaw      I   yaw angle (radians)
* return : quaternion
*-----------------------------------------------------------------------------*/
quat a2qua(double pitch, double roll, double yaw);

/* convert Euler angles to quaternion ------------------------------------------
* convert a set of Euler angles to a quaternion
* args   : vect3      att      I   Euler angles {roll, pitch, yaw} (radians)
* return : quaternion
*-----------------------------------------------------------------------------*/
quat a2qua(const vect3& att);

/* convert reversed Euler angles to Direction Cosine Matrix (DCM) --------------
* convert a set of reversed Euler angles to a Direction Cosine Matrix (DCM)
* args   : vect3      attr     I   reversed Euler angles {yaw, pitch, roll} (radians)
* return : Direction Cosine Matrix (DCM)
*-----------------------------------------------------------------------------*/
mat3 ar2mat(const vect3& attr);

/* convert reversed Euler angles to quaternion ---------------------------------
* convert a set of reversed Euler angles to a quaternion
* args   : vect3      attr     I   reversed Euler angles {yaw, pitch, roll} (radians)
* return : quaternion
*-----------------------------------------------------------------------------*/
quat ar2qua(const vect3& attr);

/* convert quaternion to Euler angles ------------------------------------------
* convert a quaternion to a set of Euler angles
* args   : quat       qnb      I   quaternion
* return : Euler angles {roll, pitch, yaw} (radians)
*-----------------------------------------------------------------------------*/
vect3 q2att(const quat& qnb);

/* convert rotation vector to quaternion ---------------------------------------
* convert a rotation vector to a quaternion
* args   : vect3      rv       I   rotation vector
* return : quaternion
*-----------------------------------------------------------------------------*/
quat rv2q(const vect3& rv);

/* convert rotation vector to Direction Cosine Matrix (DCM) --------------------
* convert a rotation vector to a Direction Cosine Matrix (DCM)
* args   : vect3      rv       I   rotation vector
* return : Direction Cosine Matrix (DCM)
*-----------------------------------------------------------------------------*/
mat3 rv2m(const vect3& rv);

/* convert quaternion to rotation vector ---------------------------------------
* convert a quaternion to a rotation vector
* args   : quat       q        I   quaternion
* return : rotation vector
*-----------------------------------------------------------------------------*/
vect3 q2rv(const quat& q);

/* calculate askew-symmetric matrix --------------------------------------------
* calculate the askew-symmetric matrix of a vector
* args   : vect3      v        I   input vector
* return : askew-symmetric matrix
*-----------------------------------------------------------------------------*/
mat3 askew(const vect3& v);

/* calculate sine of angle between two vectors ---------------------------------
* calculate the absolute value of sine of the angle between two vectors
* args   : vect3      v1       I   first input vector
*          vect3      v2       I   second input vector
* return : |sin(angle(v1, v2))|
*-----------------------------------------------------------------------------*/
double sinAng(const vect3& v1, const vect3& v2);

/* calculate first-order Markov white-noise variance ---------------------------
* calculate the first-order Markov white-noise variance
* args   : vect3      sR       I   noise standard deviation
*          vect3      tau      I   correlation time
* return : Markov white-noise variance
*-----------------------------------------------------------------------------*/
vect3 MKQt(const vect3& sR, const vect3& tau);

/* determine level attitude using single vector --------------------------------
* determine the level attitude using a single vector
* args   : vect3      fb       I   body-frame vector
*          double     yaw0     I   initial yaw angle (radians)
*          vect3      fn       I   navigation-frame vector
* return : level attitude {roll, pitch, yaw} (radians)
*-----------------------------------------------------------------------------*/
vect3 sv2att(const vect3& fb, double yaw0, const vect3& fn);

/* determine attitude using double vectors -------------------------------------
* determine the attitude using two pairs of vectors
* args   : vect3      va1      I   first vector in frame A
*          vect3      va2      I   second vector in frame A
*          vect3      vb1      I   first vector in frame B
*          vect3      vb2      I   second vector in frame B
* return : attitude {roll, pitch, yaw} (radians)
*-----------------------------------------------------------------------------*/
vect3 dv2att(const vect3& va1, const vect3& va2, const vect3& vb1, const vect3& vb2);

/* determine attitude using multiple vectors -----------------------------------
* determine the attitude using multiple pairs of vectors
* args   : int        n        I   number of vector pairs
*          vect3*     vai      I   vectors in frame A
*          vect3*     vbi      I   vectors in frame B
*          ...                 I   additional arguments
* return : attitude {roll, pitch, yaw} (radians)
*-----------------------------------------------------------------------------*/
vect3 mv2att(int n, const vect3* vai, const vect3* vbi, ...);

/* convert ENU velocity to attitude --------------------------------------------
* convert East-North-Up (ENU) velocity to attitude (pitch and yaw)
* args   : vect3      vn       I   ENU velocity vector
* return : attitude {pitch, yaw} (radians)
*-----------------------------------------------------------------------------*/
vect3 vn2att(const vect3& vn);

/* convert ENU velocity components to attitude ---------------------------------
* convert East and North velocity components to attitude (pitch and yaw)
* args   : double     vel_east I   East velocity component
*          double     vel_north I  North velocity component
* return : attitude {pitch, yaw} (radians)
*-----------------------------------------------------------------------------*/
double vn2att(const double vel_east, const double vel_north);

/* align in static base --------------------------------------------------------
* perform alignment in a static base
* args   : vect3      wmm      I   measured angular velocity
*          vect3      vmm      I   measured linear velocity
*          double     latitude I   latitude (radians)
* return : alignment result {roll, pitch, yaw} (radians)
*-----------------------------------------------------------------------------*/
vect3 Alignsb(const vect3 wmm, const vect3 vmm, double latitude);

/* calculate magnetic yaw ------------------------------------------------------
* calculate the yaw angle based on magnetic field measurements
* args   : vect3      mag      I   magnetic field vector
*          vect3      att      I   attitude {roll, pitch, yaw} (radians)
*          double     declination I magnetic declination (radians)
* return : magnetic yaw angle (radians)
*-----------------------------------------------------------------------------*/
double MagYaw(const vect3& mag, const vect3& att, double declination);

/* convert ECEF X/Y/Z to latitude/longitude/height ----------------------------
* convert a point in ECEF coordinates to geodetic coordinates
* args   : vect3      xyz      I   ECEF coordinates {X, Y, Z} (meters)
* return : geodetic coordinates {latitude, longitude, height} (radians, radians, meters)
*-----------------------------------------------------------------------------*/
vect3 xyz2blh(const vect3& xyz);

/* convert latitude/longitude/height to ECEF X/Y/Z ----------------------------
* convert a point in geodetic coordinates to ECEF coordinates
* args   : vect3      blh      I   geodetic coordinates {latitude, longitude, height} (radians, radians, meters)
* return : ECEF coordinates {X, Y, Z} (meters)
*-----------------------------------------------------------------------------*/
vect3 blh2xyz(const vect3& blh);

/* convert ECEF velocity to ENU velocity --------------------------------------
* convert a velocity vector in ECEF coordinates to ENU coordinates
* args   : vect3      Vxyz     I   velocity in ECEF coordinates {Vx, Vy, Vz} (m/s)
*          vect3      pos      I   geodetic position {latitude, longitude, height} (radians, radians, meters)
* return : velocity in ENU coordinates {Ve, Vn, Vu} (m/s)
*-----------------------------------------------------------------------------*/
vect3 Vxyz2enu(const vect3& Vxyz, const vect3& pos);

/* generate random vector -----------------------------------------------------
* generate a random vector with given mean and standard deviation
* args   : vect3      mu       I   mean vector
*          vect3      sigma    I   standard deviation vector
* return : random vector
*-----------------------------------------------------------------------------*/
vect3 randn(const vect3& mu, const vect3& sigma);

/* sort vector elements -------------------------------------------------------
* sort the elements of a vector in ascending order
* args   : vect3      v        I   input vector
* return : sorted vector
*-----------------------------------------------------------------------------*/
vect3 sort(const vect3& v);

/* calculate attraction vector ------------------------------------------------
* calculate the attraction vector based on a threshold and center
* args   : vect3      v        I   input vector
*          vect3      th       I   threshold vector
*          vect3      center   I   center vector
* return : attraction vector
*-----------------------------------------------------------------------------*/
vect3 attract(const vect3& v, const vect3& th, const vect3& center);

/* calculate force-field vector ------------------------------------------------
* calculate the force-field vector based on two input vectors and directions
* args   : vect3      fA       I   first input vector
*          vect3      fB       I   second input vector
*          char*      dir0     I   direction string for fA
*          char*      dir1     I   direction string for fB
* return : force-field vector
*-----------------------------------------------------------------------------*/
vect3 ff2muxy(const vect3& fA, const vect3& fB, const char* dir0, const char* dir1);

/* calculate column-vector multiplied by row-vector ---------------------------
* calculate the product of a column-vector and a row-vector
* args   : vect3      v1       I   column vector
*          vect3      v2       I   row vector
* return : resulting matrix
*-----------------------------------------------------------------------------*/
mat3 vxv(const vect3& v1, const vect3& v2);

/* calculate trace of matrix multiplication -----------------------------------
* calculate the trace of the product of two matrices
* args   : mat3      m1       I   first input matrix
*          mat3      m2       I   second input matrix (default: identity matrix)
* return : trace of the product of m1 and m2
*-----------------------------------------------------------------------------*/
double trMMT(const mat3& m1, const mat3& m2 = I33);

/* symmetrize a matrix --------------------------------------------------------
* symmetrize a given matrix
* args   : mat3      m        IO  input/output matrix to be symmetrized
* return : none
*-----------------------------------------------------------------------------*/
void symmetry(mat3& m);

/* calculate k-th power of a matrix -------------------------------------------
* calculate the k-th power of a matrix
* args   : mat3      m        I   input matrix
*          int       k        I   power to raise the matrix
* return : matrix raised to the k-th power
*-----------------------------------------------------------------------------*/
mat3 pow(const mat3& m, int k);

/* calculate trace of a matrix ------------------------------------------------
* calculate the trace of a matrix
* args   : mat3      m        I   input matrix
* return : trace of the matrix
*-----------------------------------------------------------------------------*/
double trace(const mat3& m);

/* calculate determinant of a matrix ------------------------------------------
* calculate the determinant of a matrix
* args   : mat3      m        I   input matrix
* return : determinant of the matrix
*-----------------------------------------------------------------------------*/
double det(const mat3& m);

/* calculate adjoint of a matrix ----------------------------------------------
* calculate the adjoint of a 3x3 matrix
* args   : mat3      m        I   input matrix
* return : adjoint of the matrix
*-----------------------------------------------------------------------------*/
mat3 adj(const mat3& m);

/* calculate inverse of a matrix ----------------------------------------------
* calculate the inverse of a 3x3 matrix
* args   : mat3      m        I   input matrix
* return : inverse of the matrix
*-----------------------------------------------------------------------------*/
mat3 inv(const mat3& m);

/* extract diagonal of a matrix -----------------------------------------------
* extract the diagonal elements of a matrix as a vector
* args   : mat3      m        I   input matrix
* return : vector containing the diagonal elements of the matrix
*-----------------------------------------------------------------------------*/
vect3 diag(const mat3& m);

/* create diagonal matrix from a vector ---------------------------------------
* create a diagonal matrix from a vector
* args   : vect3     v        I   input vector
* return : diagonal matrix
*-----------------------------------------------------------------------------*/
mat3 diag(const vect3& v);

/* calculate askew-symmetric matrix -------------------------------------------
* calculate the askew-symmetric matrix of a matrix
* args   : mat3      m        I   input matrix
*          int       I        I   index for askew operation
* return : askew-symmetric matrix
*-----------------------------------------------------------------------------*/
mat3 askew(const mat3& m, int I);

/* calculate element-wise product of two matrices -----------------------------
* calculate the element-wise product of two matrices
* args   : mat3      m1       I   first input matrix
*          mat3      m2       I   second input matrix
* return : matrix with element-wise product of m1 and m2
*-----------------------------------------------------------------------------*/
mat3 dotmul(const mat3& m1, const mat3& m2);

/* calculate matrix multiplication with transpose -----------------------------
* calculate the product of a matrix and the transpose of another matrix
* args   : mat3      m1       I   first input matrix
*          mat3      m2       I   second input matrix
* return : product of m1 and the transpose of m2
*-----------------------------------------------------------------------------*/
mat3 MMT(const mat3& m1, const mat3& m2);

/* calculate norm of a matrix -------------------------------------------------
* calculate the norm of a matrix
* args   : mat3      m        I   input matrix
* return : norm of the matrix
*-----------------------------------------------------------------------------*/
double norm(const mat3& m);

/* convert Direction Cosine Matrix (DCM) to rotation vector -------------------
* convert a Direction Cosine Matrix (DCM) to a rotation vector
* args   : mat3      Cnb      I   Direction Cosine Matrix (DCM)
* return : rotation vector
*-----------------------------------------------------------------------------*/
vect3 m2rv(const mat3& Cnb);

/* convert Direction Cosine Matrix (DCM) to quaternion ------------------------
* convert a Direction Cosine Matrix (DCM) to a quaternion
* args   : mat3      Cnb      I   Direction Cosine Matrix (DCM)
* return : quaternion
*-----------------------------------------------------------------------------*/
quat m2qua(const mat3& Cnb);

/* convert quaternion to Direction Cosine Matrix (DCM) ------------------------
* convert a quaternion to a Direction Cosine Matrix (DCM)
* args   : quat      qnb      I   quaternion
* return : Direction Cosine Matrix (DCM)
*-----------------------------------------------------------------------------*/
mat3 q2mat(const quat& qnb);

/* convert scale factor matrix to body-to-antenna matrix ----------------------
* convert a scale factor matrix to a body-to-antenna matrix
* args   : mat3      Ka       I   scale factor matrix
*          vect3*    pSfa     IO  pointer to scale factor adjustment vector (optional)
* return : body-to-antenna matrix
*-----------------------------------------------------------------------------*/
mat3 Ka2Cba(const mat3& Ka, vect3* pSfa = NULL);

/* convert body-to-antenna matrix to scale factor matrix ----------------------
* convert a body-to-antenna matrix to a scale factor matrix
* args   : mat3      Cba      I   body-to-antenna matrix
*          vect3     Sfa      I   scale factor adjustment vector
* return : scale factor matrix
*-----------------------------------------------------------------------------*/
mat3 Cba2Ka(const mat3& Cba, const vect3 Sfa = One31);

/* calculate Supper Fast Optimal Attitude Matrix (SFOAM) ----------------------
* calculate the Supper Fast Optimal Attitude Matrix (SFOAM)
* args   : mat3      B        I   input matrix
*          int       iter     I   number of iterations
* return : optimal attitude matrix
*-----------------------------------------------------------------------------*/
mat3 sfoam(const mat3& B, int iter);

/* generate random 3x3 matrix -------------------------------------------------
* generate a random 3x3 matrix with given mean and standard deviation
* args   : mat3      mu       I   mean matrix
*          double    sigma    I   standard deviation
* return : random 3x3 matrix
*-----------------------------------------------------------------------------*/
mat3 randn(const mat3& mu, const double& sigma);

/* calculate quaternion difference --------------------------------------------
* calculate the difference between two quaternions
* args   : quat       qcalcu   I   calculated quaternion
*          quat       qreal    I   real quaternion
* return : rotation vector representing the difference
*-----------------------------------------------------------------------------*/
vect3 qq2phi(const quat& qcalcu, const quat& qreal);

/* add misalignment vector to quaternion ---------------------------------------
* add a misalignment vector to a quaternion
* args   : quat       q        I   input quaternion
*          vect3      mu       I   misalignment vector
* return : resulting quaternion
*-----------------------------------------------------------------------------*/
quat addmu(const quat& q, const vect3& mu);

/* flip quaternion representation ----------------------------------------------
* flip the quaternion representation of an attitude
* args   : quat       q        I   input quaternion
* return : flipped quaternion
*-----------------------------------------------------------------------------*/
quat UpDown(const quat& q);

/* calculate absolute value of vector elements ---------------------------------
* calculate the absolute value for each element of a vector
* args   : vect       v        I   input vector
* return : vector with absolute values of each element
*-----------------------------------------------------------------------------*/
vect abs(const vect& v);

/* calculate vector norm ------------------------------------------------------
* calculate the Euclidean norm of a vector
* args   : vect       v        I   input vector
* return : Euclidean norm of the vector
*-----------------------------------------------------------------------------*/
double norm(const vect& v);

double norm1(const vect& v);
/* calculate vector infinity norm ---------------------------------------------
* calculate the infinity norm (maximum absolute value) of a vector
* args   : vect       v        I   input vector
* return : infinity norm of the vector
*-----------------------------------------------------------------------------*/
double normInf(const vect& v);

/* calculate power of vector elements -----------------------------------------
* calculate the k-th power for each element of a vector
* args   : vect       v        I   input vector
*          int        k        I   power to raise each element
* return : vector with each element raised to the k-th power
*-----------------------------------------------------------------------------*/
vect pow(const vect& v, int k);

/* generate random vector -----------------------------------------------------
* generate a random vector with given mean and standard deviation
* args   : vect       mu       I   mean vector
*          vect       sigma    I   standard deviation vector
* return : random vector
*-----------------------------------------------------------------------------*/
vect randn(const vect& mu, const vect& sigma);

/* sort vector elements -------------------------------------------------------
* sort the elements of a vector in ascending order
* args   : vect       v        I   input vector
* return : sorted vector
*-----------------------------------------------------------------------------*/
vect sort(const vect& v);

/* calculate dot product of two vectors ---------------------------------------
* calculate the dot product of two vectors
* args   : vect       v1       I   first input vector
*          vect       v2       I   second input vector
* return : dot product of the two vectors
*-----------------------------------------------------------------------------*/
double dot(const vect& v1, const vect& v2);

/* calculate element-wise product of two vectors ------------------------------
* calculate the element-wise product of two vectors
* args   : vect       v1       I   first input vector
*          vect       v2       I   second input vector
* return : vector with element-wise product of the two vectors
*-----------------------------------------------------------------------------*/
vect dotmul(const vect& v1, const vect& v2);

/* symmetrize a matrix --------------------------------------------------------
* symmetrize a given matrix
* args   : mat        m        IO  input/output matrix to be symmetrized
* return : none
*-----------------------------------------------------------------------------*/
void symmetry(mat& m);

/* calculate trace of a matrix ------------------------------------------------
* calculate the trace of a matrix
* args   : mat        m        I   input matrix
* return : trace of the matrix
*-----------------------------------------------------------------------------*/
double trace(const mat& m);

/* calculate 1-norm of a matrix -----------------------------------------------
* calculate the 1-norm (maximum column sum) of a matrix
* args   : mat        m        I   input matrix
* return : 1-norm of the matrix
*-----------------------------------------------------------------------------*/
double norm1(const mat& m);

/* calculate infinity norm of a matrix ----------------------------------------
* calculate the infinity norm (maximum row sum) of a matrix
* args   : mat        m        I   input matrix
* return : infinity norm of the matrix
*-----------------------------------------------------------------------------*/
double normInf(const mat& m);

/* calculate element-wise product of two matrices -----------------------------
* calculate the element-wise product of two matrices
* args   : mat        m1       I   first input matrix
*          mat        m2       I   second input matrix
* return : matrix with element-wise product of m1 and m2
*-----------------------------------------------------------------------------*/
mat dotmul(const mat& m1, const mat& m2);

/* extract diagonal of a matrix -----------------------------------------------
* extract the diagonal elements of a matrix as a vector
* args   : mat        m        I   input matrix
* return : vector containing the diagonal elements of the matrix
*-----------------------------------------------------------------------------*/
vect diag(const mat& m);

/* create diagonal matrix from a vector ---------------------------------------
* create a diagonal matrix from a vector
* args   : vect       v        I   input vector
* return : diagonal matrix
*-----------------------------------------------------------------------------*/
mat diag(const vect& v);

/* create identity matrix -----------------------------------------------------
* create an identity matrix of specified size
* args   : int        n        I   size of the identity matrix
* return : identity matrix
*-----------------------------------------------------------------------------*/
mat eye(int n);

/* calculate inverse of a 4x4 matrix ------------------------------------------
* calculate the inverse of a 4x4 matrix
* args   : mat        m        I   input matrix
* return : inverse of the matrix
*-----------------------------------------------------------------------------*/
mat inv4(const mat& m);

/* multiply row of a matrix by another matrix ---------------------------------
* multiply a row of a matrix by another matrix
* args   : mat        m        IO  input/output matrix
*          mat        m0       I   first input matrix
*          mat        m1       I   second input matrix
*          int        r        I   row index
*          int        fast     I   flag for fast computation (default: 0)
* return : none
*-----------------------------------------------------------------------------*/
void RowMul(mat& m, const mat& m0, const mat& m1, int r, int fast = 0);

/* multiply row of a matrix by transpose of another matrix --------------------
* multiply a row of a matrix by the transpose of another matrix
* args   : mat        m        IO  input/output matrix
*          mat        m0       I   first input matrix
*          mat        m1       I   second input matrix
*          int        r        I   row index
*          int        fast     I   flag for fast computation (default: 0)
* return : none
*-----------------------------------------------------------------------------*/
void RowMulT(mat& m, const mat& m0, const mat& m1, int r, int fast = 0);

/* scale matrix by diagonal vector --------------------------------------------
* scale a matrix by a diagonal vector and a scalar factor
* args   : vect       V        I   diagonal vector
*          mat        M        IO  input/output matrix
*          double     afa      I   scalar factor
* return : none
*-----------------------------------------------------------------------------*/
void DVMDVafa(const vect& V, mat& M, double afa);

/* generate random matrix -----------------------------------------------------
* generate a random matrix with given mean and standard deviation
* args   : mat        mu       I   mean matrix
*          double     sigma    I   standard deviation
* return : random matrix
*-----------------------------------------------------------------------------*/
mat randn(const mat& mu, const double& sigma);

/* extended atan2 function ----------------------------------------------------
* calculate atan2 with extended functionality
* args   : double     y        I   y-coordinate
*          double     x        I   x-coordinate
* return : angle in radians
*-----------------------------------------------------------------------------*/
double atan2Ex(double y, double x);

/* range limiting function ----------------------------------------------------
* limit a value within a specified range
* args   : double     val      I   input value
*          double     minVal   I   minimum limit
*          double     maxVal   I   maximum limit
* return : value within the range [minVal, maxVal]
*-----------------------------------------------------------------------------*/
double range(double val, double minVal, double maxVal);

/* calculate 1-norm of an array -----------------------------------------------
* calculate the 1-norm (sum of absolute values) of an array
* args   : const double* pd    I   pointer to the array
*          int         n       I   number of elements in the array
* return : 1-norm of the array
*-----------------------------------------------------------------------------*/
double norm1(const double* pd, int n);

/* calculate Euclidean norm of an array ---------------------------------------
* calculate the Euclidean norm of an array
* args   : const double* pd    I   pointer to the array
*          int         n       I   number of elements in the array
* return : Euclidean norm of the array
*-----------------------------------------------------------------------------*/
double norm(const float* pd, int n);

/* calculate Euclidean norm of an array ---------------------------------------
* calculate the Euclidean norm of an array
* args   : const double* pd    I   pointer to the array
*          int         n       I   number of elements in the array
* return : Euclidean norm of the array
*-----------------------------------------------------------------------------*/
double norm(const double* pd, int n);

/* calculate infinity norm of an array ----------------------------------------
* calculate the infinity norm (maximum absolute value) of an array
* args   : const double* pd    I   pointer to the array
*          int         n       I   number of elements in the array
* return : infinity norm of the array
*-----------------------------------------------------------------------------*/
double normInf(const double* pd, int n);

/* convert position to local navigation frame ---------------------------------
* convert a position vector to the local navigation frame
* args   : vect3      pos      I   position vector in geodetic coordinates
* return : transformation matrix from ECEF to local navigation frame
*-----------------------------------------------------------------------------*/
mat3 pos2Cen(const vect3& pos);

/* convert ECEF coordinates to geodetic coordinates ---------------------------
* convert a point in ECEF coordinates to geodetic coordinates
* args   : vect3      xyz      I   ECEF coordinates {X, Y, Z} (meters)
* return : geodetic coordinates {latitude, longitude, height} (radians, radians, meters)
*-----------------------------------------------------------------------------*/
vect3 xyz2blh(const vect3& xyz);

/* convert geodetic coordinates to ECEF coordinates ---------------------------
* convert a point in geodetic coordinates to ECEF coordinates
* args   : vect3      blh      I   geodetic coordinates {latitude, longitude, height} (radians, radians, meters)
* return : ECEF coordinates {X, Y, Z} (meters)
*-----------------------------------------------------------------------------*/
vect3 blh2xyz(const vect3& blh);

/* fusion of two states and covariances ---------------------------------------
* perform fusion of two states and their covariances
* args   : double*    x1       IO  state vector 1
*          double*    p1       IO  covariance matrix 1
*          const double* x2    I   state vector 2
*          const double* p2    I   covariance matrix 2
*          int         n       I   dimension of the state vectors
*          double*    xf       O   fused state vector (optional)
*          double*    pf       O   fused covariance matrix (optional)
* return : none
*-----------------------------------------------------------------------------*/
void fusion(double* x1, double* p1, const double* x2, const double* p2, int n = 9, double* xf = NULL, double* pf = NULL);

/* fusion of two vect3 states and covariances ---------------------------------
* perform fusion of two vect3 states and their covariances
* args   : vect3&     x1       IO  state vector 1
*          vect3&     p1       IO  covariance matrix 1
*          const vect3 x2      I   state vector 2
*          const vect3 p2      I   covariance matrix 2
* return : none
*-----------------------------------------------------------------------------*/
void fusion(vect3& x1, vect3& p1, const vect3 x2, const vect3 p2);

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
void fusion(vect3& x1, vect3& p1, const vect3 x2, const vect3 p2, vect3& xf, vect3& pf);

/* sort and unique observation data --------------------------------------------
* Sort observation data by time, receiver, and satellite, and remove duplicates.
* args   : obs_t *obs    IO     observation data structure (obs_t)
* return : int           O      number of unique epochs after sorting
* notes  : Duplicates are removed, and invalid observations are filtered out.
*-----------------------------------------------------------------------------*/
int checkobs(obsd_t* obs);
/* sort and unique observation data --------------------------------------------
* sort and unique observation data by time, rcv, sat
* args   : obs_t *obs    IO     observation data
* return : number of epochs
*-----------------------------------------------------------------------------*/
int  sortobs(obs_t* obs);
/* unique ephemerides ----------------------------------------------------------
* unique ephemerides in navigation data and update carrier wave length
* args   : nav_t *nav    IO     navigation data
* return : number of epochs
*-----------------------------------------------------------------------------*/
void uniqnav(nav_t* nav);
/* inner product ---------------------------------------------------------------
* inner product of vectors
* args   : double *a,*b     I   vector a,b (n x 1)
*          int    n         I   size of vector a,b
* return : a'*b
*-----------------------------------------------------------------------------*/
double dot(const double* a, const double* b, int n);
/* outer product of 3d vectors -------------------------------------------------
* outer product of 3d vectors
* args   : double *a,*b     I   vector a,b (3 x 1)
*          double *c        O   outer product (a x b) (3 x 1)
* return : none
*-----------------------------------------------------------------------------*/
void cross3(const double* a, const double* b, double* c);
/* normalize 3d vector ---------------------------------------------------------
* normalize 3d vector
* args   : double *a        I   vector a (3 x 1)
*          double *b        O   normlized vector (3 x 1) || b || = 1
* return : status (1:ok,0:error)
*-----------------------------------------------------------------------------*/
int normv3(const double* a, double* b);
/* screen by time --------------------------------------------------------------
* screening by time start, time end, and time interval
* args   : gtime_t time  I      time
*          gtime_t ts    I      time start (ts.time==0:no screening by ts)
*          gtime_t te    I      time end   (te.time==0:no screening by te)
*          double  tint  I      time interval (s) (0.0:no screen by tint)
* return : 1:on condition, 0:not on condition
*-----------------------------------------------------------------------------*/
int  screent(gtime_t time, gtime_t ts, gtime_t te, double tint);
/* satellite system+prn/slot number to satellite number ------------------------
* convert satellite system+prn/slot number to satellite number
* args   : int    sys       I   satellite system (SYS_GPS,SYS_GLO,...)
*          int    prn       I   satellite prn/slot number
* return : satellite number (0:error)
*-----------------------------------------------------------------------------*/
int  satno(int sys, int prn);
/* satellite number to satellite system ----------------------------------------
* convert satellite number to satellite system
* args   : int    sat       I   satellite number (1-MAXSAT)
*          int    *prn      IO  satellite prn/slot number (NULL: no output)
* return : satellite system (SYS_GPS,SYS_GLO,...)
*-----------------------------------------------------------------------------*/
int  satsys(int sat, int* prn);
/* satellite id to satellite number --------------------------------------------
* convert satellite id to satellite number
* args   : char   *id       I   satellite id (nn,Gnn,Rnn,Enn,Jnn,Cnn or Snn)
* return : satellite number (0: error)
* notes  : 120-138 and 193-195 are also recognized as sbas and qzss
*-----------------------------------------------------------------------------*/
int  satid2no(const char* id);
/* convert satellite id string to satellite system -----------------------------
* Convert a satellite id string to its corresponding satellite system.
* args   : const char* id   I   satellite id string (e.g., "G01", "C12", "R05", or "12")
*          int* sys         O   satellite system (SYS_GPS, SYS_GLO, SYS_GAL, SYS_QZS, SYS_CMP, etc.)
* return : none
* notes  : The function first tries to parse the id as a pure number (PRN), and assigns
*          the system based on PRN ranges. If that fails, it parses as a character code
*          followed by PRN (e.g., 'G' for GPS, 'R' for GLONASS, etc.).
*-----------------------------------------------------------------------------*/
void satid2sys(const char* id, int* sys);
/* satellite number to satellite id --------------------------------------------
* convert satellite number to satellite id
* args   : int    sat       I   satellite number
*          char   *id       O   satellite id (Gnn,Rnn,Enn,Jnn,Cnn or nnn)
* return : none
*-----------------------------------------------------------------------------*/
void satno2id(int sat, char* id);
/* obs type string to obs code -------------------------------------------------
* convert obs code type string to obs code
* args   : char   *str      I   obs code string ("1C","1P","1Y",...)
* return : obs code (CODE_???)
* notes  : obs codes are based on RINEX 3.04
*-----------------------------------------------------------------------------*/
uint8_t obs2code(const char* obs);
/* obs type string to obs code -------------------------------------------------
* convert obs code type string to obs code
* args   : char   *str   I      obs code string ("1C","1P","1Y",...)
*          int    *freq  IO     frequency (1:L1,2:L2,3:L5,4:L6,5:L7,6:L8,0:err)
*                               (NULL: no output)
* return : obs code (CODE_???)
* notes  : obs codes are based on reference [6] and qzss extension
*-----------------------------------------------------------------------------*/
unsigned char obs2code(const char* obs, int* freq);
/* system and obs code to frequency index --------------------------------------
* convert system and obs code to frequency index
* args   : int    sys       I   satellite system (SYS_???)
*          uint8_t code     I   obs code (CODE_???)
* return : frequency index (-1: error)
*                       0     1     2     3     4
*           --------------------------------------
*            GPS       L1    L2    L5     -     -
*            GLONASS   G1    G2    G3     -     -  (G1=G1,G1a,G2=G2,G2a)
*            Galileo   E1    E5b   E5a   E6   E5ab
*            QZSS      L1    L2    L5    L6     -
*            SBAS      L1     -    L5     -     -
*            BDS       B1    B2    B2a   B3   B2ab (B1=B1I,B1C,B2=B2I,B2b)
*            NavIC     L5     S     -     -     -
*-----------------------------------------------------------------------------*/
int  code2idx(int sys, uint8_t code);
/* obs code to obs code string -------------------------------------------------
* convert obs code to obs code string
* args   : uint8_t code     I   obs code (CODE_???)
* return : obs code string ("1C","1P","1P",...)
* notes  : obs codes are based on RINEX 3.04
*-----------------------------------------------------------------------------*/
char* code2obs(uint8_t code);
/* test excluded satellite -----------------------------------------------------
* test excluded satellite
* args   : int    sat       I   satellite number
*          int    svh       I   sv health flag
*          prcopt_t *opt    I   processing options (NULL: not used)
* return : status (1:excluded,0:not excluded)
*-----------------------------------------------------------------------------*/
char* code2obs(unsigned char code, int* freq);
/* set code priority -----------------------------------------------------------
* set code priority for multiple codes in a frequency
* args   : int    sys     I     system (or of SYS_???)
*          int    freq    I     frequency (1:L1,2:L2,3:L5,4:L6,5:L7,6:L8)
*          char   *pri    I     priority of codes (series of code characters)
*                               (higher priority precedes lower)
* return : none
*-----------------------------------------------------------------------------*/
void setcodepri(int sys, int freq, const char* pri);
/* get code priority -----------------------------------------------------------
* get code priority for multiple codes in a frequency
* args   : int    sys     I     system (SYS_???)
*          unsigned char code I obs code (CODE_???)
*          char   *opt    I     code options (NULL:no option)
* return : priority (15:highest-1:lowest,0:error)
*-----------------------------------------------------------------------------*/
int  getcodepri(int sys, unsigned char code, const char* opt);
/* test excluded satellite -----------------------------------------------------
* test excluded satellite
* args   : int    sat       I   satellite number
*          int    svh       I   sv health flag
*          prcopt_t *opt    I   processing options (NULL: not used)
* return : status (1:excluded,0:not excluded)
*-----------------------------------------------------------------------------*/
int satexclude(int sat, int svh, const prcopt_t* opt);
/* test SNR mask ---------------------------------------------------------------
* test SNR mask
* args   : int    base      I   rover or base-station (0:rover,1:base station)
*          int    freq      I   frequency (0:L1,1:L2,2:L3,...)
*          double el        I   elevation angle (rad)
*          double snr       I   C/N0 (dBHz)
*          snrmask_t *mask  I   SNR mask
* return : status (1:masked,0:unmasked)
*-----------------------------------------------------------------------------*/
int testsnr(int base, int freq, double el, double snr, const int flag, const float snrthres);


int limitcmpgeo(int sat);
/* string to number ------------------------------------------------------------
* convert substring in string to number
* args   : char   *s        I   string ("... nnn.nnn ...")
*          int    i,n       I   substring position and width
* return : converted number (0.0:error)
*-----------------------------------------------------------------------------*/
double  str2num(const char* s, int i, int n);
/* string to time --------------------------------------------------------------
* convert substring in string to gtime_t struct
* args   : char   *s        I   string ("... yyyy mm dd hh mm ss ...")
*          int    i,n       I   substring position and width
*          gtime_t *t       O   gtime_t struct
* return : status (0:ok,0>:error)
*-----------------------------------------------------------------------------*/
int     str2time(const char* s, int i, int n, gtime_t* t);
/* time to string --------------------------------------------------------------
* convert gtime_t struct to string
* args   : gtime_t t        I   gtime_t struct
*          char   *s        O   string ("yyyy/mm/dd hh:mm:ss.ssss")
*          int    n         I   number of decimals
* return : none
*-----------------------------------------------------------------------------*/
void    time2str(gtime_t t, char* str, int n);
/* convert calendar day/time to time -------------------------------------------
* convert calendar day/time to gtime_t struct
* args   : double *ep_       I   day/time {year,month,day,hour,min,sec}
* return : gtime_t struct
* notes  : proper in 1970-2037 or 1970-2099 (64bit time_t)
*-----------------------------------------------------------------------------*/
gtime_t epoch2time(const double* ep);
/* time to calendar day/time ---------------------------------------------------
* convert gtime_t struct to calendar day/time
* args   : gtime_t t        I   gtime_t struct
*          double *ep_       O   day/time {year,month,day,hour,min,sec}
* return : none
* notes  : proper in 1970-2037 or 1970-2099 (64bit time_t)
*-----------------------------------------------------------------------------*/
void    time2epoch(gtime_t t, double* ep);
/* gps time to time ------------------------------------------------------------
* convert week and tow in gps time to gtime_t struct
* args   : int    week      I   week number in gps time
*          double sec       I   time of week in gps time (s)
* return : gtime_t struct
*-----------------------------------------------------------------------------*/
gtime_t gpst2time(int week, double sec);
/* time to gps time ------------------------------------------------------------
* convert gtime_t struct to week and tow in gps time
* args   : gtime_t t        I   gtime_t struct
*          int    *week     IO  week number in gps time (NULL: no output)
* return : time of week in gps time (s)
*-----------------------------------------------------------------------------*/
double  time2gpst(gtime_t t, int* week);
/* galileo system time to time -------------------------------------------------
* convert week and tow in galileo system time (gst) to gtime_t struct
* args   : int    week      I   week number in gst
*          double sec       I   time of week in gst (s)
* return : gtime_t struct
*-----------------------------------------------------------------------------*/
gtime_t gst2time(int week, double sec);
/* time to galileo system time -------------------------------------------------
* convert gtime_t struct to week and tow in galileo system time (gst)
* args   : gtime_t t        I   gtime_t struct
*          int    *week     IO  week number in gst (NULL: no output)
* return : time of week in gst (s)
*-----------------------------------------------------------------------------*/
double  time2gst(gtime_t t, int* week);
/* beidou time (bdt) to time ---------------------------------------------------
* convert week and tow in beidou time (bdt) to gtime_t struct
* args   : int    week      I   week number in bdt
*          double sec       I   time of week in bdt (s)
* return : gtime_t struct
*-----------------------------------------------------------------------------*/
gtime_t bdt2time(int week, double sec);
/* time to beidouo time (bdt) --------------------------------------------------
* convert gtime_t struct to week and tow in beidou time (bdt)
* args   : gtime_t t        I   gtime_t struct
*          int    *week     IO  week number in bdt (NULL: no output)
* return : time of week in bdt (s)
*-----------------------------------------------------------------------------*/
double  time2bdt(gtime_t t, int* week);
/* get time string -------------------------------------------------------------
* get time string
* args   : gtime_t t        I   gtime_t struct
*          int    n         I   number of decimals
* return : time string
* notes  : not reentrant, do not use multiple in a function
*-----------------------------------------------------------------------------*/
char* time_str(gtime_t t, int n);
/* add time --------------------------------------------------------------------
* add time to gtime_t struct
* args   : gtime_t t        I   gtime_t struct
*          double sec       I   time to add (s)
* return : gtime_t struct (t+sec)
*-----------------------------------------------------------------------------*/
gtime_t timeadd(gtime_t t, double sec);
/* time difference -------------------------------------------------------------
* difference between gtime_t structs
* args   : gtime_t t1,t2    I   gtime_t structs
* return : time difference (t1-t2) (s)
*-----------------------------------------------------------------------------*/
double  timediff(gtime_t t1, gtime_t t2);
/* gpstime to utc --------------------------------------------------------------
* convert gpstime to utc considering leap seconds
* args   : gtime_t t        I   time expressed in gpstime
* return : time expressed in utc
* notes  : ignore slight time offset under 100 ns
*-----------------------------------------------------------------------------*/
gtime_t gpst2utc(gtime_t t);
/* utc to gpstime --------------------------------------------------------------
* convert utc to gpstime considering leap seconds
* args   : gtime_t t        I   time expressed in utc
* return : time expressed in gpstime
* notes  : ignore slight time offset under 100 ns
*-----------------------------------------------------------------------------*/
gtime_t utc2gpst(gtime_t t);
/* gpstime to bdt --------------------------------------------------------------
* convert gpstime to bdt (beidou navigation satellite system time)
* args   : gtime_t t        I   time expressed in gpstime
* return : time expressed in bdt
* notes  : ref [8] 3.3, 2006/1/1 00:00 BDT = 2006/1/1 00:00 UTC
*          no leap seconds in BDT
*          ignore slight time offset under 100 ns
*-----------------------------------------------------------------------------*/
gtime_t gpst2bdt(gtime_t t);
/* bdt to gpstime --------------------------------------------------------------
* convert bdt (beidou navigation satellite system time) to gpstime
* args   : gtime_t t        I   time expressed in bdt
* return : time expressed in gpstime
* notes  : see gpst2bdt()
*-----------------------------------------------------------------------------*/
gtime_t bdt2gpst(gtime_t t);
/* time to day of year ---------------------------------------------------------
* convert time to day of year
* args   : gtime_t t        I   gtime_t struct
* return : day of year (days)
*-----------------------------------------------------------------------------*/
double  time2doy(gtime_t t);
/* utc to gmst -----------------------------------------------------------------
* convert utc to gmst (Greenwich mean sidereal time)
* args   : gtime_t t        I   time expressed in utc
*          double ut1_utc   I   UT1-UTC (s)
* return : gmst (rad)
*-----------------------------------------------------------------------------*/
double  utc2gmst(gtime_t t, double ut1_utc);
/* read leap seconds table -----------------------------------------------------
* read leap seconds table
* args   : char    *file    I   leap seconds table file
* return : status (1:ok,0:error)
* notes  : (1) The records in the table file cosist of the following fields:
*              year month day hour min sec UTC-GPST(s)
*          (2) The date and time indicate the start UTC time for the UTC-GPST
*          (3) The date and time should be descending order.
*-----------------------------------------------------------------------------*/
int read_leaps(const char* file);
/* adjust gps week number ------------------------------------------------------
* adjust gps week number using cpu time
* args   : int   week       I   not-adjusted gps week number
* return : adjusted gps week number
*-----------------------------------------------------------------------------*/
int adjgpsweek(int week);
/* new matrix ------------------------------------------------------------------
* allocate memory of matrix
* args   : int    n,m       I   number of rows and columns of matrix
* return : matrix pointer (if n<=0 or m<=0, return NULL)
*-----------------------------------------------------------------------------*/
double* mat_mat(int n, int m);
/* new integer matrix ----------------------------------------------------------
* allocate memory of integer matrix
* args   : int    n,m       I   number of rows and columns of matrix
* return : matrix pointer (if n<=0 or m<=0, return NULL)
*-----------------------------------------------------------------------------*/
int* imat_mat(int n, int m);
/* zero matrix -----------------------------------------------------------------
* generate new zero matrix
* args   : int    n,m       I   number of rows and columns of matrix
* return : matrix pointer (if n<=0 or m<=0, return NULL)
*-----------------------------------------------------------------------------*/
double* zeros_mat(int n, int m);
/* identity matrix -------------------------------------------------------------
* generate new identity matrix
* args   : int    n         I   number of rows and columns of matrix
* return : matrix pointer (if n<=0, return NULL)
*-----------------------------------------------------------------------------*/
double* eye_mat(int n);
/* outer product of 3d vectors -------------------------------------------------
* outer product of 3d vectors
* args   : double *a,*b     I   vector a,b (3 x 1)
*          double *c        O   outer product (a x b) (3 x 1)
* return : none
*-----------------------------------------------------------------------------*/
void cross3(const double* a, const double* b, double* c);
/* normalize 3d vector ---------------------------------------------------------
* normalize 3d vector
* args   : double *a        I   vector a (3 x 1)
*          double *b        O   normlized vector (3 x 1) || b || = 1
* return : status (1:ok,0:error)
*-----------------------------------------------------------------------------*/
int  normv3(const double* a, double* b);
/* copy matrix -----------------------------------------------------------------
* copy matrix
* args   : double *A        O   destination matrix A (n x m)
*          double *B        I   source matrix B (n x m)
*          int    n,m       I   number of rows and columns of matrix
* return : none
*-----------------------------------------------------------------------------*/
void matcpy(double* A, const double* B, int n, int m);
/* multiply matrix (wrapper of blas dgemm) -------------------------------------
* multiply matrix by matrix (C=alpha*A*B+beta*C)
* args   : char   *tr       I  transpose flags ("N":normal,"T":transpose)
*          int    n,k,m     I  size of (transposed) matrix A,B
*          double alpha     I  alpha
*          double *A,*B     I  (transposed) matrix A (n x m), B (m x k)
*          double beta      I  beta
*          double *C        IO matrix C (n x k)
* return : none
*-----------------------------------------------------------------------------*/
void matmul(const char* tr, int n, int k, int m, double alpha, const double* A, const double* B, double beta, double* C);
/* inverse of matrix -----------------------------------------------------------
* inverse of matrix (A=A^-1)
* args   : double *A        IO  matrix (n x n)
*          int    n         I   size of matrix A
* return : status (0:ok,0>:error)
*-----------------------------------------------------------------------------*/
int  matinv(double* A, int n);
/* transform ecef to geodetic postion ------------------------------------------
* transform ecef position to geodetic position
* args   : double *r        I   ecef position {x,y,z} (m)
*          double *pos      O   geodetic position {lat,lon,h} (rad,m)
* return : none
* notes  : WGS84, ellipsoidal height
*-----------------------------------------------------------------------------*/
void ecef2pos(const double* r, double* pos);
/* transform geodetic to ecef position -----------------------------------------
* transform geodetic position to ecef position
* args   : double *pos      I   geodetic position {lat,lon,h} (rad,m)
*          double *r        O   ecef position {x,y,z} (m)
* return : none
* notes  : WGS84, ellipsoidal height
*-----------------------------------------------------------------------------*/
void pos2ecef(const double* pos, double* r);
/* transform ecef vector to local tangental coordinate -------------------------
* transform ecef vector to local tangental coordinate
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *r        I   vector in ecef coordinate {x,y,z}
*          double *e        O   vector in local tangental coordinate {e,n,u}
* return : none
*-----------------------------------------------------------------------------*/
void ecef2enu(const double pos, const double r, double* e);
/* transform local vector to ecef coordinate -----------------------------------
* transform local tangental coordinate vector to ecef
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *e_       I   vector in local tangental coordinate {e_,n,u}
*          double *r        O   vector in ecef coordinate {x,y,z}
* return : none
*-----------------------------------------------------------------------------*/
void enu2ecef(const double pos, const double e, double* r);
/* transform covariance to local tangental coordinate --------------------------
* transform ecef covariance to local tangental coordinate
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *P        I   covariance in ecef coordinate
*          double *Q        O   covariance in local tangental coordinate
* return : none
*-----------------------------------------------------------------------------*/
void covenu(const double pos, const double P, double* Q);
/* transform local enu coordinate covariance to xyz-ecef -----------------------
* transform local enu covariance to xyz-ecef coordinate
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *Q        I   covariance in local enu coordinate
*          double *P        O   covariance in xyz-ecef coordinate
* return : none
*-----------------------------------------------------------------------------*/
void covecef(const double* pos, const double* Q, double* P);
/* ecef to local coordinate transfromation matrix ------------------------------
* compute ecef to local coordinate transfromation matrix
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *E        O   ecef to local coord transformation matrix (3x3)
* return : none
* notes  : matirix stored by column-major order (fortran convention)
*-----------------------------------------------------------------------------*/
void xyz2enu(const double* pos, double* E);
/* convert degree to deg-min-sec -----------------------------------------------
* convert degree to degree-minute-second
* args   : double deg       I   degree
*          double *dms      O   degree-minute-second {deg,min,sec}
* return : none
*-----------------------------------------------------------------------------*/
void deg2dms(double deg, double* dms);
/* convert deg-min-sec to degree -----------------------------------------------
* convert degree-minute-second to degree
* args   : double *dms      I   degree-minute-second {deg,min,sec}
* return : degree
*-----------------------------------------------------------------------------*/
double dms2deg(const double* dms);
/* satellite carrier wave length -----------------------------------------------
* get satellite carrier wave lengths
* args   : int    sat       I   satellite number
*          int    frq       I   frequency index (0:L1,1:L2,2:L5/3,...)
*          nav_t  *nav      I   navigation messages
* return : carrier wave length (m) (0.0: error)
*-----------------------------------------------------------------------------*/
double satwavelen(int sat, int frq, const nav_t* nav);
/* satellite azimuth/elevation angle -------------------------------------------
* compute satellite azimuth/elevation angle
* args   : double *pos      I   geodetic position {lat,lon,h} (rad,m)
*          double *e        I   receiver-to-satellilte unit vevtor (ecef)
*          double *azel     IO  azimuth/elevation {az,el} (rad) (NULL: no output)
*                               (0.0<=azel[0]<2*pi,-pi/2<=azel[1]<=pi/2)
* return : elevation angle (rad)
*-----------------------------------------------------------------------------*/
double satazel(const double* pos, const double* e, double* azel);
/* geometric distance ----------------------------------------------------------
* compute geometric distance and receiver-to-satellite unit vector
* args   : double *rs       I   satellilte position (ecef at transmission) (m)
*          double *rr       I   receiver position (ecef at reception) (m)
*          double *e        O   line-of-sight vector (ecef)
* return : geometric distance (m) (0>:error/no satellite position)
* notes  : distance includes sagnac effect correction
*-----------------------------------------------------------------------------*/
double geodist(const double* rs, const double* rr, double* e);
/* compute dops ----------------------------------------------------------------
* compute DOP (dilution of precision)
* args   : int    ns        I   number of satellites
*          double *azel     I   satellite azimuth/elevation angle (rad)
*          double elmin     I   elevation cutoff angle (rad)
*          double *dop      O   DOPs {GDOP,PDOP,HDOP,VDOP}
* return : none
* notes  : dop[0]-[3] return 0 in case of dop computation error
*-----------------------------------------------------------------------------*/
void dops(int ns, const double* azel, double elmin, double* dop);

void dops_dif(int ns, const double* azel, double elmin, double* dop, const int* vsat);
/* ionosphere model ------------------------------------------------------------
* compute ionospheric delay by broadcast ionosphere model (klobuchar model)
* args   : gtime_t t        I   time (gpst)
*          double *ion      I   iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3}
*          double *pos      I   receiver position {lat,lon,h} (rad,m)
*          double *azel     I   azimuth/elevation angle {az,el} (rad)
* return : ionospheric delay (L1) (m)
*-----------------------------------------------------------------------------*/
double ionmodel(gtime_t t, const double* ion, const double* pos, const double* azel);
/* ionosphere mapping function -------------------------------------------------
* compute ionospheric delay mapping function by single layer model
* args   : double *pos      I   receiver position {lat,lon,h} (rad,m)
*          double *azel     I   azimuth/elevation angle {az,el} (rad)
* return : ionospheric mapping function
*-----------------------------------------------------------------------------*/
double ionmapf(const double* pos, const double* azel);
/* ionospheric pierce point position -------------------------------------------
* compute ionospheric pierce point (ipp) position and slant factor
* args   : double *pos      I   receiver position {lat,lon,h} (rad,m)
*          double *azel     I   azimuth/elevation angle {az,el} (rad)
*          double re        I   earth radius (km)
*          double hion      I   altitude of ionosphere (km)
*          double *posp     O   pierce point position {lat,lon,h} (rad,m)
* return : slant factor
* notes  : see ref [2], only valid on the earth surface
*          fixing bug on ref [2] A.4.4.10.1 A-22,23
*-----------------------------------------------------------------------------*/
double ionppp(const double* pos, const double* azel, double re, double hion, double* pppos);
/* troposphere model -----------------------------------------------------------
* compute tropospheric delay by standard atmosphere and saastamoinen model
* args   : gtime_t time     I   time
*          double *pos      I   receiver position {lat,lon,h} (rad,m)
*          double *azel     I   azimuth/elevation angle {az,el} (rad)
*          double humi      I   relative humidity
* return : tropospheric delay (m)
*-----------------------------------------------------------------------------*/
double tropmodel(gtime_t time, const vect3 pos, const double* azel, double humi);
/* troposphere mapping function ------------------------------------------------
* compute tropospheric mapping function by NMF
* args   : gtime_t t        I   time
*          double *pos      I   receiver position {lat,lon,h} (rad,m)
*          double *azel     I   azimuth/elevation angle {az,el} (rad)
*          double *mapfw    IO  wet mapping function (NULL: not output)
* return : dry mapping function
* note   : see ref [5] (NMF) and [9] (GMF)
*          original JGR paper of [5] has bugs in eq.(4) and (5). the corrected
*          paper is obtained from:
*          ftp://web.haystack.edu/pub/aen/nmf/NMF_JGR.pdf
*-----------------------------------------------------------------------------*/
double tropmapf(gtime_t time, const double* pos, const double* azel, double* mapfw);
/* ionospheric correction ------------------------------------------------------
* compute ionospheric correction
* args   : gtime_t time     I   time
*          nav_t  *nav      I   navigation data
*          int    sat       I   satellite number
*          double *pos      I   receiver position {lat,lon,h} (rad|m)
*          double *azel     I   azimuth/elevation angle {az,el} (rad)
*          int    ionoopt   I   ionospheric correction option (IONOOPT_???)
*          double *ion      O   ionospheric delay (L1) (m)
*          double *var      O   ionospheric delay (L1) variance (m^2)
* return : status(1:ok,0:error)
*-----------------------------------------------------------------------------*/
int ionocorr(gtime_t time, const nav_t* nav, int sat, const double* pos, const double* azel, int ionoopt, double* ion, double* var);
/* tropospheric correction -----------------------------------------------------
* compute tropospheric correction
* args   : gtime_t time     I   time
*          nav_t  *nav      I   navigation data
*          double *pos      I   receiver position {lat,lon,h} (rad|m)
*          double *azel     I   azimuth/elevation angle {az,el} (rad)
*          int    tropopt   I   tropospheric correction option (TROPOPT_???)
*          double *trp      O   tropospheric delay (m)
*          double *var      O   tropospheric delay variance (m^2)
* return : status(1:ok,0:error)
*-----------------------------------------------------------------------------*/
int tropcorr(gtime_t time, const nav_t* nav, const double* pos, const double* azel, int tropopt, double* trp, double* var);
/* set selected satellite ephemeris --------------------------------------------
* Set selected satellite ephemeris for multiple ones like LNAV - CNAV, I/NAV -
* F/NAV. Call it before calling satpos(),satposs() to use unselected one.
* args   : int    sys       I   satellite system (SYS_???)
*          int    sel       I   selection of ephemeris
*                                 GPS,QZS : 0:LNAV ,1:CNAV  (default: LNAV)
*                                 GAL     : 0:I/NAV,1:F/NAV (default: I/NAV)
*                                 others  : undefined
* return : none
* notes  : default ephemeris selection for galileo is any.
*-----------------------------------------------------------------------------*/
void setseleph(int sys, int sel);
/* get selected satellite ephemeris -------------------------------------------
* Get the selected satellite ephemeris.
* args   : int    sys       I   satellite system (SYS_???)
* return : selected ephemeris
*            refer setseleph()
*-----------------------------------------------------------------------------*/
int getseleph(int sys);
/* broadcast ephemeris to satellite clock bias ---------------------------------
* compute satellite clock bias with broadcast ephemeris (gps, galileo, qzss)
* args   : gtime_t time     I   time by satellite clock (gpst)
*          eph_t *eph       I   broadcast ephemeris
* return : satellite clock bias (s) without relativeity correction
* notes  : see ref [1],[7],[8]
*          satellite clock does not include relativity correction and tdg
*-----------------------------------------------------------------------------*/
double eph2clk(gtime_t time, const eph_t* eph);
/* broadcast ephemeris to satellite position and clock bias --------------------
* compute satellite position and clock bias with broadcast ephemeris (gps,
* galileo, qzss)
* args   : gtime_t time     I   time (gpst)
*          eph_t *eph       I   broadcast ephemeris
*          double *rs       O   satellite position (ecef) {x,y,z} (m)
*          double *dts      O   satellite clock bias (s)
*          double *var      O   satellite position and clock variance (m^2)
* return : none
* notes  : see ref [1],[7],[8]
*          satellite clock includes relativity correction without code bias
*          (tgd or bgd)
*-----------------------------------------------------------------------------*/
double geph2clk(gtime_t time, const geph_t* geph);
/* broadcast ephemeris to satellite position and clock bias --------------------
* compute satellite position and clock bias with broadcast ephemeris (gps,
* galileo, qzss)
* args   : gtime_t time     I   time (gpst)
*          eph_t *eph       I   broadcast ephemeris
*          double *rs       O   satellite position (ecef) {x,y,z} (m)
*          double *dts      O   satellite clock bias (s)
*          double *var      O   satellite position and clock variance (m^2)
* return : none
* notes  : see ref [1],[7],[8]
*          satellite clock includes relativity correction without code bias
*          (tgd or bgd)
*-----------------------------------------------------------------------------*/
void eph2pos(gtime_t time, const eph_t* eph, double* rs, double* dts, double* var);
/* glonass ephemeris to satellite position and clock bias ----------------------
* compute satellite position and clock bias with glonass ephemeris
* args   : gtime_t time     I   time (gpst)
*          geph_t *geph     I   glonass ephemeris
*          double *rs       O   satellite position {x,y,z} (ecef) (m)
*          double *dts      O   satellite clock bias (s)
*          double *var      O   satellite position and clock variance (m^2)
* return : none
* notes  : see ref [2]
*-----------------------------------------------------------------------------*/
void geph2pos(gtime_t time, const geph_t* geph, double* rs, double* dts, double* var);
/* broadcast ephemeris to satellite clock bias ---------------------------------
* compute satellite clock bias using broadcast ephemeris (GPS, Galileo, QZSS)
* args   : gtime_t time     I   time at which to compute clock bias (gpst)
*          gtime_t teph     I   ephemeris reference time (gpst)
*          int    sat       I   satellite number
*          nav_t  *nav      I   navigation data
*          double *dts      O   satellite clock bias (s)
* return : int              O   status (1:ok, 0:error)
* notes  : 
*   - Computes the satellite clock bias at the specified time using the broadcast ephemeris.
*   - The result does not include relativity correction or code bias.
*   - Returns 1 on success, 0 on failure (e.g., no valid ephemeris).
*/
int ephclk(gtime_t time, gtime_t teph, int sat, const nav_t* nav, float* dts);
/* satellite position and clock ------------------------------------------------
* compute satellite position, velocity and clock
* args   : gtime_t time     I   time (gpst)
*          gtime_t teph     I   time to select ephemeris (gpst)
*          int    sat       I   satellite number
*          nav_t  *nav      I   navigation data
*          int    ephopt    I   ephemeris option (EPHOPT_???)
*          double *rs       O   sat position and velocity (ecef)
*                               {x,y,z,vx,vy,vz} (m|m/s)
*          double *dts      O   sat clock {bias,drift} (s|s/s)
*          double *var      O   sat position and clock error variance (m^2)
*          int    *svh      O   sat health flag (-1:correction not available)
* return : status (1:ok,0:error)
* notes  : satellite position is referenced to antenna phase center
*          satellite clock does not include code bias correction (tgd or bgd)
*-----------------------------------------------------------------------------*/
int  satpos(gtime_t time, gtime_t teph, int sat, int ephopt,	const nav_t* nav, double* rs, double* dts, double* var,	int* svh);
/* satellite positions and clocks ----------------------------------------------
* compute satellite positions, velocities and clocks
* args   : gtime_t teph     I   time to select ephemeris (gpst)
*          obsd_t *obs      I   observation data
*          int    n         I   number of observation data
*          nav_t  *nav      I   navigation data
*          int    ephopt    I   ephemeris option (EPHOPT_???)
*          double *rs       O   satellite positions and velocities (ecef)
*          double *dts      O   satellite clocks
*          double *var      O   sat position and clock error variances (m^2)
*          int    *svh      O   sat health flag (-1:correction not available)
* return : none
* notes  : rs [(0:2)+i*6]= obs[i] sat position {x,y,z} (m)
*          rs [(3:5)+i*6]= obs[i] sat velocity {vx,vy,vz} (m/s)
*          dts[(0:1)+i*2]= obs[i] sat clock {bias,drift} (s|s/s)
*          var[i]        = obs[i] sat position and clock error variance (m^2)
*          svh[i]        = obs[i] sat health flag
*          if no navigation data, set 0 to rs[], dts[], var[] and svh[]
*          satellite position and clock are values at signal transmission time
*          satellite position is referenced to antenna phase center
*          satellite clock does not include code bias correction (tgd or bgd)
*          any pseudorange and broadcast ephemeris are always needed to get
*          signal transmission time
*-----------------------------------------------------------------------------*/
void satposs(gtime_t time, const obsd_t* obs, int n, const nav_t* nav, int sateph, double* rs, double* dts, double* var, int* svh, int maxsizeofsvh);
/* pseudorange measurement error variance ------------------------------------
* Compute the variance of pseudorange measurement errors.
* args   : const prcopt_t *opt I   processing options
*          double el           I   satellite elevation angle (radians)
*          int    sys          I   satellite system (SYS_GPS, SYS_GLO, etc.)
* return : double              O   variance of pseudorange measurement errors [m^2]
* notes  :
*   - This function calculates the variance of pseudorange measurement errors
*     based on the satellite system, elevation angle, and processing options.
*   - The variance is scaled by a factor specific to the satellite system:
*       - GPS: `EFACT_GPS`
*       - GLONASS: `EFACT_GLO`
*       - SBAS: `EFACT_SBS`
*   - For ionosphere-free combinations, an additional scaling factor is applied.
*-----------------------------------------------------------------------------*/
double varerr_psr(int ionoopt, double el, int sys);
/* least square estimation -----------------------------------------------------
* least square estimation by solving normal equation (x=(A*A')^-1*A*y)
* args   : double *A        I   transpose of (weighted) design matrix (n x m)
*          double *y        I   (weighted) measurements (m x 1)
*          int    n,m       I   number of parameters and measurements (n<=m)
*          double *x        O   estmated parameters (n x 1)
*          double *Q        O   esimated parameters covariance matrix (n x n)
* return : status (0:ok,0>:error)
* notes  : for weighted least square, replace A and y by A*w and w*y (w=W^(1/2))
*          matirix stored by column-major order (fortran convention)
*-----------------------------------------------------------------------------*/
int lsq(const double* A, const double* y, int n, int m, double* x, double* Q);
/* pseudorange with code bias correction -------------------------------------
* Compute pseudorange with code bias correction.
* args   : const obsd_t *obs  I   observation data for a single satellite
*          const nav_t  *nav  I   navigation data
*          const double *azel I   azimuth/elevation angles {az, el} [rad]
*          int    iter        I   iteration number (0 for the first iteration)
*          const prcopt_t *opt I  processing options
*          double *var        O   pseudorange measurement variance [m^2]
* return : double             O   corrected pseudorange [m] (0.0 if invalid)
* notes  :
*   - This function computes the pseudorange measurement for a satellite,
*     applying corrections for ionospheric delay, tropospheric delay, and
*     code biases.
*   - Dual-frequency or single-frequency measurements are supported.
*   - If the signal-to-noise ratio (SNR) is below a threshold, the measurement
*     is rejected.
*   - The function also handles Differential Code Biases (DCB) and Total Group
*     Delay (TGD) corrections.
*-----------------------------------------------------------------------------*/
double prange(const obsd_t* obs, const nav_t* nav, const double* azel, int iter, snrmask_t* mask, int ionoopt, double* var);
/* single-point positioning ----------------------------------------------------
* compute receiver position, velocity, clock bias by single-point positioning
* with pseudorange and doppler observables
* args   : obsd_t *obs      I   observation data
*          int    n         I   number of observation data
*          nav_t  *nav      I   navigation data
*          prcopt_t *opt    I   processing options
*          sol_t  *sol      IO  solution
*          double *azel     IO  azimuth/elevation angle (rad) (NULL: no output)
*          ssat_t *ssat     IO  satellite status              (NULL: no output)
*          char   *msg      O   error message for error exit
*		   int vof			I   Whether to estimate speed signs
* return : status(1:ok,0:error)
* notes  : assuming sbas-gps, galileo-gps, qzss-gps, compass-gps time offset and
*          receiver bias are negligible (only involving glonass-gps time offset
*          and receiver bias)
*-----------------------------------------------------------------------------*/
int pntpos(rtk_t* rtk, const obsd_t* obs, int n, const nav_t* nav, const prcopt_t* opt,sol_t* sol, double* azel, ssat_t* ssat);

//extern int lambda(int n, int m, const double* a, const double* Q, double* F, double* s);
///* standard positioning ------------------------------------------------------*/
//
//extern void rtkinit(rtk_t* rtk, const prcopt_t* opt);
//extern void rtkfree(rtk_t* rtk);
//extern int  rtkpos(rtk_t* rtk, const obsd_t* obs, int nobs, const nav_t* nav);

/* satellite position/clock by precise ephemeris/clock -------------------------
* compute satellite position/clock with precise ephemeris/clock
* args   : gtime_t time       I   time (gpst)
*          int    sat         I   satellite number
*          nav_t  *nav        I   navigation data
*          int    opt         I   sat postion option
*                                 (0: center of mass, 1: antenna phase center)
*          double *rs         O   sat position and velocity (ecef)
*                                 {x,y,z,vx,vy,vz} (m|m/s)
*          double *dts        O   sat clock {bias,drift} (s|s/s)
*          double *var        IO  sat position and clock error variance (m)
*                                 (NULL: no output)
* return : status (1:ok,0:error or data outage)
* notes  : clock includes relativistic correction but does not contain code bias
*          before calling the function, nav->peph, nav->ne, nav->pclk and
*          nav->nc must be set by calling readsp3(), readrnx() or readrnxt()
*          if precise clocks are not set, clocks in sp3 are used instead
*-----------------------------------------------------------------------------*/
extern int  peph2pos(gtime_t time, int sat, const nav_t* nav, int opt, double* rs, double* dts, double* var);
/* satellite antenna phase center offset ---------------------------------------
* compute satellite antenna phase center offset in ecef
* args   : gtime_t time       I   time (gpst)
*          double *rs         I   satellite position and velocity (ecef)
*                                 {x,y,z,vx,vy,vz} (m|m/s)
*          int    sat         I   satellite number
*          nav_t  *nav        I   navigation data
*          double *dant       I   satellite antenna phase center offset (ecef)
*                                 {dx,dy,dz} (m) (iono-free LC value)
* return : none
*-----------------------------------------------------------------------------*/
extern void satantoff(gtime_t time, const double* rs, int sat, const nav_t* nav, double* dant);
/*in renix cpp*/
/* read rinex header ---------------------------------------------------------
* Read and decode the RINEX file header, extracting version, type, system, and observation types.
* args   : FILE* fp       I   file pointer
*          double* ver    O   RINEX version
*          char* type     O   file type ('O', 'N', etc.)
*          int* sys       O   satellite system
*          int* tsys      O   time system
*          char tobs[][MAXOBSTYPE][4] O   observation types per system
*          nav_t* nav     IO  navigation data structure
* return : 1 if header read successfully, 0 otherwise
*-----------------------------------------------------------------------------*/
extern int readrnxh(FILE* fp, double* ver, char* type, int* sys, int* tsys, char tobs[][MAXOBSTYPE][4], nav_t* nav);
/* read rinex obs data body --------------------------------------------------
* Read and decode the body of a RINEX observation file (all epochs and satellites).
* args   : FILE* fp       I   file pointer
*          const char* opt I  option string
*          double ver     I   RINEX version
*          int* tsys      IO  time system
*          char tobs[][MAXOBSTYPE][4] I   observation types per system
*          int* flag      O   epoch flag
*          obsd_t* data   O   output observation data array
*          int rcv        I   receiver index
* return : number of observation records read, -1 on error
*-----------------------------------------------------------------------------*/
extern int readrnxobsb(FILE* fp, const char* opt, double ver, int* tsys, char tobs[][MAXOBSTYPE][4], int* flag, obsd_t* data, int rcv);
/* read rinex file -----------------------------------------------------------
* Read and decode a RINEX file (observation or navigation).
* args   : FILE* fp       I   file pointer
*          gtime_t ts     I   start time (0: no limit)
*          gtime_t te     I   end time (0: no limit)
*          double tint    I   time interval (0: no limit)
*          const char* opt I  option string
*          int flag       I   0: except for clock, 1: clock only
*          int index      I   receiver index
*          char* type     O   file type ('O', 'N', etc.)
*          obs_t* obs     IO  observation data structure
*          nav_t* nav     IO  navigation data structure
* return : 1 if successful, 0 otherwise
*-----------------------------------------------------------------------------*/
extern int readrnxfp(FILE* fp, gtime_t ts, gtime_t te, double tint, const char* opt, int flag, int index, char* type, obs_t* obs, nav_t* nav);
/* read sp3 precise ephemeris file ---------------------------------------------
* read sp3 precise ephemeris/clock files and set them to navigation data
* args   : char   *file       I   sp3-c precise ephemeris file
*                                 (wind-card * is expanded)
*          nav_t  *nav        IO  navigation data
*          int    opt         I   options (1: only observed + 2: only predicted +
*                                 4: not combined)
* return : none
* notes  : see ref [1]
*          precise ephemeris is appended and combined
*          nav->peph and nav->ne must by properly initialized before calling the
*          function
*          only files with extensions of .sp3, .SP3, .eph* and .EPH* are read
*-----------------------------------------------------------------------------*/
extern void readsp3(const char* file, nav_t* nav, int opt);
extern void readpos(const char* file, const char* rcv, double* pos);
extern int  readnav(const char* file, nav_t* nav);
extern int  savenav(const char* file, const nav_t* nav);
extern unsigned int getbitu(const unsigned char* buff, int pos, int len);
extern int          getbits(const unsigned char* buff, int pos, int len);
extern unsigned int rtk_crc24q(const unsigned char* buff, int len);
extern int init_rtcm(rtcm_t* rtcm, prelock_t* prelock, int rcv, int staid);
extern void free_rtcm(rtcm_t* rtcm);
extern int input_rtcm3(rtcm_t* rtcm, unsigned char data);


#endif