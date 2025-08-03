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

#define MAXFREQ     7                   /* max NFREQ */
#define CLIGHT      299792458.0         /* speed of light (m/s) */
#define HION        350000.0            /* ionosphere height (m) */
#define AU          149597870691.0      /* 1 AU (m) */

#define FREQ1       1.57542E9           /* L1/E1/B1C  frequency (Hz) */
#define FREQ2       1.22760E9           /* L2         frequency (Hz) */
#define FREQ5       1.17645E9           /* L5/E5a/B2a frequency (Hz) */
#define FREQ6       1.27875E9           /* E6/L6  frequency (Hz) */
#define FREQ7       1.20714E9           /* E5b    frequency (Hz) */
#define FREQ8       1.191795E9          /* E5a+b  frequency (Hz) */
#define FREQ9       2.492028E9          /* S      frequency (Hz) */
#define FREQ1_GLO   1.60200E9           /* GLONASS G1 base frequency (Hz) */
#define DFRQ1_GLO   0.56250E6           /* GLONASS G1 bias frequency (Hz/n) */
#define FREQ2_GLO   1.24600E9           /* GLONASS G2 base frequency (Hz) */
#define DFRQ2_GLO   0.43750E6           /* GLONASS G2 bias frequency (Hz/n) */
#define FREQ3_GLO   1.202025E9          /* GLONASS G3 frequency (Hz) */
#define FREQ1a_GLO  1.600995E9          /* GLONASS G1a frequency (Hz) */
#define FREQ2a_GLO  1.248060E9          /* GLONASS G2a frequency (Hz) */
#define FREQ1_CMP   1.561098E9          /* BDS B1I     frequency (Hz) */
#define FREQ2_CMP   1.20714E9           /* BDS B2I/B2b frequency (Hz) */
#define FREQ3_CMP   1.26852E9           /* BDS B3      frequency (Hz) */

#define EFACT_GPS   1.0                 /* error factor: GPS */
#define EFACT_GLO   1.5                 /* error factor: GLONASS */
#define EFACT_GAL   1.0                 /* error factor: Galileo */
#define EFACT_QZS   1.0                 /* error factor: QZSS */
#define EFACT_CMP   1.0                 /* error factor: BeiDou */
#define EFACT_IRN   1.5                 /* error factor: IRNSS */
#define EFACT_SBS   3.0                 /* error factor: SBAS */

#define SYS_NONE    0x00                /* navigation system: none */
#define SYS_GPS     0x01                /* navigation system: GPS */
#define SYS_SBS     0x02                /* navigation system: SBAS */
#define SYS_GLO     0x04                /* navigation system: GLONASS */
#define SYS_GAL     0x08                /* navigation system: Galileo */
#define SYS_QZS     0x10                /* navigation system: QZSS */
#define SYS_CMP     0x20                /* navigation system: BeiDou */
#define SYS_IRN     0x40                /* navigation system: IRNS */
#define SYS_LEO     0x80                /* navigation system: LEO */
#define SYS_ALL     0xFF                /* navigation system: all */

#define TSYS_GPS    0                   /* time system: GPS time */
#define TSYS_UTC    1                   /* time system: UTC */
#define TSYS_GLO    2                   /* time system: GLONASS time */
#define TSYS_GAL    3                   /* time system: Galileo time */
#define TSYS_QZS    4                   /* time system: QZSS time */
#define TSYS_CMP    5                   /* time system: BeiDou time */
#define TSYS_IRN    6                   /* time system: IRNSS time */

#ifndef NFREQ
#define NFREQ       3                   /* number of carrier frequencies */
#endif
#define NFREQGLO    2                   /* number of carrier frequencies of GLONASS */

#ifndef NEXOBS
#define NEXOBS      0                   /* number of extended obs codes */
#endif

#define SNR_UNIT    0.001               /* SNR unit (dBHz) */

#define MINPRNGPS   1                   /* min satellite PRN number of GPS */
#define MAXPRNGPS   32                  /* max satellite PRN number of GPS */
#define NSATGPS     (MAXPRNGPS-MINPRNGPS+1) /* number of GPS satellites */
#define NSYSGPS     1

#ifdef ENAGLO
	#define MINPRNGLO   1                   /* min satellite slot number of GLONASS */
	#define MAXPRNGLO   27                  /* max satellite slot number of GLONASS */
	#define NSATGLO     (MAXPRNGLO-MINPRNGLO+1) /* number of GLONASS satellites */
	#define NSYSGLO     1
#else
	#define MINPRNGLO   0
	#define MAXPRNGLO   0
	#define NSATGLO     0
	#define NSYSGLO     0
#endif
#ifdef ENAGAL
	#define MINPRNGAL   1                   /* min satellite PRN number of Galileo */
	#define MAXPRNGAL   36                  /* max satellite PRN number of Galileo */
	#define NSATGAL    (MAXPRNGAL-MINPRNGAL+1) /* number of Galileo satellites */
	#define NSYSGAL     1
#else
	#define MINPRNGAL   0
	#define MAXPRNGAL   0
	#define NSATGAL     0
	#define NSYSGAL     0
#endif
#ifdef ENAQZS
	#define MINPRNQZS   193                 /* min satellite PRN number of QZSS */
	#define MAXPRNQZS   202                 /* max satellite PRN number of QZSS */
	#define MINPRNQZS_S 183                 /* min satellite PRN number of QZSS L1S */
	#define MAXPRNQZS_S 191                 /* max satellite PRN number of QZSS L1S */
	#define NSATQZS     (MAXPRNQZS-MINPRNQZS+1) /* number of QZSS satellites */
	#define NSYSQZS     1
#else
	#define MINPRNQZS   0
	#define MAXPRNQZS   0
	#define MINPRNQZS_S 0
	#define MAXPRNQZS_S 0
	#define NSATQZS     0
	#define NSYSQZS     0
#endif
#ifdef ENACMP
	#define MINPRNCMP   1                   /* min satellite sat number of BeiDou */
	#define MAXPRNCMP   63                  /* max satellite sat number of BeiDou */
	#define NSATCMP     (MAXPRNCMP-MINPRNCMP+1) /* number of BeiDou satellites */
	#define NSYSCMP     1
#else
	#define MINPRNCMP   0
	#define MAXPRNCMP   0
	#define NSATCMP     0
	#define NSYSCMP     0
#endif
#ifdef ENAIRN
	#define MINPRNIRN   1                   /* min satellite sat number of IRNSS */
	#define MAXPRNIRN   14                  /* max satellite sat number of IRNSS */
	#define NSATIRN     (MAXPRNIRN-MINPRNIRN+1) /* number of IRNSS satellites */
	#define NSYSIRN     1
#else
	#define MINPRNIRN   0
	#define MAXPRNIRN   0
	#define NSATIRN     0
	#define NSYSIRN     0
#endif
#ifdef ENALEO
	#define MINPRNLEO   1                   /* min satellite sat number of LEO */
	#define MAXPRNLEO   10                  /* max satellite sat number of LEO */
	#define NSATLEO     (MAXPRNLEO-MINPRNLEO+1) /* number of LEO satellites */
	#define NSYSLEO     1
#else
	#define MINPRNLEO   0
	#define MAXPRNLEO   0
	#define NSATLEO     0
	#define NSYSLEO     0
#endif
#define NSYS        (NSYSGPS+NSYSGLO+NSYSGAL+NSYSQZS+NSYSCMP+NSYSIRN+NSYSLEO) /* number of systems */

#define MINPRNSBS   120                 /* min satellite PRN number of SBAS */
#define MAXPRNSBS   158                 /* max satellite PRN number of SBAS */
#define NSATSBS     (MAXPRNSBS-MINPRNSBS+1) /* number of SBAS satellites */

#define MAXSAT      (NSATGPS+NSATGLO+NSATGAL+NSATQZS+NSATCMP+NSATIRN+NSATSBS+NSATLEO)
/* max satellite number (1 to MAXSAT) */
#define MAXSTA      255

#ifndef MAXOBS
#define MAXOBS      96                  /* max number of obs in an epoch */
#endif
#define MAXRCV      64                  /* max receiver number (1 to MAXRCV) */
#define MAXOBSTYPE  64                  /* max number of obs type in RINEX */
#ifdef OBS_100HZ
	#define DTTOL       0.005               /* tolerance of time difference (s) */
#else
#define DTTOL       0.025               /* tolerance of time difference (s) */
#endif
#define MAXDTOE     7200.0              /* max time difference to GPS Toe (s) */
#define MAXDTOE_QZS 7200.0              /* max time difference to QZSS Toe (s) */
#define MAXDTOE_GAL 14400.0             /* max time difference to Galileo Toe (s) */
#define MAXDTOE_CMP 21600.0             /* max time difference to BeiDou Toe (s) */
#define MAXDTOE_GLO 1800.0              /* max time difference to GLONASS Toe (s) */
#define MAXDTOE_IRN 7200.0              /* max time difference to IRNSS Toe (s) */
#define MAXDTOE_SBS 360.0               /* max time difference to SBAS Toe (s) */
#define MAXDTOE_S   86400.0             /* max time difference to ephem toe (s) for other */
#define MAXGDOP     300.0               /* max GDOP */

#define INT_SWAP_TRAC 86400.0           /* swap interval of trace file (s) */
#define INT_SWAP_STAT 86400.0           /* swap interval of solution status file (s) */

#define MAXEXFILE   1024                /* max number of expanded files */
#define MAXSBSAGEF  30.0                /* max age of SBAS fast correction (s) */
#define MAXSBSAGEL  1800.0              /* max age of SBAS long term corr (s) */
#define MAXSBSURA   8                   /* max URA of SBAS satellite */
#define MAXBAND     10                  /* max SBAS band of IGP */
#define MAXNIGP     201                 /* max number of IGP in SBAS band */
#define MAXNGEO     4                   /* max number of GEO satellites */
#define MAXCOMMENT  100                 /* max number of RINEX comments */
#define MAXSTRPATH  1024                /* max length of stream path */
#define MAXSTRMSG   1024                /* max length of stream message */
#define MAXSTRRTK   8                   /* max number of stream in RTK server */
#define MAXSBSMSG   32                  /* max number of SBAS msg in RTK server */
#define MAXSOLMSG   8191                /* max length of solution message */
#define MAXRAWLEN   16384               /* max length of receiver raw message */
#define MAXERRMSG   4096                /* max length of error/warning message */
#define MAXANT      64                  /* max length of station name/antenna type */
#define MAXSOLBUF   256                 /* max number of solution buffer */
#define MAXOBSBUF   128                 /* max number of observation data buffer */
#define MAXNRPOS    16                  /* max number of reference positions */
#define MAXLEAPS    64                  /* max number of leap seconds table */
#define MAXGISLAYER 32                  /* max number of GIS data layers */
#define MAXRCVCMD   4096                /* max length of receiver commands */

#define RNX2VER     2.10                /* RINEX ver.2 default output version */
#define RNX3VER     3.00                /* RINEX ver.3 default output version */

#define OBSTYPE_PR  0x01                /* observation type: pseudorange */
#define OBSTYPE_CP  0x02                /* observation type: carrier-phase */
#define OBSTYPE_DOP 0x04                /* observation type: doppler-freq */
#define OBSTYPE_SNR 0x08                /* observation type: SNR */
#define OBSTYPE_ALL 0xFF                /* observation type: all */

#define FREQTYPE_L1 0x01                /* frequency type: L1/E1/B1 */
#define FREQTYPE_L2 0x02                /* frequency type: L2/E5b/B2 */
#define FREQTYPE_L3 0x04                /* frequency type: L5/E5a/L3 */
#define FREQTYPE_L4 0x08                /* frequency type: L6/E6/B3 */
#define FREQTYPE_L5 0x10                /* frequency type: E5ab */
#define FREQTYPE_ALL 0xFF               /* frequency type: all */

#define CODE_NONE   0                   /* obs code: none or unknown */
#define CODE_L1C    1                   /* obs code: L1C/A,G1C/A,E1C (GPS,GLO,GAL,QZS,SBS) */
#define CODE_L1P    2                   /* obs code: L1P,G1P,B1P (GPS,GLO,BDS) */
#define CODE_L1W    3                   /* obs code: L1 Z-track (GPS) */
#define CODE_L1Y    4                   /* obs code: L1Y        (GPS) */
#define CODE_L1M    5                   /* obs code: L1M        (GPS) */
#define CODE_L1N    6                   /* obs code: L1codeless,B1codeless (GPS,BDS) */
#define CODE_L1S    7                   /* obs code: L1C(D)     (GPS,QZS) */
#define CODE_L1L    8                   /* obs code: L1C(P)     (GPS,QZS) */
#define CODE_L1E    9                   /* (not used) */
#define CODE_L1A    10                  /* obs code: E1A,B1A    (GAL,BDS) */
#define CODE_L1B    11                  /* obs code: E1B        (GAL) */
#define CODE_L1X    12                  /* obs code: E1B+C,L1C(D+P),B1D+P (GAL,QZS,BDS) */
#define CODE_L1Z    13                  /* obs code: E1A+B+C,L1S (GAL,QZS) */
#define CODE_L2C    14                  /* obs code: L2C/A,G1C/A (GPS,GLO) */
#define CODE_L2D    15                  /* obs code: L2 L1C/A-(P2-P1) (GPS) */
#define CODE_L2S    16                  /* obs code: L2C(M)     (GPS,QZS) */
#define CODE_L2L    17                  /* obs code: L2C(L)     (GPS,QZS) */
#define CODE_L2X    18                  /* obs code: L2C(M+L),B1_2I+Q (GPS,QZS,BDS) */
#define CODE_L2P    19                  /* obs code: L2P,G2P    (GPS,GLO) */
#define CODE_L2W    20                  /* obs code: L2 Z-track (GPS) */
#define CODE_L2Y    21                  /* obs code: L2Y        (GPS) */
#define CODE_L2M    22                  /* obs code: L2M        (GPS) */
#define CODE_L2N    23                  /* obs code: L2codeless (GPS) */
#define CODE_L5I    24                  /* obs code: L5I,E5aI   (GPS,GAL,QZS,SBS) */
#define CODE_L5Q    25                  /* obs code: L5Q,E5aQ   (GPS,GAL,QZS,SBS) */
#define CODE_L5X    26                  /* obs code: L5I+Q,E5aI+Q,L5B+C,B2aD+P (GPS,GAL,QZS,IRN,SBS,BDS) */
#define CODE_L7I    27                  /* obs code: E5bI,B2bI  (GAL,BDS) */
#define CODE_L7Q    28                  /* obs code: E5bQ,B2bQ  (GAL,BDS) */
#define CODE_L7X    29                  /* obs code: E5bI+Q,B2bI+Q (GAL,BDS) */
#define CODE_L6A    30                  /* obs code: E6A,B3A    (GAL,BDS) */
#define CODE_L6B    31                  /* obs code: E6B        (GAL) */
#define CODE_L6C    32                  /* obs code: E6C        (GAL) */
#define CODE_L6X    33                  /* obs code: E6B+C,LEXS+L,B3I+Q (GAL,QZS,BDS) */
#define CODE_L6Z    34                  /* obs code: E6A+B+C,L6D+E (GAL,QZS) */
#define CODE_L6S    35                  /* obs code: L6S        (QZS) */
#define CODE_L6L    36                  /* obs code: L6L        (QZS) */
#define CODE_L8I    37                  /* obs code: E5abI      (GAL) */
#define CODE_L8Q    38                  /* obs code: E5abQ      (GAL) */
#define CODE_L8X    39                  /* obs code: E5abI+Q,B2abD+P (GAL,BDS) */
#define CODE_L2I    40                  /* obs code: B1_2I      (BDS) */
#define CODE_L2Q    41                  /* obs code: B1_2Q      (BDS) */
#define CODE_L6I    42                  /* obs code: B3I        (BDS) */
#define CODE_L6Q    43                  /* obs code: B3Q        (BDS) */
#define CODE_L3I    44                  /* obs code: G3I        (GLO) */
#define CODE_L3Q    45                  /* obs code: G3Q        (GLO) */
#define CODE_L3X    46                  /* obs code: G3I+Q      (GLO) */
#define CODE_L1I    47                  /* obs code: B1I        (BDS) (obsolute) */
#define CODE_L1Q    48                  /* obs code: B1Q        (BDS) (obsolute) */
#define CODE_L5A    49                  /* obs code: L5A SPS    (IRN) */
#define CODE_L5B    50                  /* obs code: L5B RS(D)  (IRN) */
#define CODE_L5C    51                  /* obs code: L5C RS(P)  (IRN) */
#define CODE_L9A    52                  /* obs code: SA SPS     (IRN) */
#define CODE_L9B    53                  /* obs code: SB RS(D)   (IRN) */
#define CODE_L9C    54                  /* obs code: SC RS(P)   (IRN) */
#define CODE_L9X    55                  /* obs code: SB+C       (IRN) */
#define CODE_L1D    56                  /* obs code: B1D        (BDS) */
#define CODE_L5D    57                  /* obs code: L5D(L5S),B2aD (QZS,BDS) */
#define CODE_L5P    58                  /* obs code: L5P(L5S),B2aP (QZS,BDS) */
#define CODE_L5Z    59                  /* obs code: L5D+P(L5S) (QZS) */
#define CODE_L6E    60                  /* obs code: L6E        (QZS) */
#define CODE_L7D    61                  /* obs code: B2bD       (BDS) */
#define CODE_L7P    62                  /* obs code: B2bP       (BDS) */
#define CODE_L7Z    63                  /* obs code: B2bD+P     (BDS) */
#define CODE_L8D    64                  /* obs code: B2abD      (BDS) */
#define CODE_L8P    65                  /* obs code: B2abP      (BDS) */
#define CODE_L4A    66                  /* obs code: G1aL1OCd   (GLO) */
#define CODE_L4B    67                  /* obs code: G1aL1OCd   (GLO) */
#define CODE_L4X    68                  /* obs code: G1al1OCd+p (GLO) */
#define MAXCODE     68                  /* max number of obs code */

#define PMODE_SINGLE 0                  /* positioning mode: single */
#define PMODE_DGPS   1                  /* positioning mode: DGPS/DGNSS */
#define PMODE_KINEMA 2                  /* positioning mode: kinematic */
#define PMODE_STATIC 3                  /* positioning mode: static */
#define PMODE_MOVEB  4                  /* positioning mode: moving-base */
#define PMODE_FIXED  5                  /* positioning mode: fixed */
#define PMODE_PPP_KINEMA 6              /* positioning mode: PPP-kinemaric */
#define PMODE_PPP_STATIC 7              /* positioning mode: PPP-static */
#define PMODE_PPP_FIXED 8               /* positioning mode: PPP-fixed */

#define SOLF_LLH    0                   /* solution format: lat/lon/height */
#define SOLF_XYZ    1                   /* solution format: x/y/z-ecef */
#define SOLF_ENU    2                   /* solution format: e/n/u-baseline */
#define SOLF_NMEA   3                   /* solution format: NMEA-183 */
#define SOLF_STAT   4                   /* solution format: solution status */
#define SOLF_GSIF   5                   /* solution format: GSI F1/F2 */

#define SOLQ_NONE   0                   /* solution status: no solution */
#define SOLQ_FIX    1                   /* solution status: fix */
#define SOLQ_FLOAT  2                   /* solution status: float */
#define SOLQ_SBAS   3                   /* solution status: SBAS */
#define SOLQ_DGPS   4                   /* solution status: DGPS/DGNSS */
#define SOLQ_SINGLE 5                   /* solution status: single */
#define SOLQ_PPP    6                   /* solution status: PPP */
#define SOLQ_DR     7                   /* solution status: dead reconing */
#define MAXSOLQ     7                   /* max number of solution status */

#define TIMES_GPST  0                   /* time system: gps time */
#define TIMES_UTC   1                   /* time system: utc */
#define TIMES_JST   2                   /* time system: jst */

#define IONOOPT_OFF 0                   /* ionosphere option: correction off */
#define IONOOPT_BRDC 1                  /* ionosphere option: broadcast model */
#define IONOOPT_SBAS 2                  /* ionosphere option: SBAS model */
#define IONOOPT_IFLC 3                  /* ionosphere option: L1/L2 iono-free LC */
#define IONOOPT_EST 4                   /* ionosphere option: estimation */
#define IONOOPT_TEC 5                   /* ionosphere option: IONEX TEC model */
#define IONOOPT_QZS 6                   /* ionosphere option: QZSS broadcast model */
#define IONOOPT_STEC 8                  /* ionosphere option: SLANT TEC model */

#define TROPOPT_OFF 0                   /* troposphere option: correction off */
#define TROPOPT_SAAS 1                  /* troposphere option: Saastamoinen model */
#define TROPOPT_SBAS 2                  /* troposphere option: SBAS model */
#define TROPOPT_EST 3                   /* troposphere option: ZTD estimation */
#define TROPOPT_ESTG 4                  /* troposphere option: ZTD+grad estimation */
#define TROPOPT_ZTD 5                   /* troposphere option: ZTD correction */

#define EPHOPT_BRDC 0                   /* ephemeris option: broadcast ephemeris */
#define EPHOPT_PREC 1                   /* ephemeris option: precise ephemeris */
#define EPHOPT_SBAS 2                   /* ephemeris option: broadcast + SBAS */
#define EPHOPT_SSRAPC 3                 /* ephemeris option: broadcast + SSR_APC */
#define EPHOPT_SSRCOM 4                 /* ephemeris option: broadcast + SSR_COM */

#define ARMODE_OFF  0                   /* AR mode: off */
#define ARMODE_CONT 1                   /* AR mode: continuous */
#define ARMODE_INST 2                   /* AR mode: instantaneous */
#define ARMODE_FIXHOLD 3                /* AR mode: fix and hold */
#define ARMODE_WLNL 4                   /* AR mode: wide lane/narrow lane */
#define ARMODE_TCAR 5                   /* AR mode: triple carrier ar */

#define SBSOPT_LCORR 1                  /* SBAS option: long term correction */
#define SBSOPT_FCORR 2                  /* SBAS option: fast correction */
#define SBSOPT_ICORR 4                  /* SBAS option: ionosphere correction */
#define SBSOPT_RANGE 8                  /* SBAS option: ranging */

#define POSOPT_POS   0                  /* pos option: LLH/XYZ */
#define POSOPT_SINGLE 1                 /* pos option: average of single pos */
#define POSOPT_FILE  2                  /* pos option: read from pos file */
#define POSOPT_RINEX 3                  /* pos option: rinex header pos */
#define POSOPT_RTCM  4                  /* pos option: rtcm/raw station pos */

#define LLI_SLIP    0x01                /* LLI: cycle-slip */
#define LLI_HALFC   0x02                /* LLI: half-cycle not resovled */
#define LLI_BOCTRK  0x04                /* LLI: boc tracking of mboc signal */
#define LLI_HALFA   0x40                /* LLI: half-cycle added */
#define LLI_HALFS   0x80                /* LLI: half-cycle subtracted */

#define P2_5        0.03125             /* 2^-5 */
#define P2_6        0.015625            /* 2^-6 */
#define P2_11       4.882812500000000E-04 /* 2^-11 */
#define P2_15       3.051757812500000E-05 /* 2^-15 */
#define P2_17       7.629394531250000E-06 /* 2^-17 */
#define P2_19       1.907348632812500E-06 /* 2^-19 */
#define P2_20       9.536743164062500E-07 /* 2^-20 */
#define P2_21       4.768371582031250E-07 /* 2^-21 */
#define P2_23       1.192092895507810E-07 /* 2^-23 */
#define P2_24       5.960464477539063E-08 /* 2^-24 */
#define P2_27       7.450580596923828E-09 /* 2^-27 */
#define P2_29       1.862645149230957E-09 /* 2^-29 */
#define P2_30       9.313225746154785E-10 /* 2^-30 */
#define P2_31       4.656612873077393E-10 /* 2^-31 */
#define P2_32       2.328306436538696E-10 /* 2^-32 */
#define P2_33       1.164153218269348E-10 /* 2^-33 */
#define P2_35       2.910383045673370E-11 /* 2^-35 */
#define P2_38       3.637978807091710E-12 /* 2^-38 */
#define P2_39       1.818989403545856E-12 /* 2^-39 */
#define P2_40       9.094947017729280E-13 /* 2^-40 */
#define P2_43       1.136868377216160E-13 /* 2^-43 */
#define P2_48       3.552713678800501E-15 /* 2^-48 */
#define P2_50       8.881784197001252E-16 /* 2^-50 */
#define P2_55       2.775557561562891E-17 /* 2^-55 */

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


typedef struct {        /* observation data record */
	gtime_t time;       /* receiver sampling time (GPST) */
	unsigned char sat, rcv; /* satellite/receiver number */
	unsigned char SNR[NFREQ + NEXOBS]; /* signal strength (0.25 dBHz) */
	unsigned char LLI[NFREQ + NEXOBS]; /* loss of lock indicator */
	unsigned char code[NFREQ + NEXOBS]; /* code indicator (CODE_???) */
	double L[NFREQ + NEXOBS]; /* observation data carrier-phase (cycle) */
	double P[NFREQ + NEXOBS]; /* observation data pseudorange (m) */
	float  D[NFREQ + NEXOBS]; /* observation data doppler frequency (Hz) */
} obsd_t;

typedef struct {        /* observation data */
	uint32_t n, nmax;   /* number of obervation data/allocated */
	obsd_t *data;       /* observation data records */
} obs_t;

typedef struct {        /* GPS/QZS/GAL broadcast ephemeris type
	* Stores broadcast ephemeris parameters for GPS, QZSS, and Galileo satellites.
	* Includes satellite orbit, clock, health, and correction parameters for navigation solutions.
	*/
	uint16_t sat;       /* satellite number */
	uint16_t iode;      /* Issue of Data Ephemeris (IODE) */
	uint16_t iodc;      /* Issue of Data Clock (IODC) */
	uint16_t sva;       /* SV accuracy (URA index) */
	uint16_t svh;       /* SV health (0:ok) */
	uint16_t week;      /* GPS/QZS: GPS week, GAL: Galileo week */
	uint8_t code;       /* GPS/QZS: code on L2, GAL/CMP: data sources */
	uint8_t flag;       /* GPS/QZS: L2 P data flag, CMP: nav type */
	gtime_t toe, toc, ttr; /* Toe: ephemeris reference time, Toc: clock data reference time, T_trans: transmission time */
	/* Satellite orbit parameters */
	double A;           /* semi-major axis (m) */
	double e;           /* orbit eccentricity */
	double i0;          /* inclination angle at reference time (rad) */
	double OMG0;        /* longitude of ascending node of orbit plane at weekly epoch (rad) */
	double omg;         /* argument of perigee (rad) */
	double M0;          /* mean anomaly at reference time (rad) */
	double deln;        /* mean motion difference from computed value (rad/s) */
	double OMGd;        /* rate of right ascension (rad/s) */
	double idot;        /* rate of inclination angle (rad/s) */
	double crc, crs;    /* amplitude of the cosine/sine harmonic correction term to the orbit radius (m) */
	double cuc, cus;    /* amplitude of the cosine/sine harmonic correction term to the argument of latitude (rad) */
	double cic, cis;    /* amplitude of the cosine/sine harmonic correction term to the angle of inclination (rad) */
	double toes;        /* Toe (s) in week */
	uint16_t fit;       /* fit interval (hours) */
	float f0, f1, f2;   /* SV clock parameters (af0: bias, af1: drift, af2: drift rate) */
	float tgd[4];       /* group delay parameters
						   GPS/QZS: tgd[0]=TGD
						   GAL    : tgd[0]=BGD E5a/E1, tgd[1]=BGD E5b/E1
						   CMP    : tgd[0]=BGD1, tgd[1]=BGD2 */
	double Adot, ndot;  /* Adot: rate of change of semi-major axis, ndot: rate of change of mean motion (for CNAV) */
} eph_t;

typedef struct {        /* GLONASS broadcast ephemeris type */
	uint16_t sat;            /* satellite number */
	uint8_t iode;       /* IODE (0-6 bit of tb field) */
	uint8_t frq;        /* satellite frequency number */
	uint16_t svh;
	uint16_t sva;
	uint16_t age;       /* satellite health, accuracy, age of operation */
	gtime_t toe;        /* epoch of epherides (gpst) */
	gtime_t tof;        /* message frame time (gpst) */
	double pos[3];      /* satellite position (ecef) (m) */
	double vel[3];      /* satellite velocity (ecef) (m/s) */
	double acc[3];      /* satellite acceleration (ecef) (m/s^2) */
	double taun, gamn;   /* SV clock bias (s)/relative freq bias */
	double dtaun;       /* delay between L1 and L2 (s) */
} geph_t;

typedef struct {        /* precise ephemeris type */
	gtime_t time;       /* time (GPST) */
	uint16_t index;          /* ephemeris index for multiple files */
	double pos[MAXSAT][4]; /* satellite position/clock (ecef) (m|s) */
	float  std[MAXSAT][4]; /* satellite position/clock std (m|s) */
	double vel[MAXSAT][4]; /* satellite velocity/clk-rate (m/s|s/s) */
	float  vst[MAXSAT][4]; /* satellite velocity/clk-rate std (m/s|s/s) */
	float  cov[MAXSAT][3]; /* satellite position covariance (m^2) */
	float  vco[MAXSAT][3]; /* satellite velocity covariance (m^2) */
} peph_t;

typedef struct {        /* precise clock type */
	gtime_t time;       /* time (GPST) */
	uint16_t index;          /* clock index for multiple files */
	float  clk[MAXSAT][1]; /* satellite clock (s) */
	float  std[MAXSAT][1]; /* satellite clock std (s) */
} pclk_t;

typedef struct {        /* SBAS ephemeris type */
	uint16_t sat;            /* satellite number */
	gtime_t t0;         /* reference epoch time (GPST) */
	gtime_t tof;        /* time of message frame (GPST) */
	uint8_t sva;            /* SV accuracy (URA index) */
	uint8_t svh;            /* SV health (0:ok) */
	double pos[3];      /* satellite position (m) (ecef) */
	double vel[3];      /* satellite velocity (m/s) (ecef) */
	double acc[3];      /* satellite acceleration (m/s^2) (ecef) */
	float  af0;
	float  af1;     /* satellite clock-offset/drift (s,s/s) */
} seph_t;

typedef struct {        /* satellite measurement and status type
	* Stores measurement results and status for each satellite in the current epoch.
	* Includes PRN, position/velocity, clock correction, variance, health, and azimuth/elevation.
	*/
	uint16_t prn[MAXOBS];        /* satellite PRN numbers */
	double   rs[MAXOBS][6];      /* satellite position/velocity/clock {x,y,z,vx,vy,vz} (m|m/s) */
	float    dts[MAXOBS][2];     /* satellite clock correction and drift {dt, dtdt} (s, s/s) */
	float    vare[MAXOBS];       /* variance of satellite position (m^2) */
	uint16_t svh[MAXOBS];        /* satellite health status */
	float    azel[MAXOBS][2];    /* satellite azimuth/elevation angles {az, el} (rad) */
} sat_t;

typedef struct {        /* station parameter type */
	char name[MAXANT]; /* marker name */
	char marker[MAXANT]; /* marker number */
	char antdes[MAXANT]; /* antenna descriptor */
	char antsno[MAXANT]; /* antenna serial number */
	char rectype[MAXANT]; /* receiver type descriptor */
	char recver[MAXANT]; /* receiver firmware version */
	char recsno[MAXANT]; /* receiver serial number */
	int antsetup;       /* antenna setup id */
	int itrf;           /* ITRF realization year */
	int deltype;        /* antenna delta type (0:enu,1:xyz) */
	double pos[3];      /* station position (ecef) (m) */
	double del[3];      /* antenna position delta (e/n/u or x/y/z) (m) */
	double hgt;         /* antenna height (m) */
	int glo_cp_align;   /* GLONASS code-phase alignment (0:no,1:yes) */
	double glo_cp_bias[4]; /* GLONASS code-phase biases {1C,1P,2C,2P} (m) */
} sta_t;

typedef struct {        /* navigation data type */
	uint16_t n, nmax;         /* number of broadcast ephemeris */
	uint16_t ng, ngmax;       /* number of glonass ephemeris */
	uint16_t ne, nemax;       /* number of precise ephemeris */
	uint16_t nc, ncmax;       /* number of precise clock */
	eph_t  *eph;         /* GPS/QZS/GAL ephemeris */
	geph_t *geph;       /* GLONASS ephemeris */
	peph_t *peph;       /* precise ephemeris */
	pclk_t *pclk;       /* precise clock */
	float utc_gps[4];  /* GPS delta-UTC parameters {A0,A1,T,W} */
	float utc_glo[4];  /* GLONASS UTC GPS time parameters */
	float utc_gal[4];  /* Galileo UTC GPS time parameters */
	float utc_qzs[4];  /* QZS UTC GPS time parameters */
	float utc_cmp[4];  /* BeiDou UTC parameters */
	float ion_gps[8];  /* GPS iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
	float ion_gal[4];  /* Galileo iono model parameters {ai0,ai1,ai2,0} */
	float ion_qzs[8];  /* QZSS iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
	float ion_cmp[8];  /* BeiDou iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
	int leaps;          /* leap seconds (s) */
	double lam[MAXSAT][NFREQ]; /* carrier wave lengths (m) */
	float cbias[MAXSAT][3];   /* code bias (0:p1-p2,1:p1-c1,2:p2-c2) (m) */
	float wlbias[MAXSAT];     /* wide-lane bias (cycle) */
	float glo_cpbias[4];    /* glonass code-phase bias {1C,1P,2C,2P} (m) */
	char glo_fcn[MAXPRNGLO + 1]; /* glonass frequency channel number + 8 */
} nav_t;

typedef struct {        /* SNR mask type */
	uint8_t ena[2];         /* enable flag {rover,base} */
	float   mask[NFREQ][9]; /* mask (dBHz) at 5,10,...85 deg */
} snrmask_t;

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