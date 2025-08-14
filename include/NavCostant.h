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
 *                            Yejin     : yejinbenjohn@qq.com
 *                            Liuhuihao :							 
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

#include <stdint.h>

#define  CONFIGSIM      // Enable simulation configuration // 启用仿真配置
#define  GNSSMODEL      // Enable GNSS model // 启用GNSS模型
#define  TRACE          // Enable trace output // 启用跟踪输出
#define  FILEIO         // Enable file input/output // 启用文件输入输出

// Frequency mode macros for each GNSS system // 各系统频点组合模式宏
#define  FREQMODE_CMP  2     // BeiDou frequency mode: 0=B1+B2, 1=B1+B2a, 2=B1+B3 // 北斗频点模式
#define  FREQMODE_GPS  0     // GPS frequency mode: 0=L1+L2, 1=L1+L5 // GPS频点模式
#define  FREQMODE_GAL  0     // Galileo frequency mode: 0=L1+L2, 1=L1+L5, 2=L1+E5b // 伽利略频点模式

// Math and utility macros // 数学与通用宏
#define SQR(x)      ((x)*(x))                  // Square of x // x的平方
#define SQRT(x)     ((x)<=0.0?0.0:sqrt(x))     // Safe square root // 安全开方
#define ROUND(x)    (int)(floor((x)+0.5))      // Round to nearest integer // 四舍五入
#define ROUND_U(x)	((unsigned int)floor((x)+0.5)) // Round to nearest unsigned int // 四舍五入为无符号整数

// Various constants // 各类常量
#define MAXEORSTA   10		   // Max number of satellites for gross error detection // 最大粗差探测卫星数
#define MAXFALSTA   20         // Max number of dropped satellites in 5 epochs // 5个历元内降星卫星数
#define DOPSIP      13.0       // Doppler threshold for phase slip // 多普勒周跳阈值
#define RECCLOCK    500.0      // Receiver clock jump threshold // 接收机钟跳阈值
#define ZEROS_MIN   1E-12      // Threshold for double zero // double类型零阈值

#define DPS_IN_KALPNT   2      // Kalman SPP Doppler usage: 1=off, 2=on // 卡尔曼单点多普勒开关
#define DPS_IN_RTD      2      // RTD Doppler usage: 1=off, 2=on // RTD多普勒开关
#define DIF_L_IN_EVL    1      // Least squares velocity: 1=off, 2=on // 最小二乘定速伪距差分开关

#define VAR_POS     SQR(30.0)  // Initial variance of receiver position (m^2) // 初始位置方差
#define VAR_VEL     SQR(10.0)  // Initial variance of receiver velocity ((m/s)^2) // 初始速度方差
#define VAR_ACC     SQR(10.0)  // Initial variance of receiver acceleration ((m/ss)^2) // 初始加速度方差
#define VAR_HWBIAS  SQR(1.0)   // Initial variance of hardware bias ((m/MHz)^2) // 初始硬件偏差方差
#define VAR_GRA     SQR(0.001) // Initial variance of gradient (m^2) // 初始梯度方差
#define INIT_ZWD    0.15       // Initial zenith wet delay (m) // 初始对流层湿延迟

#define VAR_POS_B     SQR(500.0)  // Initial variance of base position (m^2) // 基站初始位置方差
#define VAR_VEL_B     SQR(100.0)  // Initial variance of base velocity ((m/s)^2) // 基站初始速度方差
#define VAR_DOP_B     0.001       // Variance of Doppler measurement update // 多普勒测量更新方差
#define VAR_CLK_B     SQR(20.0)   // Initial variance of receiver clock correction // 接收机钟差初始方差
#define VAR_SHIFT_B   SQR(5.0)    // Initial variance of receiver clock shift // 接收机钟移初始方差

#define PRN_HWBIAS  1E-6     // Process noise of hardware bias (m/MHz/sqrt(s)) // 硬件偏差过程噪声
#define GAP_RESION  120      // Gap to reset ionosphere parameters (epochs) // 电离层参数重置间隔
#define MAXACC      30.0     // Max acceleration for Doppler slip detection (m/s^2) // 多普勒周跳最大加速度

#define VAR_HOLDAMB 0.25     // Constraint to hold ambiguity (cycle^2) // 固定模糊度约束

#define TTOL_MOVEB  (1.0+2*DTTOL) // Time sync tolerance for moving-baseline (s) // 动基线时间同步容差

// State parameter macros // 状态参数宏
#define NF(opt)     ((opt)->ionoopt==IONOOPT_IFLC?1:(opt)->nf) // Number of frequencies // 频点数
#define NP(opt)     ((opt)->dynamics==0?3:3)                   // Number of position parameters // 位置参数数
#define NI(opt)     ((opt)->ionoopt!=IONOOPT_EST?0:MAXSAT)     // Number of ionosphere parameters // 电离层参数数
#define NT(opt)     ((opt)->tropopt<TROPOPT_EST?0:((opt)->tropopt<TROPOPT_ESTG?2:6)) // Number of troposphere parameters // 对流层参数数
#define NL(opt)     ((opt)->glomodear!=2?0:NFREQGLO)           // Number of GLONASS parameters // GLONASS参数数
#define NB(opt)     ((opt)->mode<=PMODE_DGPS?0:MAXSAT*NF(opt)) // Number of phase bias parameters // 相位偏差参数数
#define NR(opt)     (NP(opt)+NI(opt)+NT(opt)+NL(opt))          // Number of real parameters // 实参数量
#define NX(opt)     (NR(opt)+NB(opt))                          // Number of estimated parameters // 估计参数量

// State variable index macros // 状态变量索引宏
#define II(s,opt)   (NP(opt)+(s)-1)                 // Ionosphere index // 电离层索引
#define IT(r,opt)   (NP(opt)+NI(opt)+NT(opt)/2*(r)) // Troposphere index // 对流层索引
#define IL(f,opt)   (NP(opt)+NI(opt)+NT(opt)+(f))   // Hardware bias index // 硬件偏差索引
#define IB2(s,f,opt,ns) (NR(opt)+ns*(f)+(s))        // Phase bias index // 相位偏差索引

#define MIN_DD        5     // Minimum number of double-difference // 最小双差数
#define MIN_DDSAT     4     // Minimum number of double-difference satellites // 最小双差卫星数

#define SMALL       1.0e-20 // Small value for numerical stability // 数值稳定用小量
#define MAXINT      32766   // Maximum value for int type // int类型最大值
#define KNOT2M      0.514444444  // Conversion: knot to meter // 节转米

// Physical and astronomical constants // 物理与天文常量
#define PI          3.1415926535897932  // Pi // 圆周率
#define D2R         (PI/180.0)          // Degrees to radians // 角度转弧度
#define R2D         (180.0/PI)          // Radians to degrees // 弧度转角度
#define CLIGHT      299792458.0         // Speed of light (m/s) // 光速
#define SC2RAD      3.1415926535898     // Semi-circle to radian // 半圆转弧度
#define AU          149597870691.0      // Astronomical unit (m) // 天文单位
#define AS2R        (D2R/3600.0)        // Arcsec to radian // 角秒转弧度

#define OMGE        7.2921151467E-5     // Earth angular velocity (rad/s) // 地球自转角速度

#define RE_WGS84    6378137.0           // Earth semimajor axis (WGS84) (m) // 地球长半轴
#define FE_WGS84    (1.0/298.257223563) // Earth flattening (WGS84) // 地球扁率

#define HION        350000.0            // Ionosphere height (m) // 电离层高度

#define MAXFREQ     7                   // Max number of frequencies // 最大频点数
#define MINREFSTARELEV 35               // Minimum reference star elevation (deg) // 最小参考星高度角
#define FREQ1       1.57542E9           // L1/E1/B1C frequency (Hz) // L1/E1/B1C频率
#define FREQ2       1.22760E9           // L2 frequency (Hz) // L2频率
#define FREQ5       1.17645E9           // L5/E5a/B2a frequency (Hz) // L5/E5a/B2a频率
#define FREQ6       1.27875E9           // E6/L6 frequency (Hz) // E6/L6频率
#define FREQ7       1.20714E9           // E5b frequency (Hz) // E5b频率
#define FREQ8       1.191795E9          // E5a+b frequency (Hz) // E5a+b频率
#define FREQ9       2.492028E9          // S frequency (Hz) // S频率
#define FREQ1_GLO   1.60200E9           // GLONASS G1 base frequency (Hz) // GLONASS G1基频
#define DFRQ1_GLO   0.56250E6           // GLONASS G1 bias frequency (Hz/n) // GLONASS G1频偏
#define FREQ2_GLO   1.24600E9           // GLONASS G2 base frequency (Hz) // GLONASS G2基频
#define DFRQ2_GLO   0.43750E6           // GLONASS G2 bias frequency (Hz/n) // GLONASS G2频偏
#define FREQ3_GLO   1.202025E9          // GLONASS G3 frequency (Hz) // GLONASS G3频率
#define FREQ1_CMP   1.561098E9          // BeiDou B1 C2I frequency (Hz) // 北斗B1频率
#define FREQ2_CMP   1.20714E9           // BeiDou B2 C7I frequency (Hz) // 北斗B2频率
#define FREQ3_CMP   1.26852E9           // BeiDou B3 C6I frequency (Hz) // 北斗B3频率
#define FREQ1c_CMP  1.57542E9           // BeiDou B1c/BDS-3 frequency (Hz) // 北斗B1c频率
#define FREQ2a_CMP  1.17645E9           // BeiDou B2a/BDS-3 frequency (Hz) // 北斗B2a频率
#define FREQ5_GAL   1.17645E9           // Galileo E5a frequency (Hz) // 伽利略E5a频率

// Error factor macros for each system // 各系统误差因子宏
#define EFACT_GPS   1.5                 // Error factor: GPS // GPS误差因子
#define EFACT_GLO   1.5                 // Error factor: GLONASS // GLONASS误差因子
#define EFACT_GAL   1.0                 // Error factor: Galileo // 伽利略误差因子
#define EFACT_QZS   1.0                 // Error factor: QZSS // QZSS误差因子
#define EFACT_CMP   1.0                 // Error factor: BeiDou // 北斗误差因子
#define EFACT_IRN   1.5                 // Error factor: IRNSS // IRNSS误差因子
#define EFACT_SBS   3.0                 // Error factor: SBAS // SBAS误差因子

// Navigation system macros // 导航系统宏
#define SYS_NONE    0x00                // Navigation system: none // 无导航系统
#define SYS_GPS     0x01                // Navigation system: GPS // GPS系统
#define SYS_SBS     0x02                // Navigation system: SBAS // SBAS系统
#define SYS_GLO     0x04                // Navigation system: GLONASS // GLONASS系统
#define SYS_GAL     0x08                // Navigation system: Galileo // 伽利略系统
#define SYS_QZS     0x10                // Navigation system: QZSS // QZSS系统
#define SYS_CMP     0x20                // Navigation system: BeiDou // 北斗系统
#define SYS_IRN     0x40                // Navigation system: IRNSS // IRNSS系统
#define SYS_LEO     0x80                // Navigation system: LEO // LEO系统
#define SYS_ALL     0xFF                // Navigation system: all // 所有导航系统

// Time system macros // 时间系统宏
#define TSYS_GPS    0                   // Time system: GPS time // GPS时间
#define TSYS_UTC    1                   // Time system: UTC // UTC时间
#define TSYS_GLO    2                   // Time system: GLONASS time // GLONASS时间
#define TSYS_GAL    3                   // Time system: Galileo time // 伽利略时间
#define TSYS_QZS    4                   // Time system: QZSS time // QZSS时间
#define TSYS_CMP    5                   // Time system: BeiDou time // 北斗时间
#define TSYS_IRN    6                   // Time system: IRNSS time // IRNSS时间

#ifndef NFREQ
#define NFREQ       2                   /* number of carrier frequencies */
#endif
#define NFREQGLO    2                   /* number of carrier frequencies of GLONASS */

#ifndef NEXOBS
#define NEXOBS      0                   /* number of extended obs codes */
#endif

#define MINPRNGPS   1                   /* min satellite PRN number of GPS */
#define MAXPRNGPS   32                  /* max satellite PRN number of GPS */
#define NSATGPS     (MAXPRNGPS-MINPRNGPS+1) /* number of GPS satellites */
#define NSYSGPS     1
#define ENAGLO
#ifdef ENAGLO
#define MINPRNGLO   1                       /* min satellite slot number of GLONASS */
#define MAXPRNGLO   32                      /* max satellite slot number of GLONASS */
#define NSATGLO     (MAXPRNGLO-MINPRNGLO+1) /* number of GLONASS satellites */
#define NSYSGLO     1
#else
#define MINPRNGLO   0
#define MAXPRNGLO   0
#define NSATGLO     0
#define NSYSGLO     0
#endif

#define ENACMP
#ifdef ENACMP
#define MINPRNCMP   1                       /* min satellite sat number of BeiDou */
#define MAXPRNCMP   64                      /* max satellite sat number of BeiDou */
#define NSATCMP     (MAXPRNCMP-MINPRNCMP+1) /* number of BeiDou satellites */
#define NSYSCMP     1
#else
#define MINPRNCMP   0
#define MAXPRNCMP   0
#define NSATCMP     0
#define NSYSCMP     0
#endif
#define ENAGAL
#ifdef ENAGAL
#define MINPRNGAL   1                      /* min satellite PRN number of Galileo */
#define MAXPRNGAL   36                     /* max satellite PRN number of Galileo */
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
#define MAXPRNQZS   199                 /* max satellite PRN number of QZSS */
#define MINPRNQZS_S 183                 /* min satellite PRN number of QZSS SAIF */
#define MAXPRNQZS_S 189                 /* max satellite PRN number of QZSS SAIF */
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

#ifdef ENAIRN
#define MINPRNIRN   1                   /* min satellite sat number of IRNSS */
#define MAXPRNIRN   7                   /* max satellite sat number of IRNSS */
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

#define MINPRNSBS   120                     /* min satellite PRN number of SBAS */
#define MAXPRNSBS   142                     /* max satellite PRN number of SBAS */
#define NSATSBS     (MAXPRNSBS-MINPRNSBS+1) /* number of SBAS satellites */

#define MAXSAT      (NSATGPS+NSATGLO+NSATGAL+NSATCMP)   /* max satellite number (1 to MAXSAT) */

#define MAXSTA      255
 
#define MAXRECOBS   80                  /*max number of reception obs in an epoch ** ?原来是40 ***/
#define MAXPNTOBS   50                  /*一个历元参与单点解算的最大卫星数 必须小于等于MAXRECOBS*/
#define MAXDIFOBS   22                  /*一个历元参与差分解算的最大卫星数*/

#define MAXEPHNUM   80                  /*星历最大存储个数*/
#define NA   3

#define MAXOBSNF      (MAXDIFOBS*NFREQ)
#define MAXOBSNFA     (MAXDIFOBS*NFREQ+NA)
#define MAXOBSNF2     (MAXOBSNF*MAXOBSNF)
#define MAXOBSNFA2    (MAXOBSNFA*MAXOBSNFA)

#define LIMITBDSGEO 0   /*单点是否采用BDS GEO卫星，以及rtd、rtk浮点选参考星时是否选用BDS GEO卫星，906A设备观测到的卫星较少，此处暂不建议做此项限制*/
#define BDSGEOFLAG(prn)   ((prn<=5)||(prn==59)||(prn==60)||(prn==62)) ?1:0  // bdsgeo

#define MAXRCV      64 /*64*/           /* max receiver number (1 to MAXRCV) */
#define MAXOBSTYPE  64                  /* max number of obs type in RINEX */
#define DTTOL       0.005               /* tolerance of time difference (s) */
#define MAXDTOE     7200.0              /* max time difference to GPS Toe (s) */
#define MAXSAVEDTOE 4*3600              /* max save time difference to GPS Toe (s) */
#define MAXDTOE_GLO 1800.0              /* max time difference to GLONASS Toe (s) */
#define MAXDTOE_SBS 360.0               /* max time difference to SBAS Toe (s) */
#define MAXDTOE_S   86400.0             /* max time difference to ephem toe (s) for other */
#define MAXGDOP     300.0               /* max GDOP */

#define MAXRNXLEN   (16*MAXOBSTYPE+4)   /* max rinex record length */

#define INT_SWAP_TRAC 86400.0           /* swap interval of trace file (s) */
#define INT_SWAP_STAT 86400.0           /* swap interval of solution status file (s) */

#define MAXEXFILE   5                   /* max number of expanded files */
#define MAXSBSAGEF  30.0                /* max age of SBAS fast correction (s) */
#define MAXSBSAGEL  1800.0              /* max age of SBAS long term corr (s) */
#define MAXSBSURA   8                   /* max URA of SBAS satellite */
#define MAXBAND     10                  /* max SBAS band of IGP */
#define MAXNIGP     201                 /* max number of IGP in SBAS band */
#define MAXNGEO     4                   /* max number of GEO satellites */
#define MAXCOMMENT  10                  /* max number of RINEX comments */
#define MAXSTRPATH  2/*1024*/           /* max length of stream path */
#define MAXSTRMSG   1024                /* max length of stream message */
#define MAXSTRRTK   8                   /* max number of stream in RTK server */
#define MAXSBSMSG   32                  /* max number of SBAS msg in RTK server */
#define MAXRAWLEN   4096                /* max length of receiver raw message */
#define MAXERRMSG   4096                /* max length of error/warning message */
#define MAXANT      20 /*64*/           /* max length of station name/antenna type */
#define MAXSOLBUF   256                 /* max number of solution buffer */
#define MAXOBSBUF   128                 /* max number of observation data buffer */
#define MAXNRPOS    16                  /* max number of reference positions */
#define MAXLEAPS    20    //64          /* max number of leap seconds table */
#define MAXGISLAYER 32                  /* max number of GIS data layers */
#define MAXRCVCMD   4096                /* max length of receiver commands */

#define RNX2VER     2.10                /* RINEX ver.2 default output version */
#define RNX3VER     3.00                /* RINEX ver.3 default output version */
#define STD_GAL_NAPA 500.0              /* error of galileo ephemeris for NAPA (m) */

#define MaxPseudorange_GPS 26E6
#define MinPseudorange_GPS 19E6
#define MaxPseudorange_GLO 26E6
#define MinPseudorange_GLO 18E6
#define MaxPseudorange_BDS 46E6
#define MinPseudorange_BDS 19E6
#define MaxPseudorange_GAL 30E6
#define MinPseudorange_GAL 19E6

#define OBSTYPE_PR  0x01                /* observation type: pseudorange */
#define OBSTYPE_CP  0x02                /* observation type: carrier-phase */
#define OBSTYPE_DOP 0x04                /* observation type: doppler-freq */
#define OBSTYPE_SNR 0x08                /* observation type: SNR */
#define OBSTYPE_ALL 0xFF                /* observation type: all */

#define FREQTYPE_L1 0x01                /* frequency type: L1/E1 */
#define FREQTYPE_L2 0x02                /* frequency type: L2/B1 */
#define FREQTYPE_L5 0x04                /* frequency type: L5/E5a/L3 */
#define FREQTYPE_L6 0x08                /* frequency type: E6/LEX/B3 */
#define FREQTYPE_L7 0x10                /* frequency type: E5b/B2 */
#define FREQTYPE_L8 0x20                /* frequency type: E5(a+b) */
#define FREQTYPE_L9 0x40                /* frequency type: S */
#define FREQTYPE_ALL 0xFF               /* frequency type: all */

#define CODE_NONE   0                   /* obs code: none or unknown */
#define CODE_L1C    1                   /* obs code: L1C/A,G1C/A,E1C (GPS,GLO,GAL,QZS,SBS) */
#define CODE_L1P    2                   /* obs code: L1P,G1P    (GPS,GLO) */
#define CODE_L1W    3                   /* obs code: L1 Z-track (GPS) */
#define CODE_L1Y    4                   /* obs code: L1Y        (GPS) */
#define CODE_L1M    5                   /* obs code: L1M        (GPS) */
#define CODE_L1N    6                   /* obs code: L1codeless (GPS) */
#define CODE_L1S    7                   /* obs code: L1C(D)     (GPS,QZS) */
#define CODE_L1L    8                   /* obs code: L1C(P)     (GPS,QZS) */
#define CODE_L1E    9                   /* (not used) */
#define CODE_L1A    10                  /* obs code: E1A        (GAL) */
#define CODE_L1B    11                  /* obs code: E1B        (GAL) */
#define CODE_L1X    12                  /* obs code: E1B+C,L1C(D+P) (GAL,QZS) */
#define CODE_L1Z    13                  /* obs code: E1A+B+C,L1SAIF (GAL,QZS) */
#define CODE_L2C    14                  /* obs code: L2C/A,G1C/A (GPS,GLO) */
#define CODE_L2D    15                  /* obs code: L2 L1C/A-(P2-P1) (GPS) */
#define CODE_L2S    16                  /* obs code: L2C(M)     (GPS,QZS) */
#define CODE_L2L    17                  /* obs code: L2C(L)     (GPS,QZS) */
#define CODE_L2X    18                  /* obs code: L2C(M+L),B1I+Q (GPS,QZS,CMP) */
#define CODE_L2P    19                  /* obs code: L2P,G2P    (GPS,GLO) */
#define CODE_L2W    20                  /* obs code: L2 Z-track (GPS) */
#define CODE_L2Y    21                  /* obs code: L2Y        (GPS) */
#define CODE_L2M    22                  /* obs code: L2M        (GPS) */
#define CODE_L2N    23                  /* obs code: L2codeless (GPS) */
#define CODE_L5I    24                  /* obs code: L5/E5aI    (GPS,GAL,QZS,SBS) */
#define CODE_L5Q    25                  /* obs code: L5/E5aQ    (GPS,GAL,QZS,SBS) */
#define CODE_L5X    26                  /* obs code: L5/E5aI+Q/L5B+C (GPS,GAL,QZS,IRN,SBS) */
#define CODE_L7I    27                  /* obs code: E5bI,B2I   (GAL,CMP) */
#define CODE_L7Q    28                  /* obs code: E5bQ,B2Q   (GAL,CMP) */
#define CODE_L7X    29                  /* obs code: E5bI+Q,B2I+Q (GAL,CMP) */
#define CODE_L6A    30                  /* obs code: E6A        (GAL) */
#define CODE_L6B    31                  /* obs code: E6B        (GAL) */
#define CODE_L6C    32                  /* obs code: E6C        (GAL) */
#define CODE_L6X    33                  /* obs code: E6B+C,LEXS+L,B3I+Q (GAL,QZS,CMP) */
#define CODE_L6Z    34                  /* obs code: E6A+B+C    (GAL) */
#define CODE_L6S    35                  /* obs code: LEXS       (QZS) */
#define CODE_L6L    36                  /* obs code: LEXL       (QZS) */
#define CODE_L8I    37                  /* obs code: E5(a+b)I   (GAL) */
#define CODE_L8Q    38                  /* obs code: E5(a+b)Q   (GAL) */
#define CODE_L8X    39                  /* obs code: E5(a+b)I+Q (GAL) */
#define CODE_L2I    40                  /* obs code: B1I        (BDS) */
#define CODE_L2Q    41                  /* obs code: B1Q        (BDS) */
#define CODE_L6I    42                  /* obs code: B3I        (BDS) */
#define CODE_L6Q    43                  /* obs code: B3Q        (BDS) */
#define CODE_L3I    44                  /* obs code: G3I        (GLO) */
#define CODE_L3Q    45                  /* obs code: G3Q        (GLO) */
#define CODE_L3X    46                  /* obs code: G3I+Q      (GLO) */
#define CODE_L1I    47                  /* obs code: B1I        (BDS) */
#define CODE_L1Q    48                  /* obs code: B1Q        (BDS) */
#define CODE_L5A    49                  /* obs code: L5A SPS    (IRN) */
#define CODE_L5B    50                  /* obs code: L5B RS(D)  (IRN) */
#define CODE_L5C    51                  /* obs code: L5C RS(P)  (IRN) */
#define CODE_L9A    52                  /* obs code: SA SPS     (IRN) */
#define CODE_L9B    53                  /* obs code: SB RS(D)   (IRN) */
#define CODE_L9C    54                  /* obs code: SC RS(P)   (IRN) */
#define CODE_L9X    55                  /* obs code: SB+C       (IRN) */
#define MAXCODE     55                  /* max number of obs code */

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
#define IONOOPT_IFLC 3                  /* ionosphere option: L1/L2 or L1/L5 iono-free LC */
#define IONOOPT_EST 4                   /* ionosphere option: estimation */
#define IONOOPT_TEC 5                   /* ionosphere option: IONEX TEC model */
#define IONOOPT_QZS 6                   /* ionosphere option: QZSS broadcast model */
#define IONOOPT_LEX 7                   /* ionosphere option: QZSS LEX ionospehre */
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
#define EPHOPT_LEX  5                   /* ephemeris option: QZSS LEX ephemeris */

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
#define POSOPT_RTCM  4                  /* pos option: rtcm station pos */
#define POSOPT_RAW   5                  /* pos option: raw station pos */

#define STR_NONE     0                  /* stream type: none */
#define STR_SERIAL   1                  /* stream type: serial */
#define STR_FILE     2                  /* stream type: file */
#define STR_TCPSVR   3                  /* stream type: TCP server */
#define STR_TCPCLI   4                  /* stream type: TCP client */
#define STR_NTRIPSVR 6                  /* stream type: NTRIP server */
#define STR_NTRIPCLI 7                  /* stream type: NTRIP client */
#define STR_FTP      8                  /* stream type: ftp */
#define STR_HTTP     9                  /* stream type: http */
#define STR_NTRIPC_S 10                 /* stream type: NTRIP caster server */
#define STR_NTRIPC_C 11                 /* stream type: NTRIP caster client */
#define STR_UDPSVR   12                 /* stream type: UDP server */
#define STR_UDPCLI   13                 /* stream type: UDP server */
#define STR_MEMBUF   14                 /* stream type: memory buffer */

#define STRFMT_RTCM2 0                  /* stream format: RTCM 2 */
#define STRFMT_RTCM3 1                  /* stream format: RTCM 3 */
#define STRFMT_OEM4  2                  /* stream format: NovAtel OEMV/4 */
#define STRFMT_OEM3  3                  /* stream format: NovAtel OEM3 */
#define STRFMT_UBX   4                  /* stream format: u-blox LEA-*T */
#define STRFMT_SS2   5                  /* stream format: NovAtel Superstar II */
#define STRFMT_CRES  6                  /* stream format: Hemisphere */
#define STRFMT_STQ   7                  /* stream format: SkyTraq S1315F */
#define STRFMT_GW10  8                  /* stream format: Furuno GW10 */
#define STRFMT_JAVAD 9                  /* stream format: JAVAD GRIL/GREIS */
#define STRFMT_NVS   10                 /* stream format: NVS NVC08C */
#define STRFMT_BINEX 11                 /* stream format: BINEX */
#define STRFMT_RT17  12                 /* stream format: Trimble RT17 */
#define STRFMT_SEPT  13                 /* stream format: Septentrio */
#define STRFMT_CMR   14                 /* stream format: CMR/CMR+ */
#define STRFMT_TERSUS 15                /* stream format: TERSUS */
#define STRFMT_LEXR  16                 /* stream format: Furuno LPY-10000 */
#define STRFMT_RINEX 17                 /* stream format: RINEX */
#define STRFMT_SP3   18                 /* stream format: SP3 */
#define STRFMT_RNXCLK 19                /* stream format: RINEX CLK */
#define STRFMT_SBAS  20                 /* stream format: SBAS messages */
#define STRFMT_NMEA  21                 /* stream format: NMEA 0183 */
#ifndef EXTLEX
#define MAXRCVFMT    15                 /* max number of receiver format */
#else
#define MAXRCVFMT    16
#endif

#define STR_MODE_R  0x1                 /* stream mode: read */
#define STR_MODE_W  0x2                 /* stream mode: write */
#define STR_MODE_RW 0x3                 /* stream mode: read/write */

#define GEOID_EMBEDDED    0             /* geoid model: embedded geoid */
#define GEOID_EGM96_M150  1             /* geoid model: EGM96 15x15" */
#define GEOID_EGM2008_M25 2             /* geoid model: EGM2008 2.5x2.5" */
#define GEOID_EGM2008_M10 3             /* geoid model: EGM2008 1.0x1.0" */
#define GEOID_GSI2000_M15 4             /* geoid model: GSI geoid 2000 1.0x1.5" */
#define GEOID_RAF09       5             /* geoid model: IGN RAF09 for France 1.5"x2" */

#define COMMENTH    "%"                 /* comment line indicator for solution */
#define MSG_DISCONN "$_DISCONNECT\r\n"  /* disconnect message */

#define DLOPT_FORCE   0x01              /* download option: force download existing */
#define DLOPT_KEEPCMP 0x02              /* download option: keep compressed file */
#define DLOPT_HOLDERR 0x04              /* download option: hold on error file */
#define DLOPT_HOLDLST 0x08              /* download option: hold on listing file */

#define LLI_SLIP    0x01                /* LLI: cycle-slip */
#define LLI_HALFC   0x02                /* LLI: half-cycle not resovled */
#define LLI_BOCTRK  0x04                /* LLI: boc tracking of mboc signal */
#define LLI_HALFA   0x40                /* LLI: half-cycle added */
#define LLI_HALFS   0x80                /* LLI: half-cycle subtracted */

#define IMUFMT_KVH  1                   /* imu data format KVH */

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

typedef struct {        /* time struct */
    long long time;    // time (s) expressed by standard time_t  // 标准time_t表示的秒数
    double sec;        // fraction of second under 1 s           // 秒的小数部分（小于1秒）
} gtime_t;

typedef struct {            /* observation data record */
    gtime_t time;           // receiver sampling time (GPST)      // 接收机采样时间（GPST）
    unsigned char sat, rcv; // satellite/receiver number          // 卫星号/接收机号
    unsigned char snr[NFREQ + NEXOBS];  // signal strength (0.25 dBHz) // 信号强度（0.25 dBHz）
    unsigned char lli[NFREQ + NEXOBS];  // loss of lock indicator      // 失锁指示
    unsigned char code[NFREQ + NEXOBS]; // code indicator (CODE_???)   // 码类型指示（CODE_???）
    double L[NFREQ + NEXOBS];           // observation data carrier-phase (cycle) // 载波相位观测值（周）
    double P[NFREQ + NEXOBS];           // observation data pseudorange (m)       // 伪距观测值（米）
    float  D[NFREQ + NEXOBS];           // observation data doppler frequency (Hz)// 多普勒频率观测值（Hz）
    int trueid;                         // true satellite id                      // 卫星真实编号
} obsd_t;

typedef struct {         /* observation data */
    int n, nmax;         // number of observation data/allocated // 观测数据数量/分配空间数
    obsd_t* data;        // observation data records             // 观测数据记录指针
} obs_t;

typedef struct {           /* GPS/QZS/GAL broadcast ephemeris type */
    int sat;               // satellite number                    // 卫星号
    int iode, iodc;        // IODE,IODC                          // 星历IODE、IODC
    int sva;               // SV accuracy (URA index)             // 卫星精度（URA索引）
    int svh;               // SV health (0:ok)                    // 卫星健康状态（0:正常）
    int week;              // GPS/QZS: gps week, GAL: galileo week// 周数
    int code;              // GPS/QZS: code on L2, GAL/CMP: data sources // L2码类型或数据源
    int flag;              // GPS/QZS: L2 P data flag, CMP: nav type     // L2 P数据标志或导航类型
    gtime_t toe, toc, ttr; // Toe,Toc,T_trans                     // 参考历元、钟参考、传输时间
    double A, e, i0, omg0, omg, m0, deln, omgd, idot; // SV orbit parameters // 卫星轨道参数
    double crc, crs, cuc, cus, cic, cis; // SV orbit correction parameters   // 轨道改正参数
    double toes;         // Toe (s) in week                      // 参考历元（周内秒）
    double fit;          // fit interval (h)                     // 适用时间间隔（小时）
    double f0, f1, f2;   // SV clock parameters (af0,af1,af2)    // 卫星钟参数
    double tgd, tgdb;    // group delay parameters                // 群延迟参数
    double adot, ndot;   // Adot,ndot for CNAV                    // 轨道变化率参数
} eph_t;

typedef struct {         /* GLONASS broadcast ephemeris type */
    int sat;             // satellite number                      // 卫星号
    int iode;            // IODE (0-6 bit of tb field)            // IODE（tb字段低6位）
    int frq;             // satellite frequency number             // 卫星频点号
    int svh, sva, age;   // satellite health, accuracy, age of operation // 卫星健康、精度、运行龄期
    gtime_t toe;         // epoch of ephemerides (gpst)           // 星历参考历元（GPST）
    gtime_t tof;         // message frame time (gpst)             // 电文帧时间（GPST）
    double pos[3];       // satellite position (ecef) (m)         // 卫星位置（ECEF，米）
    double vel[3];       // satellite velocity (ecef) (m/s)       // 卫星速度（ECEF，米/秒）
    double acc[3];       // satellite acceleration (ecef) (m/s^2) // 卫星加速度（ECEF，米/秒²）
    double taun, gamn;   // SV clock bias (s)/relative freq bias  // 卫星钟偏差/相对频率偏差
    double dtaun;        // delay between L1 and L2 (s)           // L1与L2之间延迟（秒）
} geph_t;

typedef struct {        /* SBAS ephemeris type (for lock info transfer) */
    int sat;            // satellite number                       // 卫星号
    int lock;           // lock info                              // 锁定信息
} prelock_t;

typedef struct {        /* navigation data type */
    int n, nmax;        // number of broadcast ephemeris          // 广播星历数量/分配空间数
    int ng, ngmax;      // number of glonass ephemeris            // GLONASS星历数量/分配空间数
    eph_t* eph;         // GPS/QZS/GAL ephemeris pointer          // GPS/QZS/GAL星历指针
    geph_t* geph;       // GLONASS ephemeris pointer              // GLONASS星历指针
    double utc_gps[4];  // GPS delta-UTC parameters {A0,A1,T,W}   // GPS UTC参数
    double utc_glo[4];  // GLONASS UTC GPS time parameters        // GLONASS UTC参数
    double utc_gal[4];  // Galileo UTC GPS time parameters        // Galileo UTC参数
    double utc_qzs[4];  // QZS UTC GPS time parameters            // QZS UTC参数
    double utc_cmp[4];  // BeiDou UTC parameters                  // 北斗UTC参数
    double ion_gps[8];  // GPS iono model parameters              // GPS电离层参数
    double ion_gal[4];  // Galileo iono model parameters          // Galileo电离层参数
    double ion_qzs[8];  // QZSS iono model parameters             // QZSS电离层参数
    double ion_cmp[8];  // BeiDou iono model parameters           // 北斗电离层参数
    int leaps;          // leap seconds (s)                       // 闰秒数
    double lam[MAXSAT][NFREQ];   // carrier wave lengths (m)       // 各卫星载波波长（米）
    double cbias[MAXSAT][3];     // satellite dcb (0:p1-p2,1:p1-c1,2:p2-c2) (m) // 卫星DCB差分码偏差
    double glo_cpbias[4];        // glonass code-phase bias {1C,1P,2C,2P} (m)   // GLONASS码相位偏差
    char glo_fcn[MAXPRNGLO + 1]; // glonass frequency channel number + 8        // GLONASS频点号+8
} nav_t;

typedef struct {         /* station parameter type */
    char name[MAXANT];   // marker name                             // 测站名
    char marker[MAXANT]; // marker number                           // 测站编号
    char antdes[MAXANT]; // antenna descriptor                      // 天线型号
    char antsno[MAXANT]; // antenna serial number                   // 天线序列号
    char rectype[MAXANT];// receiver type descriptor                // 接收机型号
    char recver[MAXANT]; // receiver firmware version               // 接收机固件版本
    char recsno[MAXANT]; // receiver serial number                  // 接收机序列号
    int antsetup;        // antenna setup id                        // 天线安装编号
    int itrf;            // ITRF realization year                   // ITRF年份
    int deltype;         // antenna delta type (0:enu,1:xyz)        // 天线偏移类型（0:ENU，1:XYZ）
    double pos[3];       // station position (ecef) (m)             // 测站位置（ECEF，米）
    double del[3];       // antenna position delta (e/n/u or x/y/z) (m) // 天线偏移（米）
    double hgt;          // antenna height (m)                      // 天线高（米）
} sta_t;

typedef struct {         /* solution type */
    gtime_t time;        // time (GPST)                             // 解算时间（GPST）
    double rr[6];        // position/velocity (m|m/s)               // 位置/速度（米|米/秒）
    double qr[6];        // position variance/covariance (m^2)      // 位置方差/协方差（米²）
    double dtr[6];       // receiver clock bias to time systems (s)  // 接收机钟差（秒）
    unsigned char stat;  // solution status (SOLQ_???)              // 解状态
    unsigned char ns;    // number of valid satellites               // 有效卫星数
    unsigned char float_ns; // number of valid satellites (float)    // 浮点解有效卫星数
    double age;          // age of differential (s)                  // 差分龄期（秒）
    double ratio;        // AR ratio factor for validation           // 固定解比率
    double pre_ratio;    // previous AR ratio                        // 上一历元比率
    unsigned char vtat;  // velocity status 0:fail 1:successful      // 速度解状态
    int spp_flag;        // SPP success flag                         // 单点定位成功标志
    int last_stat;       // last solution status                     // 上一解状态
    int first_flag;      // first solution flag                      // 首次解标志
    unsigned char lackflag; // lack satellite flag                   // 卫星不足标志
    unsigned char lacksat;  // lack satellite count                  // 卫星不足数量
    int saterrorlist[MAXEORSTA]; // satellite error list             // 卫星异常列表
    double rdop[4];      // rov dops {gdop,pdop,hdop,vdop}           // DOP值
    int staid;           // vrs station id                           // VRS测站编号
    int nsat[4];         // satellite count for each system [GPS GLO GAL BDS] // 各系统卫星数
    int resetflag;       // time update reset flag                   // 时间更新重置标志
} sol_t;

typedef struct { /* lg69t pvt control struct type */
    unsigned int sub;    // subtype id                               // 子类型ID
    unsigned int sta;    // Reference Station ID                     // 参考站ID
    unsigned int qual;   // GPS Quality Indicator (fix status)       // 定位质量指示
    unsigned int nsat_u; // Number of satellites in use              // 使用卫星数
    unsigned int nsat_v; // Number of satellites in view             // 可见卫星数
    double dop[3];       // hdop vdop pdop                           // DOP值
    int age;             // Age of Differentials                     // 差分龄期
    unsigned int dsta;   // differential reference station id        // 差分参考站ID
    gtime_t time;        // GNSS epoch time                          // GNSS历元时间
    double lat;          // latitude                                 // 纬度
    double lon;          // longitude                                // 经度
    double ght;          // height                                   // 高程
    double vh;           // velocity horizontal                      // 水平速度
    double vv;           // velocity vertical                        // 垂直速度
    double dir;          // course angle                             // 航向角
    double cbias;        // receiver clock bias                      // 接收机钟差
    double cdrift;       // receiver clock drift                     // 接收机钟漂
    double h;            // geoidal separation                       // 大地水准面分离
} lg69t_pvt_t;

typedef struct {        /* RTCM control struct type */
    int staid;          // station id                                // 测站编号
    int stah;           // station health                            // 测站健康状态
    int seqno;          // sequence number for rtcm 2 or iods msm    // RTCM2或MSM序号
    int outtype;        // output message type                       // 输出消息类型
    int type;           // message type                              // 电文类型
    gtime_t time;       // message time                              // 消息时间
    gtime_t time_s;     // message start time                        // 消息起始时间
    obs_t obs;          // observation data (uncorrected)            // 原始观测数据
    nav_t nav;          // satellite ephemerides                     // 卫星星历
    sta_t sta;          // station parameters                        // 测站参数
    char msg[128];      // special message                           // 特殊消息
    char msgtype[256];  // last message type                         // 上一消息类型
    char msmtype[6][128]; // msm signal types                        // MSM信号类型
    int obsflag;        // obs data complete flag (1:ok,0:not complete) // 观测数据完整标志
    int ephsat;         // update satellite of ephemeris             // 星历更新卫星号
    prelock_t prelock[MAXRECOBS * 2][NFREQ + NEXOBS]; // lock info   // 锁定信息
    int nbyte;          // number of bytes in message buffer         // 消息缓冲区字节数
    int nbit;           // number of bits in word buffer             // 字缓冲区比特数
    int len;            // message length (bytes)                    // 消息长度（字节）
    unsigned char buff[1200]; // message buffer                      // 消息缓冲区
    unsigned int word;       // word buffer for rtcm 2               // RTCM2字缓冲
    unsigned int nmsg2[100]; // message count of RTCM 2              // RTCM2消息计数
    unsigned int nmsg3[400]; // message count of RTCM 3              // RTCM3消息计数
    char opt[256];           // RTCM dependent options               // RTCM相关选项
    lg69t_pvt_t lgpvt;       // LG69T自身单点信息                    // LG69T单点信息
} rtcm_t;

typedef struct {        /* SNR mask type */
    int ena[2];         // enable flag {rover,base}                  // 使能标志{流动站,基站}
    double mask[NFREQ][9]; // mask (dBHz) at 5,10,...85 deg          // 各高度角信噪比掩码
} snrmask_t;

typedef struct {        /* processing options type */
    int mode;           // positioning mode (PMODE_???)              // 定位模式
    int soltype;        // solution type (0:forward,1:backward,2:combined) // 解算类型
    int nf;             // number of frequencies (1:L1,2:L1+L2,3:L1+L2+L5) // 频点数
    int navsys;         // navigation system                         // 导航系统
    double elmin;       // elevation mask angle (rad)                // 卫星高度角掩码（弧度）
    int snroff;         // snr on:1 off:0                            // 信噪比开关
    float snrthres;     // all snr threshold                         // 信噪比阈值
    int sateph;         // satellite ephemeris/clock (EPHOPT_???)    // 星历/钟选项
    int modear;         // AR mode                                   // 模糊度固定模式
    int glomodear;      // GLONASS AR mode                           // GLONASS模糊度模式
    int bdsmodear;      // BeiDou AR mode                            // 北斗模糊度模式
    int maxout;         // obs outage count to reset bias            // 观测中断重置计数
    int minlock;        // min lock count to fix ambiguity           // 固定模糊度最小锁定数
    int minfix;         // min fix count to hold ambiguity           // 保持模糊度最小固定数
    int ionoopt;        // ionosphere option (IONOOPT_???)           // 电离层选项
    int tropopt;        // troposphere option (TROPOPT_???)          // 对流层选项
    int dynamics;       // dynamics model (0:none,1:velociy,2:accel) // 动力学模型
    int tidecorr;       // earth tide correction                     // 潮汐改正
    int niter;          // number of filter iteration                // 滤波迭代次数
    double eratio[NFREQ]; // code/phase error ratio                  // 码/相位误差比
    double err[5];      // measurement error factor                  // 测量误差因子
    double std[3];      // initial-state std [0]bias,[1]iono [2]trop // 初始状态标准差
    double prn[6];      // process-noise std                        // 过程噪声标准差
    double sclkstab;    // satellite clock stability (sec/sec)       // 卫星钟稳定度
    double thresar[8];  // AR validation threshold                   // 固定解判别阈值
    double elmaskar;    // elevation mask of AR for rising satellite // AR高度角掩码
    double elmaskhold;  // elevation mask to hold ambiguity          // 保持模糊度高度角掩码
    double thresslip;   // slip threshold of geometry-free phase     // 周跳判别阈值
    double maxtdiff;    // max difference of time (sec)              // 最大时间差
    double maxinno;     // reject threshold of innovation (m)        // 创新量拒绝阈值
    double maxgdop;     // reject threshold of gdop                  // GDOP拒绝阈值
    double rb[3];       // base position for relative mode {x,y,z}   // 基站位置
    double snravg[4][3];// average SNR for each system               // 各系统信噪比均值
    double maxtdiffrtd; // max RTD age                              // RTD最大龄期
    double maxtdiffrtk; // max RTK age                              // RTK最大龄期
    int ratiomodel;     // ratio model                              // 比率模型
    int saterrorlist_dect[MAXEORSTA]; // error sat detection         // 卫星异常检测
    double optsnr[4];   // SNR options                              // SNR选项
} prcopt_t;

typedef struct {        /* solution options type */
    int posf;           // solution format (SOLF_???)               // 解算格式
    int times;          // time system (TIMES_???)                  // 时间系统
    int timef;          // time format (0:sssss.s,1:yyyy/mm/dd hh:mm:ss.s) // 时间格式
    int timeu;          // time digits under decimal point           // 时间小数位数
    int degf;           // latitude/longitude format                 // 纬度/经度格式
    int outhead;        // output header (0:no,1:yes)                // 输出头部
    int outopt;         // output processing options (0:no,1:yes)    // 输出处理选项
    int outvel;         // output velocity options (0:no,1:yes)      // 输出速度选项
    int datum;          // datum (0:WGS84,1:Tokyo)                   // 基准
    int height;         // height (0:ellipsoidal,1:geodetic)         // 高程类型
    int geoid;          // geoid model (0:EGM96,1:JGD2000)           // 大地水准面模型
    int solstatic;      // solution of static mode (0:all,1:single)  // 静态解选项
    int sstat;          // solution statistics level                  // 解统计级别
    int trace;          // debug trace level                         // 调试跟踪级别
    double nmeaintv[2]; // nmea output interval (s)                  // NMEA输出间隔
    char sep[64];       // field separator                           // 字段分隔符
    char prog[64];      // program name                              // 程序名
    double maxsolstd;   // max std-dev for solution output (m)       // 最大输出标准差
} solopt_t;

typedef struct {        /* satellite status type */
    unsigned char sys;  // navigation system                         // 导航系统
    unsigned char sat;  // satellite number                          // 卫星号
    double azel[2];     // azimuth/elevation angles {az,el} (rad)    // 方位/高度角（弧度）
    unsigned char vsat[NFREQ]; // valid satellite flag               // 有效卫星标志
    unsigned char snr[NFREQ];  // signal strength (0.25 dBHz)        // 信号强度
    int fix[NFREQ];            // ambiguity fix flag                  // 模糊度固定标志
    unsigned char slip[NFREQ]; // cycle-slip flag                     // 周跳标志
    unsigned char half[NFREQ]; // half-cycle valid flag               // 半周有效标志
    int lock[NFREQ];           // lock counter of phase                // 相位锁定计数
    unsigned int outc[NFREQ];  // obs outage counter of phase          // 相位观测中断计数
    double  gf;                // geometry-free phase L1-L2 (m)        // 几何无关相位
    gtime_t pt[2][NFREQ];      // previous carrier-phase time           // 上一历元载波相位时间
    double  ph[2][NFREQ];      // previous carrier-phase observable     // 上一历元载波相位观测值
    double  pd[2][NFREQ];      // previous doppler observation         // 上一历元多普勒观测值
    double  pp[2][NFREQ];      // previous pseudorange observation     // 上一历元伪距观测值
    double  vrs_phy[NFREQ];    // previous vrs pseudorange             // 上一历元VRS伪距
    int change_flag[NFREQ];    // change flag for each freq            // 频点变化标志
    gtime_t change_time;       // change time                          // 变化时间
    double bias[NFREQ];        // ambiguity bias                       // 模糊度偏差
    int rest[NFREQ];           // amb rest flag                        // 模糊度初始化标志
    int ref[NFREQ];            // reference flag                       // 参考星标志
    int snr_flag[NFREQ];       // SNR below mean flag                  // 信噪比低于均值标志
    int dxflag[NFREQ];         // ambiguity difference flag             // 模糊度差分标志
    double x[NFREQ];           // ambiguity record for drop             // 降星时模糊度记录
    int ambresetfail[NFREQ];   // ambiguity reset fail flag             // 模糊度强制初始化标志
} ssat_t;

typedef struct {
    gtime_t time;      // time                                       // 时间
    double isb;        // inter-system bias                          // 系统间偏差
} isb_t;

typedef struct {       /* RTK control/result type */
    gtime_t time;      // rtk solution time                          // RTK解算时间
    gtime_t time_d;    // rtd solution time                          // RTD解算时间
    gtime_t time_s;    // spp solution time                          // SPP解算时间
    gtime_t time_ld;   // last dynamic time                          // 上次动态时间
    gtime_t time_ls;   // last static time                           // 上次静态时间
    gtime_t time_k;    // RTK success time                           // RTK成功时间
    sol_t  sol;        // RTK solution                               // RTK解
    sol_t  sol_last;   // last solution                              // 上次任意解
    sol_t  sol_spp;    // SPP solution                               // SPP解
    sol_t  sol_rtd;    // RTD solution                               // RTD解
    sol_t  sol_rtk;    // RTK solution                               // RTK解
    sol_t  sol_sppk;   // spp kalman solution                        // 卡尔曼单点解
    double rb[6];      // base position/velocity (ecef) (m|m/s)      // 基站位置/速度
    int nx, na;        // number of float states/fixed states         // 浮点/固定状态数
    double tt;         // time difference between current and previous (s) // 历元间龄期
    double tt_d;       // RTD time difference                        // RTD龄期
    double* x, * P;    // float states and their covariance           // 浮点状态及协方差
    double* xa, * pa;  // fixed states and their covariance           // 固定状态及协方差
    double x_s[11], p_s[11 * 11]; // single position filter states    // 单点滤波状态及协方差
    double x_d[6], p_d[6 * 6];    // RTD filter states and covariance // RTD滤波状态及协方差
    double l_x_d[6], l_x[3];      // last epoch psr diff              // 上次历元伪距差分
    int nfix;                     // number of continuous fixes        // 连续固定次数
    ssat_t ssat[MAXDIFOBS];       // satellite status                  // 卫星状态
    int nsat;                     // current epoch satellite count     // 当前历元卫星数
    int presat;                   // previous epoch satellite count    // 上一历元卫星数
    int n_errsat;                 // satellites with SNR < 25          // 信噪比低于25卫星数
    int satid[MAXDIFOBS];         // double-difference satellite IDs   // 双差卫星ID
    prcopt_t opt;                 // processing options                // 处理选项
    int basechange;               // base position change flag         // 基站变化标志
    gtime_t prefixtime;           // last fixed time                   // 上次固定时间
    double n_lerr_ratio, n_slip_ratio; // error/slip ratio             // 粗差/周跳比率
    int ratiomodel;               // ratio model                       // 比率模型
    int n_errsat_30;              // satellites with SNR < 30          // 信噪比低于30卫星数
    int staid;                    // station id                        // 测站编号
    int vrs_staid;                // VRS station id                    // VRS测站编号
    int falsatnum;                // dropped satellite count            // 降星数
    ssat_t f_ssat[MAXFALSTA];     // dropped satellite info             // 降星卫星信息
    double lasttt;                // last time difference               // 上次龄期
    gtime_t lasttime;             // last time                          // 上次时间
    isb_t glo_isb;                // GLONASS ISB                        // GLONASS系统间偏差
    isb_t gal_isb;                // Galileo ISB                        // Galileo系统间偏差
    isb_t bds_isb;                // BDS ISB                            // 北斗系统间偏差
    lg69t_pvt_t lgpvt;            // LG69T PVT info                     // LG69T单点信息
    int vrs_sys_flag[4];          // VRS system exist flag              // VRS系统存在标志
    double vel;                   // velocity norm                      // 空间速度模长
    float  vrs_age;               // VRS age                            // VRS龄期
    int fix_con_num;              // continuous fix count               // 连续固定次数
    int statchange;               // status change flag                 // 状态变化标志
    int up_vrs_flag;              // VRS update flag                    // VRS更新标志
    int vrsbadnum;                // VRS bad satellite count            // VRS异常卫星数
} rtk_t;

//typedef struct { /* last epoch message */
//    int sat;                      // satellite number                   // 卫星号
//    unsigned int dpsr1;           // Prior psr detect 1:error 0:normal  // 先验伪距残差探测
//    unsigned int dpsr2[NFREQ];    // Prior detect 1:error 0:normal       // 频点先验伪距探测
//    unsigned int ddop1;           // dop detect 1:error 0:normal         // 多普勒探测
//    unsigned int ddop2[NFREQ];    // dop detect 1:error 0:normal         // 频点多普勒探测
//    unsigned int dlsr2[NFREQ];    // LSR detect                          // LSR探测
//    double d1;                    // Doppler value                       // 多普勒值
//} midmsg_t;

#define MAX_MAT_DIM     19                        /*the max dim of vect*/
#define MAX_MAT_DIM_2   MAX_MAT_DIM*MAX_MAT_DIM   /*the max dim of matrix*/
#define VARMAX          5						 /* the max number of variance */

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
#define min(x,y)    ((x)<=(y)?(x):(y))         // Minimum of x and y // 取最小值
#define max(x,y)    ((x)<=(y)?(y):(x))         // Maximum of x and y // 取最大值

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

class mat;
class mat3;
class quat;
class vect;

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
    vect3(const double* pdata);

    /* array constructor (float) --------------------------------------------------
    * initialize vect3 from a float array
    * args   : const float *pdata  I   pointer to array of 3 floats
    * return : none
    *-----------------------------------------------------------------------------*/
    vect3(const float* pdata);

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
    vect3& operator=(const double* pf);

    /* check if all elements are zeros --------------------------------------------
    * check if all components of vect3 are zero within a threshold
    * args   : const vect3 &v      I   vect3 to check
    *          double     eps      I   threshold
    * return : 1 if all elements are zero, 0 otherwise
    *-----------------------------------------------------------------------------*/
    int IsZeros(const vect3& v, double eps);

    /* check if x and y elements are zeros ----------------------------------------
    * check if x and y components of vect3 are zero within a threshold
    * args   : const vect3 &v      I   vect3 to check
    *          double     eps      I   threshold
    * return : true if x and y are zero, false otherwise
    *-----------------------------------------------------------------------------*/
    bool IsZeroXY(const vect3& v, double eps);

    /* check if any element is NaN ------------------------------------------------
    * check if any component of vect3 is NaN
    * args   : const vect3 &v      I   vect3 to check
    * return : true if any element is NaN, false otherwise
    *-----------------------------------------------------------------------------*/
    bool IsNaN(const vect3& v);

    /* vector addition ------------------------------------------------------------
    * add two vect3 objects
    * args   : const vect3 &v      I   vect3 to add
    * return : result of addition (vect3)
    *-----------------------------------------------------------------------------*/
    vect3 operator+(const vect3& v) const;

    /* vector subtraction ---------------------------------------------------------
    * subtract two vect3 objects
    * args   : const vect3 &v      I   vect3 to subtract
    * return : result of subtraction (vect3)
    *-----------------------------------------------------------------------------*/
    vect3 operator-(const vect3& v) const;

    /* vector cross multiplication ------------------------------------------------
    * cross product of two vect3 objects
    * args   : const vect3 &v      I   vect3 to cross
    * return : result of cross product (vect3)
    *-----------------------------------------------------------------------------*/
    vect3 operator*(const vect3& v) const;

    /* row-vector multiply matrix -------------------------------------------------
    * multiply vect3 (as row vector) by matrix
    * args   : const mat &m        I   matrix to multiply
    * return : result of multiplication (vect3)
    *-----------------------------------------------------------------------------*/
    vect3 operator*(const mat& m) const;

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
    vect3 operator*(const mat3& m) const;

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
    vect3 operator/(const vect3& v) const;

    /* vector addition assignment -------------------------------------------------
    * add vect3 to this vect3 (in place)
    * args   : const vect3 &v      I   vect3 to add
    * return : reference to this vect3
    *-----------------------------------------------------------------------------*/
    vect3& operator+=(const vect3& v);

    /* vector subtraction assignment ----------------------------------------------
    * subtract vect3 from this vect3 (in place)
    * args   : const vect3 &v      I   vect3 to subtract
    * return : reference to this vect3
    *-----------------------------------------------------------------------------*/
    vect3& operator-=(const vect3& v);

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
    vect3& operator/=(const vect3& v);

    /* scale multiply vector (friend) ---------------------------------------------
    * multiply scalar by vect3
    * args   : double     f        I   scalar value
    *          const vect3 &v      I   vect3 to multiply
    * return : result of multiplication (vect3)
    *-----------------------------------------------------------------------------*/
    friend vect3 operator*(double f, const vect3& v);

    /* minus (friend) -------------------------------------------------------------
    * negate vect3 (unary minus)
    * args   : const vect3 &v      I   vect3 to negate
    * return : result of negation (vect3)
    *-----------------------------------------------------------------------------*/
    friend vect3 operator-(const vect3& v);
};


#endif // NavCostant