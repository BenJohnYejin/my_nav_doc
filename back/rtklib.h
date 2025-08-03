#ifndef RTKLIB_H
#define RTKLIB_H
#include "leador_os.h"
#include "rtkport.h"

#define TRACEOPEN    1   // 1：open  0:off  存在的问题是外部无法控制日志的开启或者关闭，待考虑做外部接口函数

typedef struct {
	int i, j;
	double value;
	double value2;
}tri_mat_t;

typedef struct {
	tri_mat_t* tri;         //非零个数，乘数和被乘数时排序有所差异，乘数则以行排序，被乘数则以列排序
	int* ijnonzerosnums;    //每行非零个数，主要作为矩阵乘数  /每列非零个数，主要作为矩阵乘数  改，现为改行非零元素结束的索引
	int* ijnonzerosindex;   //每行非零个数，主要作为矩阵乘数  /每列非零个数，主要作为矩阵乘数
}tri_mat_t2;

#ifdef WIN_DLL
#define EXPORT __declspec(dllexport) /* for Windows DLL */
#else
#define EXPORT extern
#endif
//区分每个系统的频点，方便每个系统单独频点组合，后续添加按照系统区分，方便事后兼容其他频点的处理 0413 add by lyp  基本的计算模式里需要包含B1 b3 两个的组合
#define  FREQMODE_CMP  2     //0  BDS:B1+B2   1 BDS:B1+B2a  2 BDS:B1+b3
#define  FREQMODE_GPS  0     //GPS:L1+L2  // 1GPS:L1+L5
#define  FREQMODE_GAL  0     //GAL:L1+L2  // 1GAL:L1+L5     2 GAL L1+E5b
/* constants -----------------------------------------------------------------*/
#define SQR(x)      ((x)*(x))
#define SQRT(x)     ((x)<=0.0?0.0:sqrt(x))
#define MIN(x,y)    ((x)<=(y)?(x):(y))
#define MAX(x,y)    ((x)<=(y)?(y):(x))
#define ROUND(x)    (int)(floor((x)+0.5))
#define ROUND_U(x)		((unsigned int)floor((x)+0.5))
#define MAXEORSTA   10		   /*最大粗差探测的卫星数*/
#define MAXFALSTA   20         /*前5个历元内发生降星的卫星信息*/
#define DOPSIP      13.0       /* dopper for phase slip */
#define RECCLOCK    500.0      /* dectect receiver clock jump threshold */
#define ZEROS_MIN   1E-12      /* 用于判断double类型是否为零*/

#define DPS_IN_KALPNT 2        /*kalman单点是否用多普勒  1表示不用，2表示用  相关矩阵大小*该值,改该值时需慎重*/
#define DPS_IN_RTD    2        /*rtd是否用多普勒        1表示不用，2表示用  相关矩阵大小*该值,改该值时需慎重*/
#define DIF_L_IN_EVL    1      /*最小二乘定速是否用前后伪距差       1表示不用，2表示用  相关矩阵大小*该值,改该值时需慎重*/

#define VAR_POS     SQR(30.0)  /* initial variance of receiver pos (m^2) */
#define VAR_VEL     SQR(10.0)  /* initial variance of receiver vel ((m/s)^2) */
#define VAR_ACC     SQR(10.0)  /* initial variance of receiver acc ((m/ss)^2) */
#define VAR_HWBIAS  SQR(1.0)   /* initial variance of h/w bias ((m/MHz)^2) */
#define VAR_GRA     SQR(0.001) /* initial variance of gradient (m^2) */
#define INIT_ZWD    0.15       /* initial zwd (m) */

#define VAR_POS_B     SQR(500.0)  /* initial variance of receiver pos (m^2) */
#define VAR_VEL_B     SQR(100.0)  /* initial variance of receiver vel ((m/s)^2) */
#define VAR_DOP_B     0.001       /* variance of doppler measurement update */
#define VAR_CLK_B     SQR(20.0)   /* initial variance of receiver clock correction */
#define VAR_SHIFT_B   SQR(5.0)    /*initial variance of receiver clock shift */

#define PRN_HWBIAS  1E-6     /* process noise of h/w bias (m/MHz/sqrt(s)) */
#define GAP_RESION  120      /* gap to reset ionosphere parameters (epochs) */
#define MAXACC      30.0     /* max accel for doppler slip detection (m/s^2) */

#define VAR_HOLDAMB 0.25     /* constraint to hold ambiguity (cycle^2) */

#define TTOL_MOVEB  (1.0+2*DTTOL)
							 /* time sync tolerance for moving-baseline (s) */
/* number of parameters (pos,ionos,tropos,hw-bias,phase-bias,real,estimated) */
#define NF(opt)     ((opt)->ionoopt==IONOOPT_IFLC?1:(opt)->nf)
#define NP(opt)     ((opt)->dynamics==0?3:3)
#define NI(opt)     ((opt)->ionoopt!=IONOOPT_EST?0:MAXSAT)
#define NT(opt)     ((opt)->tropopt<TROPOPT_EST?0:((opt)->tropopt<TROPOPT_ESTG?2:6))
#define NL(opt)     ((opt)->glomodear!=2?0:NFREQGLO)
#define NB(opt)     ((opt)->mode<=PMODE_DGPS?0:MAXSAT*NF(opt))
#define NR(opt)     (NP(opt)+NI(opt)+NT(opt)+NL(opt))
#define NX(opt)     (NR(opt)+NB(opt))

/* state variable index */
#define II(s,opt)   (NP(opt)+(s)-1)                 /* ionos (s:satellite no) */
#define IT(r,opt)   (NP(opt)+NI(opt)+NT(opt)/2*(r)) /* tropos (r:0=rov,1:ref) */
#define IL(f,opt)   (NP(opt)+NI(opt)+NT(opt)+(f))   /* receiver h/w bias */
#define IB2(s,f,opt,ns) (NR(opt)+ns*(f)+(s)) /* phase bias (s:satno,f:freq) */

#define MIN_DD        5
#define MIN_DDSAT     4

#define SMALL       1.0e-20
#define MAXINT      32766       /*int类型最大值*/
#define KNOT2M     0.514444444  /* m/knot */

#define PI          3.1415926535897932  /* pi */
#define D2R         (PI/180.0)          /* deg to rad */
#define R2D         (180.0/PI)          /* rad to deg */
#define CLIGHT      299792458.0         /* speed of light (m/s) */
#define SC2RAD      3.1415926535898     /* semi-circle to radian (IS-GPS) */
#define AU          149597870691.0      /* 1 AU (m) */
#define AS2R        (D2R/3600.0)        /* arc sec to radian */

#define OMGE        7.2921151467E-5     /* earth angular velocity (IS-GPS) (rad/s) */

#define RE_WGS84    6378137.0           /* earth semimajor axis (WGS84) (m) */
#define FE_WGS84    (1.0/298.257223563) /* earth flattening (WGS84) */

#define HION        350000.0            /* ionosphere height (m) */

#define MAXFREQ     7                   /* max NFREQ */
#define MINREFSTARELEV 35

#define FREQ1       1.57542E9           /* L1/E1  frequency (Hz) */
#define FREQ2       1.22760E9           /* L2     frequency (Hz) */
#define FREQ5       1.17645E9           /* L5/E5a frequency (Hz) */
#define FREQ6       1.27875E9           /* E6/LEX frequency (Hz) */
#define FREQ7       1.20714E9           /* E5b    frequency (Hz) */
#define FREQ8       1.191795E9          /* E5a+b  frequency (Hz) */
#define FREQ9       2.492028E9          /* S      frequency (Hz) */
#define FREQ1_GLO   1.60200E9           /* GLONASS G1 base frequency (Hz) */
#define DFRQ1_GLO   0.56250E6           /* GLONASS G1 bias frequency (Hz/n) */
#define FREQ2_GLO   1.24600E9           /* GLONASS G2 base frequency (Hz) */
#define DFRQ2_GLO   0.43750E6           /* GLONASS G2 bias frequency (Hz/n) */
#define FREQ3_GLO   1.202025E9          /* GLONASS G3 frequency (Hz) */
#define FREQ1_CMP   1.561098E9          /* BeiDou B1 C2I frequency (Hz) */
#define FREQ2_CMP   1.20714E9           /* BeiDou B2 C7I frequency (Hz) */
#define FREQ3_CMP   1.26852E9           /* BeiDou B3 C6I frequency (Hz) */
#define FREQ1c_CMP  1.57542E9           /* BeiDou B1c/BDS-3 Signals frequency (Hz) */
#define FREQ2a_CMP  1.17645E9           /* BeiDou B2a/BDS-3 Signals frequency (Hz) */
#define FREQ5_GAL   1.17645E9   //1.17642E9           /* E5a frequency (Hz) */

#define EFACT_GPS   1.5                 /* error factor: GPS */
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
#define MAXSAVEDTOE    4*3600           /* max save time difference to GPS Toe (s) */
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

/*定义各系统伪距观测值最大最小值*/
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

#if defined(WIN32)
#define thread_t    HANDLE
#define lock_t      CRITICAL_SECTION
#define initlock(f) InitializeCriticalSection(f)
#define lock(f)     EnterCriticalSection(f)
#define unlock(f)   LeaveCriticalSection(f)
#define FILEPATHSEP '\\'
#elif defined(LINUX)
#define thread_t    pthread_t
#define lock_t      pthread_mutex_t
#define initlock(f) pthread_mutex_init(f,NULL)
#define lock(f)     pthread_mutex_lock(f)
#define unlock(f)   pthread_mutex_unlock(f)
#define FILEPATHSEP '/'
#elif defined(RTOS)

#else
#endif

#define GGA0 "$GPGGA,,,,,,,,,,,,,,,*64"
#define RMC0 "$GPRMC,,,,,,,,,,,,*55"

#define RTCM3PREAMB 0xD3        /* rtcm ver.3 frame preamble */
/* type definitions ----------------------------------------------------------*/

typedef struct {        /* time struct */
	time_t time;        /* time (s) expressed by standard time_t */
	double sec;         /* fraction of second under 1 s */
} gtime_t;

typedef struct {            /* observation data record */
	gtime_t time;           /* receiver sampling time (GPST) */
	unsigned char sat, rcv; /* satellite/receiver number */
	unsigned char snr[NFREQ + NEXOBS];  /* signal strength (0.25 dBHz) */
	unsigned char lli[NFREQ + NEXOBS];  /* loss of lock indicator */
	unsigned char code[NFREQ + NEXOBS]; /* code indicator (CODE_???) */
	double L[NFREQ + NEXOBS]; /* observation data carrier-phase (cycle) */
	double P[NFREQ + NEXOBS]; /* observation data pseudorange (m) */
	float  D[NFREQ + NEXOBS]; /* observation data doppler frequency (Hz) */
	int trueid;
} obsd_t;

typedef struct {         /* observation data */
	int n, nmax;         /* number of obervation data/allocated */
	obsd_t* data;        /* observation data records */
} obs_t;

typedef struct {           /* GPS/QZS/GAL broadcast ephemeris type */
	int sat;               /* satellite number */
	int iode, iodc;        /* IODE,IODC */
	int sva;               /* SV accuracy (URA index) */
	int svh;               /* SV health (0:ok) */
	int week;              /* GPS/QZS: gps week, GAL: galileo week */
	int code;              /* GPS/QZS: code on L2, GAL/CMP: data sources */
	int flag;              /* GPS/QZS: L2 P data flag, CMP: nav type */
	gtime_t toe, toc, ttr; /* Toe,Toc,T_trans */
	/* SV orbit parameters */
	double A, e, i0, omg0, omg, m0, deln, omgd, idot;
	double crc, crs, cuc, cus, cic, cis;
	double toes;         /* Toe (s) in week */
	double fit;          /* fit interval (h) */
	double f0, f1, f2;   /* SV clock parameters (af0,af1,af2) */
	double tgd, tgdb;    /* group delay parameters */
	double adot, ndot;   /* Adot,ndot for CNAV */
} eph_t;

typedef struct {         /* GLONASS broadcast ephemeris type */
	int sat;             /* satellite number */
	int iode;            /* IODE (0-6 bit of tb field) */
	int frq;             /* satellite frequency number */
	int svh, sva, age;   /* satellite health, accuracy, age of operation */
	gtime_t toe;         /* epoch of epherides (gpst) */
	gtime_t tof;         /* message frame time (gpst) */
	double pos[3];       /* satellite position (ecef) (m) */
	double vel[3];       /* satellite velocity (ecef) (m/s) */
	double acc[3];       /* satellite acceleration (ecef) (m/s^2) */
	double taun, gamn;   /* SV clock bias (s)/relative freq bias */
	double dtaun;        /* delay between L1 and L2 (s) */
} geph_t;

//存储前一个历元的lock信息，并进行传递
typedef struct {        /* SBAS ephemeris type */
	int sat;            /* satellite number */
	int lock;
} prelock_t;

typedef struct {        /* navigation data type */
	int n, nmax;        /* number of broadcast ephemeris */
	int ng, ngmax;      /* number of glonass ephemeris */
	eph_t* eph;         /* GPS/QZS/GAL ephemeris */
	geph_t* geph;       /* GLONASS ephemeris */
	double utc_gps[4];  /* GPS delta-UTC parameters {A0,A1,T,W} */
	double utc_glo[4];  /* GLONASS UTC GPS time parameters */
	double utc_gal[4];  /* Galileo UTC GPS time parameters */
	double utc_qzs[4];  /* QZS UTC GPS time parameters */
	double utc_cmp[4];  /* BeiDou UTC parameters */
	double ion_gps[8];  /* GPS iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
	double ion_gal[4];  /* Galileo iono model parameters {ai0,ai1,ai2,0} */
	double ion_qzs[8];  /* QZSS iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
	double ion_cmp[8];  /* BeiDou iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
	int leaps;          /* leap seconds (s) */
	double lam[MAXSAT][NFREQ];   /* carrier wave lengths (m) */
	double cbias[MAXSAT][3];     /* satellite dcb (0:p1-p2,1:p1-c1,2:p2-c2) (m) */
	double glo_cpbias[4];        /* glonass code-phase bias {1C,1P,2C,2P} (m) */
	char glo_fcn[MAXPRNGLO + 1]; /* glonass frequency channel number + 8 */
} nav_t;

typedef struct {         /* station parameter type */
	char name[MAXANT];   /* marker name */
	char marker[MAXANT]; /* marker number */
	char antdes[MAXANT]; /* antenna descriptor */
	char antsno[MAXANT]; /* antenna serial number */
	char rectype[MAXANT];/* receiver type descriptor */
	char recver[MAXANT]; /* receiver firmware version */
	char recsno[MAXANT]; /* receiver serial number */
	int antsetup;        /* antenna setup id */
	int itrf;            /* ITRF realization year */
	int deltype;         /* antenna delta type (0:enu,1:xyz) */
	double pos[3];       /* station position (ecef) (m) */
	double del[3];       /* antenna position delta (e/n/u or x/y/z) (m) */
	double hgt;          /* antenna height (m) */
} sta_t;

typedef struct {         /* solution type */
	gtime_t time;        /* time (GPST) */
	double rr[6];        /* position/velocity (m|m/s) */
	/* {x,y,z,vx,vy,vz} or {e,n,u,ve,vn,vu} */
	double  qr[6];       /* position variance/covariance (m^2) */
	/* {c_xx,c_yy,c_zz,c_xy,c_yz,c_zx} or */
	/* {c_ee,c_nn,c_uu,c_en,c_nu,c_ue} */
	
	double dtr[6];       /* receiver clock bias to time systems (s) 钟差、gloisb、galisb、bdsisb、 、接收机钟漂*/
	unsigned char stat;  /* solution status (SOLQ_???) */
	unsigned char ns;    /* number of valid satellites */
	unsigned char float_ns;   /* number of valid satellites */
	double age;          /* age of differential (s) */
	double ratio;        /* AR ratio factor for valiation */
	double pre_ratio;
	unsigned char vtat;  /* velocity statues 0:fail 1:successful */
	int spp_flag;        /*单点成功标识，0 成功，1 spp时失败，2 在1的基础上qr超限不传递下一历元*/
	int last_stat;
	int first_flag;
	unsigned char lackflag;
	unsigned char lacksat;/*单点定位卫星太少不能解算标志*/
	int saterrorlist[MAXEORSTA];
	double rdop[4];     /* rov dops {gdop,pdop,hdop,vdop}*/
	int staid;          /*vrs测站编号*/
	int nsat[4];        /*参与单点解算的各个系统卫星数[GPS GLO GAL BDS]*/
	int resetflag;      /*时间更新状态重置标识*/
} sol_t;

typedef struct { /* lg69t pvt control struct type */
	unsigned int sub;   /*subtype id*/
	unsigned int sta;   /*Reference Station ID*/
	unsigned int qual;  /*GPS Quality Indicator (fix status)   same as field <Quality> in NMEA GGA*/
	unsigned int nsat_u;/*Number of satellites in use*/
	unsigned int nsat_v;/*Number of satellites in view*/
	double dop[3];      /*hdop vdop pdop*/
	int age;            /*Age of Differentials*/
	unsigned int dsta;  /*differential reference station id*/
	gtime_t time;       /*GNSS epoch time   gpstime*/
	double lat;         /*latitude*/
	double lon;         /*longtitude*/
	double ght;         /*height*/
	double vh;          /*velocity horizontal*/
	double vv;          /*velocity vertical*/
	double dir;         /*course angle*/
	double cbias;       /*receiver clock bias*/
	double cdrift;      /*receiver clock dirft*/
	double h;           /*geoidal separation*/
} lg69t_pvt_t;

typedef struct {        /* RTCM control struct type */
	int staid;          /* station id */
	int stah;           /* station health */
	int seqno;          /* sequence number for rtcm 2 or iods msm */
	int outtype;        /* output message type */
	int type;           /*电文类型*/
	gtime_t time;       /* message time */
	gtime_t time_s;     /* message start time */
	obs_t obs;          /* observation data (uncorrected) */
	nav_t nav;          /* satellite ephemerides */
	sta_t sta;          /* station parameters */
	char msg[128];      /* special message */
	char msgtype[256];  /* last message type */
	char msmtype[6][128]; /* msm signal types */
	int obsflag;          /* obs data complete flag (1:ok,0:not complete) */
	int ephsat;           /* update satellite of ephemeris */

	prelock_t prelock[MAXRECOBS * 2][NFREQ + NEXOBS];
	int nbyte;          /* number of bytes in message buffer */
	int nbit;           /* number of bits in word buffer */
	int len;            /* message length (bytes) */
	unsigned char buff[1200];/* message buffer */
	unsigned int word;       /* word buffer for rtcm 2 */
	unsigned int nmsg2[100]; /* message count of RTCM 2 (1-99:1-99,0:other) */
	unsigned int nmsg3[400]; /* message count of RTCM 3 (1-299:1001-1299,300-399:2000-2099,0:ohter) */
	char opt[256];           /* RTCM dependent options */
	lg69t_pvt_t lgpvt;       /*LG69T自身单点信息*/
} rtcm_t;

typedef struct {        /* SNR mask type */
	int ena[2];         /* enable flag {rover,base} */
	double mask[NFREQ][9]; /* mask (dBHz) at 5,10,...85 deg */
} snrmask_t;

typedef struct {        /* processing options type */
	int mode;           /* positioning mode (PMODE_???) */
	int soltype;        /* solution type (0:forward,1:backward,2:combined) */
	int nf;             /* number of frequencies (1:L1,2:L1+L2,3:L1+L2+L5) */
	int navsys;         /* navigation system */
	double elmin;       /* elevation mask angle (rad) */
	int   snroff;       /* snr on:1 off:0 */
	float snrthres;     /* all snr thres */
	int sateph;         /* satellite ephemeris/clock (EPHOPT_???) */
	int modear;         /* AR mode (0:off,1:continuous,2:instantaneous,3:fix and hold,4:ppp-ar) */
	int glomodear;      /* GLONASS AR mode (0:off,1:on,2:auto cal,3:ext cal) */
	int bdsmodear;      /* BeiDou AR mode (0:off,1:on) */
	int maxout;         /* obs outage count to reset bias */
	int minlock;        /* min lock count to fix ambiguity */
	int minfix;         /* min fix count to hold ambiguity */
	int ionoopt;        /* ionosphere option (IONOOPT_???) */
	int tropopt;        /* troposphere option (TROPOPT_???) */
	int dynamics;       /* dynamics model (0:none,1:velociy,2:accel) */
	int tidecorr;       /* earth tide correction (0:off,1:solid,2:solid+otl+pole) */
	int niter;          /* number of filter iteration */
	double eratio[NFREQ]; /* code/phase error ratio */
	double err[5];      /* measurement error factor */
	/* [0]:reserved */
	/* [1-3]:error factor a/b/c of phase (m) */
	/* [4]:doppler frequency (hz) */
	double std[3];      /* initial-state std [0]bias,[1]iono [2]trop */
	double prn[6];      /* process-noise std [0]bias,[1]iono [2]trop [3]acch [4]accv [5] pos */
	double sclkstab;    /* satellite clock stability (sec/sec) */
	double thresar[8];  /* AR validation threshold */
	double elmaskar;    /* elevation mask of AR for rising satellite (deg) */
	double elmaskhold;  /* elevation mask to hold ambiguity (deg) */
	double thresslip;   /* slip threshold of geometry-free phase (m) */
	double maxtdiff;    /* max difference of time (sec) */
	double maxinno;     /* reject threshold of innovation (m) */
	double maxgdop;     /* reject threshold of gdop */
	double rb[3];       /* base position for relative mode {x,y,z} (ecef) (m) */
	double snravg[4][3];
	double maxtdiffrtd; /*rtd的最大差分龄期*/
	double maxtdiffrtk; /*rtk的最大差分龄期*/
	int ratiomodel;
	int saterrorlist_dect[MAXEORSTA];/*探测流动站或手机中的异常数据*/

	double optsnr[4];
} prcopt_t;

typedef struct {        /* solution options type */
	int posf;           /* solution format (SOLF_???) */
	int times;          /* time system (TIMES_???) */
	int timef;          /* time format (0:sssss.s,1:yyyy/mm/dd hh:mm:ss.s) */
	int timeu;          /* time digits under decimal point */
	int degf;           /* latitude/longitude format (0:ddd.ddd,1:ddd mm ss) */
	int outhead;        /* output header (0:no,1:yes) */
	int outopt;         /* output processing options (0:no,1:yes) */
	int outvel;         /* output velocity options (0:no,1:yes) */
	int datum;          /* datum (0:WGS84,1:Tokyo) */
	int height;         /* height (0:ellipsoidal,1:geodetic) */
	int geoid;          /* geoid model (0:EGM96,1:JGD2000) */
	int solstatic;      /* solution of static mode (0:all,1:single) */
	int sstat;          /* solution statistics level (0:off,1:states,2:residuals) */
	int trace;          /* debug trace level (0:off,1-5:debug) */
	double nmeaintv[2]; /* nmea output interval (s) (<0:no,0:all) */
	/* nmeaintv[0]:gprmc,gpgga,nmeaintv[1]:gpgsv */
	char sep[64];       /* field separator */
	char prog[64];      /* program name */
	double maxsolstd;   /* max std-dev for solution output (m) (0:all) */
} solopt_t;

typedef struct {        /* satellite status type */
	unsigned char sys;  /* navigation system */
	unsigned char sat;  /*sat NO.*/
	double azel[2];     /* azimuth/elevation angles {az,el} (rad) */
	unsigned char vsat[NFREQ]; /* valid satellite flag */
	unsigned char snr[NFREQ];  /* signal strength (0.25 dBHz) */
	int fix[NFREQ];            /* ambiguity fix flag (1:fix,2:float,3:hold) */
	unsigned char slip[NFREQ]; /* cycle-slip flag */
	unsigned char half[NFREQ]; /* half-cycle valid flag */
	int lock[NFREQ];         /* lock counter of phase */
	unsigned int outc[NFREQ];/* obs outage counter of phase */
	double  gf;              /* geometry-free phase L1-L2 (m) */
	gtime_t pt[2][NFREQ];    /* previous carrier-phase time */
	double  ph[2][NFREQ];    /* previous carrier-phase observable (cycle) */
	double  pd[2][NFREQ];    /* previous doppler observation (Hz) */
	double  pp[2][NFREQ];    /* previous pseudorange observation (meters) */
	double  vrs_phy[NFREQ];  /* (obs 10hz)  (vrs 1hz) previous pseudorange observation (meters) */

	int       change_flag[NFREQ];
	gtime_t change_time;
	double    bias[NFREQ];
	int       rest[NFREQ];/* amb rest flag 模糊度初始化 */
	int		  ref[NFREQ]; /* 双差模糊度的参考星标识 0：未参与双差 1:参考星 2：非参考星*/
	int snr_flag[NFREQ];  /*信噪比小于系统均值标志*/
	int dxflag[NFREQ];    /*观测值计算的单差模糊度和历元间传递的单差模糊度差值分段标识*/
	double x[NFREQ];      /*降星时记录模糊度*/
	int ambresetfail[NFREQ];/*载波伪距无偏差（bias==0）导致模糊度强制初始化*/
} ssat_t;

typedef struct {
	gtime_t time;
	double isb;
}isb_t;

typedef struct {       /* RTK control/result type */
	gtime_t time;      /* rtk solution time */
	gtime_t time_d;    /* rtd solution time RTD时间更新后的时间，也可理解为开始解算的时间 */
	gtime_t time_s;    /* spp solution time */
	gtime_t time_ld;
	gtime_t time_ls;
	gtime_t time_k;    /*RTK成功解的时间（RTD/FLOAT/RTK都算）*/
	sol_t  sol;        /* RTK solution */
	sol_t  sol_last;   /* last solution 上次任意解的时间*/
	sol_t  sol_spp;    /* SPP solution */
	sol_t  sol_rtd;    /* RTD solution */
	sol_t  sol_rtk;    /* RTK solution */
	sol_t  sol_sppk;   /* spp kalman solution卡尔曼单点解.time为卡尔曼单点解成功时间*/
	double rb[6];      /* base position/velocity (ecef) (m|m/s) */
	int nx, na;        /* number of float states/fixed states */
	double tt;         /* time difference between current and previous (s)上次RTK成功解算(RTD/fix/float)与本次的龄期 */
	double tt_d;	   /*上次RTD开始解算与本次的龄期*/
	double* x, * P;    /* float states and their covariance */
	double* xa, * pa;  /* fixed states and their covariance */
	double x_s[11], p_s[11 * 11];/* single postipn fliter states and their covariance 位置 速度 receiver clock bias glo gal bds系统间偏差 接收机钟漂 */
	double x_d[6], p_d[6 * 6];   /* RTD估计值rtd fliter states and their covariance */
	double l_x_d[6], l_x[3];     /* last epoch psr diff l_x_d：上次RTD时间更新状态  l_x:上次RTK时间更新状态*/
	int nfix;                    /* number of continuous fixes of ambiguity */
	ssat_t ssat[MAXDIFOBS];      /* satellite status(只储存共视卫星) */
	int nsat;              /*��ǰ��Ԫ��sat����*/
	int presat;            /*上一个历元共视星数*/
	int n_errsat;          /*信噪比低于25的卫星数*/
	int satid[MAXDIFOBS];  /*参与双差卫星的ID*/
	prcopt_t opt;          /* processing options */
	int basechange;        /* base position change *///1:base change 0:base not change
	gtime_t prefixtime;    /*上次固定的时间*/
	double n_lerr_ratio, n_slip_ratio;
	int ratiomodel;        /*0为动态阈值 1为经验阈值*/
	int n_errsat_30;
	int staid;             /*测站编号*/
	int vrs_staid;
	int falsatnum;		   /*降星数信息*/
	ssat_t f_ssat[MAXFALSTA]; /*前5个历元内发生降星的卫星信息 */
	double lasttt;
	gtime_t lasttime;
	isb_t glo_isb;
	isb_t gal_isb;
	isb_t bds_isb;
	lg69t_pvt_t lgpvt;
	int vrs_sys_flag[4];     /*vrs解析时历史中某个系统存在标识*/

	double vel;				 /*空间速度向量模长，RTD??*/
	float  vrs_age;
	int fix_con_num;
	int statchange;
	int up_vrs_flag;         /*成功更新vrs标识*/
	int vrsbadnum;
} rtk_t;
typedef struct { /* last epoch meaasge */
	int sat;
	unsigned int dpsr1;            /* Prior psr detect 1:error 0:normal 先验伪距残差探测*/
	unsigned int dpsr2[NFREQ];     /* Prior detect 1:error 0:normal */
	unsigned int ddop1;            /* dop detect 1:error 0:normal 3sigma粗差探测doppler标记位*/
	unsigned int ddop2[NFREQ];     /* dop detect 1:error 0:normal */
	unsigned int dlsr2[NFREQ];
	double d1;						/* Doppler value */
}midmsg_t;

typedef struct {        /* imu data type */
	gtime_t time;       /* time */
	int stat;           /* status */
	int seqno;          /* sequence number */
	float temp;         /* temperature (C) */
	double rot[3];      /* rotation rate {x,y,z} (rad/s) */
	double acc[3];      /* acceleration data {x,y,z} (m/s^2) */
} imud_t;

typedef struct {
	double pos[3];   //1005/1006

	int staid;       //vrs staid
}staidinfo;

typedef struct {        /* imu type */
	imud_t data;        /* imu data */
	int nbyte;          /* bytes in imu data buffer */
	unsigned char buff[256]; /* imu data buffer */
} imu_t;

typedef void fatalfunc_t(const char*); /* fatal callback function type */

/* satellites, systems, codes functions --------------------------------------*/
extern int  satno(int sys, int prn);
extern int  satsys(int sat, int* prn);
extern int  satid2no(const char* id);
extern void satid2sys(const char* id, int* sys);
extern void satno2id(int sat, char* id);
extern unsigned char obs2code(const char* obs, int* freq);
extern char* code2obs(unsigned char code, int* freq);
extern int  satexclude(int sat, int svh, const prcopt_t* opt);
extern int testsnr(int base, int freq, double el, double snr, const int flag, const float snrthres);
extern void setcodepri(int sys, int freq, const char* pri);
extern int  getcodepri(int sys, unsigned char code, const char* opt);

/* matrix and vector functions -----------------------------------------------*/
extern double* mat(int n, int m);
extern int* imat(int n, int m);
extern double* zeros(int n, int m);
extern double* eye(int n);
extern double dot(const double* a, const double* b, int n);
extern double norm(const double* a, int n);
extern void matcpy(double* A, const double* B, int n, int m);
extern void imatcpy(int* A, const int* B, int n, int m);
extern void matmul(const char* tr, int n, int k, int m, double alpha,
	const double* a, const double* b, double beta, double* c);
extern int  matinv(double* A, int n);
extern int  solve(const char* tr, const double* A, const double* Y, int n,
	int m, double* X);
extern int  lsq(const double* A, const double* y, int n, int m, double* x,
	double* Q);
extern int  filter_leador(double* x, double* P, const double* H, const double* v,
	const double* R, int n, int m, int k1, int k2);
extern int  smoother(const double* xf, const double* Qf, const double* xb,
	const double* Qb, int n, double* xs, double* Qs);
extern void matprint(const double* A, int n, int m, int p, int q);
extern void matfprint(const double* A, int n, int m, int p, int q, FILE* fp);

extern void add_fatal(fatalfunc_t* func);

/* time and string functions -------------------------------------------------*/
extern double  str2num(const char* s, int i, int n);
extern int     str2time(const char* s, int i, int n, gtime_t* t);
extern void    time2str(gtime_t t, char* str, int n);
extern gtime_t epoch2time(const double* ep);
extern void    time2epoch(gtime_t t, double* ep);
extern gtime_t gpst2time(int week, double sec);
extern double  time2gpst(gtime_t t, int* week);
extern gtime_t gst2time(int week, double sec);
extern double  time2gst(gtime_t t, int* week);
extern gtime_t bdt2time(int week, double sec);
extern double  time2bdt(gtime_t t, int* week);
extern char* time_str(gtime_t t, int n);

extern gtime_t timeadd(gtime_t t, double sec);
extern double  timediff(gtime_t t1, gtime_t t2);
extern gtime_t gpst2utc(gtime_t t);
extern gtime_t utc2gpst(gtime_t t);
extern gtime_t gpst2bdt(gtime_t t);
extern gtime_t bdt2gpst(gtime_t t);
extern gtime_t timeget(void);
extern void    timeset(gtime_t t);
extern double  time2doy(gtime_t t);
extern double  utc2gmst(gtime_t t, double ut1_utc);
extern int read_leaps(const char* file);

extern int adjgpsweek(int week);

/* coordinates transformation ------------------------------------------------*/
extern void ecef2pos(const double* r, double* pos);
extern void pos2ecef(const double* pos, double* r);
extern void ecef2enu(const double* pos, const double* r, double* e);
extern void enu2ecef(const double* pos, const double* e, double* r);
extern void covenu(const double* pos, const double* P, double* Q);
extern void covecef(const double* pos, const double* Q, double* P);
extern void xyz2enu(const double* pos, double* E);
extern void eci2ecef(gtime_t tutc, const double* erpv, double* U, double* gmst);
extern void deg2dms(double deg, double* dms, int ndec);
extern double dms2deg(const double* dms);

/* input and output functions ------------------------------------------------*/
extern void readpos(const char* file, const char* rcv, double* pos);
extern int  sortobs(obs_t* obs);
extern void uniqnav(nav_t* nav);
extern int  screent(gtime_t time, gtime_t ts, gtime_t te, double tint);
extern int  readnav(const char* file, nav_t* nav);
extern int  savenav(const char* file, const nav_t* nav);
extern void freeobs(obs_t* obs);
extern void freenav(nav_t* nav, int opt);
extern int  readblq(const char* file, const char* sta, double* odisp);
extern int readrnxh(FILE* fp, double* ver, char* type, int* sys, int* tsys,
	char tobs[][MAXOBSTYPE][4], nav_t* nav);

/* debug trace functions -----------------------------------------------------*/
extern void traceopen(const char* file);
extern void traceclose(void);
extern void tracelevel(int level);
extern void trace(int level, const char* format, ...);
extern void tracet(int level, const char* format, ...);
extern void tracemat(int level, const double* A, int n, int m, int p, int q);
extern void traceobs(rtk_t* rtk, int level, const obsd_t* obs, int n);
extern void tracenav(int level, const nav_t* nav);
extern void tracegnav(int level, const nav_t* nav);
extern void tracehnav(int level, const nav_t* nav);
extern void tracepeph(int level, const nav_t* nav);
extern void tracepclk(int level, const nav_t* nav);
extern void traceb(int level, const unsigned char* p, int n);

/* positioning models --------------------------------------------------------*/
extern double satwavelen(int sat, int frq, const nav_t* nav);
extern double satazel(const double* pos, const double* e, double* azel);
extern double geodist(const double* rs, const double* rr, double* e);
extern void dops(int ns, const double* azel, double elmin, double* dop, const int* vsat);
extern void dops_dif(int ns, const double* azel, double elmin, double* dop, const int* vsat);
/* atmosphere models ---------------------------------------------------------*/
extern double ionmodel(gtime_t t, const double* ion, const double* pos,
	const double* azel);
extern double tropmodel(gtime_t time, const double* pos, const double* azel,
	double humi);
extern double tropmapf(gtime_t time, const double* pos, const double* azel,
	double* mapfw);
extern int ionocorr(gtime_t time, const nav_t* nav, int sat, const double* pos,
	const double* azel, int ionoopt, double* ion, double* var);
extern int tropcorr(gtime_t time, const nav_t* nav, const double* pos,
	const double* azel, int tropopt, double* trp, double* var);

/* geiod models --------------------------------------------------------------*/
extern double geoidh(const double* pos);

extern int readrnxh(FILE* fp, double* ver, char* type, int* sys, int* tsys,
	char tobs[][MAXOBSTYPE][4], nav_t* nav);
extern int readrnxfp(FILE* fp, gtime_t ts, gtime_t te, double tint,
	const char* opt, int flag, int index, char* type,
	obs_t* obs, nav_t* nav);
extern int readrnxobsb(FILE* fp, const char* opt, double ver, int* tsys,
	char tobs[][MAXOBSTYPE][4], int* flag, obsd_t* data, int rcv);

/* ephemeris and clock functions ---------------------------------------------*/
extern double eph2clk(gtime_t time, const eph_t* eph);
extern double geph2clk(gtime_t time, const geph_t* geph);
extern void eph2pos(gtime_t time, const eph_t* eph, double* rs, double* dts,
	double* var);
extern void geph2pos(gtime_t time, const geph_t* geph, double* rs, double* dts,
	double* var);
extern int  satpos(gtime_t time, gtime_t teph, int sat, int ephopt,
	const nav_t* nav, double* rs, double* dts, double* var,
	int* svh);
extern void satposs(gtime_t time, const obsd_t* obs, int n, const nav_t* nav,
	int sateph, double* rs, double* dts, double* var, int* svh, int maxsizeofsvh);

/* receiver raw data functions -----------------------------------------------*/
extern unsigned int getbitu(const unsigned char* buff, int pos, int len);
extern int          getbits(const unsigned char* buff, int pos, int len);
extern unsigned int rtk_crc24q(const unsigned char* buff, int len);

/* rtcm functions ------------------------------------------------------------*/
extern int init_rtcm(rtcm_t* rtcm, prelock_t* prelock, int rcv, int staid);
extern void free_rtcm(rtcm_t* rtcm);
extern int input_rtcm3(rtcm_t* rtcm, unsigned char data);

/* integer ambiguity resolution ----------------------------------------------*/
extern int lambda(int n, int m, const double* a, const double* Q, double* F,
	double* s);
/* standard positioning ------------------------------------------------------*/
extern int pntpos(rtk_t* rtk, const obsd_t* obs, int n, const nav_t* nav,
	const prcopt_t* opt, midmsg_t* midmsg, sol_t* sol, double* azel, ssat_t* ssat, int beforedect, int vof);

/* precise positioning -------------------------------------------------------*/
extern void rtkinit(rtk_t* rtk, const prcopt_t* opt);
extern void rtkfree(rtk_t* rtk);
extern int  rtkpos(rtk_t* rtk, const obsd_t* obs, int nobs, const nav_t* nav);

/* application defined functions ---------------------------------------------*/
extern int showmsg(char* format, ...);
extern int checkbrk(const char* format, ...);

extern int ix(int sat, const int* sats, int nsat);
extern int ij(int i, int j);

extern void xy_free(void* p);
extern void* xy_malloc(size_t t);

extern int decode_rtcm3(rtcm_t* rtcm);
extern int rtcmdataread(unsigned char* data, int length, obs_t* obs, nav_t* nav,
	double* pos, prelock_t* prelock, int rcv, rtk_t* rtk, int staid);

extern void uniqnav2(nav_t* nav, gtime_t t);
extern int    g_rtcm_week;
extern double g_gps_toe;
extern int    satnum_max;
extern midmsg_t midmsg_[MAXDIFOBS * 2];

extern unsigned char* hex_string2bytes2(char* hexstr, int* len);
extern int compare(const void* p1, const void* p2);
extern int compare2(const void* p1, const void* p2);
extern int test_sys(int sys, int m);
extern int get_sys_num(int sys);
extern int* izeros(int n, int m);
extern int maxminsat(int sys, int m);
extern int zdres(int base, const obsd_t* obs, int n, const double* rs, const double* dts,
	const int* svh, const nav_t* nav, const double* rr, const prcopt_t* opt, int index,
	double* y, double* e, double* azel, midmsg_t* midmsg);
extern int rtd_position(rtk_t* rtk, const obsd_t* obs, int nu, int nr,
	const nav_t* nav, midmsg_t* midmsg, double* rs, double* dts,
	int* svh, double* y, double* e, double* azel, int ns, int* sat, int* iu, int* ir);
extern double baseline(const double* ru, const double* rb, double* dr);
extern void ddcov(const int* nb, int n, const double* Ri, const double* Rj,
	int nv, double* R);
extern int resamb(rtk_t* rtk, double* bias, double* xa);
extern int manage_resamb(rtk_t* rtk, double* bias, double* xa);
extern eph_t* seleph(gtime_t time, int sat, int iode, const nav_t* nav);
extern geph_t* selgeph(gtime_t time, int sat, int iode, const nav_t* nav);
extern int pnt_pos_solution(rtk_t* rtk, const obsd_t* obs, int n, int nu, int nr, const nav_t* nav);
extern int pnt_pos_solution2(rtk_t* rtk, const obsd_t* obs, int n, int nu, int nr, const nav_t* nav);
extern int pnt_pos_solution3(rtk_t* rtk, const obsd_t* obs, int n, int nu, int nr, const nav_t* nav);
extern void holdamb_t(rtk_t* rtk, const double* xa, int* vflag, int ns);
extern void rtkoutnmea(rtk_t* rtk, nmea_t* nmea);
extern void rtkoutcustom(rtk_t* rtk, FILE* fp);

extern void age_init(rtk_t* rtk);
extern int select_sat(obs_t obsu, obs_t obsr, nav_t* nav, prcopt_t popt, obsd_t* obs, double* rb);
extern int select_sat_v10(rtk_t* rtkt, obs_t obsu, const obs_t obsr, const nav_t* nav, prcopt_t popt, obsd_t* obs, double* rb, double* rs, double* dts, int* svh, double* var);

extern void cal_azel(prcopt_t popt, const obsd_t* obs, int n, const nav_t* nav, double* rb, double* elev);
extern int orderdopdif(int n, double* diftem);
extern double dmm2deg(double dmm);

extern int ephclk(gtime_t time, gtime_t teph, int sat, const nav_t* nav,
	double* dts);
extern int decode_gga(char* buff, nmea_gga_t* sol);
extern int gga_judge_add(const unsigned char* ggain);
extern void outlg69tpvt(lg69t_pvt_t lgpvt, nmea_t* nmea_t);
extern int split_rtcm3(rtcm_t* rtcm, unsigned char data);

extern void settracefile(char* trace_file);
extern void matmul_new(int flag, const char* tr, int n, int k, int m, double alpha,
	const double* a, const double* b, double beta, double* c);
//__attribute__((visibility("default")))int  gga_filter3(unsigned char *ggabuff,unsigned char *rmcbuff,unsigned char *resultbuff_gga,unsigned char *resultbuff_rmc,double *enuv);
int  gga_filter3(unsigned char* ggabuff, unsigned char* rmcbuff, unsigned char* resultbuff_gga, unsigned char* resultbuff_rmc, double* enuv);
extern double get_gga_distance(nmea_gga_t* solgga, nmea_gga_t lastfiltergga, double* angle);
extern void drift_adjust(double lat, double lnt, double v, double dir, double* lat_new, double* lon_new);
extern int decode_rmc(char* buff, nmea_rmc_t* sol);
extern int outggas(unsigned char* buff, const nmea_gga_t sol_gga);
extern int outrmcs(unsigned char* buff, const nmea_rmc_t sol_rmc, const nmea_gga_t sol_gga);
extern void fiter_init2();
extern void syncsolinfo(sol_t sol, sol_t* solnew);
extern void cal_azel3(prcopt_t popt, int n, double* rb, double* elev, double* rs, int* svh);

extern int limitcmpgeo(int sat);
extern void rtd_igg3test(rtk_t* rtk, const obsd_t* obs, double* x, const double* p, const double* h, double* v, double* rvar, int* sat, int n, int m, int* vflg, midmsg_t* midmsg);
extern void rtk_igg3test(rtk_t* rtk, int rflag, const obsd_t* obs, double* x, const double* p, const double* h, double* v, double* rvar, const int* sat, int n, int m, int* vflg, midmsg_t* midmsg);

#ifdef __cplusplus
}
#endif
#endif /* RTKLIB_H */
