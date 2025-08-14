/**
 * @file AppConfigPara.h
 * @brief Application Configuration Parameters and Macros for PPOI_Nav System
 *
 * This header file defines the main application configuration parameters, macro definitions, and structure types
 * for the PPOI_Nav navigation system. It includes system operation modes, filtering parameters, observation controls,
 * high-precision GNSS integration models, file paths, and other commonly used configuration options.
 * These parameters provide a unified interface and foundational support for modules such as navigation computation,
 * integrated positioning, observation data processing, and system modeling.
 *
 * @author  LEADOR PPOI Team - Yejin
 * @date    2024-09-04
 * @version 1.0
 * @copyright Copyright (c) 2020-2025 PPOI_Nav Project
 *
 * @mainpage PPOI_Nav Application Configuration Parameters
 * @section intro_sec Introduction
 * This file provides the essential application-level configuration parameters, macro definitions, and structure types
 * for the PPOI_Nav navigation and integrated positioning system. It is suitable for navigation computation,
 * integrated positioning, observation data processing, and other related scenarios, offering unified configuration
 * management and system parameter support.
 *
 * @section macro_sec Macros & Parameters Overview
 * - System modes and observation controls: CAL_TYPE, NAV_TYPE, MASK_MEAS, OD_ENABLE, etc.
 * - Filtering and algorithm parameters: N_STEP, YAW_ALIGN, YAW_STD, FB_att_TAU, Qt_gyr_h, etc.
 * - Model and feature configuration: IPOS_SER, IPOSDY_OLD, LOST_ASUME, GNSS_CHECK_LG69T, etc.
 * - State covariance and process noise: Pk_*, Qt_*, FB_*, etc.
 * - Structure definitions: sim32bin_t (simulation/observation data), config_t (configuration parameters), CONFIG_t (generic configuration item)
 *
 * @section history_sec History
 * - 2024-09-04: Initial version, defined main parameters and structures.
 * - 2024-09-10: Added model configuration options and improved comments.
 *
 * @section reference_sec References
 * - [1] PPOI_Nav Project Documentation and Design Specifications
 * - [2] PSINS: A C++ Inertial Navigation Library, https://psins.org.cn/
 * - [3] RTKLIB: An Open Source Program Package for GNSS Positioning, http://www.rtklib.com/
 *
 * @section usage_sec Usage
 * Include this header file directly to access all essential application-level configuration parameters, macro definitions,
 * and structure types for the PPOI_Nav system. It provides parameter support for navigation computation, integrated positioning,
 * and related system modules.
 *
 * @defgroup app_config Application Configuration Parameters
 * @defgroup app_struct Application Structure Types
 */

#ifndef MYNAV_APPCONFIGPARA_H
#define MYNAV_APPCONFIGPARA_H

#define CAL_TYPE   0x0        /*< Calibration mode identifier */
#define NAV_TYPE   0x1        /*< Navigation mode identifier */

#define MASK_MEAS  0377777    /*< Enable all measurements; use detailed macros for control */
#define OD_ENABLE  0x00       /*< Disable odometer and virtual odometer */

#define N_STEP     6          /*< Step size for filter or algorithm (e.g., (2*(nq+nr) +3) / 20 = 3.8 < N) */
#define YAW_ALIGN  0.5        /*< Yaw alignment RMS in degrees */
#define YAW_STD    5          /*< Yaw standard deviation in degrees */


//#define IPOS_DZ               /*< Use old IMU DZ parameters */
#ifdef IPOS_DZ
	#define FS          200
	#define TS          1.0/200    /*< Time step for DZ serial (125Hz) */
	#define DT_POS     -0.20       /*< Position time delay compensation */
	#define DT_VEL     -0.00       /*< Velocity time delay compensation */
	#define DT_YAW     -0.20       /*< Yaw time delay compensation */
	#define DT_GNSS    -0.12       /*< GNSS time delay compensation */
	//#define DT_GNSS    -0.00       /*< GNSS time delay compensation */
#endif

#define IPOS_SER              /*< Use IPOS3 100Hz serial mode */
#ifdef IPOS_SER
	#define FS          100 
	#define TS          1.0/100    /*< Time step for IPOS3 serial (100Hz) */
	#define DT_POS     -0.00       /*< Position time delay compensation */
	#define DT_VEL     -0.00       /*< Velocity time delay compensation */
	#define DT_YAW     -0.00       /*< Yaw time delay compensation */
	#define DT_GNSS    -0.12       /*< GNSS time delay compensation */
	//#define DT_GNSS    -0.00       /*< GNSS time delay compensation */
#endif


#define FB_att_TAU     TS*1.0      /*< Attitude feedback time constant */
#define FB_vel_TAU     TS*10.0     /*< Velocity feedback time constant */
#define FB_pos_TAU     TS*10.0     /*< Position feedback time constant */
#define FB_eb_TAU      TS*10.0     /*< Gyro bias feedback time constant */
#define FB_db_TAU      TS*10.0     /*< Accel bias feedback time constant */
#define FB_od_TAU      TS*10.0     /*< Odometer feedback time constant */
#define FB_gnss_yaw_TAU TS*10.0    /*< GNSS yaw feedback time constant */

#define Qt_gyr_h        0.8*DPSH   /*< Horizontal gyro noise (deg/sqrt(hr)) */
#define Qt_gyr_v        1.0*DPSH   /*< Vertical gyro noise (deg/sqrt(hr)) */
#define Pk_eb_h      10.0*DPH     /*< Horizontal gyro bias initial covariance */
#define Pk_eb_v      15.0*DPH     /*< Vertical gyro bias initial covariance */
#define Qt_acc_h     1000.0*UGPSHZ /*< Horizontal accel noise (ug/sqrt(Hz)) */
#define Qt_acc_v     1200.0*UGPSHZ /*< Vertical accel noise (ug/sqrt(Hz)) */
#define Pk_db_h       1.0*MG       /*< Horizontal accel bias initial covariance */
#define Pk_db_v       3.0*MG       /*< Vertical accel bias initial covariance */

#define Rt_yaw_gnss      0.5*DEG   /*< GNSS yaw measurement noise */

#ifdef IPOS_DZ
	#define MASK_MEAS    0377077    /*< Enable all measurements; use detailed macros for control */
	#define RB_Init          INF    /*< Infinite value for constraints */
	#define RB_Norm          0.5    /*< Infinite value for constraints */
	#define YAW_ALIGN        5.0    /*< Yaw alignment RMS in degrees */

	#define Qt_gyr_h        0.3*DPSH   /*< Horizontal gyro noise (deg/sqrt(hr)) */
	#define Qt_gyr_v        0.8*DPSH   /*< Vertical gyro noise (deg/sqrt(hr)) */

	#define Rt_yaw_gnss      5.0*DEG   /*< GNSS yaw measurement noise */
	/* IPOSDZ 20250704*/
#endif 

#define AVPINUM     -int((DT_GNSS+DT_POS)*FS) + 1


#define IPOSDY_OLD            /*< Use old IMU DY parameters */
//#define IPOS_BOSHI
//#define IPOS3_CX
//#define IPOSHS          

//#define LOW_PASS_IIR        /*< Enable 10Hz IIR low-pass filter */
#define LOST_ASUME          /*< Use speed & heading assumption when lost */
//#define GNSS_CHECK_UBOX       /*< Enable GNSS check for  receiver */
//#define GNSS_CHECK_UM482    /*< Enable GNSS check for UM482 receiver */
//#define GNSS_CHECK_UM982    /*< Enable GNSS check for UM982 receiver */
#define GNSS_CHECK_LG69T      /*< Enable GNSS check for LG69T receiver */


#ifdef IPOS_BOSHI
	#define Qt_gyr_h        0.8*DPSH   /*< Horizontal gyro noise (deg/sqrt(hr)) */
	#define Qt_gyr_v        1.0*DPSH   /*< Vertical gyro noise (deg/sqrt(hr)) */
	#define Pk_eb_h      10.0*DPH     /*< Horizontal gyro bias initial covariance */
	#define Pk_eb_v      25.0*DPH     /*< Vertical gyro bias initial covariance */
	#define RB_Init          INF       /*< Infinite value for constraints */
	#define RB_Norm          0.5       /*< Infinite value for constraints */

	#define Qt_acc_h     3000.0*UGPSHZ /*< Horizontal accel noise (ug/sqrt(Hz)) */
	#define Qt_acc_v     5000.0*UGPSHZ /*< Vertical accel noise (ug/sqrt(Hz)) */
#endif 

#ifdef IPOSDY_OLD
	#define Qt_gyr_h        0.8*DPSH   /*< Horizontal gyro noise (deg/sqrt(hr)) */
	#define Qt_gyr_v        1.0*DPSH   /*< Vertical gyro noise (deg/sqrt(hr)) */
	#define Pk_eb_h      10.0*DPH     /*< Horizontal gyro bias initial covariance */
	#define Pk_eb_v      20.0*DPH     /*< Vertical gyro bias initial covariance */
	#define RB_Init          INF      /*< Infinite value for constraints */
	#define RB_Norm          0.80    /*< Infinite value for constraints */
#endif

#ifdef IPOS3_CX
	#define RB_Init          INF       /*< Infinite value for constraints */
	#define RB_Norm          0.5       /*< Infinite value for constraints */

	#define Qt_gyr_h       0.8*DPSH   /*< Horizontal gyro noise (deg/sqrt(hr)) */
	#define Qt_gyr_v       1.0*DPSH   /*< Vertical gyro noise (deg/sqrt(hr)) */
#endif 


#define Pk_att_h        60*MIN     /*< Horizontal attitude initial covariance */
#define Pk_att_v       180*MIN     /*< Vertical attitude initial covariance */
#define Pk_vel_h       1.0         /*< Horizontal velocity initial covariance */
#define Pk_vel_v       1.0         /*< Vertical velocity initial covariance */
#define Pk_pos_h       1.0/RE      /*< Horizontal position initial covariance */
#define Pk_pos_v       3.0         /*< Vertical position initial covariance */

#define Pk_att_h_min   1e-4*DEG    /*< Minimum horizontal attitude covariance */
#define Pk_att_v_min   1e-4*DEG    /*< Minimum vertical attitude covariance */
#define Pk_vel_h_min   1e-3        /*< Minimum horizontal velocity covariance */
#define Pk_vel_v_min   1e-3        /*< Minimum vertical velocity covariance */
#define Pk_pos_h_min   1e-3/RE     /*< Minimum horizontal position covariance */
#define Pk_pos_v_min   1e-3        /*< Minimum vertical position covariance */
#define Pk_eb_h_min    1e-4*DPH    /*< Minimum horizontal gyro bias covariance */
#define Pk_eb_v_min    1e-4*DPH    /*< Minimum vertical gyro bias covariance */
#define Pk_db_h_min    1e-3*UG     /*< Minimum horizontal accel bias covariance */
#define Pk_db_v_min    1e-3*UG     /*< Minimum vertical accel bias covariance */

/** Rt in GNSS */
#define Rt_Vn_gnss_h     0.5       /*< GNSS horizontal velocity measurement noise */
#define Rt_Vn_gnss_v     0.8       /*< GNSS vertical velocity measurement noise */
#define Rt_Pos_gnss_h    0.5       /*< GNSS horizontal position measurement noise */
#define Rt_Pos_gnss_v    0.8       /*< GNSS vertical position measurement noise */

#define Rt_vn_od_h       0.1       /*< Odometer horizontal velocity measurement noise */
#define Rt_vn_od_v       0.3       /*< Odometer vertical velocity measurement noise */
#define Rt_zupt          0.1       /*< ZUPT measurement noise */
#define Rt_nhc           0.1       /*< NHC measurement noise */
#define Rt_zihr          5.0*DEG   /*< ZIHR measurement noise */

#define Qt_pos     0           /*< Process noise for position state */
#define Qt_eb      0           /*< Process noise for gyro bias state */
#define Qt_db      0           /*< Process noise for accel bias state */
#define Qt_Other   0           /*< Process noise for other states */

#define Pk_od_pitch 1.0*DEG    /*< Odometer pitch initial covariance */
#define Pk_od_scale 0.1        /*< Odometer scale initial covariance */
#define Pk_od_yaw   1.0*DEG    /*< Odometer yaw initial covariance */
#define Pk_gnss_yaw 1.0*DEG    /*< GNSS yaw initial covariance */

#define Pk_od_pitch_min 1e-3*DEG   /*< Minimum odometer pitch covariance */
#define Pk_od_scale_min 1e-4       /*< Minimum odometer scale covariance */
#define Pk_od_yaw_min   1e-3*DEG   /*< Minimum odometer yaw covariance */
#define Pk_gnss_yaw_min 1e-3*DEG   /*< Minimum GNSS yaw covariance */

#define Pk_att_h_max    360*DEG    /*< Maximum horizontal attitude covariance */
#define Pk_att_v_max    360*DEG    /*< Maximum vertical attitude covariance */
#define Pk_vel_h_max    20.0       /*< Maximum horizontal velocity covariance */
#define Pk_vel_v_max    20.0       /*< Maximum vertical velocity covariance */
#define Pk_pos_h_max    1000.0/RE  /*< Maximum horizontal position covariance */
#define Pk_pos_v_max    1000.0     /*< Maximum vertical position covariance */
#define Pk_eb_h_max     3600*DPH   /*< Maximum horizontal gyro bias covariance */
#define Pk_eb_v_max     3600*DPH   /*< Maximum vertical gyro bias covariance */
#define Pk_db_h_max     10.0*MG    /*< Maximum horizontal accel bias covariance */
#define Pk_db_v_max     10.0*MG    /*< Maximum vertical accel bias covariance */
#define Pk_od_pitch_max 10.0*DEG   /*< Maximum odometer pitch covariance */
#define Pk_od_scale_max 0.3        /*< Maximum odometer scale covariance */
#define Pk_od_yaw_max   10.0*DEG   /*< Maximum odometer yaw covariance */
#define Pk_gnss_yaw_max 10.0*DEG   /*< Maximum GNSS yaw covariance */

#define Xk_att_h_max    INF        /*< Maximum horizontal attitude state value */
#define Xk_att_v_max    INF        /*< Maximum vertical attitude state value */
#define Xk_vel_h_max    INF        /*< Maximum horizontal velocity state value */
#define Xk_vel_v_max    INF        /*< Maximum vertical velocity state value */
#define Xk_pos_h_max    INF        /*< Maximum horizontal position state value */
#define Xk_pos_v_max    INF        /*< Maximum vertical position state value */
#define Xk_eb_h_max     600.0*DPH  /*< Maximum horizontal gyro bias state value */
#define Xk_eb_v_max     100.0*DPH  /*< Maximum vertical gyro bias state value */
#define Xk_db_h_max     100.0*MG   /*< Maximum horizontal accel bias state value */
#define Xk_db_v_max     100.0*MG   /*< Maximum vertical accel bias state value */
#define Xk_od_pitch_max 90.0*DEG   /*< Maximum odometer pitch state value */
#define Xk_od_scale_max 0.5        /*< Maximum odometer scale state value */
#define Xk_od_yaw_max   90.0*DEG   /*< Maximum odometer yaw state value */
#define Xk_gnss_yaw_max 90.0*DEG   /*< Maximum GNSS yaw state value */

#define FB_att_h   30*MIN      /*< Horizontal attitude feedback gain */
#define FB_att_v   60*MIN      /*< Vertical attitude feedback gain */
#define FB_vel_h   10.0        /*< Horizontal velocity feedback gain */
#define FB_vel_v   10.0        /*< Vertical velocity feedback gain */
#define FB_pos_h   25.0/RE     /*< Horizontal position feedback gain */
#define FB_pos_v   10.0        /*< Vertical position feedback gain */
#define FB_eb_h    1.0*DPH     /*< Horizontal gyro bias feedback gain */
#define FB_eb_v    1.0*DPH     /*< Vertical gyro bias feedback gain */
#define FB_db_h    1e5*UG      /*< Horizontal accel bias feedback gain */
#define FB_db_v    1e5*UG      /*< Vertical accel bias feedback gain */
#define FB_od_pitch 5e-1*DEG   /*< Odometer pitch feedback gain */
#define FB_od_scale 1e-1       /*< Odometer scale feedback gain */
#define FB_od_yaw   5e-1*DEG   /*< Odometer yaw feedback gain */
#define FB_gnss_yaw 5e-1*DEG   /*< GNSS yaw feedback gain */


typedef struct {
	double t;              
	double wm_x;           
	double wm_y;          
	double wm_z;           
	double vm_x;           
	double vm_y;          
	double vm_z;          
	double dS;            
	double lat_gps;      
	double lon_gps;       
	double hgt_gps;      
	double dop;           
	double eastvel_gps;   
	double northvel_gps;   
	double upvel_gps;      
	double satnum;       
	double None;        
	double yaw;         
	double yawrms;         
	double lat_base;      
	double lon_base;      
	double hgt_base;     
	double pitch_base;  
	double roll_base;      
	double yaw_base;     
	double eastvel_base;  
	double northvel_base;  
	double uphvel_base;    
	double hdop;          
	double std_lat;       
	double std_lon;      
	double std_hgt;     
} sim32bin_t;

typedef struct {
	char nav_file[128];    
	char obs_file[128];   
	char trace_file[128];   
	char res_file[128];
	int  trace_lever;
} config_t;

typedef struct {
	const char* name;      
	int format;            
	void* var;            
	const char* comment;  
} CONFIG_t;




#endif //MYNAV_APPCONFIGPARA_H
