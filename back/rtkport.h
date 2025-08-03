#ifndef _RTKPORT_H
#define _RTKPORT_H
#include <stdarg.h>
#include <stdint.h>

#define BUFLENGTH   128

typedef struct {
	unsigned char gga[BUFLENGTH];
	unsigned char rmc[BUFLENGTH];
	unsigned char gsv[BUFLENGTH];
	unsigned char zda[BUFLENGTH];
	unsigned char bestvel[BUFLENGTH];
	unsigned char vtg[BUFLENGTH];
	double enuv[3];
	double snr_mean;
} nmea_t;

typedef struct {
	int      fre_flag;
	float  snr_thres;
	double  elemin;
	int      satnum_max;
	int      open_filter;
	int      open_replace;
} optconf_in_t;

typedef struct {
	double tod, lat, lon, hdop, alt, msl, tt;
	double ep[6];
	double pos[3];
	char ns, ew, ua, um;
	int i, solq, nrcv;
	float age;
	int staid;

	double lon_pre;  //滤波前的经纬度
	double lat_pre;  //

	double time;
	double rr[3];
	double vel;
	double dir;

	double angle;
	double dis_v_f;
} nmea_gga_t;

typedef struct {          /* solution type */
	double tod;
	double lat;
	double lon;
	double vel;
	double dir;
	double date;
	double ang;
	double ep[6];
	double pos[3];
	char act, ns, ew, mew, mode;
	double time;
} nmea_rmc_t;

typedef struct {
	void* (*malloc)(uint32_t size);
	void (*free)(void* buffer);
	void (*printf_info)(char* fmt, ...);//向窗口打印信息
}XYRTK_Callbacks;

void rtk_version(char ver[64]);
void set_tracelevel(int level);
int rtk_init(optconf_in_t optconf_in, XYRTK_Callbacks* callbacks);
void rtk_free(void);
int updatePvt(unsigned char* rtcm1, int rtcmLength, nmea_t* nmea);
int updateNav(unsigned char* rtcm1, int rtcmLength);
int updateVrs(unsigned char* rtcm1, int rtcmLength);
int updateObs(unsigned char* rtcm1, int rtcmLength, nmea_t* nmea);
int gga_filter(unsigned char* ggabuff, unsigned char* rmcbuff, unsigned char* resultbuff_gga, unsigned char* resultbuff_rmc);
void fiterinit(void);

#endif
