#include "rtkport.h"
#include "rtklib.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

typedef enum _speedmode {
	very_slow,
	slow,
	high
}speedmode;

typedef struct {
	int fixtimes;
	int pftimes;
	int fix_location_buffersize;

	int no_location_resettime;

	double lastggaTime;
	nmea_gga_t last_fitergga;
	double last_dir;
	double last_dir2;

	int times;
	double predic_error;
	double measure_error;
	double predic_error_lat;
	double measure_error_lat;
	double predic_error_lon;
	double measure_error_lon;
}filter_struct;

filter_struct ggafilter;
static int filterstartnum = 0;    //计数器，初始化后第一个点不作为滤波基准

//小数转度分
double decimal_to_degree(double data) {
	int hourdata = ((int)data) * 100;
	double result = hourdata + (data - (int)data) * 60;
	return result;
}

void settimes(int t) {
	ggafilter.times = t;
}

void filter_para_init() {
	ggafilter.predic_error = 4;
	ggafilter.measure_error = 3;

	ggafilter.predic_error_lat = 4;
	ggafilter.measure_error_lat = 3;

	ggafilter.predic_error_lon = 4;
	ggafilter.measure_error_lon = 3;

	ggafilter.times = 10;
}

double predic_lon(double oldvalue, double* currentvalue, double speed) {
	double result = oldvalue;
	int i = 0;
	double gauss = 0.0;
	double delta = 0.0;
	double gain = 0.0;

	int filtertimes = ggafilter.times;
	if (speed > 0.5) {
		filtertimes = ggafilter.times * 4 / 5;
	}
	if (speed > 2) {
		filtertimes = ggafilter.times * 3 / 5;
	}

	if (speed > 3) {
		filtertimes = ggafilter.times * 2 / 5;
	}

	if (speed > 7) {
		filtertimes = 1;
	}
	for (i = 0; i < filtertimes; i++) {
		gauss = sqrt(ggafilter.predic_error_lon * ggafilter.predic_error_lon + ggafilter.measure_error_lon * ggafilter.measure_error_lon);
		delta = *currentvalue - oldvalue;
		gain = sqrt((gauss * gauss) / (gauss * gauss + ggafilter.predic_error_lon * ggafilter.predic_error_lon));
		result = oldvalue + gain * delta;
		ggafilter.measure_error_lon = *currentvalue - result;

		*currentvalue = result;
	}
	return result;
}

double predic_lat(double oldvalue, double* currentvalue, double speed) {
	double result = oldvalue;
	int i = 0;
	double gauss = 0.0;
	double delta = 0.0;
	double gain = 0.0;

	int filtertimes = ggafilter.times;
	if (speed > 0.5) {
		filtertimes = ggafilter.times * 4 / 5;
	}
	if (speed > 2) {
		filtertimes = ggafilter.times * 3 / 5;
	}

	if (speed > 3) {
		filtertimes = ggafilter.times * 2 / 5;
	}

	if (speed > 7) {
		filtertimes = 1;
	}
	for (i = 0; i < filtertimes; i++) {
		gauss = sqrt(ggafilter.predic_error_lat * ggafilter.predic_error_lat + ggafilter.measure_error_lat * ggafilter.measure_error_lat);
		delta = *currentvalue - oldvalue;
		gain = sqrt((gauss * gauss) / (gauss * gauss + ggafilter.predic_error_lat * ggafilter.predic_error_lat));
		result = oldvalue + gain * delta;
		ggafilter.measure_error_lat = *currentvalue - result;

		*currentvalue = result;
	}
	return result;
}

//根据外部输入的速度rmc进行滤波
void fiterinit()
{
	ggafilter.fixtimes = 0;
	ggafilter.pftimes = 0;
	ggafilter.fix_location_buffersize = 5;
	ggafilter.no_location_resettime = 5;     // 5秒无收到定位，重置定位缓存数组

	memset(&ggafilter.last_fitergga, 0, sizeof(nmea_gga_t));
	ggafilter.lastggaTime = 0.0;
	filter_para_init();
}

void buffsplit(char* buff, char* ggabuff, char* rmcbuff)
{
	int n = 0;
	char* p, * q = NULL, * val[3] = { 0 };

	for (p = buff; *p && n < 3; p = q + 1) {
		if ((q = strchr(p, '$')) || (q = strchr(p, '\n')))
		{
			val[n++] = p; *q = '\0';
		}

		else break;
	}
	strcpy(ggabuff, val[1]);
	strcpy(rmcbuff, val[2]);
}

extern int outggas(unsigned char* buff, const nmea_gga_t sol_gga)
{
	char* p = (char*)buff, * q = NULL, sum = 0;

	double hour = 0.0, min = 0.0;
	double sec = 0.0;

	double lat = 0.0, lon = 0.0;
	double lat1 = 0.0, lon1 = 0.0;
	double lat2 = 0.0, lon2 = 0.0;

	hour = (int)sol_gga.tod / 10000;
	min = (int)(sol_gga.tod - hour * 10000) / 100;
	sec = sol_gga.tod - hour * 10000 - min * 100;

	lat = decimal_to_degree(sol_gga.lat);  //小数转为度分
	lon = decimal_to_degree(sol_gga.lon);
	lat1 = (int)lat / 100;
	lat2 = lat - lat1 * 100;

	lon1 = (int)lon / 100;
	lon2 = lon - lon1 * 100;

	p += sprintf(p, "$GPGGA,%02.0f%02.0f%05.2f,%02.0f%010.7f,%c,%03.0f%010.7f,%c,%d,%02d,%.1f,%.3f,M,%.3f,M,%.1f,%d",
		hour, min, sec, lat1, lat2, sol_gga.ns,
		lon1, lon2, sol_gga.ew, sol_gga.solq,
		sol_gga.nrcv, sol_gga.hdop, sol_gga.alt, sol_gga.msl, sol_gga.age, sol_gga.staid);

	for (q = (char*)buff + 1, sum = 0; *q; q++)
		sum ^= *q; /* check-sum */
	p += sprintf(p, "*%02X", sum);

	return p - (char*)buff;
}

extern int outrmcs(unsigned char* buff, const nmea_rmc_t sol_rmc, const nmea_gga_t sol_gga)
{
	char* p = (char*)buff, * q = NULL, sum = 0;

	double hour = 0.0, min = 0.0;
	double sec = 0.0;

	double lat = 0.0, lon = 0.0;
	double lat1 = 0.0, lon1 = 0.0;
	double lat2, lon2;
	hour = (int)sol_gga.tod / 10000;
	min = (int)(sol_gga.tod - hour * 10000) / 100;
	sec = sol_gga.tod - hour * 10000 - min * 100;

	lat = decimal_to_degree(sol_gga.lat);  //小数转为度分
	lon = decimal_to_degree(sol_gga.lon);

	lat1 = (int)lat / 100;
	lat2 = lat - lat1 * 100;

	lon1 = (int)lon / 100;
	lon2 = lon - lon1 * 100;

	p += sprintf(p, "$GPRMC,%02.0f%02.0f%05.2f,A,%02.0f%010.7f,%c,%03.0f%010.7f,%c,%4.2f,%4.2f,%06d,%.1f,%c,%c",
		hour, min, sec, lat1, lat2, sol_rmc.ns,
		lon1, lon2, sol_rmc.ew, sol_rmc.vel, sol_rmc.dir,
		(int)sol_rmc.date, sol_rmc.ang, sol_rmc.mew,
		sol_rmc.mode);
	for (q = (char*)buff + 1, sum = 0; *q; q++) sum ^= *q; /* check-sum */
	p += sprintf(p, "*%02X", sum);

	return p - (char*)buff;
}

int extract_gga(nmea_gga_t* sol)
{
	if (!sol->solq)   //定位失败不过滤
	{
		ggafilter.fixtimes = 0;
		return 0;
	}

	if (sol->solq == 4 || sol->solq == 5) {// 对固定解和浮点解不过滤 连续5个历元 不滤波
		ggafilter.fixtimes++;
		if (ggafilter.fixtimes > ggafilter.fix_location_buffersize) {
			return 0;
		}
		if (sol->vel < 4 && sol->solq == 4)  //固定解低速下不过滤       1m/s???
			return 0;
	}
	else {
		ggafilter.fixtimes = 0;
	}
	return 1;
}

//提取速度信息
int extract_rmc(nmea_rmc_t* sol_rmc)
{
	sol_rmc->vel = sol_rmc->vel * 3.6 / 1.852;
	return 1;
}

extern double dmm2deg(double dmm)
{
	return floor(dmm / 100.0) + fmod(dmm, 100.0) / 60.0;
}

/* decode nmea gxgga: fix information ----------------------------------------*/
int decode_nmeagga(char** val, int n, nmea_gga_t* sol)
{
	double tod = 0.0, lat = 0.0, lon = 0.0, hdop = 0.0, alt = 0.0, msl = 0.0;
	double pos[3] = { 0 };
	char ns = 'N', ew = 'E', ua = ' ', um = ' ';
	int i = 0, solq = 0, nrcv = 0;
	int age = 0;
	int staid = 0;
	double hour = 0.0, min = 0.0;
	double sec = 0.0;

	//trace(4,"decode_nmeagga: n=%d\n",n);

	for (i = 0; i < n; i++) {
		if (val[i] == NULL)
			continue;
		switch (i) {
		case  0: tod = atof(val[i]); break; /* time in utc (hhmmss) */
		case  1: lat = atof(val[i]); break; /* latitude (ddmm.mmm) */
		case  2: ns = *val[i];      break; /* N=north,S=south */
		case  3: lon = atof(val[i]); break; /* longitude (dddmm.mmm) */
		case  4: ew = *val[i];      break; /* E=east,W=west */
		case  5: solq = (int)atof(val[i]); break; /* fix quality */
		case  6: nrcv = (int)atof(val[i]); break; /* # of satellite tracked */
		case  7: hdop = atof(val[i]); break; /* hdop */
		case  8: alt = atof(val[i]); break; /* altitude in msl */
		case  9: ua = *val[i];      break; /* unit (M) */
		case 10: msl = atof(val[i]); break; /* height of geoid */
		case 11: um = *val[i];      break; /* unit (M) */
		case 12: age = atof(val[i]);      break; /* 差分龄期 */
		case 13: staid = (int)atof(val[i]);      break; /* 站点号 */
		}
	}
	if ((ns != 'N' && ns != 'S') || (ew != 'E' && ew != 'W')) {
		trace(2, "invalid nmea gpgga format\n");
		return 0;
	}
	if (lat == 0.0 && lon == 0.0) {
		trace(2, "invalid nmea gpgga format\n");
		return 0;
	}
	if (nrcv == 0) {
		trace(2, "invalid nmea gpgga format\n");
		return 0;
	}
	sol->tod = tod;
	sol->lat = (ns == 'N' ? 1.0 : -1.0) * dmm2deg(lat);
	sol->ns = ns;
	sol->lon = (ew == 'E' ? 1.0 : -1.0) * dmm2deg(lon);
	sol->ew = ew;
	sol->solq = solq;
	sol->nrcv = nrcv;
	sol->hdop = hdop;
	sol->alt = alt;
	sol->ua = ua;
	sol->msl = msl;
	sol->um = um;
	sol->age = age;
	sol->staid = staid;

	sol->lat_pre = lat;
	sol->lon_pre = lon;

	pos[0] = (ns == 'N' ? 1.0 : -1.0) * dmm2deg(lat) * D2R;
	pos[1] = (ew == 'E' ? 1.0 : -1.0) * dmm2deg(lon) * D2R;
	pos[2] = alt + msl;

	pos2ecef(pos, sol->rr);

	hour = (int)tod / 10000;
	min = (int)(tod - hour * 10000) / 100;
	sec = tod - hour * 10000 - min * 100;
	sol->time = hour * 3600 + min * 60 + sec;
	//trace(4, "ingga  decode_nmeagga: n=%d %f %f %f %f \n", n, sol->lat, sol->lon, sol->lat_pre, sol->lon_pre);
	return 1;
}

extern int decode_gga(char* buff, nmea_gga_t* sol)
{
	char* p = NULL, * q = NULL, * val[BUFLENGTH] = { 0 };
	int n = 0;
	char gga[BUFLENGTH] = "";
	strcpy(gga, (char*)buff);

	trace(4, "decode_nmea: buff=%s\n", buff);

	/* parse fields */
	for (p = gga; *p && n < BUFLENGTH; p = q + 1) {
		if ((q = strchr(p, ',')) || (q = strchr(p, '*'))) {
			val[n++] = p; *q = '\0';
		}

		else break;
	}
	if (n < 1)
		return 0;

	else if (!strcmp(val[0] + 3, "GGA")) { /* $xxGGA */
		return decode_nmeagga(val + 1, n - 1, sol);
	}
	return 0;
}

/* decode nmea gxrmc: recommended minumum data for gps -----------------------*/
int decode_nmearmc(char** val, int n, nmea_rmc_t* sol)
{
	double tod = 0.0, lat = 0.0, lon = 0.0, vel = 0.0, dir = 0.0, date = 0.0, ang = 0.0;
	char act = ' ', ns = 'N', ew = 'E', mew = 'E', mode = 'A';
	int i = 0;
	double hour = 0.0, min = 0.0;
	double sec = 0.0;
	//trace(4,"decode_nmearmc: n=%d\n",n);

	for (i = 0; i < n; i++) {
		if (val[i] == NULL)
			continue;
		switch (i) {
		case  0: tod = atof(val[i]); break; /* time in utc (hhmmss) */
		case  1: act = *val[i];      break; /* A=active,V=void */
		case  2: lat = atof(val[i]); break; /* latitude (ddmm.mmm) */
		case  3: ns = *val[i];      break; /* N=north,S=south */
		case  4: lon = atof(val[i]); break; /* longitude (dddmm.mmm) */
		case  5: ew = *val[i];      break; /* E=east,W=west */
		case  6: vel = atof(val[i]); break; /* speed (knots) */
		case  7: dir = atof(val[i]); break; /* track angle (deg) */
		case  8: date = atof(val[i]); break; /* date (ddmmyy) */
		case  9: ang = atof(val[i]); break; /* magnetic variation */
		case 10: mew = *val[i];      break; /* E=east,W=west */
		case 11: mode = *val[i];      break; /* mode indicator (>nmea 2) */
			/* A=autonomous,D=differential */
			/* E=estimated,N=not valid,S=simulator */
		}
	}
	if ((act != 'A' && act != 'V') || (ns != 'N' && ns != 'S') || (ew != 'E' && ew != 'W')) {
		//trace(2,"invalid nmea gprmc format\n");
		return 0;
	}
	if (lat == 0.0 && lon == 0.0) {
		trace(2, "invalid nmea gpgga format\n");
		return 0;
	}

	sol->tod = tod;
	sol->act = act;
	sol->lat = (ns == 'N' ? 1.0 : -1.0) * dmm2deg(lat);
	sol->ns = ns;
	sol->lon = (ew == 'E' ? 1.0 : -1.0) * dmm2deg(lon);
	sol->ew = ew;
	sol->vel = vel;
	sol->dir = dir;
	sol->date = date;
	sol->ang = ang;
	sol->mew = mew;
	sol->mode = mode;
	hour = (int)tod / 10000;
	min = (int)(tod - hour * 10000) / 100;
	sec = tod - hour * 10000 - min * 100;
	sol->time = hour * 3600 + min * 60 + sec;
	return 2; /* update time */
}

extern int decode_rmc(char* buff, nmea_rmc_t* sol)
{
	char* p = NULL, * q = NULL, * val[BUFLENGTH] = { 0 };
	int n = 0;

	char rmc[BUFLENGTH];
	strcpy(rmc, (char*)buff);

	trace(4, "decode_nmea: buff=%s\n", buff);

	/* parse fields */
	for (p = rmc; *p && n < BUFLENGTH; p = q + 1) {
		if ((q = strchr(p, ',')) || (q = strchr(p, '*'))) {
			val[n++] = p; *q = '\0';
		}

		else break;
	}
	if (n < 1)
		return 0;
	if (!strcmp(val[0] + 3, "RMC")) { /* $xxRMC */
		return decode_nmearmc(val + 1, n - 1, sol);
	}
	return 0;
}

double get_distance(double lat1, double lon1, double lat2, double lon2) {//根据经纬度计算两点距离
	double dlat = lat2 * D2R - lat1 * D2R;
	double dlon = lon2 * D2R - lon1 * D2R;
	double a = 0.0, c = 0.0;

	lat1 = lat1 * D2R;
	lat2 = lat2 * D2R;

	a = sin(dlat / 2) * sin(dlat / 2) +
		sin(dlon / 2) * sin(dlon / 2) * cos(lat1) * cos(lat2);

	if (a < 0 || a>1)  //避免开方运算溢出
		return 0;

	c = 2 * atan2(sqrt(a), sqrt(1 - a));

	return RE_WGS84 * c;
}

void pos2enu(double* af_pos, double* bf_pos, double* enu) {
	double rr[3] = { 0 }, rr2[3] = { 0 }, dr[3] = { 0 };
	double pos[3] = { 0 }, pos2[3] = { 0 };

	pos[0] = af_pos[0] * D2R;
	pos[1] = af_pos[1] * D2R;
	pos[2] = af_pos[2];

	pos2[0] = bf_pos[0] * D2R;
	pos2[1] = bf_pos[1] * D2R;
	pos2[2] = bf_pos[2];

	pos2ecef(pos, rr);
	pos2ecef(pos2, rr2);

	for (int i = 0; i < 3; i++)
		dr[i] = rr[i] - rr2[i];
	ecef2enu(pos2, dr, enu);
}

extern double get_gga_distance(nmea_gga_t* solgga, nmea_gga_t lastfiltergga, double* angle)
{
	double dis = 0.0;
	double lat1[3] = { 0.0 }, lat2[3] = { 0.0 };
	double enu[3] = { 0.0 };

	lat1[0] = solgga->lat;
	lat1[1] = solgga->lon;
	lat2[0] = lastfiltergga.lat;
	lat2[1] = lastfiltergga.lon;

	pos2enu(lat1, lat2, enu);

	*angle = atan2(enu[0], enu[1]) * R2D;  //斜率转换成度数
	if (*angle < 0)
		*angle += 360;   //范围由-180~180 转换成0~360

	dis = get_distance(lat1[0], lat1[1], lat2[0], lat2[1]);

	return dis;
}

void drift_adjust(double lat, double lnt, double v, double dir, double* lat_new, double* lon_new)  //跳点处理
{
	double lat1[3] = { 0.0 };
	double v_enu[3] = { 0.0 };
	double rr[3] = { 0.0 };
	double dr[3] = { 0.0 };
	//度转为弧度
	lat1[0] = lat * D2R;
	lat1[1] = lnt * D2R;

	trace(4, "before: lat1 %16.9f %16.9f\n", lat1[0] * R2D, lat1[1] * R2D);

	//经纬度转成xyz(暂忽略高程的影响)
	pos2ecef(lat1, rr);

	//航向角 度数转为弧度
	dir = dir * D2R;

	//用速度外推,
	v_enu[0] = v * sin(dir);
	v_enu[1] = v * cos(dir);
	v_enu[2] = 0;

	enu2ecef(lat1, v_enu, dr);

	rr[0] += dr[0];
	rr[1] += dr[1];
	rr[2] += dr[2];

	//xyz转为经纬度
	ecef2pos(rr, lat1);

	trace(4, "after: lat1 %16.9f %16.9f\n", lat1[0] * R2D, lat1[1] * R2D);

	*lat_new = lat1[0] * R2D;  //弧度转为度
	*lon_new = lat1[1] * R2D;
}

//针对于gga字符串形式进行滤波
void filter_location(nmea_gga_t* solgga, nmea_rmc_t* solrmc)
{
	double location_time_delta = 0.0;
	double distance = 0.0;
	double referVel = 0.0;
	double vel = 0.0;
	double angle = 0.0;  
	double lat, lon;
	double ddir = 0.0;
	double ddir_now = 0.0;
	double ddir_last = 0.0;
	double dr = 0.0;
	int vflag = 1; 

	solgga->vel = solrmc->vel * 3.6 / 1.852;
	solgga->dir = solrmc->dir;

	if (!extract_gga(solgga))
	{
		if (solgga->solq)   //连续几个历元的浮点解或者固定解传递至下一个历元作为滤波初值
		{
			memcpy(&ggafilter.last_fitergga, solgga, sizeof(nmea_gga_t));
			ggafilter.lastggaTime = solgga->time;
			filterstartnum++;
		}
		trace(5, "don't need filter and out rtk gga\n");
		return;
	}

	location_time_delta = solgga->time - ggafilter.lastggaTime;

	if (fabs(location_time_delta) < 1e-6) 
		return;

	if (location_time_delta < 1e-6) {
		location_time_delta += 86400;
	}

	if (location_time_delta > ggafilter.no_location_resettime) 
	{
		ggafilter.fixtimes = 0;
		memset(&ggafilter.last_fitergga, 0, sizeof(nmea_gga_t));
		//trace(3, "filter_location: too long.reset last_fitergga\n");
		filterstartnum = 0;
	}

	ggafilter.lastggaTime = solgga->time;
	//上个历元的GGA有效
	if (filterstartnum > 2 && !(ggafilter.last_fitergga.lat == 0.0 && ggafilter.last_fitergga.lon == 0.0 && ggafilter.last_fitergga.msl == 0.0))
	{
		distance = get_gga_distance(solgga, ggafilter.last_fitergga, &angle);

		if (fabs(location_time_delta) > 1e-6)
			referVel = distance / location_time_delta;

		vel = ggafilter.last_fitergga.vel;

		trace(4, "filter:refvel=%6.3f rmcvel=%6.3f\n", referVel, vel);
		if (vel > 2 * referVel)  //
		{
			vflag = 0;
		}

		if (referVel < vel) {
			referVel = vel;
		}
		if (vel < 0.001) {
			vel = referVel;
			//referVel=1;
		}
		// trace(4,"the change of run direction:last_dir=%6.3f dir=%6.3f angle=%6.3f v=%6.3f\n",last_dir,solrmc->dir,angle,solgga->vel);
		if (fabs(ggafilter.last_dir - solrmc->dir) > 180)
			ddir = 360 - fabs(ggafilter.last_dir - ggafilter.last_dir2);
		else
			ddir = fabs(ggafilter.last_dir - ggafilter.last_dir2);

		if (fabs(angle - solrmc->dir) > 180)
			ddir_now = 360 - fabs(angle - ggafilter.last_dir2);
		else
			ddir_now = fabs(angle - ggafilter.last_dir2);

		if (fabs(angle - ggafilter.last_dir) > 180)
			ddir_last = 360 - fabs(angle - ggafilter.last_dir);
		else
			ddir_last = fabs(angle - ggafilter.last_dir);

		trace(3, "run direction:last_dir=%6.1f dir=%6.1f angle=%6.1f v=%6.1f\n", ggafilter.last_dir, solrmc->dir, angle, solgga->vel);
		trace(4, "the change of run direction:ddir_now=%6.1f ddir_last=%6.1f ddir=%6.1f\n", ddir_now, ddir_last, ddir);
		if (ddir < 0.01)
			vflag = 0;

		if (vflag && ddir_now > 30 && ddir_last > 30 && ddir < 30)  
		{
			dr = ggafilter.last_fitergga.vel * 0.5144 * 0.5144 * location_time_delta; 
			drift_adjust(ggafilter.last_fitergga.lat, ggafilter.last_fitergga.lon, dr, ggafilter.last_dir, &lat, &lon);
			trace(4, "fiter info 1:lat=%16.9f lon=%16.9f ggafilterlan=%16.9f ggafilterlon=%16.9f\n", lat, lon, ggafilter.last_fitergga.lat, ggafilter.last_fitergga.lon);

			if ((ggafilter.last_fitergga.solq == 4 || ggafilter.last_fitergga.solq == 5) && (solgga->solq == 1 || solgga->solq == 2))
				ggafilter.pftimes = 1;
			if (solgga->solq == 4 || solgga->solq == 5)
			{
				if (ggafilter.pftimes > 1) 
				{
					lat = lat + (solgga->lat - lat) * 0.5;  //
					lon = lon + (solgga->lon - lon) * 0.5;  //
					ggafilter.pftimes--;
					if (ggafilter.pftimes < 0)
						ggafilter.pftimes = 0;
				}
				else
				{
					lat = lat + (solgga->lat - lat) * (ggafilter.fixtimes / ggafilter.fix_location_buffersize) * (30 / ddir_last);  //
					lon = lon + (solgga->lon - lon) * (ggafilter.fixtimes / ggafilter.fix_location_buffersize) * (30 / ddir_last);  //
					ggafilter.pftimes = 0;
				}
				trace(4, "float or fix:ftime=%4d\n", ggafilter.fixtimes);
			}
			if (ggafilter.pftimes)   //浮点或固定切换成rtd或单点时，后连续五个单点或者rtd进行平滑处理，自身解逐步加权
			{
				lat = lat + (solgga->lat - lat) * (ggafilter.pftimes) / 10 * (30 / ddir_last);
				lon = lon + (solgga->lon - lon) * (ggafilter.pftimes) / 10 * (30 / ddir_last);
				ggafilter.pftimes++;
				if (ggafilter.pftimes > 5)
					ggafilter.pftimes = 0;
			}
			else {
				ggafilter.pftimes = 0;
			}
		}
		else {
			lat = ggafilter.last_fitergga.lat + (solgga->lat - ggafilter.last_fitergga.lat) * vel / referVel;
			lon = ggafilter.last_fitergga.lon + (solgga->lon - ggafilter.last_fitergga.lon) * vel / referVel;
		}

		solgga->lat = lat;
		solgga->lon = lon;
	}
	else
	{
		memcpy(&ggafilter.last_fitergga, solgga, sizeof(nmea_gga_t));
	}

	filterstartnum++;
	predic_lon(ggafilter.last_fitergga.lon, &solgga->lon, solgga->vel);
	predic_lat(ggafilter.last_fitergga.lat, &solgga->lat, solgga->vel);

	ggafilter.last_dir2 = ggafilter.last_dir;
	ggafilter.last_dir = solrmc->dir;
	memcpy(&ggafilter.last_fitergga, solgga, sizeof(nmea_gga_t));
	ggafilter.lastggaTime = solgga->time;
}

int  gga_filter(unsigned char* ggabuff, unsigned char* rmcbuff, unsigned char* resultbuff_gga, unsigned char* resultbuff_rmc)
{
	nmea_gga_t sol_gga = { 0 };
	nmea_rmc_t sol_rmc = { 0 };

	char gga[BUFLENGTH] = "";
	char rmc[BUFLENGTH] = "";
	strcpy(gga, (char*)ggabuff);
	strcpy(rmc, (char*)rmcbuff);

	int n = 0;

	if (!decode_gga(gga, &sol_gga))
		return 0;
	if (!decode_rmc(rmc, &sol_rmc))
		return 0;

	filter_location(&sol_gga, &sol_rmc);

	n = outggas(resultbuff_gga, sol_gga);
	outrmcs(resultbuff_rmc, sol_rmc, sol_gga);
	return 1;
}