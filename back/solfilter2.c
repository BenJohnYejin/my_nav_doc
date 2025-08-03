#include "rtkport.h"
#include "rtklib.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

#define GGAWINDOWSIZES  10
#define STEPS           3

static double g_f_p[9] = { 0 };
static double g_f_x[3] = { 0 };

typedef struct {
	double vel;         //rmc中速度
	double dis_v;       //原始前后位置差计算的速度
	double dis_v_f;     //滤波后前后位置差计算的速度

	double acc;         //前后速度计算的加速度
	double dis_acc;     //原始前后位置差计算的加速度
	double dis_acc_f;   //滤波后前后位置差计算的加速度

	double dir;         //rmc中的航向
	double dis_dir;     //原始前后位置差计算的航向
	double dis_dir_f;   //滤波后前后位置差计算的航向

	double dir_v;       //rmc中的航向前后变化
	double dis_dir_v;   //原始前后位置差计算的航向变化
	double dis_dir_f_v; //滤波后前后位置差计算的航向

	double dir_ref;     //参考航向
	int dir_ref_flag;   //参考航向获取成功标识，0表示未能成功获取，1表示成功获取
	int driftnum;
}check_info_struct;

typedef struct {
	int fixtimes;
	int pftimes;
	int fix_location_buffersize;

	int no_location_resettime;

	double firsttime;
	double ftime;
	nmea_gga_t last_fitergga;
	nmea_gga_t fitergga[GGAWINDOWSIZES];
	double last_dir;

	double dis_v;
	double dis_ref_angle;
	double dis_ref_dif;

	int outlier_flag;

	int driftnum;

	check_info_struct check_info;
}filter_struct;

static filter_struct ggafilter;
static long filterstartnum = 0;    //计数器，初始化后第一个点不作为滤波基准

//根据外部输入的速度rmc进行滤波
extern void fiter_init2()
{
	memset(&ggafilter, 0, sizeof(filter_struct));
	ggafilter.fixtimes = 0;
	ggafilter.pftimes = 0;
	ggafilter.fix_location_buffersize = 5;
	ggafilter.no_location_resettime = 5;     // 5秒无收到定位，重置定位缓存数组

	memset(&ggafilter.last_fitergga, 0, sizeof(nmea_gga_t));
}

//对定位结果进行判断  注意sol.pos中的经纬度格式，原滤波算法中是直接采用了GGA中度分格式
int extract_gga2(nmea_gga_t* sol)
{
	if (ggafilter.outlier_flag)
	{
		ggafilter.fixtimes = 0;
		return 1;
	}

	if (sol->solq == 4 || sol->solq == 5) {// 对固定解和浮点解不过滤 连续5个历元 不滤波
		ggafilter.fixtimes++;

		if (ggafilter.fixtimes > MAXINT)  //异常处理，避免一直累加导致数值越界
			ggafilter.fixtimes = ggafilter.fix_location_buffersize + 1;

		if (ggafilter.fixtimes > ggafilter.fix_location_buffersize) {
			return 0;
		}
		if ((sol->vel < 0.5 || ggafilter.check_info.dis_v_f < 0.5) && sol->solq == 4)  //固定解低速下不过滤
			return 0;
	}
	else {
		ggafilter.fixtimes = 0;
	}
	return 1;
}

//大跳点检测
int pos_outlier_check(nmea_gga_t* solgga, double distance)
{
	double dt = 0.0;
	double dis_v = 0.0;
	double dis_acc = 0.0;
	dt = solgga->time - ggafilter.last_fitergga.time;

	if (dt <= 0.0)
		return 0;

	dis_v = distance / dt;
	dis_acc = (dis_v - ggafilter.dis_v) / dt;

	trace(4, "pos outlier check:dis_v=%f dis_acc=%f\n", dis_v, dis_acc);

	ggafilter.dis_v = dis_v;

	if (solgga->solq == 4 || solgga->solq == 5)  //浮点解或者固定解不做跳点检测
		return 0;

#if 0  //暂不支持高速运行下的单点或者rtd轨迹，此模块在此场景在经过α-β之后容易引起误判
	if (solgga->vel > 12)  //避免vrs断开时高速运行时对单点或者rtd采取了α—β滤波导致跳点误判
		return 0;

	if (dis_v > 50 || (dis_acc < -6 || dis_acc>3))  //前后位置差大于50m/s直接视为跳点 //加速度区间（-6，3），超出此区间视为跳点
	{
		return 1;
	}
#endif

	return 0;
}

void get_check_info(nmea_gga_t* solgga)
{
	double distance = 0.0;
	double angle = 0.0;
	double dt = 0.0;
	double dir_temp = 0.0;

	dt = solgga->time - ggafilter.last_fitergga.time;  //注意跨天
	if (dt < -80000)
		dt += 86400;

	distance = get_gga_distance(solgga, ggafilter.last_fitergga, &angle);

	solgga->angle = angle;
	solgga->dis_v_f = distance / dt;

	ggafilter.check_info.vel = solgga->vel;
	ggafilter.check_info.dir = solgga->dir;

	ggafilter.check_info.acc = (solgga->vel - ggafilter.last_fitergga.vel) / dt;

	dir_temp = solgga->dir - ggafilter.last_fitergga.dir;
	if (fabs(dir_temp) > 180)  //夹角之间取锐角
		dir_temp = (dir_temp > 0 ? 1 : -1) * (360 - fabs(dir_temp));
	ggafilter.check_info.dir_v = dir_temp / dt;

	//滤波后前后位置算的航向
	ggafilter.check_info.dis_v_f = distance / dt;
	ggafilter.check_info.dis_dir_f = angle;

	ggafilter.check_info.dis_acc_f = (ggafilter.check_info.dis_v_f - ggafilter.last_fitergga.dis_v_f) / dt;

	//计算航向变化的一阶项
	dir_temp = angle - ggafilter.last_fitergga.angle;
	if (fabs(dir_temp) > 180)  //夹角之间取锐角
		dir_temp = (dir_temp > 0 ? 1 : -1) * (360 - fabs(dir_temp));
	ggafilter.check_info.dis_dir_f_v = dir_temp / dt;

	trace(5, "dir check info:vel=%8.2f acc=%8.2f dis_v_f=%8.2f dis_acc_f=%8.2f\n", ggafilter.check_info.vel, ggafilter.check_info.acc, ggafilter.check_info.dis_v_f, ggafilter.check_info.dis_acc_f);
	trace(5, "dir check info:dir=%8.2f dir_v=%8.2f dis_dir_f=%8.2f dis_dir_f_v=%8.2f\n", ggafilter.check_info.dir, ggafilter.check_info.dir_v, ggafilter.check_info.dis_dir_f, ggafilter.check_info.dis_dir_f_v);
}

//获取参考航向
void get_ref_dir()
{
	//连续多个航向变化小于15/30°视为参考航向，以rmc中的航向对位置计算的航向进行检测
	ggafilter.dis_ref_angle = ggafilter.last_fitergga.dir;//ggafilter.check_info.dir;
}

void outlier_check(nmea_gga_t* solgga)
{
	double dir_temp = 0.0;
	ggafilter.outlier_flag = 0;

	if ((ggafilter.check_info.vel < 0.5 || ggafilter.check_info.dis_v_f < 0.5) && solgga->solq == 4)  //低速固定解不进行异常值判断,增加位置速度判断（避免静态定速误差过大导致误判）
		return;

	dir_temp = fabs(ggafilter.check_info.dis_dir_f - ggafilter.dis_ref_angle);  //必须是与参考航向的差值

	if (fabs(dir_temp) > 180)  //夹角之间取锐角
		dir_temp = (360 - fabs(dir_temp));

	trace(5, "the dir change:%6.2f\n", dir_temp);

	ggafilter.dis_ref_dif = dir_temp;
	if (dir_temp >= 90)        //轨迹回退
		ggafilter.outlier_flag = 3;
	else if (dir_temp >= 30 && dir_temp < 90)   //疑似航向突变
		ggafilter.outlier_flag = 2;
# if 0
	else if (dir_temp >= 15 && dir_temp < 30)   //疑似拐弯，抑或者轨迹不平滑，车载方向盘自由行程15°
		ggafilter.outlier_flag = 1;
#endif
}

void outlier_deal(nmea_gga_t* solgga, int staticflag)
{
	double dis = 0.0;
	double dt = 0.0;
	double lat = 0.0, lon = 0.0;
	double pos[3] = { 0 };

	if (!ggafilter.outlier_flag || ggafilter.outlier_flag == 1)
	{
		ggafilter.driftnum = 0;
		return;
	}

	dt = solgga->time - ggafilter.last_fitergga.time;

	if (ggafilter.outlier_flag == 3)
	{
		dis = 0.001;    //可以考虑用rmc里的速度进行替换，但需要考虑速度的误差，需考虑如何兼容缓行
		if (ggafilter.check_info.vel < 0.5)
			ggafilter.driftnum = 0;
	}
	else if (ggafilter.outlier_flag == 2)
		dis = ggafilter.check_info.dis_v_f * dt * cos(ggafilter.dis_ref_dif * D2R);

	trace(4, "outlier_deal:dis=%6.2f dt=%f\n", dis, dt);
	drift_adjust(ggafilter.last_fitergga.lat, ggafilter.last_fitergga.lon, dis, ggafilter.dis_ref_angle, &lat, &lon);

	if (staticflag)
		ggafilter.driftnum = 0;

	solgga->lat = lat + (solgga->lat - lat) * ggafilter.driftnum / 10;
	solgga->lon = lon + (solgga->lon - lon) * ggafilter.driftnum / 10;
	trace(5, "filter dirftnum:%d\n", ggafilter.driftnum);

	//更新rr
	pos[0] = solgga->lat * D2R;
	pos[1] = solgga->lon * D2R;
	pos[2] = solgga->alt;

	pos2ecef(pos, solgga->rr);

	if (ggafilter.driftnum < 10)
		ggafilter.driftnum++;
}

double xy_xn(double a, int n)
{
	int i = 0;
	double b = 1.0;
	for (i = 0; i < n; i++)
		b *= a;
	return b;
}

void polyomial(nmea_gga_t* solgga)
{
	int i = 0, j = 0;
	double dt = 0;
	int info = 0;

	double x = 0.0, y = 0, z = 0;
	static int firsttimeflag = 0;

	double aa[GGAWINDOWSIZES * STEPS] = { 0 };
	double faa[STEPS * STEPS] = { 0 };
	double faa2[STEPS * STEPS] = { 0 };

	double tt_x[GGAWINDOWSIZES] = { 0 };
	double tt_y[GGAWINDOWSIZES] = { 0 };
	double tt_z[GGAWINDOWSIZES] = { 0 };

	double bb_x[STEPS] = { 0 };
	double bb_y[STEPS] = { 0 };
	double bb_z[STEPS] = { 0 };
	double bb_tx[STEPS] = { 0 };
	double bb_ty[STEPS] = { 0 };
	double bb_tz[STEPS] = { 0 };

	double pos[3] = { 0 };

	pos[0] = solgga->lat * D2R;
	pos[1] = solgga->lon * D2R;
	pos[2] = solgga->alt;

	pos2ecef(pos, solgga->rr);

	if (!firsttimeflag)
	{
		ggafilter.firsttime = solgga->time;
		firsttimeflag = 1;
	}

	if (ggafilter.outlier_flag == 3)
		return;

	if (filterstartnum < GGAWINDOWSIZES)
	{
		memcpy(&ggafilter.fitergga[filterstartnum], solgga, sizeof(nmea_gga_t));
		return;
	}

	for (i = 0; i < GGAWINDOWSIZES - 1; i++)
		memcpy(&ggafilter.fitergga[i], &ggafilter.fitergga[i + 1], sizeof(nmea_gga_t));
	memcpy(&ggafilter.fitergga[GGAWINDOWSIZES - 1], solgga, sizeof(nmea_gga_t));

	if (!extract_gga2(solgga))
	{
		trace(4, "no need polyomial\r\n");
		return;
	}

	for (i = 0; i < GGAWINDOWSIZES; i++)
	{
		dt = ggafilter.fitergga[i].time - ggafilter.fitergga[0].time;//ggafilter.firsttime;

		for (j = 0; j < STEPS; j++)
		{
			aa[j + i * STEPS] = xy_xn(dt, STEPS - 1 - j);
		}

		tt_x[i] = ggafilter.fitergga[i].rr[0];
		tt_y[i] = ggafilter.fitergga[i].rr[1];
		tt_z[i] = ggafilter.fitergga[i].rr[2];
	}

	//x=((A'A)^-1)A'AT
	matmul("NT", STEPS, STEPS, GGAWINDOWSIZES, 1.0, aa, aa, 0.0, faa);  // A'A

	matmul("NN", STEPS, 1, GGAWINDOWSIZES, 1.0, aa, tt_x, 0.0, bb_tx);
	matmul("NN", STEPS, 1, GGAWINDOWSIZES, 1.0, aa, tt_y, 0.0, bb_ty);
	matmul("NN", STEPS, 1, GGAWINDOWSIZES, 1.0, aa, tt_z, 0.0, bb_tz);

	matcpy(faa2, faa, STEPS, STEPS);

	if (!(info = matinv(faa2, STEPS)))
	{
		matmul("NN", STEPS, 1, STEPS, 1.0, faa2, bb_tx, 0.0, bb_x);  //(A'A)^-1*A'
		matmul("NN", STEPS, 1, STEPS, 1.0, faa2, bb_ty, 0.0, bb_y);  //(A'A)^-1*A'
		matmul("NN", STEPS, 1, STEPS, 1.0, faa2, bb_tz, 0.0, bb_z);  //(A'A)^-1*A'
	}
	else {
		trace(4, "polyomial failed\r\n");
		return;
	}

	dt = solgga->time - ggafilter.fitergga[0].time;//ggafilter.firsttime;

	for (i = 0; i < STEPS; i++)
	{
		x += bb_x[i] * xy_xn(dt, STEPS - 1 - i);
		y += bb_y[i] * xy_xn(dt, STEPS - 1 - i);
		z += bb_z[i] * xy_xn(dt, STEPS - 1 - i);
	}

	trace(4, "after filter xyz:%f %f %f  x:%f y:%f z:%f\n", solgga->rr[0], solgga->rr[1], solgga->rr[2], x, y, z);

	double difx[3] = { 0 };
	difx[0] = solgga->rr[0] - x;
	difx[1] = solgga->rr[1] - y;
	difx[2] = solgga->rr[2] - z;

	trace(4, "difx:%f %f %f %f\n", difx[0], difx[1], difx[2], norm(difx, 3));

	if (norm(difx, 3) < 5)
	{
		solgga->rr[0] = x;
		solgga->rr[1] = y;
		solgga->rr[2] = z;

		ecef2pos(solgga->rr, pos);

		solgga->lat = pos[0] * R2D;
		solgga->lon = pos[1] * R2D;
		solgga->alt = pos[2];
	}

	//memcpy(&ggafilter.fitergga[GGAWINDOWSIZES-1], solgga, sizeof(nmea_gga_t));//用拟合后的值更新滑窗拐弯处容易失真
}

void gga_kalman_init(double* rr)
{
	int i = 0;
	for (i = 0; i < 3; i++)
		g_f_x[i] = rr[i];
	for (i = 0; i < 9; i++)
		g_f_p[i] = 0.0;
	g_f_p[0] = g_f_p[4] = g_f_p[8] = 300;
}

void update_q(double en_var, double u_var)
{
	double q[9] = { 0.0 }, qv[9] = { 0.0 }, pos[3] = { 0.0 };
	q[0] = q[4] = SQR(en_var);
	q[8] = SQR(u_var);

	ecef2pos(g_f_x, pos);
	covecef(pos, q, qv);

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			g_f_p[i + j * 3] += qv[i + j * 3];
		}
	}
}
void updatestate(nmea_gga_t* solgga, double* xyz_v)
{
	int i = 0;
	double dt = 0.0;

	if (ggafilter.ftime)
		dt = solgga->time - ggafilter.ftime;
	if (dt > 5 || norm(g_f_x, 3) < ZEROS_MIN)
	{
		gga_kalman_init(solgga->rr);
		ggafilter.ftime = solgga->time;
		return;
	}
	for (i = 0; i < 3; i++)
	{
		g_f_x[i] += xyz_v[i] * dt;
	}

	//如何加上噪声
	update_q(1.0 * dt, 1.0 * dt);
}

void gga_kalman_fiter(nmea_gga_t* solgga, double* enuv)
{
	int i = 0;
	int info = 0;
	double xyz_v[3] = { 0 }, pos[3] = { 0 };
	double h[9] = { 1.0 };
	double v[3] = { 0 };
	double rvar[9] = { 0 };
	double dif[3] = { 0 };

	double xp[3] = { 0 }, pp[9] = { 0 };

	ecef2pos(g_f_x, pos);

	//enuv转存xyzv
	enu2ecef(pos, enuv, xyz_v);

	trace(5, "enuv:%f %f %f  xyzv:%f %f %f\n", enuv[0], enuv[1], enuv[2], xyz_v[0], xyz_v[1], xyz_v[2]);

	//状态更新
	updatestate(solgga, xyz_v);

	matcpy(xp, g_f_x, 3, 1);
	matcpy(pp, g_f_p, 3, 3);
	//测量更新
	h[0] = h[4] = h[8] = 1.0;

	for (i = 0; i < 3; i++)
		v[i] = solgga->rr[i] - g_f_x[i];

	if (ggafilter.outlier_flag == 3) {
		rvar[0] = rvar[4] = rvar[8] = 1;
	}
	else if (ggafilter.outlier_flag == 2) {
		rvar[0] = rvar[4] = rvar[8] = 0.5;
	}
	else {
		if (solgga->solq == 1)
			rvar[0] = rvar[4] = rvar[8] = 10.0;
		if (solgga->solq == 2)
			rvar[0] = rvar[4] = rvar[8] = 5.0;
		if (solgga->solq == 4)
			rvar[0] = rvar[4] = rvar[8] = 0.01;
		if (solgga->solq == 5)
			rvar[0] = rvar[4] = rvar[8] = 0.1;
	}

	if ((info = filter_leador(xp, pp, h, v, rvar, 3, 3, 5, 10)))//(5,10)
	{
		trace(4, "filter error (info=%d)\r\n", info);
	}

	trace(5, "fiter info: %f %f %f  rr %f %f %f\n", solgga->rr[0], solgga->rr[1], solgga->rr[2], xyz_v[0], xyz_v[1], xyz_v[2]);
	trace(5, "fiter info2: %f %f %f\n", xp[0], xp[1], xp[2]);
	trace(5, "fiter info3: %f %f %f\n", g_f_x[0], g_f_x[1], g_f_x[2]);

	matcpy(g_f_x, xp, 3, 1);
	matcpy(g_f_p, pp, 3, 3);

	for (i = 0; i < 3; i++)
		dif[i] = xp[i] - solgga->rr[i];

	trace(5, "fiter dif info:%f %f %f %f\n", dif[0], dif[1], dif[2], norm(dif, 3));
	if (norm(dif, 3) > 5)
		return;

	ggafilter.ftime = solgga->time;

	if (!extract_gga2(solgga))
	{
		trace(4, "no need polyomial\r\n");
		return;
	}

	solgga->rr[0] = xp[0];
	solgga->rr[1] = xp[1];
	solgga->rr[2] = xp[2];

	ecef2pos(solgga->rr, pos);

	solgga->lat = pos[0] * R2D;
	solgga->lon = pos[1] * R2D;
	solgga->alt = pos[2];
}

//针对于gga字符串形式进行滤波
void filter_location2(nmea_gga_t* solgga, nmea_rmc_t* solrmc)
{
	double locationTimeDelta = 0.0;

	solgga->vel = solrmc->vel * 0.5144;
	solgga->dir = solrmc->dir;

	if (!solgga->solq)   //连续几个历元的浮点解或者固定解传递至下一个历元作为滤波初值
	{
		return;
	}

	if (filterstartnum > 0)
	{
		locationTimeDelta = solgga->time - ggafilter.last_fitergga.time;

		if (fabs(locationTimeDelta) < ZEROS_MIN)  //避免输入相同历元
			return;

		if (locationTimeDelta < 1e-6) {// 隔天处理,改成前向滤波，此处为负值标识跨天
			locationTimeDelta += 86400;
		}
	}

	if (locationTimeDelta > ggafilter.no_location_resettime) // 超时未收到定位，重置定位缓存数组
	{
		ggafilter.fixtimes = 0;
		memset(&ggafilter.last_fitergga, 0, sizeof(nmea_gga_t));
		filterstartnum = 0;
	}
	if (filterstartnum >= 2)
	{
		get_check_info(solgga); //获取待检测信息

		get_ref_dir();   //获取参考航向,主要以rmc里的航向为主

		outlier_check(solgga); //基于航向进行异常值检测

		outlier_deal(solgga, 0); //异常值处理
	}

	polyomial(solgga);    //多项式拟合

	filterstartnum++;
	if (filterstartnum > MAXINT)  //异常处理
		filterstartnum = GGAWINDOWSIZES + 1;

	memcpy(&ggafilter.last_fitergga, solgga, sizeof(nmea_gga_t));
}

//针对于gga字符串形式进行滤波
void filter_location3(nmea_gga_t* solgga, nmea_rmc_t* solrmc, double* enuv)
{
	double locationTimeDelta = 0.0;
	int staticflag = 0;

	if (norm(enuv, 3) < ZEROS_MIN)
		staticflag = 1;

	solgga->vel = solrmc->vel * 0.5144;
	solgga->dir = solrmc->dir;

	if (!solgga->solq)   //连续几个历元的浮点解或者固定解传递至下一个历元作为滤波初值
	{
		return;
	}

	if (filterstartnum > 0)
	{
		locationTimeDelta = solgga->time - ggafilter.last_fitergga.time;

		if (fabs(locationTimeDelta) < ZEROS_MIN)  //避免输入相同历元
			return;

		if (locationTimeDelta < 1e-6) {// 隔天处理,改成前向滤波，此处为负值标识跨天
			locationTimeDelta += 86400;
		}
	}

	if (locationTimeDelta > ggafilter.no_location_resettime) // 超时未收到定位，重置定位缓存数组
	{
		ggafilter.fixtimes = 0;
		memset(&ggafilter.last_fitergga, 0, sizeof(nmea_gga_t));
		filterstartnum = 0;
	}
	if (filterstartnum >= 2)
	{
		get_check_info(solgga); //获取待检测信息

		get_ref_dir();   //获取参考航向,主要以rmc里的航向为主

		outlier_check(solgga); //基于航向进行异常值检测

		outlier_deal(solgga, staticflag); //异常值处理

		gga_kalman_fiter(solgga, enuv);
	}

	filterstartnum++;
	if (filterstartnum > MAXINT)  //异常处理
		filterstartnum = GGAWINDOWSIZES + 1;

	memcpy(&ggafilter.last_fitergga, solgga, sizeof(nmea_gga_t));
}

int  gga_filter2(unsigned char* ggabuff, unsigned char* rmcbuff, unsigned char* resultbuff_gga, unsigned char* resultbuff_rmc)
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

	filter_location2(&sol_gga, &sol_rmc);

	n = outggas(resultbuff_gga, sol_gga);
	outrmcs(resultbuff_rmc, sol_rmc, sol_gga);
	return 1;
}

int  gga_filter3(unsigned char* ggabuff, unsigned char* rmcbuff, unsigned char* resultbuff_gga, unsigned char* resultbuff_rmc, double* enuv)
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

	filter_location3(&sol_gga, &sol_rmc, enuv);

	n = outggas(resultbuff_gga, sol_gga);
	outrmcs(resultbuff_rmc, sol_rmc, sol_gga);
	return 1;
}