#include "rtkport.h"
#include "rtklib.h"

int    g_rtcm_week = 0;
double g_gps_toe = 0.0;
int open_filter = 1;              
int open_replace = 1;              
int satnum_max = 0;                   
static gtime_t gnss_obstime = { 0 };

static nav_t gnssnavs = { 0 };          /* navigation data */
static obs_t obsr = { 0 };          /* observation data */
static double ru[3] = { 0 };          //rov coordinate
static double rb[3] = { 0 };          //base coordinate
static double rbt[3] = { 0 };         //temp base coordinate
static gtime_t lastime = { 0,0.0 };       //time of last rtcm message
static rtk_t rtk = { 0 };
static prelock_t prelock[MAXRECOBS * 3 * NFREQ] = { 0 };
XYRTK_Callbacks* cb;

static staidinfo vrsstaidinfo = { 0 };
static staidinfo obsstaidinfo = { 0 };

//prcopt_t prcopt_rtk = {      /* defaults processing options */
prcopt_t popt = {      /* defaults processing options */
				 PMODE_KINEMA,0,2,SYS_GPS | SYS_CMP | SYS_GAL,                          /* mode,soltype,nf,navsys */
				 15.0 * D2R,1,32,    /* elmin,snr on/off,snrthres */                   //1
				 1,3,0,1,                         /* sateph,modear,glomodear,bdsmodear */                //1
				 1,3,10,                          /* maxout,minlock,minfix */
				 1,1,1,0,                         /* estion,esttrop,dynamics,tidecorr */                        //1
				 1,                               /* niter */     //1
				 { 180.0 },                 /* eratio[] */                                                        //1
				 { 180.0,0.006,0.004,0.03,1.0 },  /* err[] */
				 { 30.0,0.03,0.3 },               /* std[] */
				 { 1E-4,1E-3,1E-4,1E-1,1E-2 },    /* prn[] */
				 5E-12,                           /* sclkstab */
				 { 1.5,0.9999,0.20 },             /* thresar */
				 10.0 * D2R,0.0,0.05,            /* elmaskar,almaskhold,thresslip */
				 10,3000.0,20.0,                 /* maxtdif,maxinno,maxgdop */
				 {0},                           //rb
				 { 0 }, 60,30 ,0,              // snravg maxtdiffrtd maxtdiffrtk ratiomodel
				 {0},{0}
};

#if 0
const solopt_t solopt_rtk = {   /* defaults solution output options */
	SOLF_NMEA,TIMES_GPST,1,3,    /* posf,times,timef,timeu */
	0,0,0,0,0,0,0,                /* degf,outhead,outopt,outvel,datum,height,geoid */
	0,0,5,                      /* solstatic,sstat,trace */
	{ 0.0,0.0 },                /* nmeaintv */
	" ",""                      /* separator/program name */
};
#endif

void rtk_version(char ver[64])
{
	sprintf(ver, "%s", "double_fre_lg69t_v2.4.0_906a_599b7fbd");  //此处记录的是当前提交前的版本信息 2023.0601 修复没有改算法
}

extern void settracefile(char* trace_file)
{
#ifdef WIN32
	int trace = 0;
	if (trace > 0) {
		traceclose();
		traceopen(trace_file);
		tracelevel(trace);
	}
#endif
}
int rtk_init(optconf_in_t optconf_in, XYRTK_Callbacks* callbacks)
{
	if (callbacks == NULL) return 0;

	cb = callbacks;

	cb->printf_info("Leador LIB :%s\r\n", "0001");
	popt.nf = 2;//双频SDK锁定频段值为2
	popt.nf = optconf_in.fre_flag;

	//set min elev
	if ((fabs(optconf_in.elemin) > 0) && (fabs(optconf_in.elemin) < 180))  popt.elmin = optconf_in.elemin * D2R;

	//set snrthes
	if (fabs(optconf_in.snr_thres) > 0) popt.snrthres = optconf_in.snr_thres;
	if (abs(optconf_in.satnum_max) > 0) satnum_max = optconf_in.satnum_max;

	//避免异常值输入，只考虑输入值为0-1的情况
	if (optconf_in.open_filter == 0)
		open_filter = 0;
	else if (optconf_in.open_filter == 1)
		open_filter = 1;

	if (optconf_in.open_replace == 0)
		open_replace = 0;
	else if (optconf_in.open_replace == 1)
		open_replace = 1;

	trace(2, "filter info:open_filter=%d open_replace=%d\n", open_filter, open_replace);
	rtkinit(&rtk, &popt);
	fiter_init2();

	obsr.data = (obsd_t*)xy_malloc(sizeof(obsd_t) * MAXRECOBS * 2);
	gnssnavs.eph = (eph_t*)xy_malloc(sizeof(eph_t) * MAXEPHNUM);
	gnssnavs.nmax = MAXEPHNUM;

	return 1;
}

void rtk_free()
{
	rtkfree(&rtk);
	xy_free(obsr.data);
	xy_free(gnssnavs.eph);
}

void set_tracelevel(int level)
{
	tracelevel(level);
}

// 输出解
static void  gga_filter_save(nmea_t* nmea, nmea_t* tnmea, nmea_t* fnmea, int  gga_filter_flag)
{
	int i = 0;
	if (gga_filter_flag == 1)
	{
		memset(nmea->rmc, 0, BUFLENGTH);
		memcpy(nmea->rmc, fnmea->rmc, sizeof(fnmea->rmc));

		memset(nmea->gga, 0, BUFLENGTH);
		memcpy(nmea->gga, fnmea->gga, sizeof(fnmea->gga));
	}

	//没有开启滤波的时候 没有开启或者滤波失败
	if (gga_filter_flag == 2) {
		memset(nmea->rmc, 0, BUFLENGTH);
		memcpy(nmea->rmc, tnmea->rmc, sizeof(tnmea->rmc));

		memset(nmea->gga, 0, BUFLENGTH);
		memcpy(nmea->gga, tnmea->gga, sizeof(tnmea->gga));
	}
	//即使滤波仍继承滤波前的速度
	for (i = 0; i < 3; i++) nmea->enuv[i] = tnmea->enuv[i];
}

//补点失败或者不需要补点返回0； 补点成功返回1    与上层的约定，涉及是否给imu   gga
int updatePvt(unsigned char* rtcm1, int rtcmLength, nmea_t* nmea)
{
	int i = 0;
	nmea_t tnmea = { 0 };    //滤波前
	nmea_t fnmea = { 0 };    //滤波后

	trace(3, "updatePvt\r\n");

	int ret = rtcmdataread(rtcm1, rtcmLength, NULL, NULL, NULL, NULL, 0, &rtk, 0);

	if (ret == 9)   //解析成功
	{
		if (!open_replace)
			outlg69tpvt(rtk.lgpvt, &tnmea);
	}
	else
	{
		trace(1, "LG69T pvt info decode failed\r\n");
		return 0;
	}

	if (!open_replace)  //补点
	{
		trace(1, "cfg dont need add point\r\n");
		return 0;
	}

	//通过nmea.gga判断是否需要补点,读入空的gga时需要补点
	if (gga_judge_add(nmea->gga))
	{
		trace(4, "dont need add point\r\n");
		return 0;
	}

	//根据当前观测的tow时间重新生成gga
	if (fabs(timediff(gnss_obstime, rtk.lgpvt.time)) > 2)
	{
		trace(1, "the time diff of obs and pvt is too large:%s %s\r\n", time_str(gnss_obstime, 3), time_str(rtk.lgpvt.time, 3));
		return 0;
	}

	//替换成当前观测时间并生成gga
	trace(1, "updatePvt time info:%s %s\r\n", time_str(gnss_obstime, 3), time_str(rtk.lgpvt.time, 3));
	rtk.lgpvt.time = gnss_obstime;
	outlg69tpvt(rtk.lgpvt, &tnmea);

	if (open_filter)
	{
		if (gga_filter3(tnmea.gga, tnmea.rmc, fnmea.gga, fnmea.rmc, tnmea.enuv))//滤波成功输出滤波解 否则输出rtk的解
		{
			gga_filter_save(nmea, &tnmea, &fnmea, 1);//滤波成功
		}
		else {
			gga_filter_save(nmea, &tnmea, &fnmea, 2);//滤波失败
		}
	}
	else {
		gga_filter_save(nmea, &tnmea, &fnmea, 2);//不滤波
	}

	return 1;
}

int updateNav(unsigned char* rtcm1, int rtcmLength)
{
	int ret = 0;
	ret = rtcmdataread(rtcm1, rtcmLength, NULL, &gnssnavs, NULL, NULL, 0, &rtk, 0);
	trace(3, "nav ret:%d num:%2d week=%5d toe=%.3f\r\n", ret, gnssnavs.n, g_rtcm_week, g_gps_toe);
	uniqnav(&gnssnavs);

	return ret;
}
// 同步vrs设置
//重置vrs存在的系统标识，避免中间一直缺某个系统导致差分龄期以10s周期性的出现
void resetvrssys(const obs_t* obst)
{
	int i = 0, sys = 0;

	for (i = 0; i < 4; i++)
	{
		rtk.vrs_sys_flag[i] = 0;
	}

	for (i = 0; i < obst->n; i++)
	{
		sys = satsys(obst->data[i].sat, NULL);
		if (!seleph(obst->data[i].time, obst->data[i].sat, -1, &gnssnavs))  //新增星历判断
			continue;

		switch (sys) {
		case SYS_GPS: {rtk.vrs_sys_flag[0] = 1; break; }
		case SYS_GAL: {rtk.vrs_sys_flag[1] = 1; break; }
		case SYS_CMP: {rtk.vrs_sys_flag[3] = 1; break; }
		}
	}
}

void vrs_locksys_test(const obs_t* obst, int* up_vrs_flag)
{
	int i = 0;
	int sys = 0;
	int sysnum[4] = { 0 };
	int vrs_lock_flag = 0;

	if (!rtk.basechange && rtk.vrs_age <= 10)
	{
		for (i = 0; i < obst->n; i++)
		{
			sys = satsys(obst->data[i].sat, NULL);
			if (!seleph(obst->data[i].time, obst->data[i].sat, -1, &gnssnavs))  //新增星历判断：若观测数据没有对应的卫星星历
				continue;
			switch (sys) {
			case SYS_GPS: {rtk.vrs_sys_flag[0] = 1; sysnum[0]++; break; }
			case SYS_GAL: {rtk.vrs_sys_flag[1] = 1; sysnum[1]++; break; }
			case SYS_CMP: {rtk.vrs_sys_flag[3] = 1; sysnum[3]++; break; }
			}
		}
		if ((rtk.vrs_sys_flag[0] && sysnum[0] == 0) || (rtk.vrs_sys_flag[1] && sysnum[1] == 0) || (rtk.vrs_sys_flag[3] && sysnum[3] == 0))
		{
			vrs_lock_flag = 1;
			trace(1, "the vrs data is lock of sys,dont updata vrs\r\n");
		}

		if (!vrs_lock_flag)
		{
			*up_vrs_flag = 1;
		}
	}
	else {
		*up_vrs_flag = 1;
	}


	if ((*up_vrs_flag))
		resetvrssys(obst);
}
void check_vrs_station_change(int ret)
{
	double diffpos[3] = { 0 };
	if ((ret == 5 || ret == 6) && norm(rbt, 3) > 0.001) {
		diffpos[0] = rbt[0] - rb[0];
		diffpos[1] = rbt[1] - rb[1];
		diffpos[2] = rbt[2] - rb[2];
		if (norm(diffpos, 3) > 1.0 || vrsstaidinfo.staid != rtk.staid) {  
			vrsstaidinfo.staid = rtk.staid;
			rtk.vrs_staid = rtk.staid;
			//age_init(&rtk);
			rtk.opt.rb[0] = rbt[0];
			rtk.opt.rb[1] = rbt[1];
			rtk.opt.rb[2] = rbt[2];
			rb[0] = rbt[0];
			rb[1] = rbt[1];
			rb[2] = rbt[2];
			rtk.basechange = 1;
			trace(2, "base station update!new cor is %f %f %f\n", rbt[0], rbt[1], rbt[2]);
		}
		else {
			rtk.basechange = 0;
		}
	}
}

int updateVrs(unsigned char* rtcm1, int rtcmLength)
{
	int ret = 0;
	int i = 0;

	int up_vrs_flag = 0;

	obsd_t obs0 = { 0 };

	obs_t obst = { 0 };
	obst.data = (obsd_t*)xy_malloc(sizeof(obsd_t) * MAXRECOBS * 2);
	obst.n = 0;
	for (i = 0; i < MAXRECOBS * 2; i++)
		obst.data[i] = obs0;//obst基准站观测数据
	ret = rtcmdataread(rtcm1, rtcmLength, &obst, &gnssnavs, rbt, prelock + MAXRECOBS * NFREQ, 2, &rtk, vrsstaidinfo.staid);
	trace(1, "0vrs sat num:%2d\r\n", obst.n);
	sortobs(&obst);

	trace(1, "vrs sat num:%2d\r\n", obst.n);
	check_vrs_station_change(ret);
	vrs_locksys_test(&obst, &up_vrs_flag);

	if (obst.n > 4 && up_vrs_flag)
	{
		//* base position by single point positioning */
		for (i = 0; i < obst.n; i++)   obsr.data[i] = obst.data[i];
		obsr.n = obst.n;
		obsr.nmax = obst.nmax;
		rtk.up_vrs_flag = up_vrs_flag;

		trace(1, "1005     : x=%16.3f y=%16.3f z=%16.3f\r\n", rbt[0], rbt[1], rbt[2]);
	}
	else
	{
		trace(1, "the vrs decode failed\n");
	}

	if (obst.data)
	{
		xy_free(obst.data); obst.data = NULL; obst.n = obst.nmax = 0;
	}
	return ret;
}

//obs数据预处理 1hz
int obsdata_pretreat(rtk_t* rtkt, obs_t* obsu, obsd_t* obs, double* rs, double* dts, int* svh, double* var)
{
	double dt = 0.0;
	int nobs = 0;
	char tstr[32] = "";
	if (obsu->n <= 0)  
		return 0;
	gnss_obstime = obsu->data[0].time;

	dt = timediff(obsu->data[0].time, lastime);
	if (gnssnavs.n > MAXRECOBS * 1.5)
		uniqnav2(&gnssnavs, obsu->data[0].time);
	time2str(obsu->data[0].time, tstr, 2);
	trace(1, "obs: time=%s n=%d rcv=%2d tdiff=%f nav: %d\r\n", tstr, obsu->n, obsu->data[0].rcv, rtk.opt.maxtdiff, gnssnavs.n);
	if (dt < 0.001)
	{
		return 0;
	}
	else {
		lastime.time = obsu->data[0].time.time;
		lastime.sec = obsu->data[0].time.sec;
	}

	/* exclude satellites */
	nobs = select_sat(*obsu, obsr, &gnssnavs, rtk.opt, obs, rb);
	if (nobs <= 0) {
		return 0;
	}

	return nobs;
}

void inputvrsdata(rtk_t* rtkt, obs_t obsr_t, obsd_t* obs, double* rbt, int* nobs)
{
	char tstr[32] = "";
	int i = 0, j = 0, n = 0;
	double difage = 0.0;

	obs_t obsu = { 0 };
	obsu.data = (obsd_t*)xy_malloc(sizeof(obsd_t) * MAXRECOBS);

	//更新obsu
	for (i = 0; i < (*nobs); i++) obsu.data[i] = obs[i];
	obsu.n = (*nobs);

	if (obsr_t.n <= 0) //未收到vrs卫星数据
	{
		trace(1, "no vrs data\r\n");
		freeobs(&obsu);
		return;
	}

	if (norm(rbt, 3) <= 0.0) {   
		trace(1, "no base position\r\n");
		freeobs(&obsu);
		return;
	}
	for (i = 0; i < 3; i++) rtkt->opt.rb[i] = rbt[i];    //todo 换站了但只更新了vrs观测数据未更新坐标此时该如何处理

	time2str(obsr.data[0].time, tstr, 2);
	rtkt->vrs_age = (float)timediff(obsu.data[0].time, obsr_t.data[0].time);
	trace(1, "vrs: time=%s n=%d rcv=%2d maxtdiffrtk=%f rtk.sol.vrs_age=%f\r\n", tstr, obsr_t.n, obsr_t.data[0].rcv, rtkt->opt.maxtdiffrtk, rtkt->vrs_age);

	difage = timediff(obsr.data[0].time, obsu.data[0].time);
	if (fabs(difage) >= rtkt->opt.maxtdiffrtd)
	{
		trace(1, "the diff age of vrs and obs is too big:%d\n", difage);
		freeobs(&obsu);
		return;
	}

	/* exclude satellites 根据设置的系统选择obs*/
	for (i = n = 0; i < obsr_t.n; i++) {
		if ((satsys(obsr_t.data[i].sat, NULL) & popt.navsys)) obsr_t.data[n++] = obsr_t.data[i];
	}
	if (n <= 0)  
	{
		trace(1, "no match condition vrs data\r\n");
		freeobs(&obsu);
		return;
	}

	for (i = 0; i < n && (*nobs) < MAXRECOBS * 2; i++)
	{
		for (j = 0; j < obsu.n && j < MAXRECOBS; j++)
		{
			if (obsr_t.data[i].sat == obsu.data[j].sat)
			{
				obs[(*nobs)++] = obsr_t.data[i];
				break;
			}
		}
	}
	trace(1, "vrs:nobs=%2d rb=%f %f %f\n \r\n", *nobs, rb[0], rb[1], rb[2]);
	freeobs(&obsu);
}

void inintnmea(nmea_t* nmea)
{
	memset(nmea, 0, sizeof(nmea_t));
	memcpy(nmea->gga, GGA0, sizeof(GGA0));
	memcpy(nmea->rmc, RMC0, sizeof(RMC0));
}

int updateObs(unsigned char* rtcm1, int rtcmLength, nmea_t* nmea)
{
	int i = 0, j = 0;
	int n = 0, nobs = 0;
	double snr_mean = 0.0;

	inintnmea(nmea);

	obsd_t obs0 = { 0 };
	obs_t obsu = { 0 };
	obsu.data = (obsd_t*)xy_malloc(sizeof(obsd_t) * MAXRECOBS);
	for (i = 0; i < MAXRECOBS; i++)
		obsu.data[i] = obs0;

	obsd_t obs[MAXRECOBS * 2] = { 0 };
	double rs[MAXRECOBS * 6 * 2] = { 0.0 };
	double dts[MAXRECOBS * 2 * 2] = { 0.0 };
	int svh[MAXRECOBS * 2] = { 0.0 };
	double var[MAXRECOBS] = { 0.0 };  //只在单点时使用，只需一倍内存空间

	memset(obs, 0, sizeof(obs));

	nmea_t tnmea = { 0 }, fnmea = { 0 };

	inintnmea(&tnmea);
	inintnmea(&fnmea);

	rtk.vrs_age = 0;
	trace(1, "obs rtk.sol.vrs_age=%f\r\n", rtk.vrs_age);
	rtk.opt = popt;
	if (gnssnavs.n < 5) {//收到卫星数小于5不能解算
		trace(1, "no eph data\r\n");
		freeobs(&obsu);
		return 0;
	}

	rtk.staid = 0;  //增加初始化
	rtcmdataread(rtcm1, rtcmLength, &obsu, &gnssnavs, ru, prelock, 1, &rtk, obsstaidinfo.staid);

	trace(1, "obs rtcm decode obs_staid:%p\n", &obsstaidinfo.staid);
	if (obsu.n <= 0){
		trace(1, "obs data decode failed\r\n");
		freeobs(&obsu);
		return 0;
	}


	if (obsstaidinfo.staid && obsstaidinfo.staid != rtk.staid)  //异常处理，避免实时测站id从内存块读取异常
	{
		trace(1, "the obs staid decode maybe wrong:gnssstaidinfo.obs_staid=%d staid=%d\r\n", obsstaidinfo.staid, rtk.staid);
		obsstaidinfo.staid = 0; //重置id，避免实时内存紊乱导致obsid数值变化
		freeobs(&obsu);
		return 0;
	}
	obsstaidinfo.staid = rtk.staid;
	gnss_obstime = obsu.data[0].time;
	sortobs(&obsu);
	for (i = 0; i < obsu.n; i++){
		if ((obsu.data[i].snr[0] * 0.25) < 1e-6)
			continue;
		snr_mean += obsu.data[i].snr[0] * 0.25;
		n++;
	}
	if (n > 0)nmea->snr_mean = snr_mean / n;
	trace(1, "obs:n=%2d snr_mean=%f\r\n", n, nmea->snr_mean);
	//整理代码到obsdata_pretreat 中
	nobs = obsdata_pretreat(&rtk, &obsu, obs, rs, dts, svh, var);
	if (nobs <= 0){
		trace(1, "select failed,maybe bug\n");
		freeobs(&obsu);
		return 0;
	}
	//输入vrs数据到解算结构体obs中
	inputvrsdata(&rtk, obsr, obs, rb, &nobs);// no change
	if (rtkpos(&rtk, obs, nobs, &gnssnavs) < 0){
		trace(1, "rtk failed\n");
		memset(&rtk.sol, 0, sizeof(sol_t));//失败清空解接着用替换的
		rtkoutnmea(&rtk, &tnmea);
		freeobs(&obsu);
		return 0;
	}
	rtkoutnmea(&rtk, &tnmea);
	if (open_filter)//滤波
	{
		if (gga_filter3(tnmea.gga, tnmea.rmc, fnmea.gga, fnmea.rmc, tnmea.enuv))//滤波成功输出滤波解 否则输出rtk的解
		{
			gga_filter_save(nmea, &tnmea, &fnmea, 1);
			trace(4, "gga_filter sucess gga=%s\r\n", nmea->gga);
		}else {
			gga_filter_save(nmea, &tnmea, &fnmea, 2);
			trace(3, "gga_filter fail gga=%s\r\n", nmea->gga);
		}
	}else {
		gga_filter_save(nmea, &tnmea, &fnmea, 2);
		trace(3, " no open filter  gga=%s\r\n", nmea->gga);
	}
	trace(1, "x:%f,y:%f,z:%fstat:%d\r\n", rtk.sol.rr[0], rtk.sol.rr[1], rtk.sol.rr[2], rtk.sol.stat);
	trace(1, "final outgga=%s\r\n", nmea->gga);
	xy_free(obsu.data); obsu.data = NULL; obsu.n = obsu.nmax = 0;
	return 1;
}

#if defined(WIN32)
void postposs() {
#define NUMSYS      7                   /* number of systems */
	/************************配置********************************/
	//注意用该rinex文件流动站与基站观测初始时间必须对齐
	char* infile[1024] =
	{
		"E:/1.Leador_data/RTKpost_data/20250226/A35new/n01.25P",
		"E:/1.Leador_data/RTKpost_data/20250226/A35new/n01.25O",
		"E:/1.Leador_data/RTKpost_data/20250226/base/Test0570.25O"
		
		//"F:/20241223GPS/A/N31L063873580822.24P",
		//"F:/20241223GPS/A/N31L063873580822.24O",
		//"F:/20241223GPS/B/N31L064603580826.24O"
	};
	char outfile[1024] = "D:\\Desktop\\1.pos";

	int start_end_time_limit = 0;//0为不限制起止时间 1为限制
	char* buff1 = "2022 3 9 1 29 28.0";
	char* buff2 = "2022 3 9 2  4  8.0";
	//下面代码还需要指定参考站位置
	/************************初始化**********************/
	FILE* fp[4];
	gtime_t ts = { 0 }, te = { 0 };
	double ti = 0.0, tu = 0.0;
	int i, j, n1 = 1, n2 = 1;
	nmea_t result;
	prcopt_t prcopt = popt;
	char opt[2] = "\0", type[2] = "\0";
	int sys, tsys = TSYS_GPS; double ver;
	gtime_t startime = { 0 }, endtime = { 0 };
	str2time(buff1, 0, 26, &startime);
	str2time(buff2, 0, 26, &endtime);
	obs_t obsu;//流动站
	obsu.data = (obsd_t*)xy_malloc(sizeof(obsd_t) * MAXRECOBS);
	obs_t obsr;//基准站
	obsr.data = (obsd_t*)xy_malloc(sizeof(obsd_t) * MAXRECOBS * 2);
	obsd_t data1[MAXRECOBS], data2[MAXRECOBS * 2], obs[MAXRECOBS * 2];
	nav_t nav = { 0 }; //sta_t sta[2];
	char tobs1[NUMSYS][MAXOBSTYPE][4] = { {""} };
	char tobs2[NUMSYS][MAXOBSTYPE][4] = { {""} };
	char tracefile[100];//statfile[100]/*,path[100]*/,*ext;
	int n, nr = 0, flag1 = 1, flag2 = 1, flag = 0, flagtrace = 1;
	rtk_t rtk;
	int tracel = 5;
	/*打开文件*/
	for (i = 0; i < 3; i++) fp[i] = fopen(infile[i], "r");
	fp[3] = fopen(outfile, "w");
	for (i = 0; i < 4; i++) {
		if (!fp[i]) {
			for (j = 0; j < 4; j++) if (fp[j]) fclose(fp[j]);
			perror("原始或输出文件打开错误");
			return;
		}
	}
	/*读导航文件*/
	readrnxfp(fp[0], ts, te, ti, opt, 0, 0, type, NULL, &nav);//读导航文件
	uniqnav(&nav);
	/*读基准观测文件*/
	if (!readrnxh(fp[1], &ver, type, &sys, &tsys, tobs1, NULL)) return;//流动站
	if (!readrnxh(fp[2], &ver, type, &sys, &tsys, tobs2, NULL)) return;//基准站
	/* open trace file */
	if (flagtrace && tracel > 0) {
		strcpy(tracefile, outfile);
		strcat(tracefile, ".trace");
		tracelevel(tracel);
		traceopen(tracefile);
	}
	rtkinit(&rtk, &prcopt);
	while (!feof(fp[1])) {//开始解算循环
		trace(4, "*****************************************************************************************************************\r\n");
		n1 = 0; n2 = 0;
		n1 = readrnxobsb(fp[1], opt, ver, &tsys, tobs1, &flag, data1, 1);  //移动站
		//if (n1 < 0 && n2 < 0) break;
		if (timediff(data1[0].time, data2[0].time) > 1.0001) { //t1>t2 当移动站播发新的观测值时，解析基准站数据
			flag2 = 1;
		}else {
			flag2 = 0;
		}
		/*如果流动站时间滞后一秒了，就解析基准站*/
		if (flag2) n2 = readrnxobsb(fp[2], opt, ver, &tsys, tobs2, &flag, data2, 2);  //vrs基准站
		/*开始终止时间限制*/
		if (start_end_time_limit == 1) {
			if (timediff(data1[0].time, startime) < 0 || timediff(data1[0].time, endtime) > 0){
				perror("流动站观测数据起止时间筛选...");
				continue;
			}
		}
		/*流动基准观测信息赋值*/
		n = 0;
		for (i = 0; i < n1; i++){
			data1[i].rcv = 1;
			obsu.data[n++] = data1[i];
		}
		obsu.n = n;
		if (n2 > 0){
			nr = 0;
			for (i = 0; i < n2; i++){
				data2[i].rcv = 2;
				obsr.data[nr++] = data2[i];
			}
		}
		obsr.n = nr;
		//obsu.n = n;		
		//流动基准观测信息分类
		trace(4, "readrinex rover: time=%s %ld\r\n", time_str(data1[0].time, 3), data1[0].time.time);
		trace(4, "rover obs=\r\n"); traceobs(&rtk, 4, data1, n1);
		sortobs(&obsu);//移动

		trace(4, "readrinex base: time=%s %ld\r\n", time_str(data2[0].time, 3), data2[0].time.time);
		trace(4, "base obs=\r\n"); traceobs(&rtk, 4, data2, n2);
		sortobs(&obsr);//基准

		if (obsu.n < 5){
			trace(4, "nu_ satnum less than 4\r\n");
			continue;
		}
		/*指定参考站位置*/
	/*	rtk.opt.rb[0] = -2276388.3991;
		rtk.opt.rb[1] = 5012763.3037;
		rtk.opt.rb[2] = 3209642.8528;*/
		rtk.opt.rb[0] = -2273225.633;
		rtk.opt.rb[1] = 5012023.190;
		rtk.opt.rb[2] = 3213012.310;

		n = select_sat(obsu, obsr, &nav, popt, obs, rtk.opt.rb);//加入了选星
		//选完星后将obsu与obsr更新到obs里
		for (i = 0; i < n; i++) obsu.data[i] = obs[i];
		obsu.n = n;
		for (i = 0; i < nr; i++){
			for (j = 0; j < obsu.n; j++){
				if (obsr.data[i].sat == obsu.data[j].sat){
					obs[n++] = obsr.data[i];
				}
			}
		}
		if (rtkpos(&rtk, obs, n, &nav) < 0) continue;

		rtk.sol.last_stat = rtk.sol.stat;
		rtkoutnmea(&rtk, &result);
		fprintf(fp[3], "%s\n", result.gga);
		checkbrk("processing : %s Q=%d", time_str(rtk.sol.time, 3), rtk.sol.stat);
	}

	rtkfree(&rtk);
	xy_free(obsu.data);
	xy_free(obsr.data);
	obsu.data = NULL;
	obsr.data = NULL;
	for (i = 0; i < 4; i++) fclose(fp[i]);
	traceclose();
	return;
}
#endif

