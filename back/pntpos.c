#include "rtklib.h"

/* constants -----------------------------------------------------------------*/
#define NX_SPP          (4+3)   /* # of estimated parameters */ 
#define CUTOFF_ANGLE 18         /*计算高度角时的截止高度角*/

#define MAXITR      10          /* max number of iteration for point pos */
#define ERR_ION     5.0         /* ionospheric delay std (m) */
#define ERR_TROP    3.0         /* tropspheric delay std (m) */
#define ERR_SAAS    0.3         /* saastamoinen model error std (m) */
#define ERR_BRDCI   0.5         /* broadcast iono model error factor */
#define ERR_CBIAS   0.3         /* code bias error std (m) */
#define REL_HUMI    0.7         /* relative humidity for saastamoinen model */

#define NS 11                   /* 待估参数*/
#define THRE_SATNUM 5

#define VEL_VAR     0.05
#define VAR_DOP     0.001       /* variance of doppler measurement update */
#define VAR_CLK     SQR(20.0)   /* initial variance of receiver clock correction */
#define VAR_SHIFT   SQR(5.0)    /*initial variance of receiver clock shift */
static double pntsnrthres = 0.0;
static double pntelvthres = 0.0;
static int glo_isb_estflag = 0;        //gal isb estimate yes or not: 0 no    1 yes
static int gal_isb_estflag = 0;        //gal isb estimate yes or not: 0 no    1 yes
static int bds_isb_estflag = 0;        //gal isb estimate yes or not: 0 no    1 yes

static gtime_t obsstartime = { 0 };
static int startimeinitflag = 0;//开机初始化标志

/* get tgd parameter (m) -----------------------------------------------------*/
static double gettgd(int sat, const nav_t* nav)
{
	int i = 0;
	for (i = 0; i < nav->n; i++) {
		if (nav->eph[i].sat != sat) continue;
		return CLIGHT * nav->eph[i].tgd;
	}
	return 0.0;
}

/* get tgd1 parameter for BDS (m) --------------------------------------------*/
double gettgd1(int sat, const nav_t* nav)
{
	if (nav == NULL) {
		return -1.0;
	}
	int i = 0;
	for (i = 0; i < nav->n; i++) {
		if (nav->eph[i].sat != sat)
			continue;
		return CLIGHT * nav->eph[i].tgdb;
	}
	return 0.0;
}

/* mutipath correct-------------------------------------------------------------
* BeiDou satellite-induced code pseudorange variations correct
* args   : rtk_t *rtk       IO  rtk control/result struct
		   obsd_t *obs      IO  observation data
		   int    n         I   number of observation data
		   nav_t  *nav      I   navigation messages
* note   :https://blog.csdn.net/cxy0711/article/details/133896266
*
* -----------------------------------------------------------------------------*/
double bds_multipath_corr(const int sat, const double* azel, int i)
{
	int j = 0, prn = 0, b = 0;
	double dmp[3] = { 0.0 }, elev = 0.0, a = 0.0;
	const static double IGSOCOEF[3][10] = {		/* m */
		{-0.55,-0.40,-0.34,-0.23,-0.15,-0.04,0.09,0.19,0.27,0.35},	//B1
		{-0.71,-0.36,-0.33,-0.19,-0.14,-0.03,0.08,0.17,0.24,0.33},	//B2
		{-0.27,-0.23,-0.21,-0.15,-0.11,-0.04,0.05,0.14,0.19,0.32},	//B3
	};
	const static double MEOCOEF[3][10] = {		/* m */
		{-0.47,-0.38,-0.32,-0.23,-0.11,0.06,0.34,0.69,0.97,1.05},	//B1
		{-0.40,-0.31,-0.26,-0.18,-0.06,0.09,0.28,0.48,0.64,0.69},	//B2
		{-0.22,-0.15,-0.13,-0.10,-0.04,0.05,0.14,0.27,0.36,0.47},	//B3
	};

	if (satsys(sat, &prn) != SYS_CMP) return 0.0;

	if (prn <= 5) return 0.0;

	elev = azel[1] * R2D;

	if (elev <= 0.0) return 0.0;

	for (j = 0; j < 3; j++) dmp[j] = 0.0;

	a = elev * 0.1;
	b = (int)a;

	if (prn >= 6 && prn < 11) { // IGSO(C06, C07, C08, C09, C10)
		if (b < 0) {
			for (j = 0; j < 3; j++) dmp[j] = IGSOCOEF[j][0];
		}
		else if (b >= 9) {
			for (j = 0; j < 3; j++) dmp[j] = IGSOCOEF[j][9];
		}
		else {
			for (j = 0; j < 3; j++) dmp[j] = IGSOCOEF[j][b] * (1.0 - a + b) + IGSOCOEF[j][b + 1] * (a - b);
		}
	}
	else if (prn >= 11 && prn < 15) {   // MEO(C11, C12, C13, C14)
		if (b < 0) {
			for (j = 0; j < 3; j++) dmp[j] = MEOCOEF[j][0];
		}
		else if (b >= 9) {
			for (j = 0; j < 3; j++) dmp[j] = MEOCOEF[j][9];
		}
		else {
			for (j = 0; j < 3; j++) dmp[j] = MEOCOEF[j][b] * (1.0 - a + b) + MEOCOEF[j][b + 1] * (a - b);
		}
	}
	return dmp[i];
}

/* psendorange with code bias correction -------------------------------------*/
//改进不同:伪距残差中信噪比检测有高度角相关信噪比动态阈值改成35的静态阈值；增加三频接口；北斗2卫星增加多路径改正（高度角依赖模型）
static double prange(const obsd_t* obs, const nav_t* nav, const double* azel,
	int iter, const prcopt_t* opt, double* var)
{
	/*稳健性判断*/
	if (obs == NULL || nav == NULL || azel == NULL || opt == NULL || var == NULL) {
		return -1.0;
	}
	/*变量初始化*/
	const double* lam = nav->lam[obs->sat - 1];
	double pc = 0.0, p1 = 0.0, p2 = 0.0, p3 = 0.0, p1_p2 = 0.0, p1_c1 = 0.0, p2_c2 = 0.0, gamma = 0.0, gamma1 = 0.0;
	int i = 0, j = 1, k = 2, sys = 0, npr = 0;
	double test = 0.0;
	*var = 0.0;
	if (!(sys = satsys(obs->sat, NULL))) {
		return 0.0;
	}

	if (NFREQ >= 3 && (sys & SYS_GAL))/*L1-L2 for GPS/GLO/QZS, L1-L5 for GAL/SBS 根据系统选择频点*/
		j = 2;
	if (NFREQ < 2 || lam[i] == 0.0 || lam[j] == 0.0) {//备份操作，只要有星历就有波长，其实是判断有没有星历，TODO单频退出？？？
		return 0.0;
	}
	if (NFREQ < 3)k = 0;//三频的策略与双频兼容:避免仅双频时（nav->lam只储存两个频）变量k=2的时候lam[k]，obs->P[k] 越界

	if (testsnr(0, i, azel[1], obs->snr[i] * 0.25, opt->snroff, pntsnrthres)) {
		trace(4, "snr mask: %s sat=%2d el=%.1f snr=%.1f\n",time_str(obs->time, 0), obs->sat, azel[1] * R2D, obs->snr[i] * 0.25);
		return 0.0;
	}
	/*!计算DCB改正*/
	gamma = SQR(lam[j]) / SQR(lam[i]); // f1^2/f2^2
	gamma1 = SQR(lam[k]) / SQR(lam[i]); // f1^2/f3^2   三频时的处理策略，可用一频比三频
	p1 = obs->P[i]; p2 = obs->P[j]; p3 = obs->P[k];

	p1_p2 = nav->cbias[obs->sat - 1][0];//不同频率的码偏差
	p1_c1 = nav->cbias[obs->sat - 1][1];//同频载波伪距码偏差
	p2_c2 = nav->cbias[obs->sat - 1][2];
	if (fabs(p1_c1) > 1000.0)
		p1_c1 = 0.0;
	if (fabs(p1_p2) > 1000.0)
		p1_p2 = 0.0;
	if (fabs(p2_c2) > 1000.0)
		p2_c2 = 0.0;
	/* TDG/DCB改正算法*/
	if (p1_p2 == 0.0 && (sys & (SYS_GPS | SYS_GAL | SYS_CMP))) {
		test = gettgd(obs->sat, nav);
		p1_p2 = (1.0 - gamma) * gettgd(obs->sat, nav);
	}
	if (p1 == 0.0 && p2 == 0.0 && p3 == 0.0) {/* single-frequency */
		return 0.0;
	}

	if (p1 != 0.0 && obs->D[i] != 0.0f) {//原版RTKLIB里的单频的方法
		pc = (p1 + (obs->code[i] == CODE_L1C ? p1_c1 : 0)) - p1_p2 / (1.0 - gamma);  
	}
# if 0
	else if (p2 != 0.0 && obs->D[j] != 0.0f) {
		pc = (p2 + (obs->code[j] == CODE_L2C ? p2_c2 : 0)) - gamma * p1_p2 / (1.0 - gamma);
	}
#endif
	if (sys == SYS_CMP) {
		if (p1 != 0.0 && obs->D[i] != 0.0f) {
			pc = p1 - gettgd(obs->sat, nav) + bds_multipath_corr(obs->sat, azel, i); /* tgd1 correction */
		}
# if 0
		else if (p2 != 0.0) {
			pc = p2 - gettgd1(obs->sat, nav) + bds_multipath_corr(obs->sat, azel, j); /* tgd2 correction */
		}
#endif
	}
	if (p1 == 0.0 && p2 == 0.0 && p3 != 0.0 && obs->D[k] != 0.0f) {
		pc = p3;
	}
	//trace(5, "PC=%16.9f P1_C1=%16.9f P1_P2=%16.9f gamma=%16.9f\r\n", PC, P1_C1, P1_P2, gamma);
	*var = SQR(ERR_CBIAS);
	return pc;
}
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
extern int ionocorr(gtime_t time, const nav_t* nav, int sat, const double* pos,
	const double* azel, int ionoopt, double* ion, double* var)
{
	//trace(5,"ionocorr: time=%s opt=%d sat=%2d pos=%.3f %.3f azel=%.3f %.3f\r\n",
	//      time_str(time,3),ionoopt,sat,pos[0]*R2D,pos[1]*R2D,azel[0]*R2D,
	//      azel[1]*R2D);

	/* broadcast model */
	if (ionoopt == IONOOPT_BRDC) {
		*ion = ionmodel(time, nav->ion_gps, pos, azel);
		*var = SQR(*ion * ERR_BRDCI);
		return 1;
	}
	*ion = 0.0;
	*var = ionoopt == IONOOPT_OFF ? SQR(ERR_ION) : 0.0;
	return 1;
}
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
extern int tropcorr(gtime_t time, const nav_t* nav, const double* pos,
	const double* azel, int tropopt, double* trp, double* var)
{
	//trace(5,"tropcorr: time=%s opt=%d pos=%.3f %.3f azel=%.3f %.3f\r\n",
	//      time_str(time,3),tropopt,pos[0]*R2D,pos[1]*R2D,azel[0]*R2D,
	//      azel[1]*R2D);

	/* saastamoinen model */
	if (tropopt == TROPOPT_SAAS || tropopt == TROPOPT_EST || tropopt == TROPOPT_ESTG) {
		*trp = tropmodel(time, pos, azel, REL_HUMI);
		*var = SQR(ERR_SAAS / (sin(azel[1]) + 0.1));
		return 1;
	}
	/* sbas troposphere model */
# if 0
	if (tropopt == TROPOPT_SBAS) {
		//*trp=sbstropcorr(time,pos,azel,var);
		return 1;
	}
# endif
	/* no correction */
	* trp = 0.0;
	*var = tropopt == TROPOPT_OFF ? SQR(ERR_TROP) : 0.0;
	return 1;
}

/* pseudorange measurement error variance 随即游走模型，高度角定权（有待优化）未加信噪比定权------------------------------------*/
double varerr_psr(const prcopt_t* opt, double el, int sys) {
	if (opt == NULL) {
		return -1.0;
	}
	double fact = 0.0, varr = 0.0;
	fact = sys == SYS_GLO ? EFACT_GLO : (sys == SYS_SBS ? EFACT_SBS : EFACT_GPS);//只对GLO与sbs和其他系统设置系统权重了（GPS与GLO是等权的）。都为等权的，相当于没做处理
	varr = SQR(opt->err[0]) * (SQR(opt->err[1]) + SQR(opt->err[2]) / sin(el));
	if (opt->ionoopt == IONOOPT_IFLC)
		varr *= SQR(3.0); /* iono-free */
	return SQR(fact) * varr;
}

/*
*在解算中剔除粗差较大的星
*/
int obsdect_out(const obsd_t* obs, int* saterrorlist)
{
	for (int j = 0; j < MAXEORSTA; j++) {
		if (obs->sat == saterrorlist[j])//利用前面最大最小伪距粗差探测对伪距进行剔除然后标记卫星，此函数对标记的卫星进行剔除
			return 1;
	}
	return 0;
}

static int rescode(int iter, const obsd_t* obs, int n, const double* rs,
	const double* dts, const double* vare, const int* svh, const nav_t* nav,
	const double* x, const prcopt_t* opt, double* v, double* H, double* var,
	double* azel, int* vsat, double* resp, int* ns, sol_t* sol, int* saterrorlist, int beforedect)
{
	/*稳健性判断*/
	if (obs == NULL || rs == NULL || dts == NULL || vare == NULL || svh == NULL || nav == NULL
		|| x == NULL || opt == NULL || v == NULL || H == NULL || var == NULL || azel == NULL
		|| vsat == NULL || resp == NULL || ns == NULL || sol == NULL) {
		return -1;
	}
	/*变量初始化*/
	double r = 0.0, dion = 0.0, dtrp = 0.0, vmeas = 0.0, vion = 0.0, vtrp = 0.0, rr[3] = { 0.0 }, pos[3] = { 0.0 }, dtr = 0.0, e[3] = { 0.0 }, P = 0.0, lam_l1 = 0.0;
	int i = 0, j = 0, nv = 0, sys = 0, mask[4] = { 0 };
	int weekn = 0;
	double weeks = 0.0;
	int prn = 0;
	double avg_snr = 0;	//查找最小信噪比和所有参与解算卫星的信噪比
	double min_snr = 0;
	double max_snr = 0;
	const double lam_carr[MAXFREQ] = { /* carrier wave length (m) */
		CLIGHT / FREQ1,CLIGHT / FREQ2,CLIGHT / FREQ5,CLIGHT / FREQ6,CLIGHT / FREQ7,
		CLIGHT / FREQ8,CLIGHT / FREQ9
	};
	weeks = time2gpst(obs[0].time, &weekn);//weekn为周，weeks为周内秒
	for (i = 0; i < 3; i++)
		rr[i] = x[i]; dtr = x[3];//卫星钟差
	
	ecef2pos(rr, pos);
	/*逐个卫星计算伪距残差*/
	for (i = *ns = 0; i < n && i < MAXPNTOBS; i++) {   //MAXOBS=120 n为obs数量
		vsat[i] = 0;
		azel[i * 2] = azel[1 + i * 2] = resp[i] = 0.0;
		if (!(sys = satsys(obs[i].sat, NULL)))
			continue;
		if (satexclude(obs[i].sat, svh[i], opt)) {
			//trace->trace(3, "pntpos::rescode, satexclude sat=%d %d %d\n", obs[i].sat,svh[i],opt->exsats[obs[i].sat - 1]);
			continue;
		}
		if (i < n - 1 && i < MAXPNTOBS - 1 && obs[i].sat == obs[i + 1].sat) {/*拒接重复观测值*/
			i++;
			trace(3, "Pntpos::rescode, pntpos_continue_reject duplicated data sat=%d\r\n", obs[i].sat);
			continue;
		}
		if ((r = geodist(rs + i * 6, rr, e)) <= 0.0 || satazel(pos, e, azel + i * 2) < pntelvthres) {
			trace(5, "pntpos_continue_geometric sat=%d r=%f elevation=%f\r\n", obs[i].sat, r, satazel(pos, e, azel + i * 2) * R2D);
			continue;
		}
		if (obsdect_out(obs + i, saterrorlist)) {
			trace(4, "(%2d)Pntpos::rescode sat obs error.sat:%d\r\n", iter, obs[i].sat);
			continue;
		}
		if ((sys = limitcmpgeo(obs[i].sat)) <= SYS_NONE) {
			trace(4, "(%2d)Pntpos::GEO limited:%d\r\n", iter, obs[i].sat);
			continue;
		}
		if ((P = prange(obs + i, nav, azel + i * 2, iter, opt, &vmeas)) == 0.0) {//vmeas码偏差改正的方差，写死固定的
			trace(5, "pntpos_continue_pseudorange with code bias correction sat=%d\r\n", obs[i].sat); // 跳到这里面的原因是，根据观测数据的卫星号导航星历里面没有，导致r算不出来
			continue;
		}
		if (!ionocorr(obs[i].time, nav, obs[i].sat, pos, azel + i * 2, iter > 0 ? opt->ionoopt : IONOOPT_BRDC, &dion, &vion)) {
			continue;
		}
		if ((lam_l1 = nav->lam[obs[i].sat - 1][0]) > 0.0) {// GPS-L1 -> L1/B1
			dion *= SQR(lam_l1 / lam_carr[0]);
		}
		if (!tropcorr(obs[i].time, nav, pos, azel + i * 2, iter > 0 ? opt->tropopt : TROPOPT_SAAS, &dtrp, &vtrp)) {
			continue;
		}
		/*pseudorange residual*/
		v[nv] = P - (r + dtr - CLIGHT * dts[i * 2] + dion + dtrp);
		/*播发错误星历判断*/
		if (!beforedect && norm(rr, 3) > 0 && v[nv] > 10000) {  //针对K803有播发错误星历的现象，此处可修复轨迹中断后重新定位，但无法修复刚开机时播发有错误星历时无法修复
			trace(5, "rescode:the v is too big,SAT=%d v[%2d]=%10.3f P=%16.9f r=%16.3f dtr=%9.3f dts=%9.3f dion=%9.3f dtrp=%9.3f x[6]=%9.3f\r\n", obs[i].sat, nv, v[nv], P, r, dtr, CLIGHT * dts[i * 2], dion, dtrp, x[6]);
			continue;
		}
		/*design matrix*/ 
		for (j = 0; j < NX_SPP; j++)
			H[j + nv * NX_SPP] = j < 3 ? -e[j] : (j == 3 ? 1.0 : 0.0);
		/*计算平均信噪比和最大最小信噪比*/
		avg_snr += obs[i].snr[0] * 0.25;
		if (obs[i].snr[0] * 0.25 < min_snr || (nv == 0))
			min_snr = obs[i].snr[0] * 0.25;
		if (obs[i].snr[0] * 0.25 > max_snr || (nv == 0))
			max_snr = obs[i].snr[0] * 0.25;
		/*time system and receiver bias offset correction*/
		if (sys == SYS_GLO) {
			v[nv] -= x[4];
			if (glo_isb_estflag){
				H[4 + nv * NX_SPP] = 1.0;
				mask[1] = 1;
			}
		}else if (sys == SYS_GAL) {
			v[nv] -= x[5];
			if (gal_isb_estflag){
				H[5 + nv * NX_SPP] = 1.0;
				mask[2] = 1;
			}
		}else if (sys == SYS_CMP) {
			v[nv] -= x[6];
			if (bds_isb_estflag){
				H[6 + nv * NX_SPP] = 1.0;
				mask[3] = 1;
			}
		}else{
			mask[0] = 1;
		}
		vsat[i] = 1;
		resp[i] = v[nv];
		(*ns)++;
		trace(5, "rescode::SAT=%d v[%2d]=%10.3f P=%16.9f r=%16.3f dtr=%9.3f dts=%9.3f dion=%9.3f dtrp=%9.3f x[6]=%9.3f\r\n", obs[i].sat, nv, v[nv], P, r, dtr, CLIGHT *dts[i * 2], dion, dtrp, x[6]);	
		/*error variance*/
		var[nv++] = varerr_psr(opt, azel[1 + i * 2], sys) + vare[i] + vmeas + vion + vtrp;//var为对应卫星方程的方差,决定加权最小二乘的对应方程的权重，方差越小，权重越大.vmeas为std方差
		trace(5,"rescode::variance：SAT=%d var=%f varerr_psr(opt, azel[1 + i * 2], sys) =%f vare[i]=%f vmeas =%f  vion=%f  vtrp=%f\n", obs[i].sat,var[nv],varerr_psr(opt, azel[1 + i * 2], sys), vare[i] , vmeas , vion , vtrp);
	}
	/*计算观测值信噪比均值和最小信噪比*/
	if (nv > 1)
		avg_snr = avg_snr / nv;

	trace(4, "nv=%d avg snr is %f,min snr is %f\r\n", nv, avg_snr, min_snr);
	avg_snr = 0.0;

	// constraint to avoid rank-deficient约束以避免秩亏
	for (i = 0; i < 4; i++) {
		if (mask[i])//相应系统时间偏差标志mask[i]
			continue;
		v[nv] = 0.0;
		for (j = 0; j < NX_SPP; j++)
			H[j + nv * NX_SPP] = j == i + 3 ? 1.0 : 0.0;
		var[nv++] = 0.01;
	}
	return nv;
}

#if 0
/* pseudorange residuals -----------------------------------------------------*/
static int rescode2(int iter, const obsd_t* obs, int n, const double* rs,
	const double* dts, const double* vare, const int* svh, const nav_t* nav,
	const double* x, const prcopt_t* opt, double* v, double* H, double* var,
	double* azel, int* vsat, double* resp, int* ns, sol_t* sol, const int* vsatc, int* vflag)
{
	if (obs == NULL || rs == NULL || dts == NULL || vare == NULL || svh == NULL || nav == NULL
		|| x == NULL || opt == NULL || v == NULL || H == NULL || var == NULL || azel == NULL
		|| vsat == NULL || resp == NULL || ns == NULL || sol == NULL) {
		return -1;
	}

	double r = 0.0, dion = 0.0, dtrp = 0.0, vmeas = 0.0, vion = 0.0, vtrp = 0.0, rr[3] = { 0.0 }, pos[3] = { 0.0 }, dtr = 0.0, e[3] = { 0.0 }, P = 0.0, lam_l1 = 0.0;
	int i = 0, j = 0, nv = 0, sys = 0, mask[4] = { 0 };
	int weekn = 0;
	double weeks = 0.0;

	//查找最小信噪比和所有参与解算卫星的信噪比
	double avg_snr = 0;
	double min_snr = 0;
	double max_snr = 0;

	weeks = time2gpst(obs[0].time, &weekn);
	for (i = 0; i < 3; i++)
		rr[i] = x[i]; dtr = x[3];
	ecef2pos(rr, pos);
	for (i = *ns = 0; i < n && i < MAXPNTOBS; i++) {   //MAXOBS=120
		vsat[i] = 0;
		azel[i * 2] = azel[1 + i * 2] = resp[i] = 0.0;
		if (!(sys = satsys(obs[i].sat, NULL)))
			continue;
		if (!vsatc)
			continue;

		// reject duplicated observation data
		if (i < n - 1 && i < MAXPNTOBS - 1 && obs[i].sat == obs[i + 1].sat) {
			i++;
			trace(4, "Pntpos::rescode, pntpos_continue_reject duplicated data sat=%d\r\n", obs[i].sat);
			continue;
		}
		// excluded satellite?
		if (satexclude(obs[i].sat, svh[i], opt)) {
			//trace->trace(3, "Pntpos::rescode, satexclude sat=%d %d %d\n", obs[i].sat,svh[i],opt->exsats[obs[i].sat - 1]);
			continue;
		}

		//剔除粗差较大的星
		if (obsdect_out(obs + i, opt)) {
			trace(4, "(%2d)Pntpos::rescode sat obs error.sat:%d\r\n", iter, obs[i].sat);
			continue;
		}

		// geometric distance/azimuth/elevation angle
		if ((r = geodist(rs + i * 6, rr, e)) <= 0.0 || satazel(pos, e, azel + i * 2) < opt->elmin) {
			trace(5, "pntpos_continue_geometric sat=%d r=%f elevation=%f\r\n", obs[i].sat, r, satazel(pos, e, azel + i * 2) * R2D);
			continue;
		}

		// psudorange with code bias correction
		if ((P = prange(obs + i, nav, azel + i * 2, iter, opt, &vmeas)) == 0.0) {
			trace(5, "pntpos_continue_pseudorange with code bias correction sat=%d\r\n", obs[i].sat);
			continue;
		}
#  if 0
		if (rtkcmn->testsnr(0, i, azel[1], obs[i].snr[0] * 0.25, opt->snroff, opt->snrthres,
			&opt->snrmask)) {
			trace->trace(4, "Pntpos::rescode testsnr_continue \n");
			continue;
		}
#endif
		// ionospheric corrections
		if (!ionocorr(obs[i].time, nav, obs[i].sat, pos, azel + i * 2, iter > 0 ? opt->ionoopt : IONOOPT_BRDC, &dion, &vion))
			continue;

		// GPS-L1 -> L1/B1
		if ((lam_l1 = nav->lam[obs[i].sat - 1][0]) > 0.0) {
			dion *= SQR(lam_l1 / lam_carr[0]);
		}
		// tropospheric corrections
		if (!tropcorr(obs[i].time, nav, pos, azel + i * 2, iter > 0 ? opt->tropopt : TROPOPT_SAAS, &dtrp, &vtrp)) {
			continue;
		}
		// pseudorange residual
		v[nv] = P - (r + dtr - CLIGHT * dts[i * 2] + dion + dtrp);

		// design matrix
		for (j = 0; j < NX_SPP; j++)
			H[j + nv * NX_SPP] = j < 3 ? -e[j] : (j == 3 ? 1.0 : 0.0);

		avg_snr += obs[i].snr[0] * 0.25;
		if (obs[i].snr[0] * 0.25 < min_snr || (nv == 0))
			min_snr = obs[i].snr[0] * 0.25;
		if (obs[i].snr[0] * 0.25 > max_snr || (nv == 0))
			max_snr = obs[i].snr[0] * 0.25;

		// time system and receiver bias offset correction
		if (sys == SYS_GLO) {
			v[nv] -= x[4];
			H[4 + nv * NX_SPP] = 1.0;
			mask[1] = 1;
		}
		else if (sys == SYS_GAL) {
			v[nv] -= x[5];
			H[5 + nv * NX_SPP] = 1.0;
			mask[2] = 1;
		}
		else if (sys == SYS_CMP) {
			v[nv] -= x[6];
			H[6 + nv * NX_SPP] = 1.0;
			mask[3] = 1;
		}
		else
			mask[0] = 1;

		/* 只记录可用的i */
		vflag[nv] = (i << 8);

		vsat[i] = 1;
		resp[i] = v[nv];
		(*ns)++;
		trace(5, "SAT=%d v[%2d]=%10.3f P=%16.9f r=%16.9f dtr=%9.7f dts=%9.7f dion=%9.7f dtrp=%9.7f x[6]=%9.7f\r\n", obs[i].sat, nv, v[nv], P, r, dtr, dts[i * 2], dion, dtrp, x[6]);

		// error variance
		var[nv++] = varerr_psr(opt, azel[1 + i * 2], sys) + vare[i] + vmeas + vion + vtrp;
	}
	//计算观测值信噪比均值和最小信噪比

	if (nv > 1)
		avg_snr = avg_snr / nv;

	trace(4, "nv=%d avg snr is %f,min snr is %f\r\n", nv, avg_snr, min_snr);
	avg_snr = 0.0;

	// constraint to avoid rank-deficient
	for (i = 0; i < 4; i++) {
		if (mask[i])
			continue;
		v[nv] = 0.0;
		for (j = 0; j < NX_SPP; j++)
			H[j + nv * NX_SPP] = j == i + 3 ? 1.0 : 0.0;
		var[nv++] = 0.01;
	}
	return nv;
}
#endif

/*查找是否在上个历元被判为粗差，是则返回1，否则返回0*/
int find_outlier_in_preepoch(int sat)
{
	int i = 0;
	for (i = 0; i < MAXDIFOBS; i++){
		if (sat == midmsg_[i].sat && (midmsg_[i].dpsr1 || midmsg_[i].dpsr2[0])){
			trace(4, "in pre eopch the sat is outlier:sat=%4d\n", sat);
			return 1;
		}
	}
	return 0;
}

int decpsrout(int n, const obsd_t* obs, double* v, int* vsat, int* sta, int flag)
{
	if (obs == NULL || v == NULL || vsat == NULL || sta == NULL) {
		return -1;
	}
	int  i = 0, j = 0, gpssatnum = 0, hasoutlier = 0;
	double coef = 3;   //粗差判断阈值系数
	double gmean = 0.0, gstd = 0.0, glim = 0.0, glim_min = 0.0, glim_max = 0.0;
	double gval[MAXPNTOBS] = { 0.0 };
	glim_max = 30.0;
	glim_min = 6.0;

	if (flag) {
		for (i = 0, j = 0; i < n; i++) {
			if (vsat[i] == 1) {
				if (!find_outlier_in_preepoch(obs[i].sat))  {
					gval[gpssatnum] = v[j];//可用卫星观测的伪距残差（不是粗差的）
					gpssatnum++;//可用卫星数
				}
			}
			if (vsat[i] != 0) j++;
		}
	}else {
		for (i = 0, j = 0; i < n; i++) {
			if (vsat[i] == 1) {
				gval[gpssatnum] = v[j];
				gpssatnum++;
			}
			if (vsat[i] != 0) j++;
		}
	}
	if (gpssatnum < 4) {
		return 1;
	}
	orderdopdif(gpssatnum, gval);
	gmean = gval[gpssatnum / 2]; 
	gstd = 0.0;
	for (j = 0, i = 1; i < gpssatnum - 1; i++) {
		if (fabs(gval[i] - gmean) < 3 * glim_max) {
			gstd = gstd + (gval[i] - gmean) * (gval[i] - gmean);
			j++;
		}
	}
	if (j <= 3) {
		return 1;
	}
	if (j > 0)
		glim = sqrt(gstd / j);
	trace(5, "decpsrout:GMean=%6.3f GLim=%6.3f\n", gmean, glim);
	if (glim < glim_min)
		glim = glim_min;
	for (j = 0, i = 0; i < n; i++) {
		if (vsat[i] == 1) {
			if (find_outlier_in_preepoch(obs[i].sat))
			{
				coef = 2.5;//上次有粗差系数降权
			}else {
				coef = 3.0;
			}
			if (fabs(v[j] - gmean) >= coef * glim) {//原理？？？
				hasoutlier = -1;
				*sta = obs[i].sat;
				break;
			}
		}
		if (vsat[i] != 0)
			j++;
	}
	return (hasoutlier);
}
/*return -1探测到了 0没探测到*/
int decpsrsys(int n, const obsd_t* obs, double* v, int* vsat, int* sat)
{
	if (obs == NULL || v == NULL || vsat == NULL || sat == NULL) {
		return -1;
	}
	int hasoutlier = 0, flag = 0;
	hasoutlier = decpsrout(n, obs, v, vsat, sat, 1); //一次只能探测出一颗星,0表示没探测出粗差，1表示不满足粗差探测的条件，-1表示探测出粗差
	if (hasoutlier == 1) 
		hasoutlier = decpsrout(n, obs, v, vsat, sat, 0);
	if (hasoutlier == -1) flag = -1;
	return flag;
}

int decpsrerror(int i, int beforedetct, int n, const obsd_t* obs, double* v, int* vsat, double* var, int* dect, int* saterrorlist)
{
	/*稳健性判断*/
	if (obs == NULL || v == NULL || vsat == NULL || var == NULL || dect == NULL || saterrorlist == NULL) {
		return -1;
	}
	/*变量初始化*/
	int itea = 0, m = 0, j = 0, k = 0, sat = 0, h = 0;
	
	for (itea = 0; itea < MAXEORSTA; itea++) 
	{										   
		sat = 0;

		if (i == 0 && beforedetct == 1) {//每历元检测一次就可以，不需要每次最小二乘迭代都检测
			dect[itea] = decpsrsys(n, obs, v, vsat, &sat);//返回-1为探测到了（循环探测所有卫星，但只能检测出一颗卫星就返回）
			if (!dect[itea])
				break;
			if (satsys(sat, NULL) != SYS_NONE)
				saterrorlist[h++] = sat;
		}
		if (beforedetct == 1 && dect[itea] == -1) {//如果探测到粗差,其本历元相应的卫星vsat为-1，方差无限大
			for (j = k = 0; j < n; j++) {
				if (vsat[j] == 0)
					continue;
				for (m = 0; m < MAXEORSTA; m++) {
					if (!saterrorlist[m])
						continue;
					if (obs[j].sat == saterrorlist[m]) {
						vsat[j] = -1;  
						var[k] = 1e16;
						trace(4, "decpsrerror sat=%2d\r\n", obs[j].sat);
					}
				}
				k++;
			}
		}
	}
	return 1;
}
int decpsrerror_add(int n, double* v, int* vsat, double* var, double vvf)
{
	/*变量初始化*/
	double vj[MAXPNTOBS] = { 0.0 };
	int i = 0;
	int j = 0;
	double abs_vj = 0.0;
	for (i = 0; i < n; i++) {
		/*稳健性判断*/
		if (vsat[i] != 1)
			continue;
		if (fabs(vvf) < 1e-10 || fabs(var[j]) < 1e-10) continue;
		vj[j] = (v[j] / vvf) / var[j];
		if (fabs(vj[j]) > 1.96) {
			var[j] = 1e16;
			vsat[i] = -1;
		}
		abs_vj = fabs(vj[j]);
		if (abs_vj < 0.5) {
			var[j] *= 0.5;
		}
		else if (abs_vj < 1.0) {
			// var[j] = var[j] 这行可以直接删除，因为不需要任何操作
		}
		else if (abs_vj < 1.5) {
			var[j] *= 2.0;
		}
		else if (abs_vj < 1.96) {
			var[j] *= 4.0;
		}
		j++;
	}

	return 1;
}

int dopflag(int i, const midmsg_t* midmsg, const unsigned char* snr, const int* vsatc_o, const double* vector)
{
	if (midmsg == NULL || snr == NULL || vsatc_o == NULL || vector == NULL) {
		return -1;
	}
	if (midmsg[i].d1 == 0.0 || !(*vsatc_o) || norm(vector, 3) <= 0.0 || midmsg[i].ddop1)//|| ((*SNR)*0.25)<15.0
	{
		return 0;
	}else {
		return 1;
	}
}
int lsqvv(double* H, double* var, int n, int m, double* R, double* qvv)
{
	if (H == NULL || var == NULL || R == NULL || qvv == NULL) {
		return -1;
	}
	if (m < n)
		return -1;
	int info = 0, i = 0, k = 0;
	double sig = 0.0, * P = NULL;
	double ata[16] = { 0 }, h1[MAXPNTOBS * 4 * DIF_L_IN_EVL] = { 0 }, htp[MAXPNTOBS * 4 * DIF_L_IN_EVL] = { 0 }, qhtp[MAXPNTOBS * 4 * DIF_L_IN_EVL] = { 0 };
	P = zeros(m, m);
	matcpy(h1, H, n, m);
	
	for (i = 0; i < m; i++) {
		sig = sqrt(var[i]);//先根号求标准差
		for (k = 0; k < n; k++)
			h1[k + i * n] /= sig;
	}
	for (i = 0; i < m; i++) {
		P[i + i * m] = 1 / var[i];
		R[i + i * m] = 1.0;
	}
	/*H'*(H*P*H')-1*H*P*/
	matmul("NT", n, n, m, 1.0, h1, h1, 0.0, ata);  /* Q=H*P*H'*/
	if (!(info = matinv(ata, n))) {/*Q-1*/
		matmul("NN", n, m, m, 1.0, H, P, 0.0, htp); /* H*P */
		matmul("NN", n, m, n, 1.0, ata, htp, 0.0, qhtp); /* Q-1*(HP) */
		matmul("TN", m, m, n, -1.0, H, qhtp, 1.0, R); /* H'*Q-1*(HP) */
		for (i = 0; i < m; i++) {
			for (k = 0; k < m; k++)
				qvv[k + i * m] = R[k + i * m] * var[i];
		}
	}
	xy_free(P);
	return(info);
}

void statissolsat(const obsd_t* obs, int n, sol_t* sol, int* vsat)
{
	int i = 0;
	int sys = 0;
	for (i = 0; i < 4; i++)
		sol->nsat[i] = 0;
	for (i = 0; i < n; i++)
	{
		if (vsat[i] < 1)
			continue;
		sys = satsys(obs[i].sat, NULL);
		if (sys == SYS_GPS) sol->nsat[0]++;
		if (sys == SYS_GLO) sol->nsat[1]++;
		if (sys == SYS_GAL) sol->nsat[2]++;
		if (sys == SYS_CMP) sol->nsat[3]++;
	}
	sol->ns = sol->nsat[0] + sol->nsat[1] + sol->nsat[2] + sol->nsat[3];
}
/* estimate receiver position ------------------------------------------------*/
static int estpos(const obsd_t* obs, int n, const double* rs, const double* dts,
	const double* vare, const int* svh, const nav_t* nav,
	const prcopt_t* opt, sol_t* sol, double* azel, int* vsat,
	double* resp, char* msg, int beforedect){
	/*初始化变量*/
	double x[NX_SPP] = { 0 }, dx[NX_SPP] = { 0.0 }, Q[NX_SPP * NX_SPP] = { 0.0 }, sig = 1.0;
	int i = 0, j = 0, k = 0, info = 0, stat = 0, nv = 0, ns = 0;
	double v[MAXPNTOBS + 4] = { 0.0 }, var[MAXPNTOBS + 4] = { 0.0 };
	double H[(MAXPNTOBS + 4) * NX_SPP] = { 0.0 };
	int saterrorlist[MAXEORSTA] = { 0 }, dect[MAXEORSTA] = { 0 };
	double dop[4] = { 0 };
	double vv = 0.0, vvp = 0.0;
	double vvf = 0.0;
	sol->stat = SOLQ_NONE;//重置stat标识
	sol->lacksat = 0;

	trace(3, "estpos  :obs n=%d\r\n", n);
	/*!前历元估计量传递*/
	for (i = 0; i < 3; i++) x[i] = sol->rr[i];
	if (beforedect) {//2.系统间偏差及接收机钟差传递
		for (i = 3; i < NX_SPP; i++) x[i] = sol->dtr[i - 3];
	}
	if (!glo_isb_estflag || fabs(x[4]) < 1e-5){
		x[4] = sol->dtr[1];
	}
	if (!gal_isb_estflag || fabs(x[5]) < 1e-5){
		x[5] = sol->dtr[2];
	}
	if (!bds_isb_estflag || fabs(x[6]) < 1e-5){
		x[6] = sol->dtr[3];
	}
	//!最小二乘解算开始
	for (i = 0; i < MAXITR; i++) {

		trace(5, "Pntpos::estpos, spp estpos x[0]=%19.6lf x[1]=%19.6lf x[2]=%19.6lf x[3]=%19.6lf x[4]=%19.6lf x[5]=%19.6lf x[6]=%19.6lf\r\n",
			x[0], x[1], x[2], x[3], x[4], x[5], x[6]);
		nv = rescode(i, obs, n, rs, dts, vare, svh, nav, x, opt, v, H, var, azel, vsat, resp, &ns, sol, saterrorlist, beforedect);
		trace(4, "spp detec v (i=%2d) nv=%2d,before=%d\r\n", i, nv, beforedect); tracemat(5, v, nv, 1, 12, 6);//https://blog.csdn.net/weixin_42109941/article/details/128861403  tracemat详解
		trace(4, "H=\r\n");
		tracemat(5, H, NX_SPP, nv, 12, 6);
		if (nv > NX_SPP) {
			vvf = SQRT((dot(v, v, nv) / (nv - NX_SPP)));
			trace(4, "Pntpos::estpos, spp detec v (i=%2d) nv=%2d,before=%d nv - NX_SPP=%d,vvf=%f\r\n", i, nv, beforedect, nv - NX_SPP, vvf);
			if (beforedect) {
				decpsrerror_add(n, v, vsat, var, vvf);
				decpsrerror(i, beforedect, n, obs, v, vsat, var, dect, saterrorlist);
				if (i == 0)
					memcpy(sol->saterrorlist, saterrorlist, sizeof(int) * MAXEORSTA);
			}
		}else if (nv <= NX_SPP) {
			sol->lacksat = 1;
			trace(3, "lack of valid sats ns=%d\r\n", nv);
			break;
		}
		/* weight by variance */
		for (j = 0; j < nv; j++) {
			if (var[j] < 1e-10)continue;
			sig = sqrt(var[j]);
			v[j] /= sig;
			for (k = 0; k < NX_SPP; k++) H[k + j * NX_SPP] /= sig;
		}
		/* least square estimation */
		if ((info = lsq(H, v, NX_SPP, nv, dx, Q))) {
			sprintf(msg, "lsq error info=%d", info);
			break;
		}
		for (j = 0; j < NX_SPP; j++) x[j] += dx[j];
		/*收敛判断*/
		if (norm(dx, NX_SPP) < 1E-4) {
			vv = dot(v, v, nv);
			vvp = SQRT(dot(v, v, nv) / (nv - NX_SPP));
			trace(4, "pntpos spp residual sum = %6.3lf nv=%d average=%6.3lf vvf=%6.3f\r\n", vv, nv, vvp, vvf);
			/*结果检测*/
			if (((opt->mode > PMODE_DGPS && beforedect) ? (vvp < 2.0 && vvf < 10) : vvp < 500.0) && vv > 0.01) {
				dops(n, azel, opt->elmin, dop, vsat);
				if (obs[0].rcv == 1) {
					for (j = 0; j < 4; j++) sol->rdop[j] = dop[j];
				}
				if (dop[0] <= 0.0 || dop[1] > 6 || dop[0] > opt->maxgdop) {
					trace(4, "gdop error nv=%d gdop=%.1f pdop=%.1f\r\n", nv, dop[0], dop[1]);
					stat = 0;
					sol->stat = SOLQ_NONE;
					return stat;
				}
				/*通过后赋值*/
				stat = 1;
				sol->stat = SOLQ_SINGLE;
				trace(4, "spp successful. residual sum = %6.3lf\r\n", vv);
				statissolsat(obs, n, sol, vsat);//统计参与解算的各个卫星系统卫星个数
				sol->time = obs[0].time;
				sol->dtr[0] = x[3];       /* receiver clock bias (m) */
				if (sol->nsat[1])
					sol->dtr[1] = x[4];       /* glo-gps time offset (m) */
				if (sol->nsat[2])
					sol->dtr[2] = x[5];       /* gal-gps time offset (m) */
				if (sol->nsat[3])
					sol->dtr[3] = x[6];       /* bds-gps time offset (m) */
				for (j = 0; j < 3; j++) sol->rr[j] = j < 3 ? x[j] : 0.0;
				for (j = 0; j < 3; j++) sol->qr[j] = Q[j + j * NX_SPP];
				sol->qr[3] = Q[1];    /* cov xy */
				sol->qr[4] = Q[2 + NX_SPP]; /* cov yz */
				sol->qr[5] = Q[2];    /* cov zx */
				sol->age = sol->ratio = 0.0;
			}else {
				trace(4, "spp failed,residual sum = %6.3lf\r\n", vv);
				sol->stat = SOLQ_NONE;
				stat = 0;
			}
			return stat;
		}
	}
	if (i >= MAXITR){
		trace(4, "iteration divergent i=%d\r\n", i);
		sol->stat = SOLQ_NONE;
	}
	return 0;
}

#if 0
//最小二乘单点增加循环踢星策略
/* estimate receiver position ------------------------------------------------*/
static int estpos2(const obsd_t* obs, int n, const double* rs, const double* dts,
	const double* vare, const int* svh, const nav_t* nav,
	const prcopt_t* opt, sol_t* sol, double* azel, int* vsat,
	double* resp, char* msg, int beforedect)
{
	double x[NX_SPP] = { 0 }, dx[NX_SPP] = { 0.0 }, Q[NX_SPP * NX_SPP] = { 0.0 }, * v = NULL, * H = NULL, * var = NULL, sig = 1.0;
	int i = 0, j = 0, k = 0, info = 0, stat = 0, nv = 0, ns = 0, ns2 = 0;

	int saterrorlist[MAXEORSTA] = { 0 }, dect[MAXEORSTA] = { 0 };
	double dop[4] = { 0 };

	double* qvv = NULL, * R = NULL, dT = 0.0, maxdt = 0.0;
	int m = 0, q = 0, eorsat = 0, num = 0, circul = 0, * vsatc = NULL, * vflag = NULL;

	double vv = 0.0, vvp = 0.0;

	trace(2, "estpos  : n=%d\r\n", n);

	v = mat(n + 4, 1); H = mat(NX_SPP, n + 4); var = mat(n + 4, 1);
	vsatc = izeros(n, 1);
	vflag = izeros(n, 1);

	for (i = 0; i < 3; i++)
		x[i] = sol->rr[i];
	if (beforedect) {
		for (i = 3; i < NX_SPP; i++)
			x[i] = sol->dtr[i - 3];
	}

	for (i = 0; i < n; i++)
		vsatc[i] = 1;

loop:for (i = 0; i < MAXITR; i++) {
	trace(5, "Pntpos::estpos, spp estpos x[0]=%19.6lf x[1]=%19.6lf x[2]=%19.6lf x[3]=%19.6lf x[4]=%19.6lf x[5]=%19.6lf x[6]=%19.6lf\r\n",
		x[0], x[1], x[2], x[3], x[4], x[5], x[6]);//前一历元的结果

	/* pseudorange residuals */
	nv = rescode2(i, obs, n, rs, dts, vare, svh, nav, x, opt, v, H, var, azel, vsat, resp, &ns, sol, vsatc, vflag);
	trace(4, "spp detec v (i=%2d) nv=%2d,before=%d\r\n", i, nv, beforedect);
	tracemat(5, v, nv, 1, 12, 6);

	double vvf = SQRT((dot(v, v, nv) / (nv - NX_SPP)));//单位权中误差
	trace(4, "Pntpos::estpos, spp detec v (i=%2d) nv=%2d,before=%d\r\n", i, nv, beforedect);

	if (beforedect) {
		decpsrerror_add(n, v, vsat, var, vvf);
		decpsrerror(i, beforedect, n, obs, v, vsat, var, dect, saterrorlist);
		if (i == 0)
			memcpy(sol->saterrorlist, saterrorlist, sizeof(int) * MAXEORSTA);
	}

	if (nv <= NX_SPP) {
		trace(2, "lack of valid sats ns=%d\r\n", nv);
		break;
	}
	/* weight by variance */
	for (j = 0; j < nv; j++) {
		sig = sqrt(var[j]);
		v[j] /= sig;
		for (k = 0; k < NX_SPP; k++) H[k + j * NX_SPP] /= sig;
	}
	/* least square estimation */
	if ((info = lsq(H, v, NX_SPP, nv, dx, Q))) {
		sprintf(msg, "lsq error info=%d", info);
		break;
	}
	for (j = 0; j < NX_SPP; j++) x[j] += dx[j];

	if (norm(dx, NX_SPP) < 1E-4) {
		qvv = zeros(nv, nv); R = zeros(nv, nv);
		for (q = 0; q < nv; q++)
			var[q] = 1.0;
		lsqvv(H, var, 4, nv, R, qvv);
		for (m = 0, ns2 = 0; m < n && ns2 < nv; m++) {
			if (!vsat[i]) {
				continue;
			}
			dT = fabs(v[ns2]) / (0.05 * sqrt(qvv[ns2 + ns2 * nv]));//0.05*
			if (dT > 2.5 && maxdt < dT) {
				maxdt = dT; eorsat = ns2;
			}
			ns2++;
		}
		if (qvv) {
			xy_free(qvv);
			qvv = NULL;
		}
		if (R) {
			xy_free(R);
			R = NULL;
		}
		if (maxdt > 1e-6) {
			vsatc[(vflag[eorsat] >> 8) & 0xFF] = 0; num++;
		}
		trace(3, "estpos maxdt = %6.3lf\n", maxdt);
		if (num > 0 && circul < 5 && (nv - 4) >= 3) {
			circul++;
			for (j = 0; j < 3; j++)
				x[j] = sol->rr[j];
			if (beforedect) {
				for (j = 3; j < NX_SPP; j++)
					x[j] = sol->dtr[j - 3];
			}
			else {
				for (j = 3; j < NX_SPP; j++)
					x[j] = 0.0;
			}
			goto loop;
		}

		vv = dot(v, v, nv);
		vvp = SQRT(dot(v, v, nv) / (nv - NX_SPP));//单位权中误差
		trace(4, "pntpos spp residual sum = %6.3lf nv=%d average=%6.3lf\r\n", vv, nv, vvp);

		if (((opt->mode > PMODE_DGPS /*&&beforedect*/) ? vvp < 1.0 : vvp < 500.0) && vv > 0.01) {
			dops(n, azel, opt->elmin, dop, vsat);
			if (obs[0].rcv == 1) {
				for (j = 0; j < 4; j++) sol->rdop[j] = dop[j];
			}
			if (dop[0] <= 0.0 || dop[1] > 6 || dop[0] > opt->maxgdop) {
				trace(2, "gdop error nv=%d gdop=%.1f pdop=%.1f\r\n", nv, dop[0], dop[1]);
				stat = 0;
				sol->stat = SOLQ_NONE;
				xy_free(v); xy_free(H); xy_free(var);
				return stat;
			}
			stat = 1;
			sol->stat = SOLQ_SINGLE;

			trace(2, "spp successful. residual sum = %6.3lf\r\n", vv);

			sol->time = obs[0].time;
			sol->dtr[0] = x[3];       /* receiver clock bias (m) */
			sol->dtr[1] = x[4];       /* glo-gps time offset (m) */
			sol->dtr[2] = x[5];       /* gal-gps time offset (m) */
			sol->dtr[3] = x[6];       /* bds-gps time offset (m) */
			for (j = 0; j < 6; j++) sol->rr[j] = j < 3 ? x[j] : 0.0;
			for (j = 0; j < 3; j++) sol->qr[j] = (float)Q[j + j * NX_SPP];
			sol->qr[3] = (float)Q[1];    /* cov xy */
			sol->qr[4] = (float)Q[2 + NX_SPP]; /* cov yz */
			sol->qr[5] = (float)Q[2];    /* cov zx */
			sol->ns = (unsigned char)ns;
			sol->age = sol->ratio = 0.0;
		}
		else {
			trace(2, "spp failed,residual sum = %6.3lf\r\n", vv);
			sol->stat = SOLQ_NONE;
			stat = 0;
		}
		xy_free(v); xy_free(H); xy_free(var);

		return stat;
	}
}
if (i >= MAXITR)
{
	trace(2, "iteration divergent i=%d\r\n", i);
	sol->stat = SOLQ_NONE;
}

xy_free(v); xy_free(H); xy_free(var);

return 0;
}
#endif

//获取前后历元载波差
int get_pl_dif(const prcopt_t* opt, const obsd_t* obs, const obs_t* preobs, double* dif)
{
	int i = 0;
	double dt = 0.0;
	for (i = 0; i < preobs->n; i++)
	{
		if (obs->sat != preobs->data[i].sat)
			continue;

		if (obs->lli[0])
		{
			return 0;
		}
		else {
			if (fabs(preobs->data[i].L[0]) > 1e-6 && fabs(obs->L[0]) > 1e-6)
			{
				dt = timediff(obs->time, preobs->data[i].time);
				if (fabs(dt) < 1e-6)
					return 0;
				*dif = -(obs->L[0] - preobs->data[i].L[0]) / dt;
				//trace(4,"the L dif of epoch:%s %4d %9.3f %9.3f\n",time_str(obs->time,3),obs->sat,obs->D[f],*dif);
				return 1;
			}
		}
	}
	return 0;
}

/* doppler residuals ---------------------------------------------------------*/
int resdop(int iter, const prcopt_t* opt, const obsd_t* obs, midmsg_t* midmsg,
	int n, const double* rs, const double* dts, const nav_t* nav, const double* rr,
	const double* x, const double* azel, const int* vsatc,
	const ssat_t* ssat, double* v, double* resd, double* var, double* H, int* vflag, double* cjump)
{
	if (opt == NULL || obs == NULL || midmsg == NULL || rs == NULL || dts == NULL || nav == NULL
		|| rr == NULL || x == NULL || azel == NULL || vsatc == NULL || ssat == NULL
		|| v == NULL || resd == NULL || var == NULL || H == NULL || vflag == NULL) {
		return -1;
	}
	double lam = 0.0, rate = 0.0, pos[3] = { 0.0 }, E[9] = { 0.0 }, a[3] = { 0.0 }, e[3] = { 0.0 }, vs[3] = { 0.0 }, cosel = 0.0;
	int sys = 0, i = 0, j = 0, nv = 0;
	double azels[MAXPNTOBS * 2 * DIF_L_IN_EVL] = { 0 }, dop[4] = { 0 };
	double vdop[MAXPNTOBS] = { 0.0 };
	double vdif[MAXPNTOBS] = { 0.0 };
	double dif = 0.0;
	int ndop = 0;
	int ndif = 0;

	ecef2pos(rr, pos);
	xyz2enu(pos, E);
	for (i = 0; i < n && i < MAXPNTOBS; i++)
	{
		sys = satsys(midmsg[i].sat, NULL);
		lam = nav->lam[midmsg[i].sat - 1][0];
		if (obs[i].snr[0] < opt->snrthres)continue;//TODO12
		if (!dopflag(i, midmsg, obs[i].snr + 0, vsatc + i, rs + 3 + i * 6))
			continue;
		/* 只记录可用的i */
		vflag[nv] = (i << 8);//相当于i*2的8次方，i*256
		/* line-of-sight vector in ecef */
		cosel = cos(azel[1 + i * 2]);
		a[0] = sin(azel[i * 2]) * cosel;
		a[1] = cos(azel[i * 2]) * cosel;
		a[2] = sin(azel[1 + i * 2]);
		matmul("TN", 3, 1, 3, 1.0, E, a, 0.0, e);
		/* satellite velocity relative to receiver in ecef */
		for (j = 0; j < 3; j++) vs[j] = rs[j + 3 + i * 6] - x[j];//相对位置
		/* range rate with earth rotation correction */
		rate = dot(vs, e, 3) + OMGE / CLIGHT * (rs[4 + i * 6] * rr[0] + rs[1 + i * 6] * x[0] -
			rs[3 + i * 6] * rr[1] - rs[i * 6] * x[1]);//e单位观测量 dot(vs, e, 3) 考虑了地球自转的用户和卫星之间的几何距离变化率
		v[nv] = -lam * midmsg[i].d1 - (rate + x[3] - CLIGHT * dts[1 + i * 2]);
		
		//前后相位差定速代码
		resd[nv] = v[nv];
		vdop[nv] = v[nv];
		var[nv] = 100.0;

		trace(5, "Pntpos::estpos, Doppler: nv=%2d sat=%2d D=%9.3f v=%19.6f vd_rate=%9.3f\r\n", nv, midmsg[i].sat, midmsg[i].d1, v[nv], dot(vs, e, 3));
		/* design matrix */
		for (j = 0; j < 4; j++)
			H[j + nv * 4] = j < 3 ? -e[j] : 1.0;//更新对应速度的H阵有区别， 下面是新增
		azels[nv * 2] = azel[i * 2];
		azels[1 + nv * 2] = azel[1 + i * 2];
		nv++;
		ndop++;
	}

#  if 0  //暂取消前后相位差进行定速
	double vdop_mean = 0.0;
	for (i = 0; i < ndop; i++){
		vdop_mean += vdop[i];
	}
	vdop_mean /= ndop;

	for (i = 0; i < n && i < MAXPNTOBS; i++){
		sys = satsys(midmsg[i].sat, NULL);
		lam = nav->lam[midmsg[i].sat - 1][0];
		if (obs[i].snr[0] < opt->snrthres)
			continue;//TODO12
		if (!dopflag(i, midmsg, obs[i].snr + 0, vsat + i, vsatc + i, rs + 3 + i * 6))
			continue;
		/* 只记录可用的i */
		vflag[nv] = (i << 8);
		/* line-of-sight vector in ecef */
		cosel = cos(azel[1 + i * 2]);
		a[0] = sin(azel[i * 2]) * cosel;
		a[1] = cos(azel[i * 2]) * cosel;
		a[2] = sin(azel[1 + i * 2]);
		matmul("TN", 3, 1, 3, 1.0, E, a, 0.0, e);
		/* satellite velocity relative to receiver in ecef */
		for (j = 0; j < 3; j++) vs[j] = rs[j + 3 + i * 6] - x[j];
		/* range rate with earth rotation correction */
		rate = dot(vs, e, 3) + OMGE / CLIGHT * (rs[4 + i * 6] * rr[0] + rs[1 + i * 6] * x[0] -
			rs[3 + i * 6] * rr[1] - rs[i * 6] * x[1]);
		dif = 0.0;
		if (!get_pl_dif(opt, obs + i, &pre_obsu, &dif)) {
			continue;
		}
		v[nv] = -lam * dif - (rate + x[3] - CLIGHT * dts[1 + i * 2]);
		v[nv] -= *cjump;
		resd[nv] = v[nv];

		var[nv] = 1;

		vdif[ndif] = v[nv];

		trace(5, "Pntpos::estpos, D %s nv=%2d sat=%2d %s dop=%9.3f dif=%9.3f v=%8.3f dop-dif=%9.3f\r\n", time_str(obs[i].time, 2), nv, midmsg[i].sat, "L", midmsg[i].D1, dif, v[nv], midmsg[i].D1 - dif);
		/* design matrix */
		for (j = 0; j < 4; j++)
			H[j + nv * 4] = j < 3 ? -e[j] : 1.0;
		azels[nv * 2] = azel[i * 2];
		azels[1 + nv * 2] = azel[1 + i * 2];
		nv++;
		ndif++;
	}
	double vdif_mean = 0.0;
	if (ndif > 0)
	{
		for (i = 0; i < ndif; i++)
		{
			vdif_mean += vdif[i];
		}
		vdif_mean /= ndif;
	}
	//相位差钟跳处理
	if (!iter)
	{
		for (i = ndop; i < nv; i++)
		{
			j = (vflag[i] >> 8) & 0xFF;
			if (obs[j].lli[0])
				continue;
			//*cjump = vdif_mean-vdop_mean;   //TODO 此处钟跳修复导致部分动态样本固定率下降15%，此处为风险项暂不开启，暂保留
		}
		trace(4, "clock jump repair:%9.3f %9.3f %9.3f\n", vdop_mean, vdif_mean, vdif_mean - vdop_mean);
	}
#endif
	dops(nv, azels, opt->elmin, dop, NULL);
	if (dop[0] <= 0.0 || dop[1] > 6 || dop[0] > opt->maxgdop){
		trace(3, "estevl failed by dops:nv=%d gdop=%.1f pdop=%.1f\r\n", nv, dop[0], dop[1]);
		nv = 0;
	}
	return nv;
}

int dopfilter(double* H, double* v, double* var, int nv, int* vsatc, int* vflag) {
	int vnv = 0, i = 0, j = 0;
	if (H == NULL || v == NULL || nv == 0) {
		return nv;
	}

	double hh[MAXPNTOBS * DIF_L_IN_EVL * 4] = { 0.0 }, vv[MAXPNTOBS * DIF_L_IN_EVL] = { 0.0 }, qvv[MAXPNTOBS * DIF_L_IN_EVL] = { 0.0 };
	double vvar[MAXPNTOBS * DIF_L_IN_EVL] = { 0.0 };

	for (i = 0; i < nv; i++) {
		qvv[i] = v[i];
	}
	//qsort(qvv, nv, sizeof(double), compare);
	orderdopdif(nv, qvv);

	double mid = qvv[nv / 2];
	for (i = 0; i < nv; i++) {
		if (fabs(v[i] - mid) > 20) {//剔除极端数据
			vsatc[(vflag[i] >> 8) & 0xFF] = 0;
			trace(5, "dopfilter wrong vv = %f\n", v[i]);
		}
		else {
			vv[vnv] = v[i];
			vvar[vnv] = var[i];
			for (j = 0; j < 4; j++) {
				hh[j + vnv * 4] = H[j + i * 4];
			}
			vnv++;
		}
	}
	memcpy(v, vv, (size_t)(sizeof(double) * MAXPNTOBS * DIF_L_IN_EVL));
	memcpy(var, vvar, (size_t)(sizeof(double) * MAXPNTOBS * DIF_L_IN_EVL));
	memcpy(H, hh, (size_t)(sizeof(double) * MAXPNTOBS * DIF_L_IN_EVL * 4));
	return vnv;
}

/* dop filter */
int dopfilter2(double* H, double* v, double* var, int nv, int* vsatc, int* vflag, midmsg_t* midmsg) {
	if (H == NULL || v == NULL || nv == 0) {
		return nv;
	}
	int vnv = 0, i = 0, j = 0;
	double hh[MAXPNTOBS * DIF_L_IN_EVL * 4] = { 0.0 }, vv[MAXPNTOBS * DIF_L_IN_EVL] = { 0.0 }, qvv[MAXPNTOBS * DIF_L_IN_EVL] = { 0.0 };
	double vvar[MAXPNTOBS * DIF_L_IN_EVL] = { 0.0 };
	double dstd = 0.0, stdmin = 0.3, stdmax = 2.0;
	int num = 0;
	for (i = 0; i < nv; i++) {
		j = (vflag[i] >> 8) & 0xFF;
		if (midmsg[j].dpsr1 == 1)
			continue;
		qvv[num++] = v[i];
	}
		//qsort(qvv, nv, sizeof(double), compare);
	orderdopdif(num, qvv);
	double mid = qvv[num / 2];
	for (i = 1; i < num - 1; i++){
		dstd += (qvv[i] - mid) * (qvv[i] - mid);
	}
	if ((num - 2) > 0) {
		dstd = sqrt(dstd / (num - 2));
	}
	if (dstd < stdmin) dstd = stdmin;
	if (dstd > stdmax) dstd = stdmax;
	trace(5, "dopfilter2:mid=%6.3f dstd=%6.3f\n", mid, dstd);
	for (i = 0; i < nv; i++) {
		if (fabs(v[i] - mid) > 3 * dstd) {
			j = (vflag[i] >> 8) & 0xFF;
			vsatc[j] = 0;
			midmsg[j].ddop1 = 1;
			trace(5, "dopfilter wrong vv = %f\n", v[i]);
		}else {
			vv[vnv] = v[i];
			vvar[vnv] = var[i];
			for (j = 0; j < 4; j++) {
				hh[j + vnv * 4] = H[j + i * 4];
			}
			vnv++;
		}
	}
	memcpy(v, vv, (size_t)(sizeof(double) * MAXPNTOBS * DIF_L_IN_EVL));
	memcpy(var, vvar, (size_t)(sizeof(double) * MAXPNTOBS * DIF_L_IN_EVL));
	memcpy(H, hh, (size_t)(sizeof(double) * MAXPNTOBS * DIF_L_IN_EVL * 4));
	return vnv;
}

/* estimate receiver velocity ------------------------------------------------*/
static void estvel(const obsd_t* obs, midmsg_t* midmsg, int n, const double* rs,
	const double* dts, const nav_t* nav, const prcopt_t* opt, sol_t* sol, const double* azel, const ssat_t* ssat)
{
	/*稳健性判断*/
	if (obs == NULL || midmsg == NULL || rs == NULL || dts == NULL || nav == NULL || opt == NULL ||
		sol == NULL || azel == NULL || ssat == NULL) {
		return;
	}
	/*变量初始化*/
	double x[4] = { 0 }, dx[4] = { 0 }, Q[16] = { 0 }, * R = NULL, * qvv = NULL, dtt = 0.0, maxdt = 0.0;//x最小二乘预测4个量，分别为xyz三方向速度加钟漂
	int i = 0, j = 0, m = 0, q = 0, ns = 0, nv = 0, eorsat = 0, num = 0, circul = 0;
	int week = 0;
	double sec = 0.0;
	double cjump = 0.0;
	sec = time2gpst(sol->time, &week);
	sol->vtat = 0;
	double v[MAXPNTOBS * DIF_L_IN_EVL] = { 0.0 }, H[MAXPNTOBS * DIF_L_IN_EVL * 4] = { 0.0 }, var[MAXPNTOBS * DIF_L_IN_EVL] = { 0.0 }, resd[MAXPNTOBS * DIF_L_IN_EVL] = { 0.0 };
	int vflag[MAXPNTOBS * DIF_L_IN_EVL] = { 0 }, vsatc[MAXPNTOBS * DIF_L_IN_EVL] = { 0 };
	double v_t[MAXPNTOBS * DIF_L_IN_EVL] = { 0.0 };
	double sig = 0.0;
	int k = 0;
	double vv = 0.0;
	qvv = zeros(n * DIF_L_IN_EVL, n * DIF_L_IN_EVL); R = zeros(n * DIF_L_IN_EVL, n * DIF_L_IN_EVL);
	for (i = 0; i < n * DIF_L_IN_EVL; i++) {
		vsatc[i] = 1;
	}
	for (i = 0; i < 3; i++) {
		x[i] = sol->rr[i + 3];
	}
	x[3] = sol->dtr[5];//接收机钟漂
/*最小二乘多普勒测速*/
loop:for (i = 0; i < MAXITR; i++){
	num = 0; eorsat = 0; maxdt = 0.0;
	nv = resdop(i, opt, obs, midmsg, n, rs, dts, nav, sol->rr, x, azel, vsatc, ssat, v, resd, var, H, vflag, &cjump);
	if (i == 0) {
		nv = dopfilter2(H, v, var, nv, vsatc, vflag, midmsg);
	}
	if (nv < 4){
		trace(4, "estvel dopres nv=%d <4\r\n", nv);
		break;
	}
#if 0 //暂取消前后相位差进行定速
	for (j = 0; j < nv; j++) {
		sig = sqrt(var[j]);
		v[j] /= sig;
		for (k = 0; k < 4; k++) H[k + j * 4] /= sig;
	}
#endif
	/* least square estimation */
	if (lsq(H, v, 4, nv, dx, Q)) {
		trace(4, "estvel lsq error!\r\n");
		break;
	}
	trace(3, "estvel :%dth nv= %d vel is %f %f %f satclc(Receiver) drift is %f\r\n", i, nv, dx[0], dx[1], dx[2], dx[3]);
	for (j = 0; j < 4; j++)
		x[j] += dx[j];
	/* 收敛判断 */
	if (norm(dx, 4) < 1E-6) {
		memset(qvv, 0, (size_t)sizeof(double) * n * n * DIF_L_IN_EVL * DIF_L_IN_EVL);
		memset(R, 0, (size_t)sizeof(double) * n * n * DIF_L_IN_EVL * DIF_L_IN_EVL);
		for (q = 0; q < nv; q++)
			var[q] = 1.0;//等权
		lsqvv(H, var, 4, nv, R, qvv);
		for (m = 0, ns = 0; m < n * DIF_L_IN_EVL && ns < nv; m++) {
			if (!dopflag(m, midmsg, obs[m].snr + 0, vsatc + m, rs + 3 + m * 6)) {
				continue;
			}
			dtt = fabs(v[ns]) / (0.05 * sqrt(qvv[ns + ns * nv]));
			if (dtt > 2.5 && maxdt < dtt) {
				maxdt = dtt; eorsat = ns;
			}
			ns++;
		}
		if (maxdt > 0.0) {
			j = (vflag[eorsat] >> 8) & 0xFF;
			vsatc[j] = 0;
			//midmsg[j].DDop1=1;
			num++;//检测出的粗差数目
		}
		trace(3, "estvel maxdt = %6.3lf\r\n", maxdt);
		/*有粗差重新初始化，剔除粗差后重新算最小二乘*/
		if (num > 0 && circul < 5 && (nv - 4) >= 3) {
			circul++;
			nv = resdop(i, opt, obs, midmsg, n, rs, dts, nav, sol->rr, x, azel, vsatc, ssat, v_t, resd, var, H, vflag, &cjump);
			if (nv >= 4){
				cjump = 0.0;
				for (j = 0; j < 3; j++)
					x[j] = sol->rr[j + 3];
				x[3] = sol->dtr[5];
				goto loop;
			}
		}
		/*没粗差的话测速更新*/
		vv = dot(v, v, ns) / (ns - 4);
		if (vv < 1.0){
			trace(4, "estvel successful vv=%f maxdt=%f\r\n", vv, maxdt);
			sol->vtat = 1;
			for (q = 0; q < 3; q++)
				sol->rr[q + 3] = x[q];
			sol->dtr[5] = x[3];
			trace(4, "final vel is %4.3f %4.3f %4.3f clock drift is %4.3f \r\n", x[0], x[1], x[2], x[3]);
		}else {
			trace(4, "estvel failure vv=%f maxdt=%f\r\n", vv, maxdt);
			sol->vtat = 0;
		}
		break;
	}
}
xy_free(qvv);
xy_free(R);
}

/* store mid message */
void storemidmsg(const int rcv, const obsd_t* obs, const nav_t* navcur,
	double* rs, double* dts, double* azel, int n, int* svh, int* satlist, midmsg_t* midmsg)
{
	if (obs == NULL || navcur == NULL || rs == NULL || dts == NULL || azel == NULL ||
		svh == NULL || satlist == NULL || midmsg == NULL) {
		return;
	}
	int i = 0, j = 0, sys = 0, f = 0;
	memset(midmsg, 0, (size_t)sizeof(midmsg_t) * MAXDIFOBS * 2);

	for (i = 0; i < n; i++) {
		sys = satsys(obs[i].sat, NULL);
		midmsg[i].sat = obs[i].sat;
		midmsg[i].d1 = obs[i].D[0];
		midmsg[i].dpsr1 = 0;
		midmsg[i].ddop1 = 0;

		for (j = 0; j < NFREQ; j++)
		{
			midmsg[i].dpsr2[j] = 0;
			midmsg[i].dlsr2[j] = 0;
			midmsg[i].ddop2[j] = 0;
		}

		for (j = 0; j < MAXEORSTA; j++) {
			if (!satlist[j]) continue;
			if (satlist[j] == obs[i].sat) midmsg[i].dpsr1 = 1;
		}
	}
}
void obsdect(int nu, int nr, const obsd_t* obs, rtk_t* rtk) {
	int i = 0, j = 0, k = 0;
	if (nr < 1 || nu < 1)
		return;
	double dts = fabs(timediff(obs[0].time, obs[nu].time));
	double dis = 0;
	for (i = 0; i < nu; i++) {
		for (j = 0; j < nr; j++) {
			if ((obs[i].sat == obs[j + nu].sat) && (dts < 5.0))
			{
				dis = fabs(obs[i].P[0] - obs[j + nu].P[0]);
				trace(4, "obsdect: %s sat:%d obs diff is %f! %16.3f %16.3f\n", time_str(obs[0].time, 2), obs[i].sat, dis, obs[i].P[0], obs[j + nu].P[0]);
				if (dis > 290000.0)
				{
					if (k >= MAXEORSTA) {
						trace(4, "the num of bad sat is too large\n");
						continue;
					}
					rtk->opt.saterrorlist_dect[k] = obs[i].sat;
					trace(4, "obsdect: %s sat:%d is dect!\n", time_str(obs[0].time, 2), obs[i].sat);
					k++;
				}
			}
		}
	}
}
void jdgestibs(rtk_t* rtk, sol_t* sol, const obsd_t* obs, int n)
{
	/*变量初始化*/
	double dtglo = 0;//距离上次glo系统间偏差估计时差
	double dtgal = 0;
	double dtbds = 0;
	int ngps = 0;
	int nglo = 0;
	int ngal = 0;
	int nbds = 0;
	int sys = 0;
	/*系统内卫星数记录*/
	for (int i = 0; i < n; i++)  {//SNR大于35且不为GEO的卫星数
		if ((sys = limitcmpgeo(obs[i].sat)) <= SYS_NONE)
			continue;
		if (obs[i].snr[0] * 0.25 < 35)
			continue;
		if (sys == SYS_GPS)ngps++;
		if (sys == SYS_GLO)nglo++;
		if (sys == SYS_GAL)ngal++;
		if (sys == SYS_CMP)nbds++;
	}

	trace(4, "jdgestibs：ngps=%4d nglo=%4d ngal=%4d nbds=%4d\n", ngps, nglo, ngal, nbds);
	if (rtk->glo_isb.time.time)
		dtglo = fabs(timediff(obs[0].time, rtk->glo_isb.time));
	if ((ngps < 2 || nglo < 3) && dtglo > 0 && dtglo < (60 * 30)){
		glo_isb_estflag = 0;
		sol->dtr[1] = rtk->glo_isb.isb;
		trace(4, "jdgestibs：GLO pnt dont estimat isb:dt=%4f nglo=%4d glo_gps=%6.3f\n", dtglo, nglo, sol->dtr[1]);
	}else {
		glo_isb_estflag = 1;
		trace(4, "jdgestibs：GLO pnt need estimat isb: dt=%4f nglo=%4d\n", dtglo, nglo);
	}
	if (rtk->gal_isb.time.time)
		dtgal = fabs(timediff(obs[0].time, rtk->gal_isb.time));
	if ((ngps < 2 || ngal < 3) && dtgal > 0 && dtgal < (60 * 30)){
		gal_isb_estflag = 0;
		sol->dtr[2] = rtk->gal_isb.isb;
		trace(4, "Gal pnt dont estimat isb:dt=%4f ngal=%4d gal_gps=%6.3f\n", dtgal, ngal, sol->dtr[2]);
	}else {
		gal_isb_estflag = 1;
		trace(4, "GAL pnt need estimat isb: dt=%4f ngal=%4d\n", dtgal, ngal);
	}
	if (rtk->bds_isb.time.time)
		dtbds = fabs(timediff(obs[0].time, rtk->bds_isb.time));
	if ((ngps < 2 || nbds < 3) && dtbds > 0 && dtbds < (60 * 30)){
		bds_isb_estflag = 0;
		sol->dtr[3] = rtk->bds_isb.isb;
		trace(4, "BDS pnt dont estimat isb:dt=%4f nbds=%4d bds_gps=%6.3f\n", dtbds, nbds, sol->dtr[3]);
	}else {
		bds_isb_estflag = 1;
		trace(4, "BDS pnt need estimat isb: dt=%4f nbds=%4d\n", dtbds, nbds);
	}
}

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
extern int pntpos(rtk_t* rtk, const obsd_t* obs, int n, const nav_t* nav,
	const prcopt_t* opt, midmsg_t* midmsg, sol_t* sol, double* azel, ssat_t* ssat, int beforedect, int vof)
{
	/*变量初始化*/
	prcopt_t opt_ = *opt;
	int i = 0, stat = 0, vsat[MAXPNTOBS] = { 0 }, svh[MAXPNTOBS] = { 0 };
	double rs[6 * MAXPNTOBS] = { 0.0 }, dts[2 * MAXPNTOBS] = { 0.0 }, var[MAXPNTOBS] = { 0.0 }, azel_[2 * MAXPNTOBS] = { 0.0 }, resp[MAXPNTOBS] = { 0.0 };
	sol->lackflag = 0;
	sol->lacksat = 0;
	sol->stat = SOLQ_NONE;
	if (beforedect)
		jdgestibs(rtk, sol, obs, n);
	trace(2, "pntpos  : tobs=%s n_sat=%d beforedect=%d\r\n", time_str(obs[0].time, 3), n, beforedect);
	/*对流层电离层处理模式*/
	if (opt_.mode != PMODE_SINGLE) { /* for precise positioning */
		opt_.ionoopt = IONOOPT_BRDC;
		opt_.tropopt = TROPOPT_SAAS;
	}
	/* satellite positons, velocities and clocks */
	satposs(obs[0].time, obs, n, nav, opt_.sateph, rs, dts, var, svh, MAXPNTOBS);
	/* estimate receiver position with pseudorange */
	stat = estpos(obs, n, rs, dts, var, svh, nav, &opt_, sol, azel_, vsat, resp, NULL, beforedect);
	/* store midmsg */
	if (midmsg)
		storemidmsg(1, obs, nav, rs, dts, azel_, n, svh, sol->saterrorlist, midmsg);
	/* estimate receiver velocity with doppler */
	if (vof) {
		if (midmsg)
			estvel(obs, midmsg, n, rs, dts, nav, &opt_, sol, azel_, ssat);
	}
	if (azel) {
		for (i = 0; i < n * 2; i++) azel[i] = azel_[i];
	}
	return stat;
}

int udpos_pnt(rtk_t* rtk, double tt, double st) {
	if (rtk == NULL) {
		return -1;
	}
	int stt = 20, sff = 5;
	double* F = NULL, * fp = NULL, * xp = NULL, Q[9] = { 0 }, qv[9] = { 0 }, qac[16] = { 0 }, pos[3] = { 0.0 };
	int i = 0, j = 0;
	trace(3, "pntpos_kalman: tt=%6.3f tt=%6.3f\r\n", tt, st);
	/*initialization*/
	if (norm(rtk->x_s, 3) <= 0.0 || tt > 5 || st > 10 || !rtk->sol.vtat) {
		if (rtk->sol.stat == SOLQ_NONE)
			return 1;//要最小二乘法有解才能作为初始解赋值
		if (rtk->sol.vtat == 0) {
			for (i = 0; i < 3; i++) {
				rtk->sol.rr[i + 3] = 0.0001;
			}
		}
		for (i = 0; i < 3; i++) {
			if (rtk->sol.rr[i + 3] == 0.0)
				rtk->sol.rr[i + 3] = 0.0001;
			rtk->x_s[i] = rtk->sol.rr[i];
			rtk->x_s[i + 3] = rtk->sol.rr[i + 3];
		}
		rtk->x_s[6] = rtk->sol.dtr[0];        /*receiver clock bias(m)*/
		rtk->x_s[7] = rtk->sol.dtr[1];        /*glo-gps time offset(m)*/
		rtk->x_s[8] = rtk->sol.dtr[3];        /*bds-gps time offset(m)*/
		rtk->x_s[9] = rtk->sol.dtr[5];        /*receiver clock shift*/

		for (i = 0; i < 10; i++) {
			for (j = 0; j < 10; j++) {
				if (i < 3) {
					rtk->p_s[i + j * NS] = rtk->p_s[j + i * NS] = i == j ? VAR_POS_B : 0.0;
				}
				else if (i < 6) {
					rtk->p_s[i + j * NS] = rtk->p_s[j + i * NS] = i == j ? VAR_VEL_B : 0.0;
				}
				else if (i < 9) {
					rtk->p_s[i + j * NS] = rtk->p_s[j + i * NS] = i == j ? VAR_CLK_B : 0.0;
				}
				else {
					rtk->p_s[i + j * NS] = rtk->p_s[j + i * NS] = i == j ? VAR_SHIFT_B : 0.0;
				}
			}
		}
		return 0;
	}
	if (rtk->x_s[7] == 0)
		rtk->x_s[7] = rtk->sol.dtr[1];//最小二乘估计的
	if (rtk->x_s[8] == 0)
		rtk->x_s[8] = rtk->sol.dtr[3];

	if (rtk->opt.mode > PMODE_DGPS && (rtk->sol_last.stat == SOLQ_FIX || rtk->sol_last.stat == SOLQ_FLOAT || rtk->sol_last.stat == SOLQ_DGPS)) {
		for (i = 0; i < 6; i++) {
			rtk->x_s[i] = rtk->sol_last.rr[i];
			tt = timediff(rtk->sol.time, rtk->sol_last.time);
		}
	}
	if (rtk->sol.vtat /*&& !rtk->sol.maxdTflag*/) 
	{
		for (i = 0; i < 3; i++)
			rtk->x_s[i + 3] = rtk->sol.rr[i + 3];
	}

	/*state transition*/
	F = eye(NS);
	fp = mat(NS, NS);
	xp = mat(NS, 1);
	for (i = 0; i < 3; i++)
		F[i + (i + 3) * NS] = tt;
	/*x=F*x,P=F*P*F+Q加的Q在后面加的*/
	matmul("NN", NS, 1, NS, 1.0, F, rtk->x_s, 0.0, xp);
	matmul("NN", NS, NS, NS, 1.0, F, rtk->p_s, 0.0, fp);
	matmul("NT", NS, NS, NS, 1.0, fp, F, 0.0, rtk->p_s);
	matcpy(rtk->x_s, xp, NS, 1);

	/*process noise matrix*/
	for (i = 0; i < 3; i++) {//对接收机钟差、GLO\GLA\BDS、接收机钟漂加入过程噪声，可能不对
		for (j = 0; j < 3; j++) {
			qac[i + j * 4] = i == j ? stt * tt + sff * pow(tt, 3) / 3 : stt * tt;
		}
		qac[i + 3 * 4] = qac[3 + i * 4] = sff * pow(tt, 2) / 2;
	}
	qac[3 + 3 * 4] = sff * tt;
	Q[0] = Q[4] = SQR(0.5 * tt); Q[8] = SQR(0.05 * tt);
	ecef2pos(rtk->x_s, pos);
	covecef(pos, Q, qv);
	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			rtk->p_s[i + 3 + (j + 3) * NS] += qv[i + j * 3];
		}
	}
	for (i = 0; i < 4; i++) {
		for (j = 0; j < 4; j++) {
			rtk->p_s[i + 6 + (j + 6) * NS] += qac[i + j * 4];//对角元素赋值，接收机钟差、GLO\GLA\BDS、接收机钟漂
		}
	}
	xy_free(F); xy_free(fp); xy_free(xp);
	return 0;
}
int udpos_pnt2(rtk_t* rtk, double tt, double st, double lt, int flag) {
	/*稳健性判断*/
	if (rtk == NULL) {
		return -1;
	}
	/*变量初始化*/
	int stt = 20, sff = 5;
	double Q[9] = { 0 }, qv[9] = { 0 }, qac[25] = { 0 }, pos[3] = { 0.0 };
	double F[NS * NS] = { 0.0 }, fp[NS * NS] = { 0.0 }, xp[NS] = { 0.0 };
	int i = 0, j = 0;
	rtk->sol_sppk.resetflag = 0;
	trace(3, "pntpos_kalman: pntdt_tt=%6.3f kalmdt_st=%6.3f udpos_x=%f %f %f\r\n", tt, st,rtk->x_s[3], rtk->x_s[4], rtk->x_s[5]);
	/**时间更新判断是否需要初始化*/
	if (flag || norm(rtk->x_s, 3) <= 0.0 || tt > 5 || st > 10 || !rtk->sol.vtat || lt > 5) {
		trace(3, "init using normal method\n");
		rtk->sol_sppk.resetflag = 1;
		if (rtk->sol.stat == SOLQ_NONE) {
			trace(3, "init using normal method failed because SOLQ_NONE\n");
			return 1;
		}
		/*速度位置初始化：多普勒测速失败则速度初始化极小值，位置用本次单点解的位置*/
		if (rtk->sol.vtat == 0) {
			for (i = 0; i < 3; i++) {
				rtk->sol.rr[i + 3] = 0.0001;
			}
		}
		for (i = 0; i < 3; i++) {
			if (rtk->sol.rr[i + 3] == 0.0)
				rtk->sol.rr[i + 3] = 0.0001;
			rtk->x_s[i] = rtk->sol.rr[i];//状态量初始化为上一次的
			rtk->x_s[i + 3] = rtk->sol.rr[i + 3];
		}
		trace(3, "after timeinit pnt2.the init pos is %f %f %f %f %f %f \n", rtk->x_s[0], rtk->x_s[1], rtk->x_s[2], rtk->x_s[3], rtk->x_s[4], rtk->x_s[5]);
		/*钟差、系统时间偏差、钟漂初始化*/
		rtk->x_s[6] = rtk->sol.dtr[0];        /*receiver clock bias(m)*/
		rtk->x_s[7] = rtk->sol.dtr[1];        /*glo-gps time offset(m)*/
		rtk->x_s[8] = rtk->sol.dtr[2];        /*gal-gps time offset(m)*/
		rtk->x_s[9] = rtk->sol.dtr[3];        /*bds-gps time offset(m)*/
		rtk->x_s[10]= rtk->sol.dtr[5];        /*receiver clock shift*/
		/*相应的方差阵初始化*/
		for (i = 0; i < 11; i++) {
			for (j = 0; j < 11; j++) {
				if (i < 3) {
					if (i == j) {
						rtk->p_s[i + j * NS] = VAR_POS_B;
						rtk->p_s[j + i * NS] = VAR_POS_B;
					}else {
						rtk->p_s[i + j * NS] = 0.0;
						rtk->p_s[j + i * NS] = 0.0;
					}
				}else if (i < 6) {
					if (i == j) {
						rtk->p_s[i + j * NS] = VAR_VEL_B;
						rtk->p_s[j + i * NS] = VAR_VEL_B;
					}else {
						rtk->p_s[i + j * NS] = 0.0;
						rtk->p_s[j + i * NS] = 0.0;
					}
				}else if (i < 10) {
					if (i == j) {
						rtk->p_s[i + j * NS] = VAR_CLK_B;
						rtk->p_s[j + i * NS] = VAR_CLK_B;
					}else {
						rtk->p_s[i + j * NS] = 0.0;
						rtk->p_s[j + i * NS] = 0.0;
					}
				}else {
					if (i == j) {
						rtk->p_s[i + j * NS] = VAR_SHIFT_B;
						rtk->p_s[j + i * NS] = VAR_SHIFT_B;
					}else {
						rtk->p_s[i + j * NS] = 0.0;
						rtk->p_s[j + i * NS] = 0.0;
					}
				}
			}
		}
		return 0;
	}
	/*三大系统间偏差初始化*/
	if (rtk->x_s[7] == 0.0)
		rtk->x_s[7] = rtk->sol.dtr[1];
	if (rtk->x_s[8] == 0.0)
		rtk->x_s[8] = rtk->sol.dtr[2];
	if (rtk->x_s[9] == 0.0)
		rtk->x_s[9] = rtk->sol.dtr[3];
	if (rtk->sol.nsat[0] && !rtk->sol_last.nsat[0]){
		rtk->x_s[7] = rtk->sol.dtr[1];
		rtk->x_s[8] = rtk->sol.dtr[2];
		rtk->x_s[9] = rtk->sol.dtr[3];
	}
	if ((rtk->sol.stat == SOLQ_SINGLE) && (fabs(rtk->sol.dtr[0]) > 299 || fabs(rtk->x_s[6]) > 299)){//根据接收机钟差和
		trace(3, "in here\n");
		rtk->x_s[6] = rtk->sol.dtr[0];        /*receiver clock bias(m)*/
		rtk->x_s[7] = rtk->sol.dtr[1];        /*glo-gps time offset(m)*/
		rtk->x_s[8] = rtk->sol.dtr[2];        /*gal-gps time offset(m)*/
		rtk->x_s[9] = rtk->sol.dtr[3];        /*bds-gps time offset(m)*/
		rtk->x_s[10] = rtk->sol.dtr[5];        /*receiver clock shift*/
	}
	
	/**时间更新*/
	if (rtk->opt.mode > PMODE_DGPS && (rtk->sol_last.stat == SOLQ_FIX || rtk->sol_last.stat == SOLQ_FLOAT || rtk->sol_last.stat == SOLQ_DGPS) && lt <= 5) {
		trace(3, "udpos_pnt2::using FIX FLOAT RTD init sol x_s last time is %s sol time is %s last flag is%d\n", time_str(rtk->sol_last.time, 3), time_str(rtk->sol.time, 3), rtk->sol_last.stat);
		for (i = 0; i < 6; i++) {
			rtk->x_s[i] = rtk->sol_last.rr[i];
		}
		tt = timediff(rtk->sol.time, rtk->sol_last.time);
		trace(3, "lt=%f tt=%f\n", lt, tt);
	}
	if (rtk->sol.vtat) {//TODO 速度处理，不稳健
		trace(3, "init spped\n");
		if (tt < 3){
			for (i = 0; i < 3; i++)
				rtk->x_s[i + 3] = (rtk->sol.rr[i + 3] + rtk->x_s[i + 3]) / 2;//上次量测与这次单点取均值
		}else{
			for (i = 0; i < 3; i++)
				rtk->x_s[i + 3] = rtk->sol.rr[i + 3];
		}
	}
	trace(3, "after init.the cor  is %f %f %f %f %f %f \n", rtk->x_s[0], rtk->x_s[1], rtk->x_s[2], rtk->x_s[3], rtk->x_s[4], rtk->x_s[5]);
	trace(3, "after init.the cor2 is %f %f %f %f %f \n", rtk->x_s[6], rtk->x_s[7], rtk->x_s[8], rtk->x_s[9], rtk->x_s[10]);
	
	/**state transition*/
	for (i = 0; i < NS; i++)
		F[i + i * NS] = 1.0;
	for (i = 0; i < 3; i++)
		F[i + (i + 3) * NS] = tt;
	/** x=F*x,P=F*P*FT+Q 暂时未加Q下面加了*/
	matmul("NN", NS, 1, NS, 1.0, F, rtk->x_s, 0.0, xp);
	matmul("NN", NS, NS, NS, 1.0, F, rtk->p_s, 0.0, fp);
	matmul("NT", NS, NS, NS, 1.0, fp, F, 0.0, rtk->p_s);
	matcpy(rtk->x_s, xp, NS, 1);
	/**process noise matrix*/
	for (i = 0; i < 4; i++) {
		for (j = 0; j < 4; j++) {
			if (i == j) {
				qac[i + j * 5] = stt * tt + sff * pow(tt, 3) / 3;
			}else {
				qac[i + j * 5] = stt * tt;//非对角阵设置过程噪声  20*解遇上个历元解时间差
			}
		}
		/*设置接收机钟漂非对角阵过程噪声Q阵*/
		qac[i + 4 * 5] = sff * pow(tt, 2) / 2;
		qac[4 + i * 5] = sff * pow(tt, 2) / 2;
	}
	qac[4 + 4 * 5] = sff * tt;
	/*速度设置过程噪声*/
	Q[0] = SQR(0.5 * tt);
	Q[4] = SQR(0.5 * tt);
	Q[8] = SQR(0.05 * tt);
	ecef2pos(rtk->x_s, pos);
	covecef(pos, Q, qv);
	/*放入过程噪声阵*/
	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			rtk->p_s[i + 3 + (j + 3) * NS] += qv[i + j * 3];/*速度设置过程噪声*/
		}
	}
	for (i = 0; i < 5; i++) {
		for (j = 0; j < 5; j++) {
			rtk->p_s[i + 6 + (j + 6) * NS] += qac[i + j * 5];
		}
	}
	return 0;
}

//调整钟差系数随定位状态进行更新，测试效果一般，暂不开启，但代码建议先保留
int udpos_pnt3(rtk_t* rtk, double tt, double st, double lt) {
	if (rtk == NULL) {
		return -1;
	}
	int stt = 20, sff = 5;
	double Q[9] = { 0 }, qv[9] = { 0 }, qac[25] = { 0 }, pos[3] = { 0.0 };
	double F[NS * NS] = { 0.0 }, fp[NS * NS] = { 0.0 }, xp[NS] = { 0.0 };
	int i = 0, j = 0;
	trace(3, "pntpos_kalman: tt=%6.3f st=%6.3f\r\n", tt, st);
	/*initialization*/
	if (norm(rtk->x_s, 3) <= 0.0 || tt > 5 || st > 10 || !rtk->sol.vtat || lt > 5) {
		trace(3, "init using normal method\n");
		if (rtk->sol.stat == SOLQ_NONE)
			return 1;
		if (rtk->sol.vtat == 0) {
			for (i = 0; i < 3; i++) {
				rtk->sol.rr[i + 3] = 0.0001;
			}
		}
		for (i = 0; i < 3; i++) {
			if (rtk->sol.rr[i + 3] == 0.0)
				rtk->sol.rr[i + 3] = 0.0001;
			rtk->x_s[i] = rtk->sol.rr[i];
			rtk->x_s[i + 3] = rtk->sol.rr[i + 3];
		}
		trace(3, "after init.the init pos is %f %f %f %f %f %f \n", rtk->x_s[0], rtk->x_s[1], rtk->x_s[2], rtk->x_s[3], rtk->x_s[4], rtk->x_s[5]);
		rtk->x_s[6] = rtk->sol.dtr[0];        /*receiver clock bias(m)*/
		rtk->x_s[7] = rtk->sol.dtr[1];        /*glo-gps time offset(m)*/
		rtk->x_s[8] = rtk->sol.dtr[2];        /*gal-gps time offset(m)*/
		rtk->x_s[9] = rtk->sol.dtr[3];        /*bds-gps time offset(m)*/
		rtk->x_s[10] = rtk->sol.dtr[5];       /*receiver clock shift*/

		for (i = 0; i < 11; i++) {
			for (j = 0; j < 11; j++) {
				if (i < 3) {
					if (i == j) {
						rtk->p_s[i + j * NS] = VAR_POS_B;
						rtk->p_s[j + i * NS] = VAR_POS_B;
					}
					else {
						rtk->p_s[i + j * NS] = 0.0;
						rtk->p_s[j + i * NS] = 0.0;
					}
				}
				else if (i < 6) {
					if (i == j) {
						rtk->p_s[i + j * NS] = VAR_VEL_B;
						rtk->p_s[j + i * NS] = VAR_VEL_B;
					}
					else {
						rtk->p_s[i + j * NS] = 0.0;
						rtk->p_s[j + i * NS] = 0.0;
					}
				}
				else if (i < 10) {
					if (i == j) {
						rtk->p_s[i + j * NS] = VAR_CLK_B;
						rtk->p_s[j + i * NS] = VAR_CLK_B;
					}
					else {
						rtk->p_s[i + j * NS] = 0.0;
						rtk->p_s[j + i * NS] = 0.0;
					}
				}
				else {
					if (i == j) {
						rtk->p_s[i + j * NS] = VAR_SHIFT_B;
						rtk->p_s[j + i * NS] = VAR_SHIFT_B;
					}
					else {
						rtk->p_s[i + j * NS] = 0.0;
						rtk->p_s[j + i * NS] = 0.0;
					}
				}
			}
		}
		return 0;
	}
	if (rtk->x_s[7] == 0.0)
		rtk->x_s[7] = rtk->sol.dtr[1];
	if (rtk->x_s[8] == 0.0)
		rtk->x_s[8] = rtk->sol.dtr[2];
	if (rtk->x_s[9] == 0.0)
		rtk->x_s[9] = rtk->sol.dtr[3];

	if (rtk->opt.mode > PMODE_DGPS && (rtk->sol_last.stat == SOLQ_FIX || rtk->sol_last.stat == SOLQ_FLOAT || rtk->sol_last.stat == SOLQ_DGPS) && lt <= 5) {
		trace(3, "using FIX FLOAT RTD init sol x_s last time is %s sol time is %s last flag is%d\n", time_str(rtk->sol_last.time, 3), time_str(rtk->sol.time, 3), rtk->sol_last.stat);
		for (i = 0; i < 6; i++) {
			rtk->x_s[i] = rtk->sol_last.rr[i];
		}
		tt = timediff(rtk->sol.time, rtk->sol_last.time);
		trace(3, "lt=%f tt=%f\n", lt, tt);
	}

	if (rtk->sol.vtat)   //TODO
	{
		rtk->x[10] = rtk->sol.dtr[5];
		trace(3, "init spped\n");
		for (i = 0; i < 3; i++)
			rtk->x_s[i + 3] = rtk->sol.rr[i + 3];
	}

	if (fabs(rtk->sol.dtr[0] - rtk->x_s[6]) > 1e5) //钟跳判断
	{
		trace(3, "clock jump in here\n");
		rtk->x_s[6] = rtk->sol.dtr[0];        /*receiver clock bias(m)*/
		rtk->x_s[7] = rtk->sol.dtr[1];        /*glo-gps time offset(m)*/
		rtk->x_s[8] = rtk->sol.dtr[2];        /*glo-gps time offset(m)*/
		rtk->x_s[9] = rtk->sol.dtr[3];        /*bds-gps time offset(m)*/
	}
	else
	{
		rtk->x_s[6] += rtk->x_s[10] * tt;
	}

	trace(3, "after init.the cor is %f %f %f %f %f %f \n", rtk->x_s[0], rtk->x_s[1], rtk->x_s[2], rtk->x_s[3], rtk->x_s[4], rtk->x_s[5]);

	/*state transition*/
	for (i = 0; i < NS; i++)
		F[i + i * NS] = 1.0;
	for (i = 0; i < 3; i++)
		F[i + (i + 3) * NS] = tt;
	/*x=F*x,P=F*P*F+Q*/
	matmul("NN", NS, 1, NS, 1.0, F, rtk->x_s, 0.0, xp);
	matmul("NN", NS, NS, NS, 1.0, F, rtk->p_s, 0.0, fp);
	matmul("NT", NS, NS, NS, 1.0, fp, F, 0.0, rtk->p_s);
	matcpy(rtk->x_s, xp, NS, 1);

	/*process noise matrix*/
	for (i = 0; i < 4; i++) {
		for (j = 0; j < 4; j++) {
			if (i == j) {
				qac[i + j * 5] = stt * tt + sff * pow(tt, 3) / 3;
			}
			else {
				qac[i + j * 5] = stt * tt;
			}
		}
		qac[i + 4 * 5] = sff * pow(tt, 2) / 2;
		qac[4 + i * 5] = sff * pow(tt, 2) / 2;
	}
	qac[4 + 4 * 5] = sff * tt;
	Q[0] = SQR(0.5 * tt);
	Q[4] = SQR(0.5 * tt);
	Q[8] = SQR(0.05 * tt);
	ecef2pos(rtk->x_s, pos);
	covecef(pos, Q, qv);
	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			rtk->p_s[i + 3 + (j + 3) * NS] += qv[i + j * 3];
		}
	}
	for (i = 0; i < 5; i++) {
		for (j = 0; j < 5; j++) {
			rtk->p_s[i + 6 + (j + 6) * NS] += qac[i + j * 5];
		}
	}
	return 0;
}
int var_update(double* rrj, double* v, int npse, int ndop, int* prvpos, int* dopvpos, midmsg_t* midmsg, int* vsat) {
	if (rrj == NULL || v == NULL || prvpos == NULL || dopvpos == NULL || midmsg == NULL) {
		return 0;
	}
	int i = 0, j = 0;
	double vpr[MAXPNTOBS] = { 0.0 }, vdop[MAXPNTOBS] = { 0.0 };
	double prvvmid = 0.0, prvstd = 0.0, prvmin = 6.0, prvmax = 15.0, prvthre = 0.0;
	double dopvvmid = 0.0, dopvstd = 0.0, dopmin = 1.0, dopvmax = 6.0, dopvthre = 0.0;
	for (i = 0; i < npse; i++) 
		vpr[i] = v[i];
	for (i = 0; i < ndop; i++) 
		vdop[i] = v[i + npse];
	if (npse >= 5) {
		orderdopdif(npse, vpr);
		prvvmid = npse % 2 == 1 ? vpr[npse / 2] : vpr[npse / 2 - 1];//todo 取绝对值的中位数更好
		if (prvvmid <= 5) {
			prvmax = 15.0;
		}
		else if (prvvmid <= 10) {
			prvmax = 20.0;
		}
		else if (prvvmid <= 20) {
			prvmax = 30.0;
		}
		else {
			prvmax = prvvmid + 10;
		}
		j = 0;
		for (i = 1; i < npse - 1; i++) {//程序已经去除掉排序后的两端极大极小值了,为了求的阈值不被影响
			if (fabs(vpr[i] - prvvmid) < prvmax) {//若未有伪距残差的异常点出现
				prvstd = prvstd + (vpr[i] - prvvmid) * (vpr[i] - prvvmid);
				j++;
			}
		}
		if (j > 0)
			prvstd = sqrt(prvstd / j);//求出所有正常伪距残差的标准差
		prvthre = prvstd;
		if (prvthre < prvmin)prvthre = prvmin;
	}
	if (ndop >= 5) {
		orderdopdif(ndop, vdop);
		dopvvmid = ndop % 2 == 1 ? vdop[ndop / 2] : vdop[ndop / 2 - 1];
		j = 0;
		for (i = 1; i < ndop - 1; i++) {
			if (fabs(vdop[i] - dopvvmid) < dopvmax) {
				dopvstd = dopvstd + (vdop[i] - dopvvmid) * (vdop[i] - dopvvmid);
				j++;
			}
		}
		if (j > 0)
			dopvstd = sqrt(dopvstd / j);
		dopvthre = dopvstd;
		if (dopvthre < dopmin)dopvthre = dopmin;
	}
	trace(5, "var_update::prvstd=%f prvvmid=%f dopvstd=%f dopvvmid=%f\n", prvstd, prvvmid, dopvstd, dopvvmid);
	for (i = 0; i < npse; i++) {
		if (fabs(v[i] - prvvmid) > 3 * prvthre && prvthre != 0) {
			rrj[i] = fabs(v[i]) * 1e6;
			midmsg[prvpos[i]].dpsr1 = 1;
			vsat[i] = -1;
		}
	}
	for (i = npse; i < npse + ndop; i++) {
		if (fabs(v[i] - dopvvmid) > 3 * dopvthre && dopvthre != 0) {
			rrj[i] = fabs(v[i]) * 1e6;
			midmsg[dopvpos[i - npse]].ddop1 = 1;
		}
	}
	if (dopvstd > 2.0 && dopvvmid > 10.0)
		return 1;

	return 0;
}

int zdpse(rtk_t* rtk, const obsd_t* obs, int ns, double* rs, double* dts, double* var, int* svh, const nav_t* nav, const prcopt_t* opt,
	midmsg_t* midmsg, double* azel, int vtat, double* xp, double* v, double* H, double* R, int* vsat) {
	/*稳健性判断*/
	if (obs == NULL || rs == NULL || dts == NULL || svh == NULL || nav == NULL || opt == NULL || midmsg == NULL || azel == NULL ||
		xp == NULL || v == NULL || H == NULL || R == NULL) {
		return -1;
	}
	double r = 0, dion = 0, dtrp = 0, vmeas = 0, vion = 0, vtrp = 0, rr[3] = { 0 }, pos[3] = { 0 }, e[3] = { 0 }, dtr = 0, P = 0, lam_L1 = 0, vs[3] = { 0 }, rate = 0, lam = 0;
	double* hi = NULL;
	int dopvpos[MAXPNTOBS] = { 0 }, prvpos[MAXPNTOBS] = { 0 };
	double rrmat[MAXPNTOBS * DPS_IN_KALPNT] = { 0.0 };
	double azel_[MAXPNTOBS * 2] = { 0.0 };
	double dop[4] = { 0 };
	int prn = 0;
	const double lam_carr[MAXFREQ] = { /* carrier wave length (m) */
		CLIGHT / FREQ1,CLIGHT / FREQ2,CLIGHT / FREQ5,CLIGHT / FREQ6,CLIGHT / FREQ7,
		CLIGHT / FREQ8,CLIGHT / FREQ9
	};
	int i = 0, j = 0, k = 0, nv = 0, sys = 0;
	int npse = 0, ndop = 0;
	for (i = 0; i < 3; i++) {
		rr[i] = xp[i];//时间更新后的x
	}
	dtr = xp[6];
	ecef2pos(rr, pos);
	/**计算伪距残差*/
	for (i = 0; i < ns && i < MAXPNTOBS; i++) {
		azel[i * 2] = azel[1 + i * 2] = 0.0;
		vsat[i] = 0;
		if (!(sys = satsys(obs[i].sat, NULL)))
			continue;
		if (midmsg[i].dpsr1 == 1) {
			trace(5, "zdpse midmsg DPsr error continue sat=%2d\r\n", midmsg[i].sat);
			continue;
		}
		/* excluded satellite? */
		if (satexclude(obs[i].sat, svh[i], opt))
			continue;
		/* reject duplicated observation data */
		if (i < ns - 1 && i < MAXPNTOBS - 1 && obs[i].sat == obs[i + 1].sat) {
			i++;
			continue;
		}
		if ((sys = limitcmpgeo(obs[i].sat)) <= SYS_NONE)
			continue;
		/* geometric distance/azimuth/elevation angle */
		if ((r = geodist(rs + i * 6, rr, e)) <= 0.0 || satazel(pos, e, azel + i * 2) < opt->elmin) {
			continue;
		}
		/* psudorange with code bias correction */
		if ((P = prange(obs + i, nav, azel + i * 2, 1, opt, &vmeas)) == 0.0) { //这里的控制参数1 改为0的话就是不pass snr低的 有差异，不明显，当前案例说明kalman中snr可以不限制，为了防止更低的，所以还是限制
			continue;
		}
		/* ionospheric corrections */
		if (!ionocorr(obs[i].time, nav, obs[i].sat, pos, azel + i * 2, IONOOPT_BRDC, &dion, &vion)) {
			continue;
		}
		if ((lam_L1 = nav->lam[obs[i].sat - 1][0]) > 0.0) {/* GPS-L1 -> L1/B1 */
			dion *= SQR(lam_L1 / lam_carr[0]);
		}
		/* tropospheric corrections */
		if (!tropcorr(obs[i].time, nav, pos, azel + i * 2, TROPOPT_SAAS, &dtrp, &vtrp)) {
			continue;
		}
		/* pseudorange residual */
		v[nv] = P - (r + dtr - CLIGHT * dts[i * 2] + dion + dtrp);
		/* design matrix */
		if (H) {
			hi = H + nv * NS;//当 Hi 被赋值为 H + nv * NS 时，实际上是将 Hi 指向了 H 指针所指向的内存位置加上偏移量的位置。这意味着 Hi 和 H 指向的是同一段内存。
			for (k = 0; k < NS; k++)
				hi[k] = 0.0;
			for (k = 0; k < 3; k++) {//位置是卫地距视线矢量
				hi[k] = -e[k];
			}
			hi[6] = 1.0;//接收机钟差、三大系统间偏差为单位阵
		}
		if (sys == SYS_GLO) {
			v[nv] -= xp[7];
			hi[7] = 1;
		}
		else if (sys == SYS_GAL) {
			v[nv] -= xp[8];
			hi[8] = 1;
		}
		else if (sys == SYS_CMP) {
			v[nv] -= xp[9];
			hi[9] = 1;
		}
		/*计算伪距测量误差*/
		rrmat[nv] = varerr_psr(opt, azel[1 + i * 2], sys) + vmeas + vion + vtrp;
		//trace(5, "zdpse:SAT=%d v[%2d]=%10.3f P=%16.9f r=%16.9f dtr=%9.7f dts=%9.7f dion=%9.7f dtrp=%9.7f\r\n", obs[i].sat, nv, v[nv], P, r, dtr, CLIGHT *dts[i * 2], dion, dtrp);
		if (fabs(v[nv]) > 200) {
			trace(3, "P out nv=%2d sat=%2d v=%10.3f Ri=%10.3f     snr=%3.1f el=%3.1f \r\n", nv, obs[i].sat, v[nv], rrmat[nv], obs[i].snr[0] * 0.25, azel[1 + i * 2] * R2D);
			continue;
		}
		trace(3, "P nv=%2d sat=%2d v=%10.3f rrmat=%10.3f     snr=%3.1f el=%3.1f \r\n", nv, obs[i].sat, v[nv], rrmat[nv], obs[i].snr[0] * 0.25, azel[1 + i * 2] * R2D);
		prvpos[npse] = i;
		azel_[2 * npse] = azel[i * 2];
		azel_[2 * npse + 1] = azel[1 + i * 2];
		vsat[i] = 1;
		nv++;
		npse++;
	}
	if (nv < THRE_SATNUM) {
		return nv;
	}
	/**dopper observables*/
	for (i = 0; i < ns && i < MAXPNTOBS; i++) {
		if (vtat == 0)//最小二乘单点时多普勒测速失败
			break;
		if ((sys = limitcmpgeo(obs[i].sat)) <= SYS_NONE)
			continue;
		/* excluded satellite? */
		if (satexclude(obs[i].sat, svh[i], opt))
			continue;
		/*test doppler observables*/
		if (obs[i].D[0] == 0.0f || nav->lam[obs[i].sat - 1][0] == 0.0)
			continue;
		if (midmsg[i].dpsr1 == 1 || midmsg[i].ddop1 == 1)
			continue;//TODO
		/* geometric distance/azimuth/elevation angle */
		if ((r = geodist(rs + i * 6, rr, e)) <= 0.0 ||
			satazel(pos, e, azel + i * 2) < opt->elmin) {
			//trace(4, "pntpos_continue_geometric sat=%d r=%f elevation=%f\n", obs[i].sat,r,satazel(pos, e, azel + i * 2)*R2D);
			continue;
		}
		lam = nav->lam[obs[i].sat - 1][0];
		/*satellite velocity relative to receiver in ecef*/
		for (j = 0; j < 3; j++) {
			vs[j] = rs[j + 3 + i * 6] - xp[j + 3];
		}
		/*range rate with earth rotation correction*/
		rate = dot(vs, e, 3) + OMGE / CLIGHT * (rs[4 + i * 6] * rr[0] + rs[1 + i * 6] * xp[3] -
			rs[3 + i * 6] * rr[1] - rs[i * 6] * xp[4]);
		v[nv] = -lam * obs[i].D[0] - (rate + xp[NS - 1] - CLIGHT * dts[1 + i * 2]);
		/*design matrix*/
		if (H) {
			hi = H + nv * NS;
			for (k = 0; k < NS; k++)
				hi[k] = 0.0;
			for (k = 0; k < 3; k++) {
				hi[k + 3] = -e[k];
			}
			hi[NS - 1] = 1;
		}
		if (fabs(v[nv]) > 20)//多普勒残差异常阈值判断
			continue;
		rrmat[nv] = 0.1 * varerr_psr(opt, azel[1 + i * 2], sys);
		trace(3, "D nv=%2d sat=%d v=%10.3f rrmat=%10.3f\r\n", nv, obs[i].sat, v[nv], rrmat[nv]);
		dopvpos[ndop] = i;
		nv++;
		ndop++;
	}
#  if 1
	if (rtk->sol_sppk.resetflag)//当kalman初始化时需考虑几何构型，初始化完成之后可不考虑几何构型，待在rtk的基础上已新增该约束
	{
		dops(npse, azel_, opt->elmin, dop, NULL);
		trace(4, "kalman_pnt:the dops of kalman_pnt:nv=%d gdop=%.1f pdop=%.1f\r\n", npse, dop[0], dop[1]);
		if (dop[0] <= 0.0 || dop[1] > 5 || dop[0] > rtk->opt.maxgdop){
			trace(4, "RTD:gdop error nv=%d gdop=%.1f pdop=%.1f\n", nv, dop[0], dop[1]);
			nv = 0;
		}
	}
#endif
	/*3sigma粗差探测*/
	if (var_update(rrmat, v, npse, ndop, prvpos, dopvpos, midmsg, vsat))//var_update_grubs(Ri,v,npse,ndop,prvpos,dopvpos,midmsg);
		nv = 0;
	/*R阵赋值*/
	for (i = 0; i < nv; i++) {
		R[i + i * nv] = rrmat[i];
	}
	return nv;
}
int pntpos_kalm(rtk_t* rtk, const obsd_t* obs, int n, const nav_t* nav, const prcopt_t* opt, midmsg_t* midmsg, sol_t* sol) {
	/*稳健性判断*/
	if (rtk == NULL || obs == NULL || nav == NULL || opt == NULL || midmsg == NULL || sol == NULL) {
		return -1;
	}
	trace(5, "RtkDynamicPntpos:: pntposB\r\n");
	if (n < 5) {/* no observation return */
		return 1;
	}
	/*变量初始化*/
	double* H = NULL, * R = NULL;
	double xp[NS] = { 0.0 }, pp[NS * NS] = { 0.0 };
	double v[MAXPNTOBS * DPS_IN_KALPNT] = { 0.0 }, rs[MAXPNTOBS * 6] = { 0.0 }, dts[MAXPNTOBS * 2] = { 0.0 }, var[MAXPNTOBS] = { 0.0 }, azel_[MAXPNTOBS * 2] = { 0.0 };
	int i = 0, stat = SOLQ_SINGLE, info = 0, nv = 0;
	int svh[MAXPNTOBS], vsat[MAXPNTOBS];
	double dif[3] = { 0 };
	double dt = 0.0, st = 0.0, lt = 0.0;
	double vv = 0.0;
	double kvv = 0.0;
	if (rtk->time_s.time != 0)
		dt = fabs(timediff(obs[0].time, rtk->time_s));
	if (rtk->sol_sppk.time.time != 0)
		st = fabs(timediff(obs[0].time, rtk->sol_sppk.time));
	if (rtk->sol_last.time.time != 0)
		lt = fabs(timediff(obs[0].time, rtk->sol_last.time));
	/**kalman filter state update*/
	if (udpos_pnt2(rtk, dt, st, lt, 0)) {
		stat = SOLQ_NONE;
		trace(4, "pntpos udpos_pnt error.\r\n");
		sol->lackflag = 1;
		sol->stat = SOLQ_NONE;
		return 1;
	}
	/*todo临时策略:通过dif判断是否需要时间更新初始化，只适合车载不适合机载*/
	for (i = 0; i < 3; i++)
		dif[i] = rtk->x_s[i] - rtk->sol.rr[i];//更新后的位置是否发散
	trace(4, "the dif of kalmanpnt initial and pnt sol:%6.3f %6.3f %6.3f %6.3f location_qrf=%f\n", dif[0], dif[1], dif[2], norm(dif, 3), norm(rtk->sol.qr, 3));
	if (rtk->sol.stat && lt >= 3 && norm(dif, 3) > 20) {//TODO临时策略  前后差差20m就初始化位置
		trace(4, "the diff of pntpos udpos_pnt and sol.rr is too big,need reset.\r\n");
		/*时间更新强制初始化*/
		if (udpos_pnt2(rtk, dt, st, lt, 1)) {
			stat = SOLQ_NONE;
			trace(4, "pntpos udpos_pnt error.\r\n");
			sol->lackflag = 1;
			sol->stat = SOLQ_NONE;
			return 1;
		}
	}
	/**测量更新->变量初始化*/
	rtk->time_s = obs[0].time;
	H = zeros(NS, n * DPS_IN_KALPNT);
	R = zeros(n * DPS_IN_KALPNT, n * DPS_IN_KALPNT);
	for (i = 0; i < MAXPNTOBS; i++) {
		vsat[i] = 0;
	}
	matcpy(xp, rtk->x_s, NS, 1);
	matcpy(pp, rtk->p_s, NS, NS);
	satposs(obs[0].time, obs, n, nav, opt->sateph, rs, dts, var, svh, MAXPNTOBS);
	for (i = 0; i < 1; i++) {
		if ((nv = zdpse(rtk, obs, n, rs, dts, var, svh, nav, opt, midmsg, azel_, rtk->sol.vtat, xp, v, H, R, vsat)) < THRE_SATNUM) {
			trace(4, "inadequate satellites.nv=%d\r\n", nv);
			stat = SOLQ_NONE;
			break;
		}
		matcpy(pp, rtk->p_s, NS, NS);
		info = filter_leador(xp, pp, H, v, R, NS, nv, 2, 5);
		if ((info)) {
			trace(1, "kalman spp filter error(info=%d)\n", info);
			stat = SOLQ_NONE;
			break;
		}
	}
	/*解正确性判断*/
	if (stat != SOLQ_NONE)
	{
		if ((nv = zdpse(rtk, obs, n, rs, dts, var, svh, nav, opt, midmsg, azel_, 0, xp, v, H, R, vsat)) < THRE_SATNUM) {
			trace(4, "inadequate satellites.nv=%d\r\n", nv);
			stat = SOLQ_NONE;
		}
		else {
			vv = sqrt(dot(v, v, nv) / nv);
			for (i = 0; i < nv; i++)
				v[i] /= sqrt(R[i + i * nv]);
			kvv = sqrt(dot(v, v, nv) / nv);
			trace(4, "pntpos_kalm:kvv=%16.3f vv=%16.3f\n", kvv, vv);
		}
		//
		if (kvv > 10 && vv > 10)
			stat = SOLQ_NONE;
	}
	/* save solution data*/
	if (stat != SOLQ_NONE) {
		matcpy(rtk->x_s, xp, NS, 1);
		matcpy(rtk->p_s, pp, NS, NS);
		for (i = 0; i < 6; i++) {
			sol->rr[i] = rtk->x_s[i];
			sol->qr[i] = rtk->p_s[i + i * NS];
		}
		statissolsat(obs, n, sol, vsat);
		sol->dtr[0] = rtk->x_s[6];        /*receiver clock bias(m)*/
		if (sol->nsat[1])
			sol->dtr[1] = rtk->x_s[7];        /*glo-gps time offset(m)*/
		else
			rtk->x_s[7] = rtk->sol.dtr[1];
		if (sol->nsat[2])
			sol->dtr[2] = rtk->x_s[8];        /*gal-gps time offset(m)*/
		else
			rtk->x_s[8] = rtk->sol.dtr[2];
		if (sol->nsat[3])
			sol->dtr[3] = rtk->x_s[9];        /*bds-gps time offset(m)*/
		else
			rtk->x_s[9] = rtk->sol.dtr[3];
		sol->dtr[5] = rtk->x_s[10];       /*receiver clock shift*/
		sol->time = obs[0].time;
		rtk->time_ls = obs[0].time;
	}
	for (i = 0; i < 4; i++)
		sol->rdop[i] = rtk->sol.rdop[i];
	sol->age = rtk->sol.age;
	sol->vtat = rtk->sol.vtat;
	sol->stat = (unsigned char)stat;
	sol->first_flag = rtk->sol.first_flag;
	/*打印结果*/
	trace(2, "kalman_stat=%d\r\n", stat);
	trace(4, "kalam: rr[0]=%12.3f rr[1]=%12.3f rr[2]=%12.3f rr[3]=%12.3f rr[4]=%12.3f rr[5]=%12.3f dt[0]=%12.3f dt[1]=%12.3f dt[2]=%12.3f dt[3]=%12.3f dt[5]=%12.3f\r\n", sol->rr[0],
		sol->rr[1], sol->rr[2], sol->rr[3], sol->rr[4], sol->rr[5], rtk->x_s[6], rtk->x_s[7], rtk->x_s[8], rtk->x_s[9], rtk->x_s[10]);
	/*释放内存*/
	xy_free(H); xy_free(R);
	return stat == SOLQ_NONE;
}
int cal_sat_azel(rtk_t* rtk, const obsd_t* obs, int nu, const nav_t* nav, double* elev)
{
	int i = 0;
	double rr[3] = { 0.0 };
	sol_t  sol = { 0 };
	if (norm(rtk->sol.rr, 3) > 0.001) {//有移动站坐标
		for (i = 0; i < 3; i++)  rr[i] = rtk->sol.rr[i];
		trace(4, "cal_sat_azel:get the pos by rtk->sol\n");
	}else {
		if (norm(rtk->opt.rb, 3) > 0.001) {
			for (i = 0; i < 3; i++)  rr[i] = rtk->opt.rb[i];
			trace(4, "cal_sat_azel:get the pos by rtk->rb\n");
		}else {  
			pntpos(rtk, obs, nu, nav, &rtk->opt, NULL, &sol, NULL, NULL, 0, 0);  //此处仅需要一个大概的位置
			if (sol.stat){
				for (i = 0; i < 3; i++)  rr[i] = sol.rr[i];
				trace(4, "cal_sat_azel:get the pos by pnt\n");
			}
		}
	}
	if (norm(rr, 3) > 0.001){
		cal_azel(rtk->opt, obs, nu, nav, rr, elev);
		trace(4, "cal_sat_azel:calzael success\n");
		return 1;
	}
	trace(4, "cal_sat_azel:calzael failed\n");
	return 0;
}

//从大到小排序
int order_snr_obsandelev(int n, obsd_t* obs_t, double* elev_t) {
	int i = 0, j = 0, k = 0;
	obsd_t temp = { 0 };
	double temp_e[2] = { 0 };

	for (i = 0; i < n - 1; i++) {
		k = i;
		for (j = i + 1; j < n; j++) {
			if (obs_t[k].snr[0] < obs_t[j].snr[0]) {
				temp = obs_t[i];
				temp_e[0] = elev_t[i * 2];
				temp_e[1] = elev_t[i * 2 + 1];

				obs_t[i] = obs_t[j];
				elev_t[i * 2] = elev_t[j * 2];
				elev_t[i * 2 + 1] = elev_t[j * 2 + 1];

				obs_t[j] = temp;
				elev_t[j * 2] = temp_e[0];
				elev_t[j * 2 + 1] = temp_e[1];
			}
		}
	}
	return 1;
}

int choose_highsnr_sat(rtk_t* rtk, const obsd_t* obs, int nu, const nav_t* nav)
{
	/*初始化变量*/
	int i = 0;
	int j = 0;
	int cal_elve_flag = 0;
	obsd_t obs_t[MAXPNTOBS] = { 0 };
	double dop[4] = { 0 };
	double dt = 0.0;
	int sys = 0;
	int k = 0;
	int stat = 0;
	double elev[MAXPNTOBS * 2] = { 0.0 };
	double elev_t[MAXPNTOBS * 2] = { 0.0 };
	/*!计算高度角：计算顺序：移动站坐标->基准站坐标->单点计算坐标*/
	if (cal_sat_azel(rtk, obs, nu, nav, elev)) {
		cal_elve_flag = 1;
	}
	j = 0;
	for (i = 0; i < nu; i++) {
		if (!ephclk(obs[i].time, obs[i].time, obs[i].sat, nav, &dt)) {
			continue;
		}
		/*检测当前星是否为北斗GEO卫星*/
		sys = limitcmpgeo(obs[i].sat);  
		if (sys <= SYS_NONE)
			continue;
		/*多普勒观测值检测*/
		if (obs[i].D[0] == 0.0f)
			continue;
		/*截止高度角限制18度*/
		if (cal_elve_flag){
			if (elev[i * 2 + 1] < CUTOFF_ANGLE * D2R) continue;
			elev_t[j * 2] = elev[i * 2];
			elev_t[j * 2 + 1] = elev[i * 2 + 1];
			obs_t[j++] = obs[i];
		}else {
			obs_t[j++] = obs[i];
		}
	}
	if (j <= 0) return 0;
	order_snr_obsandelev(j, obs_t, elev_t);
	trace(4, "choose_highsnr_sat::pnt snr7=%6.3f Pntsnrthres=%6.3f satnum=%4d\n", obs_t[7].snr[0] * 0.25, pntsnrthres, j);
	int minsat_choose = 5;
	if (obs_t[7].snr[0] * 0.25 >= 35) minsat_choose = 7;
	for (i = 0; i < j; i++)
	{
		if (obs_t[i].snr[0]*0.25 < 35) {
			return  0;
		}
		trace(3, "choose_highsnr_sat::minsat_choose =%d \n", minsat_choose);
		/*挨个试信噪比高的 最小 卫星组且通过dop阈值来解算看是否成功*/
		if (i >= minsat_choose)
		{
			trace(3, "i=%4d:\n", i);
			if (cal_elve_flag){
				pntelvthres = CUTOFF_ANGLE * D2R;//截止高度角设置为18度
				
				for (k = 0; k < i + 1; k++)//打印选择的最小卫星合集
					trace(4, "choose_highsnr_sat::choose high snr sat:%4d %6.1f %6.1f %6.1f\n", obs_t[k].sat, obs_t[k].snr[0] * 0.25, elev_t[k * 2] * R2D, elev_t[k * 2 + 1] * R2D);

				dops(i + 1, elev_t, rtk->opt.elmin, dop, NULL);
				trace(5, "choose_highsnr_sat::dop is: G:%f P:%f H:%f V:%f\n", dop[0], dop[1], dop[2], dop[3]);
				if (dop[0] > 0.0 && dop[1] < 5 && dop[0] < rtk->opt.maxgdop)//此处的逻辑是如果dop满足那么利用i+1颗计算 不满足则进行下一个i的循环
				{
					trace(4, "choose_highsnr_sat::The minimum satellite collection pntpos：\n");
					stat = pntpos(rtk, obs_t, i + 1, nav, &rtk->opt, NULL, &rtk->sol, NULL, NULL, 0, 0);//无粗差探测的单点？？？为什么要用无粗差探测？？？这块计算单点的作用？？？
					return  stat;
				}
				else{
					trace(4, "choose_highsnr_sat::dop failed  i=%d\n",i);
				}
			}else{
				pntelvthres = 20 * D2R;
				trace(4, "choose_highsnr_sat::cal_sat_azel failed,begin pntpots without dops:j=%4d\n", i);
				stat = pntpos(rtk, obs_t, i + 1, nav, &rtk->opt, midmsg_, &rtk->sol, NULL, NULL, 0, 0);
				return  stat;
			}
		}
	}
	return 0;//星太少了不满足解算要求
}

extern int pnt_pos_solution(rtk_t* rtk, const obsd_t* obs, int n, int nu, int nr, const nav_t* nav) {
	if (rtk == NULL || obs == NULL || nav == NULL) {
		return -1;
	}

	double dts = 0.0, snr_avg = 0.0;
	int sns = 0.0, sys = 0.0;
	double this_snr = 0.0;
	int m = 0, f = 0;
	int stat_info = 0;
	int i = 0, beforedect = 0;  //beforedect 是否打开粗差探测
	/* 设置变量 */
	prcopt_t* opt = &rtk->opt;
	double rangeadd = 0.0;
	double dt = 0.0;
	memset(&midmsg_, 0, sizeof(midmsg_));     //新增粗差结构体初始化，避免掉星时相应卫星对应的粗差标识未被初始化而传递至下一个历元导致误判
	memset(rtk->opt.saterrorlist_dect, 0, MAXEORSTA * sizeof(int));
	obsdect(nu, nr, obs, rtk);
	if (rtk->sol.time.time != 0 && rtk->sol_last.time.time != 0)
		dt = fabs(timediff(rtk->sol_last.time, rtk->sol.time));
	trace(2, "before sol_last pos %12.3f %12.3f %12.3f dts=%f\r\n", rtk->sol_last.rr[0], rtk->sol_last.rr[1], rtk->sol_last.rr[2], dts);

	//TODO
	if (rtk->sol.spp_flag == 2){
		memcpy(&rtk->sol, &rtk->sol_last, sizeof(sol_t));  //TODO12
	}

	if (rtk->sol.spp_flag != 2){
		if (dt < 15) memcpy(&rtk->sol, &rtk->sol_last, sizeof(sol_t));  //TODO12
	}

	/* 打开粗差探测， 除了初始历元 */
	if (norm(rtk->sol.rr, 3))
		beforedect = 1;
	rtk->sol.spp_flag = 0;//TODO 19

	/* 设置信噪比阈值 */
//    for (m = 0; m < 4; m++) {//4系统
//        for(f=0;f<opt->nf;f++)
//        {
//            snr_avg = 0.0;
//            sns = 0;
//            for (i = 0; i < nu; i++) {
//                sys = satsys(obs[i].sat, NULL);
//                if (!test_sys(sys, m))continue;
//                this_snr = obs[i].snr[f] * 0.25;
//                if (this_snr < opt->snrthres)continue;
//                snr_avg += this_snr;
//                sns++;//大于20的卫星数
//            }
//            if (sns > 0)snr_avg /= sns;//每个系统的信噪比均值
//            opt->snravg[m][f] = snr_avg;
//        }

//    } //计算每个系统的信噪比均值，作为选择基准星的参考

	if (rtk->sol.time.time)
		dts = fabs(timediff(obs[0].time, rtk->sol.time));//求时间中断多少秒

	rangeadd = norm(rtk->sol.rr + 3, 3) * dts;

	pntsnrthres = opt->snrthres;

	trace(2, "before pntpos %12.3f %12.3f %12.3f dts=%f\r\n", rtk->sol.rr[0], rtk->sol.rr[1], rtk->sol.rr[2], dts);
	/* 单点定位 */
	if (beforedect && ((dts < 5.0 && dts>0) || rangeadd < 10)) {   //如果时间中断太多或者位置变化太大，可能估计的位置会与实际位置偏差较大，影响到验前粗差检测
		for (i = 0; i < 3; i++) {
			rtk->sol.rr[i] = rtk->sol.rr[i] + rtk->sol.rr[i + 3] * dts;
		}
		rtk->sol.dtr[0] = rtk->sol.dtr[0] + rtk->sol.dtr[5] * dts;
		if (!pntpos(rtk, obs, nu, nav, opt, midmsg_, &rtk->sol, NULL, rtk->ssat, 1, 1)) {
			trace(3, "pntpos failure(beforedect)\r\n");

			rtk->sol.spp_flag = 1;
			if (rtk->sol.first_flag == 0)
				return -1;
		}
	}
	else {
		if (!pntpos(rtk, obs, nu, nav, opt, midmsg_, &rtk->sol, NULL, rtk->ssat, 0, 1)) {//与上一个pntpos差异点0和1
			trace(3, "pntpos failure(no beforedect)\r\n");

			rtk->sol.spp_flag = 1;
			if (rtk->sol.first_flag == 0)return -1;
		}
	}

	trace(2, "dtr[0]=%6.6lf dtr[1]=%6.6lf dtr[2]=%6.6lf dtr[3]=%6.6lf dtr[4]=%6.6lf dtr[5]=%6.6lf\r\n", rtk->sol.dtr[0], rtk->sol.dtr[1], rtk->sol.dtr[2], rtk->sol.dtr[3], rtk->sol.dtr[4], rtk->sol.dtr[5]);
	trace(2, "after sol:        %s pos[0]=%12.6lf pos[1]=%12.6lf pos[2]=%12.6lf vel[0]=%6.6lf vel[1]=%6.6lf vel[2]=%6.6lf vel=%6.6lf\r\n",
		time_str(rtk->sol.time, 3), rtk->sol.rr[0], rtk->sol.rr[1], rtk->sol.rr[2], rtk->sol.rr[3], rtk->sol.rr[4], rtk->sol.rr[5], norm(rtk->sol.rr + 3, 3));

	dts = fabs(timediff(obs[0].time, rtk->sol_sppk.time)); 
	if (dts <= 0)
		return 0;
	memcpy(&rtk->sol_sppk, &rtk->sol, sizeof(sol_t));

	if (rtk->time_s.time == 0) {
		rtk->time_s = rtk->sol.time;
	}
	rtk->time = obs[0].time;

	trace(2, "start spp kalman\r\n");

	if ((stat_info = pntpos_kalm(rtk, obs, nu, nav, opt, midmsg_, &rtk->sol_sppk)) == 1) {
		if (rtk->sol_sppk.lackflag == 1) {
			trace(2, "kalman spp failure!\r\n");
			rtk->sol_spp.stat = 0;
			return 0;//直接返回如果取sol_spp作为输出，spp存的是上一个历元的信息，如果直接返回则这里加上rtk->sol_spp.stat = SOLQ_NONE;
		}
	}
	if (rtk->sol_sppk.stat == SOLQ_SINGLE) {
		memcpy(&rtk->sol, &rtk->sol_sppk, sizeof(sol_t));
	}

	if (rtk->sol_sppk.stat == SOLQ_SINGLE)trace(4, "after kal kalmanspp:        %s pos[0]=%12.6lf pos[1]=%12.6lf pos[2]=%12.6lf vel[0]=%6.6lf vel[1]=%6.6lf vel[2]=%6.6lf vel=%6.6lf\r\n",
		time_str(rtk->sol_sppk.time, 3), rtk->sol_sppk.rr[0], rtk->sol_sppk.rr[1], rtk->sol_sppk.rr[2], rtk->sol_sppk.rr[3], rtk->sol_sppk.rr[4], rtk->sol_sppk.rr[5], norm(rtk->sol_sppk.rr + 3, 3));
	if (rtk->sol_sppk.stat != SOLQ_SINGLE)trace(4, "after kal  sppk no pos,and sol:        %s pos[0]=%12.6lf pos[1]=%12.6lf pos[2]=%12.6lf vel[0]=%6.6lf vel[1]=%6.6lf vel[2]=%6.6lf vel=%6.6lf\r\n",
		time_str(rtk->sol.time, 3), rtk->sol.rr[0], rtk->sol.rr[1], rtk->sol.rr[2], rtk->sol.rr[3], rtk->sol.rr[4], rtk->sol.rr[5], norm(rtk->sol.rr + 3, 3));
	memcpy(&rtk->sol_spp, &rtk->sol, sizeof(sol_t));
	if (rtk->sol.first_flag == 0)rtk->sol.first_flag++;
	return 1;
}

//单点定位，包括最小二乘和卡尔曼
extern int pnt_pos_solution2(rtk_t* rtk, const obsd_t* obs, int n, int nu, int nr, const nav_t* nav) {
	if (rtk == NULL || obs == NULL || nav == NULL) {
		return -1;
	}

	double dts = 0.0, snr_avg = 0.0;
	int sns = 0.0, sys = 0.0;
	double this_snr = 0.0;
	int m = 0, f = 0;
	int stat_info = 0;
	int i = 0, beforedect = 0;  //beforedect 是否打开粗差探测
	int prn = 0;
	/* 设置变量 */
	prcopt_t* opt = &rtk->opt;
	double dt = 0.0;
	double snr[MAXPNTOBS] = { 0 };
	memset(&midmsg_, 0, sizeof(midmsg_));     //新增粗差结构体初始化，避免掉星时相应卫星对应的粗差标识未被初始化而传递至下一个历元导致误判

	memset(rtk->opt.saterrorlist_dect, 0, MAXEORSTA * sizeof(int));
	//obsdect(nu, nr, obs, rtk);

	if (rtk->sol.time.time != 0 && rtk->sol_last.time.time != 0)
		dt = fabs(timediff(rtk->sol_last.time, rtk->sol.time));
	trace(2, "before sol_last pos %12.3f %12.3f %12.3f dts=%f\r\n", rtk->sol_last.rr[0], rtk->sol_last.rr[1], rtk->sol_last.rr[2], dts);

	//TODO
	if (rtk->sol.spp_flag == 2)
	{
		memcpy(&rtk->sol, &rtk->sol_last, sizeof(sol_t));  //TODO12
	}

	if (rtk->sol.spp_flag != 2)
	{
		if (dt < 15) memcpy(&rtk->sol, &rtk->sol_last, sizeof(sol_t));  //TODO12
	}

	/* 打开粗差探测， 除了初始历元 */
	if (norm(rtk->sol.rr, 3))
		beforedect = 1;
	rtk->sol.spp_flag = 0;//TODO 19

	/* 设置信噪比阈值 */
	for (m = 0; m < 4; m++) {
		for (f = 0; f < opt->nf; f++)
		{
			snr_avg = 0.0;
			sns = 0;
			for (i = 0; i < nu; i++) {
				sys = satsys(obs[i].sat, &prn);
				if (!test_sys(sys, m))continue;
				this_snr = obs[i].snr[f] * 0.25;
				if (this_snr < opt->snrthres)continue;
				if (f == 0)
				{
					if (sys == SYS_CMP && prn <= 5)  //北斗C01~C05不参与统计
						snr[i] = 0;
					else
						snr[i] = obs[i].snr[0] * 0.25;
				}
				snr_avg += this_snr;
				sns++;
			}
			if (sns > 0)snr_avg /= sns;
			opt->snravg[m][f] = snr_avg;
		}
	} //计算每个系统的信噪比均值，作为选择基准星的参考

	if (rtk->sol.time.time)
		dts = fabs(timediff(obs[0].time, rtk->sol.time));

	trace(2, "before pntpos %12.3f %12.3f %12.3f %12.3f %12.3f %12.3f dts=%f\r\n", rtk->sol.rr[0], rtk->sol.rr[1], rtk->sol.rr[2], rtk->sol.rr[3], rtk->sol.rr[4], rtk->sol.rr[5], dts);
	pntsnrthres = opt->snrthres;
	pntelvthres = opt->elmin;

	/* 单点定位 */
	if (beforedect && ((dts < 5.0 && dts>0))) {  
		
		for (i = 0; i < 3; i++) {
			rtk->sol.rr[i] = rtk->sol.rr[i] + rtk->sol.rr[i + 3] * dts;
		}
		rtk->sol.dtr[0] = rtk->sol.dtr[0] + rtk->sol.dtr[5] * dts;
		if (!pntpos(rtk, obs, nu, nav, opt, midmsg_, &rtk->sol, NULL, rtk->ssat, 1, 1)) {
			trace(3, "pntpos failure(beforedect)\r\n");

			rtk->sol.spp_flag = 1;
			if (rtk->sol.first_flag == 0)
				return -1;
			if (rtk->sol.lacksat)
				return -1;
		}
	}
	else {
		//qsort(snr,(size_t)nu,sizeof (double),compare2);
		orderdopdif(nu, snr);
		rtk->sol.stat = 0;
		//if(snr[7]>=30 || snr[6]>=30)
		if (snr[7] > 0)
		{
			if (norm(opt->rb, 3) < 0.001) 
			{
				pntsnrthres = 30;        //TODO  此处设置太小会放入较多的卫星而增加放入有粗差卫星的概率，太大容易导致缺少足够的卫星数来进行单点定位
			}
			else {
				pntsnrthres = MAX(snr[7], 30);
			}

			pntelvthres = 20 * D2R;

			trace(4, "pnt snr7=%6.3f Pntsnrthres=%6.3f\n", snr[7], pntsnrthres);
			if (!pntpos(rtk, obs, nu, nav, opt, midmsg_, &rtk->sol, NULL, rtk->ssat, 0, 1)) {
				trace(3, "pntpos failure(no beforedect)\r\n");

				rtk->sol.spp_flag = 1;
				if (rtk->sol.first_flag == 0)return -1;
				if (rtk->sol.lacksat)
					return -1;
			}
		}

		if (rtk->sol.stat)
		{
			pntsnrthres = opt->snrthres;
			pntelvthres = opt->elmin;
			if (!pntpos(rtk, obs, nu, nav, opt, midmsg_, &rtk->sol, NULL, rtk->ssat, 1, 1)) {
				trace(3, "pntpos failure(no beforedect)\r\n");

				rtk->sol.spp_flag = 1;
				if (rtk->sol.first_flag == 0)return -1;
				if (rtk->sol.lacksat)
					return -1;
			}
		}
		else {  //否则不开启粗差探测
			pntsnrthres = opt->snrthres;
			pntelvthres = opt->elmin;
			if (!pntpos(rtk, obs, nu, nav, opt, midmsg_, &rtk->sol, NULL, rtk->ssat, 0, 1)) {
				trace(3, "pntpos failure(no beforedect)\r\n");

				rtk->sol.spp_flag = 1;
				if (rtk->sol.first_flag == 0)return -1;
				if (rtk->sol.lacksat)
					return -1;
			}
		}
	}

	trace(4, "dtr[0]=%6.6lf dtr[1]=%6.6lf dtr[2]=%6.6lf dtr[3]=%6.6lf dtr[4]=%6.6lf dtr[5]=%6.6lf\r\n", rtk->sol.dtr[0], rtk->sol.dtr[1], rtk->sol.dtr[2], rtk->sol.dtr[3], rtk->sol.dtr[4], rtk->sol.dtr[5]);
	trace(2, "after sol:        %s pos[0]=%12.6lf pos[1]=%12.6lf pos[2]=%12.6lf vel[0]=%6.6lf vel[1]=%6.6lf vel[2]=%6.6lf vel=%6.6lf\r\n",
		time_str(rtk->sol.time, 3), rtk->sol.rr[0], rtk->sol.rr[1], rtk->sol.rr[2], rtk->sol.rr[3], rtk->sol.rr[4], rtk->sol.rr[5], norm(rtk->sol.rr + 3, 3));


	dts = fabs(timediff(obs[0].time, rtk->sol_sppk.time)); //上个历元的卡尔曼单点解的时间
	if (dts <= 0)
		return 0;
	memcpy(&rtk->sol_sppk, &rtk->sol, sizeof(sol_t));

	if (rtk->time_s.time == 0) {
		rtk->time_s = rtk->sol.time;
	}
	rtk->time = obs[0].time;

	trace(2, "start spp kalman\r\n");

	if ((stat_info = pntpos_kalm(rtk, obs, nu, nav, opt, midmsg_, &rtk->sol_sppk)) == 1) {
		if (rtk->sol_sppk.lackflag == 1) {
			trace(2, "kalman spp failure!\r\n");
			rtk->sol_spp.stat = 0;
			return 0;//直接返回如果取sol_spp作为输出，spp存的是上一个历元的信息，如果直接返回则这里加上rtk->sol_spp.stat = SOLQ_NONE;
		}
	}
	if (rtk->sol_sppk.stat == SOLQ_SINGLE) {
		memcpy(&rtk->sol, &rtk->sol_sppk, sizeof(sol_t));
	}

	if (rtk->sol_sppk.stat == SOLQ_SINGLE)trace(4, "after kal kalmanspp:        %s pos[0]=%12.6lf pos[1]=%12.6lf pos[2]=%12.6lf vel[0]=%6.6lf vel[1]=%6.6lf vel[2]=%6.6lf vel=%6.6lf\r\n",
		time_str(rtk->sol_sppk.time, 3), rtk->sol_sppk.rr[0], rtk->sol_sppk.rr[1], rtk->sol_sppk.rr[2], rtk->sol_sppk.rr[3], rtk->sol_sppk.rr[4], rtk->sol_sppk.rr[5], norm(rtk->sol_sppk.rr + 3, 3));
	if (rtk->sol_sppk.stat != SOLQ_SINGLE)trace(4, "after kal  sppk no pos,and sol:        %s pos[0]=%12.6lf pos[1]=%12.6lf pos[2]=%12.6lf vel[0]=%6.6lf vel[1]=%6.6lf vel[2]=%6.6lf vel=%6.6lf\r\n",
		time_str(rtk->sol.time, 3), rtk->sol.rr[0], rtk->sol.rr[1], rtk->sol.rr[2], rtk->sol.rr[3], rtk->sol.rr[4], rtk->sol.rr[5], norm(rtk->sol.rr + 3, 3));
	memcpy(&rtk->sol_spp, &rtk->sol, sizeof(sol_t));
	if (rtk->sol.first_flag == 0)rtk->sol.first_flag++;
	return 1;
}

int judgenewsys(sol_t* sol, const obsd_t* obs, int nu)
{
	/*变量初始化*/
	int i = 0, m = 0;
	int sys = 0;
	int sysflag[4] = { 0 };//观测值中存在的系统标记 GREC
	double dt = 0.0;
	/*为避免误判，仅在开机前三分钟进行该项判断，并且仅判断1次*/
	if (!startimeinitflag) {
		obsstartime = obs[0].time;
	}
	startimeinitflag = 1;
	dt = timediff(obs[0].time, obsstartime);
	if (dt > 180) {
		return 0;
	}
	/*储存历史系统*/
	for (i = 0; i < nu; i++)
	{
		if ((sys = limitcmpgeo(obs[i].sat)) <= SYS_NONE)
			continue;
		m = get_sys_num(sys);
		if (m < 0)
			continue;
		sysflag[m] = 1;
	}
	/*判断是否新上系统*/
	for (m = 0; m < 4; m++){
		if (!sol->nsat[m] && sysflag[m])  //存在的问题，钟差都是相对于gps而言，那新上gps如何处理
			return 1;
	}
	return 0;
}

extern int pnt_pos_solution3(rtk_t* rtk, const obsd_t* obs, int n, int nu, int nr, const nav_t* nav) {
	/*稳健性判断*/
	if (rtk == NULL || obs == NULL || nav == NULL) {
		return -1;
	}
	/* 变量初始化 */
	double dts = 0.0, snr_avg = 0.0;
	int sns = 0.0, sys = 0.0;
	double this_snr = 0.0;
	int m = 0, f = 0;
	int stat_info;
	int i = 0, Location_forecast = 0;  //beforedect 是否打开位置预测
	int prn = 0;
	prcopt_t* opt = &rtk->opt;
	double dt = 0.0;
	//memset(&midmsg_, 0, sizeof(midmsg_));     //新增粗差结构体初始化，避免掉星时相应卫星对应的粗差标识未被初始化而传递至下一个历元导致误判
	

	memset(rtk->opt.saterrorlist_dect, 0, MAXEORSTA * sizeof(int));
	//obsdect(nu, nr, obs, rtk);

	if (rtk->sol.time.time != 0 && rtk->sol_last.time.time != 0) {
		dt = fabs(timediff(rtk->sol_last.time, rtk->sol.time));
	}
	trace(2, "pnt_pos_solution3：before sol_last pos %12.3f %12.3f %12.3f dts=%f\r\n", rtk->sol_last.rr[0], rtk->sol_last.rr[1], rtk->sol_last.rr[2], dt);
	memset(&rtk->sol, 0, sizeof(sol_t));  //TODO12
	if (rtk->sol.spp_flag == 2){
		memcpy(&rtk->sol, &rtk->sol_last, sizeof(sol_t));  //TODO12  该行代码为qr超限但是传递到了下一历元
	}
	if (rtk->sol.spp_flag != 2){//单点失败或成功，也就是只要qr不超限，任意解算零期不能太长，都可以传递当前历元
		if (dt < 15) memcpy(&rtk->sol, &rtk->sol_last, sizeof(sol_t));  //TODO12
	}

	if (norm(rtk->sol.rr, 3) > ZEROS_MIN && !judgenewsys(&rtk->sol, obs, nu)){
		Location_forecast = 1;
	}
	rtk->sol.spp_flag = 0;//TODO 19

	/*! 设置信噪比阈值：计算每个系统的信噪比均值，作为选择基准星的参考 */
	for (m = 0; m < 4; m++) {
		for (f = 0; f < opt->nf; f++)
		{
			snr_avg = 0.0;
			sns = 0;
			for (i = 0; i < nu; i++) {
				sys = satsys(obs[i].sat, &prn);
				if (!test_sys(sys, m))continue;
				this_snr = obs[i].snr[f] * 0.25;
				if (this_snr < opt->snrthres)continue;
				snr_avg += this_snr;
				sns++;
			}
			if (sns > 0)snr_avg /= sns;
			opt->snravg[m][f] = snr_avg;
		}
	} 
	if (rtk->sol.time.time)
		dts = fabs(timediff(obs[0].time, rtk->sol.time));
	trace(2, "pnt_pos_solution3：before pntpos %12.3f %12.3f %12.3f %12.3f %12.3f %12.3f dts=%f\r\n", rtk->sol.rr[0], rtk->sol.rr[1], rtk->sol.rr[2], rtk->sol.rr[3], rtk->sol.rr[4], rtk->sol.rr[5], dts);
	pntsnrthres = opt->snrthres;
	pntelvthres = opt->elmin;
	/********************最小二乘单点定位开始*****************/	
	if (Location_forecast && ((dts <= 5.0 && dts > 0))) {   
		for (i = 0; i < 3; i++) {//todo用上个历元的位置和速度预测这个历元的位置
			rtk->sol.rr[i] = rtk->sol.rr[i] + rtk->sol.rr[i + 3] * dts;
		}
		if (!pntpos(rtk, obs, nu, nav, opt, midmsg_, &rtk->sol, NULL, rtk->ssat, 1, 1)) {
			trace(3, "pnt_pos_solution3：pntpos failure(Location_forecast)\r\n");
			rtk->sol.spp_flag = 1;
			if (rtk->sol.first_flag == 0 || rtk->sol.lacksat)
				return -1;
		}
	}else {
		rtk->sol.stat = 0;
		rtk->sol.vtat = 0;   //TODO
		if (choose_highsnr_sat(rtk, obs, nu, nav))//未开启位置预测
		{
			rtk->sol.spp_flag = 0;
			pntelvthres = opt->elmin;
			if (!pntpos(rtk, obs, nu, nav, opt, midmsg_, &rtk->sol, NULL, rtk->ssat, 1, 1)) {//能过的话就再算一遍
				trace(3, "pnt_pos_solution3：pntpos failure(no Location_forecast)\r\n");
				rtk->sol.spp_flag = 1;
				if (rtk->sol.first_flag == 0 || rtk->sol.lacksat)return -1;
			}
		}else {  //否则不开启粗差探测
			memset(&midmsg_, 0, sizeof(midmsg_));     //新增粗差结构体初始化，避免掉星时相应卫星对应的粗差标识未被初始化而传递至下一个历元导致误判
# if 0   //此处的代码暂时不删除 但是算法中目前没有用到
			rtk->sol.spp_flag = 0;
			if (!pntpos(obs, nu, nav, opt, midmsg_, &rtk->sol, NULL, rtk->ssat, 0, 1)) {
				trace(3, "pntpos failure(no beforedect)\r\n");
				rtk->sol.spp_flag = 1;
				if (rtk->sol.first_flag == 0 || rtk->sol.lacksat)return -1;
			}
#endif
		}
	}
	trace(4, "pnt_pos_solution3:dtr[0]=%6.6lf dtr[1]=%6.6lf dtr[2]=%6.6lf dtr[3]=%6.6lf dtr[4]=%6.6lf dtr[5]=%6.6lf\r\n", rtk->sol.dtr[0], rtk->sol.dtr[1], rtk->sol.dtr[2], rtk->sol.dtr[3], rtk->sol.dtr[4], rtk->sol.dtr[5]);
	trace(2, "pnt_pos_solution3:after sol:        %s pos[0]=%12.6lf pos[1]=%12.6lf pos[2]=%12.6lf vel[0]=%6.6lf vel[1]=%6.6lf vel[2]=%6.6lf vel=%6.6lf\r\n",
		time_str(rtk->sol.time, 3), rtk->sol.rr[0], rtk->sol.rr[1], rtk->sol.rr[2], rtk->sol.rr[3], rtk->sol.rr[4], rtk->sol.rr[5], norm(rtk->sol.rr + 3, 3));	
	if (!rtk->sol.vtat) {//最小二乘单点定速失败则此历元不计算
		rtk->sol.stat = 0;
		return -1;
	}
	dts = fabs(timediff(obs[0].time, rtk->sol_sppk.time)); //上次的卡尔曼单点解的时间
	if (dts <= 0) return 0;
	//memcpy(&rtk->sol_sppk, &rtk->sol, sizeof(sol_t));
	if (rtk->time_s.time == 0) {
		rtk->time_s = rtk->sol.time;//首次初始化
	}
	rtk->time = obs[0].time;//为此历元的时间
	trace(2, "pnt_pos_solution3:start spp kalman\r\n");
	/*卡尔曼单点开始*/
	if ((stat_info = pntpos_kalm(rtk, obs, nu, nav, opt, midmsg_, &rtk->sol_sppk)) == 1) {
		if (rtk->sol_sppk.lackflag == 1) {
			trace(2, "pnt_pos_solution3:kalman spp failure!\r\n");
			rtk->sol_spp.stat = 0;
			return 0;
		}
	}
	if (rtk->sol_sppk.stat == SOLQ_SINGLE) {
		memcpy(&rtk->sol, &rtk->sol_sppk, sizeof(sol_t));
	}
	/*初步判断环境是否利于进行系统间偏差估算*/
	if (rtk->sol.stat && rtk->sol.nsat[0] >= 2)
	{
		if (rtk->sol.nsat[1] > 3)
		{
			trace(4, "pnt_pos_solution3:updata glo isb:isbtime=%s isb=%6.3f dtr=%6.3f\n", time_str(rtk->glo_isb.time, 3), rtk->glo_isb.isb, rtk->sol.dtr[1]);
			rtk->glo_isb.time = obs[0].time;
			rtk->glo_isb.isb = rtk->sol.dtr[1];  /* glo-gps time offset (m) */
		}
		if (rtk->sol.nsat[2] > 3)
		{
			trace(4, "pnt_pos_solution3:updata gal isb:isbtime=%s isb=%6.3f dtr=%6.3f\n", time_str(rtk->gal_isb.time, 3), rtk->gal_isb.isb, rtk->sol.dtr[2]);
			rtk->gal_isb.time = obs[0].time;
			rtk->gal_isb.isb = rtk->sol.dtr[2];  /* gal-gps time offset (m) */
		}
		if (rtk->sol.nsat[3] > 3)
		{
			trace(4, "pnt_pos_solution3:updata bds isb:isbtime=%s isb=%6.3f dtr=%6.3f\n", time_str(rtk->bds_isb.time, 3), rtk->bds_isb.isb, rtk->sol.dtr[3]);
			rtk->bds_isb.time = obs[0].time;
			rtk->bds_isb.isb = rtk->sol.dtr[3];  /* gal-gps time offset (m) */
		}
	}

	if (rtk->sol_sppk.stat == SOLQ_SINGLE)trace(4, "pnt_pos_solution3:after kal kalmanspp:        %s pos[0]=%12.6lf pos[1]=%12.6lf pos[2]=%12.6lf vel[0]=%6.6lf vel[1]=%6.6lf vel[2]=%6.6lf vel=%6.6lf\r\n",
		time_str(rtk->sol_sppk.time, 3), rtk->sol_sppk.rr[0], rtk->sol_sppk.rr[1], rtk->sol_sppk.rr[2], rtk->sol_sppk.rr[3], rtk->sol_sppk.rr[4], rtk->sol_sppk.rr[5], norm(rtk->sol_sppk.rr + 3, 3));
	if (rtk->sol_sppk.stat != SOLQ_SINGLE)trace(4, "pnt_pos_solution3:after kal  sppk no pos,and sol:        %s pos[0]=%12.6lf pos[1]=%12.6lf pos[2]=%12.6lf vel[0]=%6.6lf vel[1]=%6.6lf vel[2]=%6.6lf vel=%6.6lf\r\n",
		time_str(rtk->sol.time, 3), rtk->sol.rr[0], rtk->sol.rr[1], rtk->sol.rr[2], rtk->sol.rr[3], rtk->sol.rr[4], rtk->sol.rr[5], norm(rtk->sol.rr + 3, 3));
	memcpy(&rtk->sol_spp, &rtk->sol, sizeof(sol_t));
	if (rtk->sol.first_flag == 0)rtk->sol.first_flag++;
	return 1;
}