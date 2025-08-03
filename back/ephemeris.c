#include "rtklib.h"

/* constants and macros ------------------------------------------------------*/

#define SQR(x)   ((x)*(x))

#define RE_GLO   6378136.0        /* radius of earth (m)            ref [2] */
#define MU_GPS   3.9860050E14     /* gravitational constant         ref [1] */
#define MU_GLO   3.9860044E14     /* gravitational constant         ref [2] */
#define MU_GAL   3.986004418E14   /* earth gravitational constant   ref [7] */
#define MU_CMP   3.986004418E14   /* earth gravitational constant   ref [9] */
#define J2_GLO   1.0826257E-3     /* 2nd zonal harmonic of geopot   ref [2] */

#define OMGE_GLO 7.292115E-5      /* earth angular velocity (rad/s) ref [2] */
#define OMGE_GAL 7.2921151467E-5  /* earth angular velocity (rad/s) ref [7] */
#define OMGE_CMP 7.292115E-5      /* earth angular velocity (rad/s) ref [9] */

#define SIN_5 -0.0871557427476582 /* sin(-5.0 deg) */
#define COS_5  0.9961946980917456 /* cos(-5.0 deg) */

#define ERREPH_GLO 5.0            /* error of glonass ephemeris (m) */
#define TSTEP    60.0             /* integration step glonass ephemeris (s) */
#define RTOL_KEPLER 1E-14         /* relative tolerance for Kepler equation */

#define DEFURASSR 0.15            /* default accurary of ssr corr (m) */
#define MAXECORSSR 10.0           /* max orbit correction of ssr (m) */
#define MAXCCORSSR (1E-6*CLIGHT)  /* max clock correction of ssr (m) */
#define MAXAGESSR 90.0            /* max age of ssr orbit and clock (s) */
#define MAXAGESSR_HRCLK 10.0      /* max age of ssr high-rate clock (s) */
#define STD_BRDCCLK 30.0          /* error of broadcast clock (m) */

#define MAX_ITER_KEPLER 10        /* max number of iteration of Kelpler */

/* variance by ura ephemeris (ref [1] 20.3.3.3.1.1) --------------------------*/
static double var_uraeph(int ura)
{
	const double ura_value[] = {
		2.4,3.4,4.85,6.85,9.65,13.65,24.0,48.0,96.0,192.0,384.0,768.0,1536.0,
		3072.0,6144.0
	};
	return ura < 0 || 15 < ura ? SQR(6144.0) : SQR(ura_value[ura]);
}

//add
static double var_uraeph2(int sys, int ura)
{
	const double ura_value[] = {
		2.4,3.4,4.85,6.85,9.65,13.65,24.0,48.0,96.0,192.0,384.0,768.0,1536.0,
		3072.0,6144.0
	};
	if (sys == SYS_GAL) { /* galileo sisa (ref [7] 5.1.11) */

		if (ura == 0)return SQR(ura_value[ura]);
		if (ura <= 49) return SQR(ura * 0.01);
		if (ura <= 74) return SQR(0.5 + (ura - 50) * 0.02);
		if (ura <= 99) return SQR(1.0 + (ura - 75) * 0.04);
		if (ura <= 125) return SQR(2.0 + (ura - 100) * 0.16);
		return SQR(STD_GAL_NAPA);
	}
	else { /* gps ura (ref [1] 20.3.3.3.1.1) */
		return ura < 0 || 14 < ura ? SQR(6144.0) : SQR(ura_value[ura]);
	}
}

/* broadcast ephemeris to satellite clock bias ---------------------------------
* compute satellite clock bias with broadcast ephemeris (gps, galileo, qzss)
* args   : gtime_t time     I   time by satellite clock (gpst)
*          eph_t *eph       I   broadcast ephemeris
* return : satellite clock bias (s) without relativeity correction
* notes  : see ref [1],[7],[8]
*          satellite clock does not include relativity correction and tdg
*-----------------------------------------------------------------------------*/
extern double eph2clk(gtime_t time, const eph_t* eph)
{
	double t;
	int i;

	//trace(4,"eph2clk : time=%s toc=%s sat=%2d f0=%19.18f f1=%19.18f f2=%19.18f\r\n",time_str(time,3), time_str(eph->toc, 3),eph->sat, eph->f0, eph->f1, eph->f2);

	t = timediff(time, eph->toc);

	for (i = 0; i < 2; i++) {
		t -= eph->f0 + eph->f1 * t + eph->f2 * t * t;
	}
	return eph->f0 + eph->f1 * t + eph->f2 * t * t;
}
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
extern void eph2pos(gtime_t time, const eph_t* eph, double* rs, double* dts,
	double* var)
{
	double tk, M, E, Ek, sinE, cosE, u, r, i, O, sin2u, cos2u, x, y, sinO, cosO, cosi, mu, omge;
	double xg, yg, zg, sino, coso;
	int n, sys, prn;

	if (eph->A <= 0.0) {
		rs[0] = rs[1] = rs[2] = *dts = *var = 0.0;
		return;
	}
	tk = timediff(time, eph->toe);

	switch ((sys = satsys(eph->sat, &prn))) {
	case SYS_GAL: mu = MU_GAL; omge = OMGE_GAL; break;
	case SYS_CMP: mu = MU_CMP; omge = OMGE_CMP; break;
	default:      mu = MU_GPS; omge = OMGE;     break;
	}

	M = eph->m0 + (sqrt(mu / (eph->A * eph->A * eph->A)) + eph->deln) * tk;

	for (n = 0, E = M, Ek = 0.0; fabs(E - Ek) > RTOL_KEPLER && n < MAX_ITER_KEPLER; n++) {
		Ek = E; E -= (E - eph->e * sin(E) - M) / (1.0 - eph->e * cos(E));
	}
	if (n >= MAX_ITER_KEPLER) {
		trace(2, "eph2pos: kepler iteration overflow sat=%2d\r\n", eph->sat);
		return;
	}
	sinE = sin(E); cosE = cos(E);

	//trace(4,"kepler: sat=%2d e=%8.5f n=%2d del=%10.3e tk=%19.6f\r\n",eph->sat,eph->e,n,E-Ek,tk);

	u = atan2(sqrt(1.0 - eph->e * eph->e) * sinE, cosE - eph->e) + eph->omg;
	r = eph->A * (1.0 - eph->e * cosE);
	i = eph->i0 + eph->idot * tk;
	sin2u = sin(2.0 * u); cos2u = cos(2.0 * u);
	u += eph->cus * sin2u + eph->cuc * cos2u;
	r += eph->crs * sin2u + eph->crc * cos2u;
	i += eph->cis * sin2u + eph->cic * cos2u;
	x = r * cos(u); y = r * sin(u); cosi = cos(i);

	/* beidou geo satellite (ref [9]) */
	if (sys == SYS_CMP && BDSGEOFLAG(prn)) {
		O = eph->omg0 + eph->omgd * tk - omge * eph->toes;
		sinO = sin(O); cosO = cos(O);
		xg = x * cosO - y * cosi * sinO;
		yg = x * sinO + y * cosi * cosO;
		zg = y * sin(i);
		sino = sin(omge * tk); coso = cos(omge * tk);
		rs[0] = xg * coso + yg * sino * COS_5 + zg * sino * SIN_5;
		rs[1] = -xg * sino + yg * coso * COS_5 + zg * coso * SIN_5;
		rs[2] = -yg * SIN_5 + zg * COS_5;
	}
	else {
		O = eph->omg0 + (eph->omgd - omge) * tk - omge * eph->toes;
		sinO = sin(O); cosO = cos(O);
		rs[0] = x * cosO - y * cosi * sinO;
		rs[1] = x * sinO + y * cosi * cosO;
		rs[2] = y * sin(i);
	}
	tk = timediff(time, eph->toc);
	*dts = eph->f0 + eph->f1 * tk + eph->f2 * tk * tk;

	//trace(4, "eph2pos : sat=%2d dts=%19.18f f0=%19.18f f1=%19.18f f2=%19.18f tk=%19.12f toe=%19d\r\n",
	//    eph->sat, *dts, eph->f0, eph->f1, eph->f2, tk,eph->toe.time);

	/* relativity correction */
	*dts -= 2.0 * sqrt(mu * eph->A) * eph->e * sinE / SQR(CLIGHT);

	//trace(4, "eph2pos 2: sat=%2d dts=%19.18f A=%19.18f e=%19.18f sinE=%19.18f mu=%19.12f \r\n",
	//     eph->sat, *dts, eph->A, eph->e, sinE, mu);

	/* position and clock error variance */

	*var = var_uraeph2(sys, eph->sva);
}
/* glonass orbit differential equations --------------------------------------*/
static void deq(const double* x, double* xdot, const double* acc)
{
	double a, b, c, r2 = dot(x, x, 3), r3 = r2 * sqrt(r2), omg2 = SQR(OMGE_GLO);

	if (r2 <= 0.0) {
		xdot[0] = xdot[1] = xdot[2] = xdot[3] = xdot[4] = xdot[5] = 0.0;
		return;
	}
	/* ref [2] A.3.1.2 with bug fix for xdot[4],xdot[5] */
	a = 1.5 * J2_GLO * MU_GLO * SQR(RE_GLO) / r2 / r3; /* 3/2*J2*mu*Ae^2/r^5 */
	b = 5.0 * x[2] * x[2] / r2;                    /* 5*z^2/r^2 */
	c = -MU_GLO / r3 - a * (1.0 - b);                /* -mu/r^3-a(1-b) */
	xdot[0] = x[3]; xdot[1] = x[4]; xdot[2] = x[5];
	xdot[3] = (c + omg2) * x[0] + 2.0 * OMGE_GLO * x[4] + acc[0];
	xdot[4] = (c + omg2) * x[1] - 2.0 * OMGE_GLO * x[3] + acc[1];
	xdot[5] = (c - 2.0 * a) * x[2] + acc[2];
}
/* glonass position and velocity by numerical integration --------------------*/
static void glorbit(double t, double* x, const double* acc)
{
	double k1[6] = { 0 }, k2[6] = { 0 }, k3[6] = { 0 }, k4[6] = { 0 }, w[6] = { 0 };
	int i = 0;

	deq(x, k1, acc); for (i = 0; i < 6; i++) w[i] = x[i] + k1[i] * t / 2.0;
	deq(w, k2, acc); for (i = 0; i < 6; i++) w[i] = x[i] + k2[i] * t / 2.0;
	deq(w, k3, acc); for (i = 0; i < 6; i++) w[i] = x[i] + k3[i] * t;
	deq(w, k4, acc);
	for (i = 0; i < 6; i++) x[i] += (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]) * t / 6.0;
}
/* glonass ephemeris to satellite clock bias -----------------------------------
* compute satellite clock bias with glonass ephemeris
* args   : gtime_t time     I   time by satellite clock (gpst)
*          geph_t *geph     I   glonass ephemeris
* return : satellite clock bias (s)
* notes  : see ref [2]
*-----------------------------------------------------------------------------*/
extern double geph2clk(gtime_t time, const geph_t* geph)
{
	double t = 0;
	int i = 0;

	//trace(4, "geph2clk: time=%s sat=%2d\r\n", time_str(time, 3), geph->sat);

	t = timediff(time, geph->toe);

	for (i = 0; i < 2; i++) {
		t -= -geph->taun + geph->gamn * t;
	}
	return -geph->taun + geph->gamn * t;
}
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
extern void geph2pos(gtime_t time, const geph_t* geph, double* rs, double* dts,
	double* var)
{
	double t = 0, tt = 0, x[6] = { 0 };
	int i = 0;

	//trace(4, "geph2pos: time=%s sat=%2d\r\n", time_str(time, 3), geph->sat);

	t = timediff(time, geph->toe);

	*dts = -geph->taun + geph->gamn * t;

	for (i = 0; i < 3; i++) {
		x[i] = geph->pos[i];
		x[i + 3] = geph->vel[i];
	}
	for (tt = t < 0.0 ? -TSTEP : TSTEP; fabs(t) > 1E-9; t -= tt) {
		if (fabs(t) < TSTEP) tt = t;
		glorbit(tt, x, geph->acc);
	}
	for (i = 0; i < 3; i++) rs[i] = x[i];

	*var = SQR(ERREPH_GLO);
}

/* select ephememeris --------------------------------------------------------*/
extern eph_t* seleph(gtime_t time, int sat, int iode, const nav_t* nav)
{
	double t = 0, tmax = 0, tmin = 0;
	int i = 0, j = -1;

	//trace(4,"seleph  : time=%s sat=%2d iode=%d\r\n",time_str(time,3),sat,iode);

	tmax = MAXDTOE + 1.0;
	tmin = tmax + 1.0;

	for (i = 0; i < nav->n; i++) {
		if (nav->eph[i].sat != sat) continue;
		if (iode >= 0 && nav->eph[i].iode != iode) continue;//输入的IODE代表不匹配iode
		if ((t = fabs(timediff(nav->eph[i].toe, time))) > tmax) continue;//此时time为obs的时间
		if (iode >= 0) return nav->eph + i;
		if (t <= tmin) { j = i; tmin = t; } /* toe closest to time */
	}
	if (iode >= 0 || j < 0) {
		trace(3, "no broadcast ephemeris: %s sat=%2d iode=%3d\r\n", time_str(time, 0),
			sat, iode);
		return NULL;
	}
	return nav->eph + j;
}
/* select glonass ephememeris ------------------------------------------------*/
extern geph_t* selgeph(gtime_t time, int sat, int iode, const nav_t* nav)
{
	double t = 0, tmax = MAXDTOE_GLO, tmin = tmax + 1.0;
	int i = 0, j = -1;

	//trace(4,"selgeph : time=%s sat=%2d iode=%2d\r\n",time_str(time,3),sat,iode);

	for (i = 0; i < nav->ng; i++) {
		if (nav->geph[i].sat != sat) continue;
		if (iode >= 0 && nav->geph[i].iode != iode) continue;
		if ((t = fabs(timediff(nav->geph[i].toe, time))) > tmax) continue;
		if (iode >= 0) return nav->geph + i;
		if (t <= tmin) { j = i; tmin = t; } /* toe closest to time */
	}
	if (iode >= 0 || j < 0) {
		trace(3, "no glonass ephemeris  : %s sat=%2d iode=%2d\r\n", time_str(time, 0),
			sat, iode);
		return NULL;
	}
	return nav->geph + j;
}

/* satellite clock with broadcast ephemeris ----------------------------------*/
extern int ephclk(gtime_t time, gtime_t teph, int sat, const nav_t* nav,
	double* dts)
{
	eph_t* eph = { 0 };
	geph_t* geph = { 0 };
	int sys = 0;
	int svh = 0;

	//trace(4,"ephclk  : time=%s sat=%2d\r\n",time_str(time,3),sat);

	sys = satsys(sat, NULL);

	if (sys == SYS_GPS || sys == SYS_GAL || sys == SYS_QZS || sys == SYS_CMP) {
		if (!(eph = seleph(teph, sat, -1, nav))) return 0;//选星历
		*dts = eph2clk(time, eph);//求广播卫星钟差
		svh = eph->svh;//改进
	}
	else if (sys == SYS_GLO) {
		if (!(geph = selgeph(teph, sat, -1, nav))) return 0;
		*dts = geph2clk(time, geph);
		svh = geph->svh;//改进
	}

	else return 0;

	if (svh) //星历不健康则返回0，改进
		return 0;

	return 1;
}
/* satellite position and clock by broadcast ephemeris -----------------------*/
static int ephpos(gtime_t time, gtime_t teph, int sat, const nav_t* nav,
	int iode, double* rs, double* dts, double* var, int* svh)
{
	eph_t* eph = { 0 };
	geph_t* geph = { 0 };
	double rst[3] = { 0 }, dtst[1] = { 0 }, tt = 1E-3;
	int i = 0, sys = 0;

	//trace(4,"ephpos  : time=%s sat=%2d iode=%d\r\n",time_str(time,3),sat,iode);

	sys = satsys(sat, NULL);

	*svh = -1;

	if (sys == SYS_GPS || sys == SYS_GAL || sys == SYS_QZS || sys == SYS_CMP) {
		if (!(eph = seleph(teph, sat, iode, nav))) return 0;//选择合适星历

		eph2pos(time, eph, rs, dts, var);
		time = timeadd(time, tt);
		eph2pos(time, eph, rst, dtst, var);//为了求钟漂
		*svh = eph->svh;
	}
	else if (sys == SYS_GLO) {
		if (!(geph = selgeph(teph, sat, iode, nav))) return 0;
		geph2pos(time, geph, rs, dts, var);
		time = timeadd(time, tt);
		geph2pos(time, geph, rst, dtst, var);
		*svh = geph->svh;
	}

	else return 0;

	/* satellite velocity and clock drift by differential approx */
	for (i = 0; i < 3; i++) rs[i + 3] = (rst[i] - rs[i]) / tt;
	dts[1] = (dtst[0] - dts[0]) / tt;

	return 1;
}
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
extern int satpos(gtime_t time, gtime_t teph, int sat, int ephopt,
	const nav_t* nav, double* rs, double* dts, double* var,
	int* svh)
{
	//trace(4,"satpos  : time=%s sat=%2d ephopt=%d\r\n",time_str(time,3),sat,ephopt);

	*svh = 0;
	return ephpos(time, teph, sat, nav, -1, rs, dts, var, svh);
}
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
extern void satposs(gtime_t teph, const obsd_t* obs, int n, const nav_t* nav,
	int ephopt, double* rs, double* dts, double* var, int* svh, int maxsizeofsvh)
{
	/*变量初始化*/
	gtime_t time[2 * MAXPNTOBS] = { {0} };
	double dt, pr;
	int i, j;
	trace(3, "satposs : teph=%s n=%d ephopt=%d\r\n", time_str(teph, 3), n, ephopt);
	/*逐个计算卫星位置*/
	for (i = 0; i < n && i < 2 * MAXPNTOBS && i < maxsizeofsvh; i++) { //这里maxsizeofsvh防止随便更改宏定义导致数据越界卡死
		for (j = 0; j < 6; j++) rs[j + i * 6] = 0.0;
		for (j = 0; j < 2; j++) dts[j + i * 2] = 0.0;
		var[i] = 0.0; svh[i] = 0;
		/* search any psuedorange */
		for (j = 0, pr = 0.0; j < NFREQ; j++) {
			if ((pr = obs[i].P[j]) != 0.0) break;
		}
		if (j >= NFREQ) {
			trace(2, "satposs :no pseudorange %s sat=%2d\r\n", time_str(obs[i].time, 3), obs[i].sat);
			continue;
		}
		/* transmission time by satellite clock */
		time[i] = timeadd(obs[i].time, -pr / CLIGHT);
		/* satellite clock bias by broadcast ephemeris */
		if (!ephclk(time[i], teph, obs[i].sat, nav, &dt)) {
			trace(4, "satposs :no broadcast clock %s sat=%2d\r\n", time_str(time[i], 3), obs[i].sat);//可能根据obs文件卫星未找到nav匹配卫星
			continue;
		}
		time[i] = timeadd(time[i], -dt);
		/* satellite position and clock at transmission time */
		if (!satpos(time[i], teph, obs[i].sat, ephopt, nav, rs + i * 6, dts + i * 2, var + i,
			svh + i)) {
			trace(4, "satposs :no ephemeris %s sat=%2d\r\n", time_str(time[i], 3), obs[i].sat);
			continue;
		}
		/* if no precise clock available, use broadcast clock instead */
		if (dts[i * 2] == 0.0) {
			if (!ephclk(time[i], teph, obs[i].sat, nav, dts + i * 2)) continue;
			dts[1 + i * 2] = 0.0;
			*var = SQR(STD_BRDCCLK);
		}
	}
}
/*计算卫星高度角+（截至高度角与svh滤除）*/
extern void cal_azel(prcopt_t popt, const obsd_t* obs, int n, const nav_t* nav, double* rr, double* elev)
{
	int i = 0, svh[MAXRECOBS] = { 0 };
	double rs[6 * MAXRECOBS] = { 0.0 }, dts[2 * MAXRECOBS] = { 0.0 }, var[MAXRECOBS] = { 0.0 }, azel[2 * MAXRECOBS] = { 0.0 };
	double r = 0.0, pos[3], e[3];

	satposs(obs[0].time, obs, n, nav, popt.sateph, rs, dts, var, svh, MAXRECOBS);
	if (norm(rr, 3) < 1e-6){
		trace(1, "no rr to cal azel\n");
		return;
	}
	ecef2pos(rr, pos);
	for (i = 0; i < n; i++){
		/* geometric distance/azimuth/elevation angle */
		if ((r = geodist(rs + i * 6, rr, e)) <= 0.0 ||
			satazel(pos, e, azel + i * 2) < popt.elmin || svh[i]) continue;
		elev[2 * i] = azel[2 * i];
		elev[2 * i + 1] = azel[2 * i + 1];
	}
}
// 10hz 05月copy 但是没有用 为了保证算法只重构 代码 不改算法 所以这里和对应的数据预处理10hz 都没有调用
//10hz计算高度角时，每次计算卫星位置算不过来，所以在cal_azel中去掉satposs计算卫星位置，不自算
extern void cal_azel3(prcopt_t popt, int n, double* rb, double* elev, double* rs, int* svh)
{
	int i = 0;
	double azel[2 * MAXRECOBS] = { 0.0 };
	double r = 0.0, pos[3] = { 0 }, e[3] = { 0 };

	ecef2pos(rb, pos);

	for (i = 0; i < n; i++)
	{
		/* geometric distance/azimuth/elevation angle */
		if ((r = geodist(rs + i * 6, rb, e)) <= 0.0 ||
			satazel(pos, e, azel + i * 2) < popt.elmin || svh[i]) continue;

		elev[2 * i] = azel[2 * i];
		elev[2 * i + 1] = azel[2 * i + 1];
	}
}