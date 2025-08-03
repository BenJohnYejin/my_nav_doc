/*------------------------------------------------------------------------------
* rtcm3.c : rtcm ver.3 message decorder functions
 constants -----------------------------------------------------------------*/

#include "rtklib.h"

#define PRUNIT_GPS  299792.458  /* rtcm ver.3 unit of gps pseudorange (m) */
#define PRUNIT_GLO  599584.916  /* rtcm ver.3 unit of glonass pseudorange (m) */
#define RANGE_MS    (CLIGHT*0.001)      /* range in 1 ms */

#define P2_10       0.0009765625          /* 2^-10 */
#define P2_34       5.820766091346740E-11 /* 2^-34 */
#define P2_46       1.421085471520200E-14 /* 2^-46 */
#define P2_59       1.734723475976810E-18 /* 2^-59 */
#define P2_66       1.355252715606880E-20 /* 2^-66 */

 /* type definition -----------------------------------------------------------*/

typedef struct {                    /* multi-signal-message header type */
	unsigned char iod;              /* issue of data station */
	unsigned char time_s;           /* cumulative session transmitting time */
	unsigned char clk_str;          /* clock steering indicator */
	unsigned char clk_ext;          /* external clock indicator */
	unsigned char smooth;           /* divergence free smoothing indicator */
	unsigned char tint_s;           /* soothing interval */
	unsigned char nsat, nsig;        /* number of satellites/signals */
	unsigned char sats[64];         /* satellites */
	unsigned char sigs[32];         /* signals */
	unsigned char cellmask[64];     /* cell mask */
} msm_h_t;

/* ssr update intervals ------------------------------------------------------*/
//static const double ssrudint[16]={
//    1,2,5,10,15,30,60,120,240,300,600,900,1800,3600,7200,10800
//};
/* get sign-magnitude bits ---------------------------------------------------*/
static double getbitg(const unsigned char* buff, int pos, int len)
{
	double value = getbitu(buff, pos + 1, len - 1);
	return getbitu(buff, pos, 1) ? -value : value;
}

static gtime_t get_gpsweek(double toe)
{
	//printf("week=%5d toe=%16.3f\r\n", m_RtcmWeek, m_gps_toe);
	if (g_gps_toe < 14401 && toe>590399) {    //星历跨周，跨周时有效星历的开头值小于4h，观测电文时间大于6天20时0分0秒
		return gpst2time(g_rtcm_week - 1, toe);
	}
	else if (toe < 14401 && g_gps_toe>518600) {   //观测电文跨周，防止部分卫星周六星历不更新，限制星历最小周内秒为6天
		return gpst2time(g_rtcm_week + 1, toe);
	}
	else {
		return gpst2time(g_rtcm_week, toe);
	}
}

static void adjday_glot3(rtcm_t* rtcm, double tod)
{
	gtime_t time = { 0 };
	double tow = 0, tod_p = 0;
	int week = 0;

	tow = 0.0;

	if (rtcm->time.time == 0) rtcm->time = get_gpsweek(tow + tod);
	time = timeadd(gpst2utc(rtcm->time), 10800.0); /* glonass time */
	tow = time2gpst(time, &week);
	tod_p = fmod(tow, 86400.0); tow -= tod_p;
	if (tod < tod_p - 43200.0) tod += 86400.0;
	else if (tod > tod_p + 43200.0) tod -= 86400.0;
	time = gpst2time(week, tow + tod);
	rtcm->time = utc2gpst(timeadd(time, -10800.0));
}
/* adjust weekly rollover of gps time ----------------------------------------*/
static void adjweek3(rtcm_t* rtcm, double tow)
{
	double tow_p = 0;
	int week = 0;

	/* if no time, get cpu time */
	if (rtcm->time.time == 0) rtcm->time = get_gpsweek(tow);
	tow_p = time2gpst(rtcm->time, &week);
	if (tow < tow_p - 302400.0) tow += 604800.0;
	else if (tow > tow_p + 302400.0) tow -= 604800.0;

	rtcm->time = gpst2time(week, tow);
}
/* adjust weekly rollover of bdt time ----------------------------------------*/
static int adjbdtweek(int week)
{
	int w;
	//(void)time2bdt(gpst2bdt(utc2gpst(timeget())),&w);
	//if (w<1) w=1; /* use 2006/1/1 if time is earlier than 2006/1/1 */
	w = 760;  //use 2020/07/30 if time is earlier than 2020/07/30 */
	return week + (w - week + 512) / 1024 * 1024;
}

/* loss-of-lock indicator ----------------------------------------------------*/
static int lossoflock(rtcm_t* rtcm, int sat, int index, int freq, int lock)
{
	int prelock = 0, i = 0;
	for (i = 0; i < MAXRECOBS; i++)
	{
		if (sat == rtcm->prelock[i][freq].sat)
		{
			prelock = rtcm->prelock[i][freq].lock;
			break;
		}
	}
	if (i == MAXRECOBS)
		prelock = 0;
	int lli = (!lock && !prelock) || lock < prelock;
	rtcm->prelock[index][freq].sat = sat;
	rtcm->prelock[index][freq].lock = (unsigned short)lock;
	return lli;
}

/* get observation data index ------------------------------------------------*/
static int obsindex(obs_t* obs, gtime_t time, int sat)
{
	int i = 0, j = 0;

	for (i = 0; i < obs->n; i++) {
		if (obs->data[i].sat == sat) return i; /* field already exists */
	}
	if (i >= MAXRECOBS * obs->data[0].rcv)
		return -1; /* overflow */

	/* add new field */
	obs->data[i].time = time;
	obs->data[i].sat = sat;
	for (j = 0; j < NFREQ + NEXOBS; j++) {
		obs->data[i].L[j] = obs->data[i].P[j] = 0.0;
		obs->data[i].D[j] = 0.0;
		obs->data[i].snr[j] = obs->data[i].lli[j] = obs->data[i].code[j] = 0;
	}
	obs->n++;
	return i;
}
/* test station id consistency -----------------------------------------------*/
static int test_staid(rtcm_t* rtcm, int staid)
{
	char* p = "";
	int type = 0, id = 0;

	/* test station id option */
	if ((p = strstr(rtcm->opt, "-STA=")) && sscanf(p, "-STA=%d", &id) == 1) {
		if (staid != id) return 0;
	}
	/* save station id */
	if (rtcm->staid == 0 || rtcm->obsflag) {
		rtcm->staid = staid;
	}
	else if (staid != rtcm->staid) {
		type = getbitu(rtcm->buff, 24, 12);
		trace(2, "rtcm3 %d staid invalid id=%d %d\n", type, staid, rtcm->staid);

		/* reset station id if station id error */
		//rtcm->staid=0;
		return 0;
	}
	return 1;
}

/* get signed 38bit field ----------------------------------------------------*/
static double getbits_38(const unsigned char* buff, int pos)
{
	return (double)getbits(buff, pos, 32) * 64.0 + getbitu(buff, pos + 32, 6);
}
/* decode type 1005: stationary rtk reference station arp --------------------*/
static int decode_type1005(rtcm_t* rtcm)
{
	double rr[3] = { 0 }, re[3] = { 0 }, pos[3] = { 0 };
	char* msg = "";
	int i = 24 + 12, j = 0, staid = 0, itrf = 0;

	if (i + 140 == rtcm->len * 8) {
		staid = getbitu(rtcm->buff, i, 12); i += 12;
		itrf = getbitu(rtcm->buff, i, 6); i += 6 + 4;
		rr[0] = getbits_38(rtcm->buff, i); i += 38 + 2;
		rr[1] = getbits_38(rtcm->buff, i); i += 38 + 2;
		rr[2] = getbits_38(rtcm->buff, i);
	}
	else {
		trace(2, "rtcm3 1005 length error: len=%d\r\n", rtcm->len);
		return -1;
	}
	if (rtcm->outtype) {
		msg = rtcm->msgtype + strlen(rtcm->msgtype);
		for (j = 0; j < 3; j++) re[j] = rr[j] * 0.0001;
		ecef2pos(re, pos);
		sprintf(msg, " staid=%4d pos=%.8f %.8f %.3f", staid, pos[0] * R2D, pos[1] * R2D,
			pos[2]);
	}
	/* test station id */
	//if (!test_staid(rtcm,staid)) return -1;
	rtcm->staid = staid;
	trace(4, "1005 staid:%d\n", staid);

	rtcm->sta.deltype = 0; /* xyz */
	for (j = 0; j < 3; j++) {
		rtcm->sta.pos[j] = rr[j] * 0.0001;
		rtcm->sta.del[j] = 0.0;
	}
	rtcm->sta.hgt = 0.0;
	rtcm->sta.itrf = itrf;
	return 5;
}
/* decode type 1006: stationary rtk reference station arp with height --------*/
static int decode_type1006(rtcm_t* rtcm)
{
	double rr[3] = { 0 }, re[3] = { 0 }, pos[3] = { 0 }, anth = 0;
	char* msg = "";
	int i = 24 + 12, j = 0, staid = 0, itrf = 0;

	if (i + 156 <= rtcm->len * 8) {
		staid = getbitu(rtcm->buff, i, 12); i += 12;
		itrf = getbitu(rtcm->buff, i, 6); i += 6 + 4;
		rr[0] = getbits_38(rtcm->buff, i); i += 38 + 2;
		rr[1] = getbits_38(rtcm->buff, i); i += 38 + 2;
		rr[2] = getbits_38(rtcm->buff, i); i += 38;
		anth = getbitu(rtcm->buff, i, 16);
	}
	else {
		trace(2, "rtcm3 1006 length error: len=%d\r\n", rtcm->len);
		return -1;
	}
	if (rtcm->outtype) {
		msg = rtcm->msgtype + strlen(rtcm->msgtype);
		for (j = 0; j < 3; j++) re[j] = rr[j] * 0.0001;
		ecef2pos(re, pos);
		sprintf(msg, " staid=%4d pos=%.8f %.8f %.3f anth=%.3f", staid, pos[0] * R2D,
			pos[1] * R2D, pos[2], anth);
	}
	/* test station id */
	//if (!test_staid(rtcm,staid)) return -1;

	rtcm->sta.deltype = 1; /* xyz */
	for (j = 0; j < 3; j++) {
		rtcm->sta.pos[j] = rr[j] * 0.0001;
		rtcm->sta.del[j] = 0.0;
	}
	rtcm->sta.hgt = anth * 0.0001;
	rtcm->sta.itrf = itrf;
	return 5;
}
/* decode type 1007: antenna descriptor --------------------------------------*/
static int decode_type1007(rtcm_t* rtcm)
{
	char des[32] = "";
	char* msg = "";
	int i = 24 + 12, j = 0, staid = 0, n = 0, setup = 0;

	n = getbitu(rtcm->buff, i + 12, 8);

	if (i + 28 + 8 * n <= rtcm->len * 8) {
		staid = getbitu(rtcm->buff, i, 12); i += 12 + 8;
		for (j = 0; j < n && j < 31; j++) {
			des[j] = (char)getbitu(rtcm->buff, i, 8); i += 8;
		}
		setup = getbitu(rtcm->buff, i, 8);
	}
	else {
		trace(2, "rtcm3 1007 length error: len=%d\r\n", rtcm->len);
		return -1;
	}
	if (rtcm->outtype) {
		msg = rtcm->msgtype + strlen(rtcm->msgtype);
		sprintf(msg, " staid=%4d", staid);
	}
	/* test station id */
	//if (!test_staid(rtcm,staid)) return -1;

	strncpy(rtcm->sta.antdes, des, n); rtcm->sta.antdes[n] = '\0';
	rtcm->sta.antsetup = setup;
	rtcm->sta.antsno[0] = '\0';
	return 5;
}
/* decode type 1008: antenna descriptor & serial number ----------------------*/
static int decode_type1008(rtcm_t* rtcm)
{
	char des[32] = "", sno[32] = "";
	char* msg = "";
	int i = 24 + 12, j = 0, staid = 0, n = 0, m = 0, setup = 0;

	n = getbitu(rtcm->buff, i + 12, 8);
	m = getbitu(rtcm->buff, i + 28 + 8 * n, 8);

	if (i + 36 + 8 * (n + m) <= rtcm->len * 8) {
		staid = getbitu(rtcm->buff, i, 12); i += 12 + 8;
		for (j = 0; j < n && j < 31; j++) {
			des[j] = (char)getbitu(rtcm->buff, i, 8); i += 8;
		}
		setup = getbitu(rtcm->buff, i, 8); i += 8 + 8;
		for (j = 0; j < m && j < 31; j++) {
			sno[j] = (char)getbitu(rtcm->buff, i, 8); i += 8;
		}
	}
	else {
		trace(2, "rtcm3 1008 length error: len=%d\r\n", rtcm->len);
		return -1;
	}
	if (rtcm->outtype) {
		msg = rtcm->msgtype + strlen(rtcm->msgtype);
		sprintf(msg, " staid=%4d", staid);
	}
	/* test station id */
	//if (!test_staid(rtcm,staid)) return -1;

	strncpy(rtcm->sta.antdes, des, n); rtcm->sta.antdes[n] = '\0';
	rtcm->sta.antsetup = setup;
	strncpy(rtcm->sta.antsno, sno, m); rtcm->sta.antsno[m] = '\0';
	return 5;
}

/* decode type 1019: gps ephemerides -----------------------------------------*/
static int decode_type1019(rtcm_t* rtcm)
{
	eph_t eph = { 0 };
	double toc = 0, sqrta = 0;
	char* msg = "";
	int i = 24 + 12, prn = 0, sat = 0, week = 0, sys = SYS_GPS;

	if (i + 476 <= rtcm->len * 8) {
		prn = getbitu(rtcm->buff, i, 6);              i += 6;
		week = getbitu(rtcm->buff, i, 10);              i += 10;
		eph.sva = getbitu(rtcm->buff, i, 4);              i += 4;
		eph.code = getbitu(rtcm->buff, i, 2);              i += 2;
		eph.idot = getbits(rtcm->buff, i, 14) * P2_43 * SC2RAD; i += 14;
		eph.iode = getbitu(rtcm->buff, i, 8);              i += 8;
		toc = getbitu(rtcm->buff, i, 16) * 16.0;         i += 16;
		eph.f2 = getbits(rtcm->buff, i, 8) * P2_55;        i += 8;
		eph.f1 = getbits(rtcm->buff, i, 16) * P2_43;        i += 16;
		eph.f0 = getbits(rtcm->buff, i, 22) * P2_31;        i += 22;
		eph.iodc = getbitu(rtcm->buff, i, 10);              i += 10;
		eph.crs = getbits(rtcm->buff, i, 16) * P2_5;         i += 16;
		eph.deln = getbits(rtcm->buff, i, 16) * P2_43 * SC2RAD; i += 16;
		eph.m0 = getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD; i += 32;
		eph.cuc = getbits(rtcm->buff, i, 16) * P2_29;        i += 16;
		eph.e = getbitu(rtcm->buff, i, 32) * P2_33;        i += 32;
		eph.cus = getbits(rtcm->buff, i, 16) * P2_29;        i += 16;
		sqrta = getbitu(rtcm->buff, i, 32) * P2_19;        i += 32;
		eph.toes = getbitu(rtcm->buff, i, 16) * 16.0;         i += 16;
		eph.cic = getbits(rtcm->buff, i, 16) * P2_29;        i += 16;
		eph.omg0 = getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD; i += 32;
		eph.cis = getbits(rtcm->buff, i, 16) * P2_29;        i += 16;
		eph.i0 = getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD; i += 32;
		eph.crc = getbits(rtcm->buff, i, 16) * P2_5;         i += 16;
		eph.omg = getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD; i += 32;
		eph.omgd = getbits(rtcm->buff, i, 24) * P2_43 * SC2RAD; i += 24;
		eph.tgd = getbits(rtcm->buff, i, 8) * P2_31;        i += 8;
		eph.svh = getbitu(rtcm->buff, i, 6);              i += 6;
		eph.flag = getbitu(rtcm->buff, i, 1);              i += 1;
		eph.fit = getbitu(rtcm->buff, i, 1) ? 0.0 : 4.0; /* 0:4hr,1:>4hr */
	}
	else {
		trace(2, "rtcm3 1019 length error: len=%d\r\n", rtcm->len);
		return -1;
	}
	if (prn >= 40) {
		sys = SYS_SBS; prn += 80;
	}
	//trace(4,"decode_type1019: prn=%d iode=%d toe=%.0f\r\n",prn,eph.iode,eph.toes);

	if (rtcm->outtype) {
		msg = rtcm->msgtype + strlen(rtcm->msgtype);
		sprintf(msg, " prn=%2d iode=%3d iodc=%3d week=%d toe=%6.0f toc=%6.0f svh=%02X",
			prn, eph.iode, eph.iodc, week, eph.toes, toc, eph.svh);
	}
	if (!(sat = satno(sys, prn))) {
		trace(2, "rtcm3 1019 satellite number error: prn=%d\r\n", prn);
		return -1;
	}
	eph.sat = sat;
	eph.week = adjgpsweek(week);
	g_rtcm_week = eph.week;

	eph.toe = gpst2time(eph.week, eph.toes);
	g_gps_toe = eph.toes;

	eph.toc = gpst2time(eph.week, toc);
	eph.ttr = rtcm->time;
	eph.A = sqrta * sqrta;

	if (rtcm->ephsat < (MAXRECOBS * 2))
		rtcm->nav.eph[rtcm->ephsat++] = eph;
	//rtcm->ephsat=sat;
	return 2;
}
/* decode type 1020: glonass ephemerides -------------------------------------*/
static int decode_type1020(rtcm_t* rtcm)
{
	geph_t geph = { 0 };
	double tk_h = 0, tk_m = 0, tk_s = 0, toe = 0, tow = 0, tod = 0, tof = 0;
	char* msg = "";
	int i = 24 + 12, prn = 0, sat = 0, week = 0, tb = 0, bn = 0, sys = SYS_GLO;

	if (i + 348 <= rtcm->len * 8) {
		prn = getbitu(rtcm->buff, i, 6);           i += 6;
		geph.frq = getbitu(rtcm->buff, i, 5) - 7;         i += 5 + 2 + 2;
		tk_h = getbitu(rtcm->buff, i, 5);           i += 5;
		tk_m = getbitu(rtcm->buff, i, 6);           i += 6;
		tk_s = getbitu(rtcm->buff, i, 1) * 30.0;      i += 1;
		bn = getbitu(rtcm->buff, i, 1);           i += 1 + 1;
		tb = getbitu(rtcm->buff, i, 7);           i += 7;
		geph.vel[0] = getbitg(rtcm->buff, i, 24) * P2_20 * 1E3; i += 24;
		geph.pos[0] = getbitg(rtcm->buff, i, 27) * P2_11 * 1E3; i += 27;
		geph.acc[0] = getbitg(rtcm->buff, i, 5) * P2_30 * 1E3; i += 5;
		geph.vel[1] = getbitg(rtcm->buff, i, 24) * P2_20 * 1E3; i += 24;
		geph.pos[1] = getbitg(rtcm->buff, i, 27) * P2_11 * 1E3; i += 27;
		geph.acc[1] = getbitg(rtcm->buff, i, 5) * P2_30 * 1E3; i += 5;
		geph.vel[2] = getbitg(rtcm->buff, i, 24) * P2_20 * 1E3; i += 24;
		geph.pos[2] = getbitg(rtcm->buff, i, 27) * P2_11 * 1E3; i += 27;
		geph.acc[2] = getbitg(rtcm->buff, i, 5) * P2_30 * 1E3; i += 5 + 1;
		geph.gamn = getbitg(rtcm->buff, i, 11) * P2_40;     i += 11 + 3;
		geph.taun = getbitg(rtcm->buff, i, 22) * P2_30;
	}
	else {
		trace(2, "rtcm3 1020 length error: len=%d\r\n", rtcm->len);
		return -1;
	}
	if (!(sat = satno(sys, prn))) {
		trace(2, "rtcm3 1020 satellite number error: prn=%d\r\n", prn);
		return -1;
	}
	trace(4, "decode_type1020: prn=%d tk=%02.0f:%02.0f:%02.0f\r\n", prn, tk_h, tk_m, tk_s);

	if (rtcm->outtype) {
		msg = rtcm->msgtype + strlen(rtcm->msgtype);
		sprintf(msg, " prn=%2d tk=%02.0f:%02.0f:%02.0f frq=%2d bn=%d tb=%d",
			prn, tk_h, tk_m, tk_s, geph.frq, bn, tb);
	}
	geph.sat = sat;
	geph.svh = bn;
	geph.iode = tb & 0x7F;
	if (rtcm->time.time == 0) rtcm->time = utc2gpst(timeget());
	tow = time2gpst(gpst2utc(rtcm->time), &week);
	tod = fmod(tow, 86400.0); tow -= tod;
	tof = tk_h * 3600.0 + tk_m * 60.0 + tk_s - 10800.0; /* lt->utc */
	if (tof < tod - 43200.0) tof += 86400.0;
	else if (tof > tod + 43200.0) tof -= 86400.0;
	geph.tof = utc2gpst(gpst2time(week, tow + tof));
	toe = tb * 900.0 - 10800.0; /* lt->utc */
	if (toe < tod - 43200.0) toe += 86400.0;
	else if (toe > tod + 43200.0) toe -= 86400.0;
	geph.toe = utc2gpst(gpst2time(week, tow + toe)); /* utc->gpst */

	if (!strstr(rtcm->opt, "-EPHALL")) {
		if (fabs(timediff(geph.toe, rtcm->nav.geph[prn - 1].toe)) < 1.0 &&
			geph.svh == rtcm->nav.geph[prn - 1].svh) return 0; /* unchanged */
	}
	rtcm->nav.geph[prn - 1] = geph;
	rtcm->ephsat = sat;
	return 2;
}

/* decode type 1033: receiver and antenna descriptor -------------------------*/
static int decode_type1033(rtcm_t* rtcm)
{
	char des[32] = "", sno[32] = "", rec[32] = "", ver[32] = "", rsn[32] = "";
	char* msg = "";
	int i = 24 + 12, j = 0, staid = 0, n = 0, m = 0, n1 = 0, n2 = 0, n3 = 0, setup = 0;

	n = getbitu(rtcm->buff, i + 12, 8);
	m = getbitu(rtcm->buff, i + 28 + 8 * n, 8);
	n1 = getbitu(rtcm->buff, i + 36 + 8 * (n + m), 8);
	n2 = getbitu(rtcm->buff, i + 44 + 8 * (n + m + n1), 8);
	n3 = getbitu(rtcm->buff, i + 52 + 8 * (n + m + n1 + n2), 8);

	if (i + 60 + 8 * (n + m + n1 + n2 + n3) <= rtcm->len * 8) {
		staid = getbitu(rtcm->buff, i, 12); i += 12 + 8;
		for (j = 0; j < n && j < 31; j++) {
			des[j] = (char)getbitu(rtcm->buff, i, 8); i += 8;
		}
		setup = getbitu(rtcm->buff, i, 8); i += 8 + 8;
		for (j = 0; j < m && j < 31; j++) {
			sno[j] = (char)getbitu(rtcm->buff, i, 8); i += 8;
		}
		i += 8;
		for (j = 0; j < n1 && j < 31; j++) {
			rec[j] = (char)getbitu(rtcm->buff, i, 8); i += 8;
		}
		i += 8;
		for (j = 0; j < n2 && j < 31; j++) {
			ver[j] = (char)getbitu(rtcm->buff, i, 8); i += 8;
		}
		i += 8;
		for (j = 0; j < n3 && j < 31; j++) {
			rsn[j] = (char)getbitu(rtcm->buff, i, 8); i += 8;
		}
	}
	else {
		trace(2, "rtcm3 1033 length error: len=%d\r\n", rtcm->len);
		return -1;
	}
	if (rtcm->outtype) {
		msg = rtcm->msgtype + strlen(rtcm->msgtype);
		sprintf(msg, " staid=%4d", staid);
	}
	/* test station id */
	//if (!test_staid(rtcm,staid)) return -1;

	strncpy(rtcm->sta.antdes, des, n); rtcm->sta.antdes[n] = '\0';
	rtcm->sta.antsetup = setup;
	strncpy(rtcm->sta.antsno, sno, m); rtcm->sta.antsno[m] = '\0';
	strncpy(rtcm->sta.rectype, rec, n1); rtcm->sta.rectype[n1] = '\0';
	strncpy(rtcm->sta.recver, ver, n2); rtcm->sta.recver[n2] = '\0';
	strncpy(rtcm->sta.recsno, rsn, n3); rtcm->sta.recsno[n3] = '\0';

	trace(3, "rtcm3 1033: ant=%s:%s rec=%s:%s:%s\r\n", des, sno, rec, ver, rsn);
	return 5;
}

/* decode type 1044: qzss ephemerides (ref [15]) -----------------------------*/
static int decode_type1044(rtcm_t* rtcm)
{
	eph_t eph = { 0 };
	double toc = 0, sqrta = 0;
	char* msg = "";
	int i = 24 + 12, prn = 0, sat = 0, week = 0, sys = SYS_QZS;

	if (i + 473 <= rtcm->len * 8) {
		prn = getbitu(rtcm->buff, i, 4) + 192;          i += 4;
		toc = getbitu(rtcm->buff, i, 16) * 16.0;         i += 16;
		eph.f2 = getbits(rtcm->buff, i, 8) * P2_55;        i += 8;
		eph.f1 = getbits(rtcm->buff, i, 16) * P2_43;        i += 16;
		eph.f0 = getbits(rtcm->buff, i, 22) * P2_31;        i += 22;
		eph.iode = getbitu(rtcm->buff, i, 8);              i += 8;
		eph.crs = getbits(rtcm->buff, i, 16) * P2_5;         i += 16;
		eph.deln = getbits(rtcm->buff, i, 16) * P2_43 * SC2RAD; i += 16;
		eph.m0 = getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD; i += 32;
		eph.cuc = getbits(rtcm->buff, i, 16) * P2_29;        i += 16;
		eph.e = getbitu(rtcm->buff, i, 32) * P2_33;        i += 32;
		eph.cus = getbits(rtcm->buff, i, 16) * P2_29;        i += 16;
		sqrta = getbitu(rtcm->buff, i, 32) * P2_19;        i += 32;
		eph.toes = getbitu(rtcm->buff, i, 16) * 16.0;         i += 16;
		eph.cic = getbits(rtcm->buff, i, 16) * P2_29;        i += 16;
		eph.omg0 = getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD; i += 32;
		eph.cis = getbits(rtcm->buff, i, 16) * P2_29;        i += 16;
		eph.i0 = getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD; i += 32;
		eph.crc = getbits(rtcm->buff, i, 16) * P2_5;         i += 16;
		eph.omg = getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD; i += 32;
		eph.omgd = getbits(rtcm->buff, i, 24) * P2_43 * SC2RAD; i += 24;
		eph.idot = getbits(rtcm->buff, i, 14) * P2_43 * SC2RAD; i += 14;
		eph.code = getbitu(rtcm->buff, i, 2);              i += 2;
		week = getbitu(rtcm->buff, i, 10);              i += 10;
		eph.sva = getbitu(rtcm->buff, i, 4);              i += 4;
		eph.svh = getbitu(rtcm->buff, i, 6);              i += 6;
		eph.tgd = getbits(rtcm->buff, i, 8) * P2_31;        i += 8;
		eph.iodc = getbitu(rtcm->buff, i, 10);              i += 10;
		eph.fit = getbitu(rtcm->buff, i, 1) ? 0.0 : 2.0; /* 0:2hr,1:>2hr */
	}
	else {
		trace(2, "rtcm3 1044 length error: len=%d\r\n", rtcm->len);
		return -1;
	}
	trace(4, "decode_type1044: prn=%d iode=%d toe=%.0f\r\n", prn, eph.iode, eph.toes);

	if (rtcm->outtype) {
		msg = rtcm->msgtype + strlen(rtcm->msgtype);
		sprintf(msg, " prn=%3d iode=%3d iodc=%3d week=%d toe=%6.0f toc=%6.0f svh=%02X",
			prn, eph.iode, eph.iodc, week, eph.toes, toc, eph.svh);
	}
	if (!(sat = satno(sys, prn))) {
		trace(2, "rtcm3 1044 satellite number error: prn=%d\r\n", prn);
		return -1;
	}
	eph.sat = sat;
	eph.week = adjgpsweek(week);
	eph.toe = gpst2time(eph.week, eph.toes);
	eph.toc = gpst2time(eph.week, toc);
	eph.ttr = rtcm->time;
	eph.A = sqrta * sqrta;
	if (!strstr(rtcm->opt, "-EPHALL")) {
		if (eph.iode == rtcm->nav.eph[sat - 1].iode &&
			eph.iodc == rtcm->nav.eph[sat - 1].iodc) return 0; /* unchanged */
	}
	rtcm->nav.eph[sat - 1] = eph;
	rtcm->ephsat = sat;
	return 2;
}
/* decode type 1045: galileo satellite ephemerides (ref [15]) ----------------*/
static int decode_type1045(rtcm_t* rtcm)
{
	eph_t eph = { 0 };
	double toc = 0, sqrta = 0;
	char* msg = "";
	int i = 24 + 12, prn = 0, sat = 0, week = 0, e5a_hs = 0, e5a_dvs = 0, sys = SYS_GAL;

	if (i + 484 <= rtcm->len * 8) {
		prn = getbitu(rtcm->buff, i, 6);              i += 6;
		week = getbitu(rtcm->buff, i, 12);              i += 12; /* gst-week */
		eph.iode = getbitu(rtcm->buff, i, 10);              i += 10;
		eph.sva = getbitu(rtcm->buff, i, 8);              i += 8;
		eph.idot = getbits(rtcm->buff, i, 14) * P2_43 * SC2RAD; i += 14;
		toc = getbitu(rtcm->buff, i, 14) * 60.0;         i += 14;
		eph.f2 = getbits(rtcm->buff, i, 6) * P2_59;        i += 6;
		eph.f1 = getbits(rtcm->buff, i, 21) * P2_46;        i += 21;
		eph.f0 = getbits(rtcm->buff, i, 31) * P2_34;        i += 31;
		eph.crs = getbits(rtcm->buff, i, 16) * P2_5;         i += 16;
		eph.deln = getbits(rtcm->buff, i, 16) * P2_43 * SC2RAD; i += 16;
		eph.m0 = getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD; i += 32;
		eph.cuc = getbits(rtcm->buff, i, 16) * P2_29;        i += 16;
		eph.e = getbitu(rtcm->buff, i, 32) * P2_33;        i += 32;
		eph.cus = getbits(rtcm->buff, i, 16) * P2_29;        i += 16;
		sqrta = getbitu(rtcm->buff, i, 32) * P2_19;        i += 32;
		eph.toes = getbitu(rtcm->buff, i, 14) * 60.0;         i += 14;
		eph.cic = getbits(rtcm->buff, i, 16) * P2_29;        i += 16;
		eph.omg0 = getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD; i += 32;
		eph.cis = getbits(rtcm->buff, i, 16) * P2_29;        i += 16;
		eph.i0 = getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD; i += 32;
		eph.crc = getbits(rtcm->buff, i, 16) * P2_5;         i += 16;
		eph.omg = getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD; i += 32;
		eph.omgd = getbits(rtcm->buff, i, 24) * P2_43 * SC2RAD; i += 24;
		eph.tgd = getbits(rtcm->buff, i, 10) * P2_32;        i += 10; /* E5a/E1 */
		e5a_hs = getbitu(rtcm->buff, i, 2);              i += 2; /* OSHS */
		e5a_dvs = getbitu(rtcm->buff, i, 1);              i += 1; /* OSDVS */
		//rsv       =getbitu(rtcm->buff,i, 7);
	}
	else {
		trace(2, "rtcm3 1045 length error: len=%d\r\n", rtcm->len);
		return -1;
	}
	trace(4, "decode_type1045: prn=%d iode=%d toe=%.0f\r\n", prn, eph.iode, eph.toes);

	if (rtcm->outtype) {
		msg = rtcm->msgtype + strlen(rtcm->msgtype);
		sprintf(msg, " prn=%2d iode=%3d week=%d toe=%6.0f toc=%6.0f hs=%d dvs=%d",
			prn, eph.iode, week, eph.toes, toc, e5a_hs, e5a_dvs);
	}
	if (!(sat = satno(sys, prn))) {
		trace(2, "rtcm3 1045 satellite number error: prn=%d\r\n", prn);
		return -1;
	}
	eph.sat = sat;
	eph.week = week + 1024; /* gal-week = gst-week + 1024 */
	eph.toe = gpst2time(eph.week, eph.toes);
	eph.toc = gpst2time(eph.week, toc);
	eph.ttr = rtcm->time;
	eph.A = sqrta * sqrta;
	eph.svh = (e5a_hs << 4) + (e5a_dvs << 3);
	eph.code = 2; /* data source = f/nav e5a */

	rtcm->nav.eph[rtcm->ephsat++] = eph;
	//rtcm->ephsat=sat;
	return 2;
}
/* decode type 1046: galileo satellite ephemerides (extension for IGS MGEX) --*/
static int decode_type1046(rtcm_t* rtcm)
{
	eph_t eph = { 0 };
	double toc = 0, sqrta = 0;
	char* msg = "";
	int type = getbitu(rtcm->buff, 24, 12);
	int i = 24 + 12, prn = 0, sat = 0, week = 0, e5b_hs = 0, e5b_dvs = 0, e1_hs = 0, e1_dvs = 0, sys = SYS_GAL;

	if (i + 492 <= rtcm->len * 8) {
		prn = getbitu(rtcm->buff, i, 6);              i += 6;
		week = getbitu(rtcm->buff, i, 12);              i += 12; /* gst-week */
		eph.iode = getbitu(rtcm->buff, i, 10);              i += 10;
		eph.sva = getbitu(rtcm->buff, i, 8);              i += 8;
		eph.idot = getbits(rtcm->buff, i, 14) * P2_43 * SC2RAD; i += 14;
		toc = getbitu(rtcm->buff, i, 14) * 60.0;         i += 14;
		eph.f2 = getbits(rtcm->buff, i, 6) * P2_59;        i += 6;
		eph.f1 = getbits(rtcm->buff, i, 21) * P2_46;        i += 21;
		eph.f0 = getbits(rtcm->buff, i, 31) * P2_34;        i += 31;
		eph.crs = getbits(rtcm->buff, i, 16) * P2_5;         i += 16;
		eph.deln = getbits(rtcm->buff, i, 16) * P2_43 * SC2RAD; i += 16;
		eph.m0 = getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD; i += 32;
		eph.cuc = getbits(rtcm->buff, i, 16) * P2_29;        i += 16;
		eph.e = getbitu(rtcm->buff, i, 32) * P2_33;        i += 32;
		eph.cus = getbits(rtcm->buff, i, 16) * P2_29;        i += 16;
		sqrta = getbitu(rtcm->buff, i, 32) * P2_19;        i += 32;
		eph.toes = getbitu(rtcm->buff, i, 14) * 60.0;         i += 14;
		eph.cic = getbits(rtcm->buff, i, 16) * P2_29;        i += 16;
		eph.omg0 = getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD; i += 32;
		eph.cis = getbits(rtcm->buff, i, 16) * P2_29;        i += 16;
		eph.i0 = getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD; i += 32;
		eph.crc = getbits(rtcm->buff, i, 16) * P2_5;         i += 16;
		eph.omg = getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD; i += 32;
		eph.omgd = getbits(rtcm->buff, i, 24) * P2_43 * SC2RAD; i += 24;
		eph.tgd = getbits(rtcm->buff, i, 10) * P2_32;        i += 10; /* E5a/E1 */
		eph.tgdb = getbits(rtcm->buff, i, 10) * P2_32;        i += 10; /* E5b/E1 */
		e5b_hs = getbitu(rtcm->buff, i, 2);              i += 2; /* OSHS */
		e5b_dvs = getbitu(rtcm->buff, i, 1);              i += 1; /* OSDVS */
		e1_hs = getbitu(rtcm->buff, i, 2);              i += 2; /* E1 OSHS */
		e1_dvs = getbitu(rtcm->buff, i, 1);              i += 1; /* E1 OSDVS */

		//rsv       =getbitu(rtcm->buff,i, 7);
	}
	else {
		trace(2, "rtcm3 1046 length error: len=%d\r\n", rtcm->len);
		return -1;
	}
	trace(4, "decode_type1046: prn=%d iode=%d toe=%.0f\r\n", prn, eph.iode, eph.toes);

	if (!(sat = satno(sys, prn))) {
		trace(2, "rtcm3 1046 satellite number error: prn=%d\r\n", prn);
		return -1;
	}
	eph.sva = 0;  //避免gal参与单点解算时权重较小，此处强制置零
	eph.sat = sat;
	eph.week = week + 1024; /* gal-week = gst-week + 1024 */
	eph.toe = gpst2time(eph.week, eph.toes);
	eph.toc = gpst2time(eph.week, toc);
	eph.ttr = rtcm->time;
	eph.A = sqrta * sqrta;
	eph.svh = (e5b_hs << 7) + (e5b_dvs << 6) + (e1_hs << 1) + (e1_dvs << 0);
	eph.code = 2; /* data source = f/nav e5a */

	if (rtcm->ephsat < (MAXRECOBS * 2))
		rtcm->nav.eph[rtcm->ephsat++] = eph;
	//rtcm->ephsat=sat;
	return 2;
}
/* decode type 1047: beidou ephemerides (tentative mt and format) ------------*/
static int decode_type1047(rtcm_t* rtcm)
{
	eph_t eph = { 0 };
	double toc = 0, sqrta = 0;
	char* msg = "";
	int i = 24 + 12, prn = 0, sat = 0, week = 0, sys = SYS_CMP;

	if (i + 476 <= rtcm->len * 8) {
		prn = getbitu(rtcm->buff, i, 6);              i += 6;
		week = getbitu(rtcm->buff, i, 10);              i += 10;
		eph.sva = getbitu(rtcm->buff, i, 4);              i += 4;
		eph.code = getbitu(rtcm->buff, i, 2);              i += 2;
		eph.idot = getbits(rtcm->buff, i, 14) * P2_43 * SC2RAD; i += 14;
		eph.iode = getbitu(rtcm->buff, i, 8);              i += 8;
		toc = getbitu(rtcm->buff, i, 16) * 16.0;         i += 16;
		eph.f2 = getbits(rtcm->buff, i, 8) * P2_55;        i += 8;
		eph.f1 = getbits(rtcm->buff, i, 16) * P2_43;        i += 16;
		eph.f0 = getbits(rtcm->buff, i, 22) * P2_31;        i += 22;
		eph.iodc = getbitu(rtcm->buff, i, 10);              i += 10;
		eph.crs = getbits(rtcm->buff, i, 16) * P2_5;         i += 16;
		eph.deln = getbits(rtcm->buff, i, 16) * P2_43 * SC2RAD; i += 16;
		eph.m0 = getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD; i += 32;
		eph.cuc = getbits(rtcm->buff, i, 16) * P2_29;        i += 16;
		eph.e = getbitu(rtcm->buff, i, 32) * P2_33;        i += 32;
		eph.cus = getbits(rtcm->buff, i, 16) * P2_29;        i += 16;
		sqrta = getbitu(rtcm->buff, i, 32) * P2_19;        i += 32;
		eph.toes = getbitu(rtcm->buff, i, 16) * 16.0;         i += 16;
		eph.cic = getbits(rtcm->buff, i, 16) * P2_29;        i += 16;
		eph.omg0 = getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD; i += 32;
		eph.cis = getbits(rtcm->buff, i, 16) * P2_29;        i += 16;
		eph.i0 = getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD; i += 32;
		eph.crc = getbits(rtcm->buff, i, 16) * P2_5;         i += 16;
		eph.omg = getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD; i += 32;
		eph.omgd = getbits(rtcm->buff, i, 24) * P2_43 * SC2RAD; i += 24;
		eph.tgd = getbits(rtcm->buff, i, 8) * P2_31;        i += 8;
		eph.svh = getbitu(rtcm->buff, i, 6);              i += 6;
		eph.flag = getbitu(rtcm->buff, i, 1);              i += 1;
		eph.fit = getbitu(rtcm->buff, i, 1) ? 0.0 : 4.0; /* 0:4hr,1:>4hr */
	}
	else {
		trace(2, "rtcm3 1047 length error: len=%d\r\n", rtcm->len);
		return -1;
	}
	trace(4, "decode_type1047: prn=%d iode=%d toe=%.0f\r\n", prn, eph.iode, eph.toes);

	if (rtcm->outtype) {
		msg = rtcm->msgtype + strlen(rtcm->msgtype);
		sprintf(msg, " prn=%2d iode=%3d iodc=%3d week=%d toe=%6.0f toc=%6.0f svh=%02X",
			prn, eph.iode, eph.iodc, week, eph.toes, toc, eph.svh);
	}
	if (!(sat = satno(sys, prn))) {
		trace(2, "rtcm3 1047 satellite number error: prn=%d\r\n", prn);
		return -1;
	}
	eph.sat = sat;
	eph.week = adjbdtweek(week);
	eph.toe = bdt2gpst(bdt2time(eph.week, eph.toes)); /* bdt -> gpst */
	eph.toc = bdt2gpst(bdt2time(eph.week, toc));      /* bdt -> gpst */
	eph.ttr = rtcm->time;
	eph.A = sqrta * sqrta;
	if (rtcm->ephsat < (MAXRECOBS * 2))
		rtcm->nav.eph[rtcm->ephsat++] = eph;
	return 2;
}
/* decode type 63: beidou ephemerides (rtcm draft) ---------------------------*/
static int decode_type63(rtcm_t* rtcm)
{
	eph_t eph = { 0 };
	double toc = 0, sqrta = 0;
	char* msg = "";
	int i = 24 + 12, prn = 0, sat = 0, week = 0, sys = SYS_CMP;

	if (i + 499 <= rtcm->len * 8) {
		prn = (int)getbitu(rtcm->buff, i, 6);              i += 6;
		week = (int)getbitu(rtcm->buff, i, 13);              i += 13;
		eph.sva = (int)getbitu(rtcm->buff, i, 4);              i += 4;
		eph.idot = getbits(rtcm->buff, i, 14) * P2_43 * SC2RAD; i += 14;
		eph.iode = (int)getbitu(rtcm->buff, i, 5);              i += 5; /* AODE */
		toc = getbitu(rtcm->buff, i, 17) * 8.0;          i += 17;
		eph.f2 = getbits(rtcm->buff, i, 11) * P2_66;        i += 11;
		eph.f1 = getbits(rtcm->buff, i, 22) * P2_50;        i += 22;
		eph.f0 = getbits(rtcm->buff, i, 24) * P2_33;        i += 24;
		eph.iodc = (int)getbitu(rtcm->buff, i, 5);              i += 5; /* AODC */
		eph.crs = getbits(rtcm->buff, i, 18) * P2_6;         i += 18;
		eph.deln = getbits(rtcm->buff, i, 16) * P2_43 * SC2RAD; i += 16;
		eph.m0 = getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD; i += 32;
		eph.cuc = getbits(rtcm->buff, i, 18) * P2_31;        i += 18;
		eph.e = getbitu(rtcm->buff, i, 32) * P2_33;        i += 32;
		eph.cus = getbits(rtcm->buff, i, 18) * P2_31;        i += 18;
		sqrta = getbitu(rtcm->buff, i, 32) * P2_19;        i += 32;
		eph.toes = getbitu(rtcm->buff, i, 17) * 8.0;          i += 17;
		eph.cic = getbits(rtcm->buff, i, 18) * P2_31;        i += 18;
		eph.omg0 = getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD; i += 32;
		eph.cis = getbits(rtcm->buff, i, 18) * P2_31;        i += 18;
		eph.i0 = getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD; i += 32;
		eph.crc = getbits(rtcm->buff, i, 18) * P2_6;         i += 18;
		eph.omg = getbits(rtcm->buff, i, 32) * P2_31 * SC2RAD; i += 32;
		eph.omgd = getbits(rtcm->buff, i, 24) * P2_43 * SC2RAD; i += 24;
		eph.tgd = getbits(rtcm->buff, i, 10) * 1E-10;        i += 10;
		eph.tgdb = getbits(rtcm->buff, i, 10) * 1E-10;        i += 10;
		eph.svh = (int)getbitu(rtcm->buff, i, 1);              i += 1;
	}
	else {
		trace(2, "rtcm3 63 length error: len=%d\r\n", rtcm->len);
		return -1;
	}
	//trace(4,"decode_type63: prn=%d iode=%d toe=%.0f\r\n",prn,eph.iode,eph.toes);

	if (rtcm->outtype) {
		msg = rtcm->msgtype + strlen(rtcm->msgtype);
		trace(5, " prn=%2d iode=%3d iodc=%3d week=%d toe=%6.0f toc=%6.0f svh=%02X",
			prn, eph.iode, eph.iodc, week, eph.toes, toc, eph.svh);
	}
	if (!(sat = satno(sys, prn))) {
		trace(2, "rtcm3 63 satellite number error: prn=%d\r\n", prn);
		return -1;
	}
	eph.sat = sat;
	eph.week = adjbdtweek(week);
	g_rtcm_week = eph.week + 1356;

	eph.toe = bdt2gpst(bdt2time(eph.week, eph.toes)); /* bdt -> gpst */
	g_gps_toe = eph.toes;

	eph.toc = bdt2gpst(bdt2time(eph.week, toc));      /* bdt -> gpst */
	eph.ttr = rtcm->time;
	eph.A = sqrta * sqrta;

	trace(2, "decode_type63: prn=%d iode=%d toe=%.0f toe is %s  sqrtA=%f svh=%d toc=%f cis=%f\r\n", prn, eph.iode, eph.toes, time_str(eph.toe, 3), sqrta, eph.svh, toc, eph.cis * 1e6);

	if (rtcm->ephsat < (MAXRECOBS * 2))
		rtcm->nav.eph[rtcm->ephsat++] = eph;
	return 2;
}

/* get signal index ----------------------------------------------------------*/
static void sigindex(int sys, const unsigned char* code, const int* freq, int n,
	const char* opt, int* ind)
{
	int i = 0, nex = 0, pri = 0, pri_h[8] = { 0 }, index[8] = { 0 }, ex[32] = { 0 };

	/* test code priority */
	for (i = 0; i < n; i++) {
		if (!code[i]) continue;

		if (freq[i] > NFREQ) { /* save as extended signal if freq > NFREQ */
			ex[i] = 1;
			continue;
		}
		/* code priority */
		pri = getcodepri(sys, code[i], opt);

		/* select highest priority signal */
		if (pri > pri_h[freq[i] - 1]) {
			if (index[freq[i] - 1]) ex[index[freq[i] - 1] - 1] = 1;
			pri_h[freq[i] - 1] = pri;
			index[freq[i] - 1] = i + 1;
		}
		else ex[i] = 1;
	}
	/* signal index in obs data */
	for (i = nex = 0; i < n; i++) {
		if (ex[i] == 0) ind[i] = freq[i] - 1;
		else if (nex < NEXOBS) ind[i] = NFREQ + nex++;
		else { /* no space in obs data */
			trace(2, "rtcm msm: no space in obs data sys=%d code=%d\r\n", sys, code[i]);
			ind[i] = -1;
		}
#if 0
		trace(2, "sig pos: sys=%d code=%d ex=%d ind=%d\n", sys, code[i], ex[i], ind[i]);
#endif
	}
}
/* save obs data in msm message ----------------------------------------------*/
static void save_msm_obs(rtcm_t* rtcm, int sys, msm_h_t* h, const double* r,
	const double* pr, const double* cp, const double* rr,
	const double* rrf, const double* cnr, const int* lock,
	const int* ex, const int* half)
{
	const char* sig[32];
	double tt = 0, wl = 0;
	unsigned char code[32];
	char* msm_type = "", * q = NULL;
	int i = 0, j = 0, k = 0, type = 0, prn = 0, sat = 0, fn = 0, index = 0, freq[32] = { 0 }, ind[32] = { 0 };

	/* msm signal id table -------------------------------------------------------*/
	const char* msm_sig_gps[32] = {
		/* GPS: ref [13] table 3.5-87, ref [14][15] table 3.5-91 */
		""  ,"1C","1P","1W","1Y","1M",""  ,"2C","2P","2W","2Y","2M", /*  1-12 */
		""  ,""  ,"2S","2L","2X",""  ,""  ,""  ,""  ,"5I","5Q","5X", /* 13-24 */
		""  ,""  ,""  ,""  ,""  ,"1S","1L","1X"                      /* 25-32 */
	};
	const char* msm_sig_glo[32] = {
		/* GLONASS: ref [13] table 3.5-93, ref [14][15] table 3.5-97 */
		""  ,"1C","1P",""  ,""  ,""  ,""  ,"2C","2P",""  ,"3I","3Q",
		"3X",""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,
		""  ,""  ,""  ,""  ,""  ,""  ,""  ,""
	};
	const char* msm_sig_gal[32] = {
		/* Galileo: ref [15] table 3.5-100 */
		""  ,"1C","1A","1B","1X","1Z",""  ,"6C","6A","6B","6X","6Z",
		""  ,"7I","7Q","7X",""  ,"8I","8Q","8X",""  ,"5I","5Q","5X",
		""  ,""  ,""  ,""  ,""  ,""  ,""  ,""
	};
	const char* msm_sig_qzs[32] = {
		/* QZSS: ref [15] table 3.5-103 */
		""  ,"1C",""  ,""  ,""  ,""  ,""  ,""  ,"6S","6L","6X",""  ,
		""  ,""  ,"2S","2L","2X",""  ,""  ,""  ,""  ,"5I","5Q","5X",
		""  ,""  ,""  ,""  ,""  ,"1S","1L","1X"
	};
	const char* msm_sig_sbs[32] = {
		/* SBAS: ref [13] table 3.5-T+005 */
		""  ,"1C",""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,
		""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,"5I","5Q","5X",
		""  ,""  ,""  ,""  ,""  ,""  ,""  ,""
	};
	const char* msm_sig_cmp[32] = {
		/* BeiDou: ref [15] table 3.5-106 */
		""  ,"1I","1Q","1X",""  ,""  ,""  ,"6I","6Q","6X",""  ,""  , /*  1-12 */
		""  ,"7I","7Q","7X",""  ,""  ,"5I"  ,""  ,""  ,""  ,"5I"  ,""  ,/* 13-24 */
		""  ,""  ,""  ,""  ,""  ,""  ,""  ,""
	};

	type = getbitu(rtcm->buff, 24, 12);

	switch (sys) {
	case SYS_GPS: msm_type = q = rtcm->msmtype[0]; break;
	case SYS_GLO: msm_type = q = rtcm->msmtype[1]; break;
	case SYS_GAL: msm_type = q = rtcm->msmtype[2]; break;
	case SYS_QZS: msm_type = q = rtcm->msmtype[3]; break;
	case SYS_SBS: msm_type = q = rtcm->msmtype[4]; break;
	case SYS_CMP: msm_type = q = rtcm->msmtype[5]; break;
	}
	/* id to signal */
	for (i = 0; i < h->nsig; i++) {
		// trace(3,"i=%2d sys=%2d sigs=%d\n",i,sys,h->sigs[i]);
		switch (sys) {
		case SYS_GPS: sig[i] = msm_sig_gps[h->sigs[i] - 1]; break;
		case SYS_GLO: sig[i] = msm_sig_glo[h->sigs[i] - 1]; break;
		case SYS_GAL: sig[i] = msm_sig_gal[h->sigs[i] - 1]; break;
		case SYS_QZS: sig[i] = msm_sig_qzs[h->sigs[i] - 1]; break;
		case SYS_SBS: sig[i] = msm_sig_sbs[h->sigs[i] - 1]; break;
		case SYS_CMP: sig[i] = msm_sig_cmp[h->sigs[i] - 1]; break;
		default: sig[i] = ""; break;
		}

		/* signal to rinex obs type */
		code[i] = obs2code(sig[i], freq + i);
		//trace(3,"i=%2d sys=%2d sigs=%d freq=%d sig=%s \n",i,sys,h->sigs[i],freq[i],sig[i]);
		/* freqency index for beidou */
		if (sys == SYS_CMP) {
			if (FREQMODE_CMP == 0)//B1 B2  B3模式
			{
				if (freq[i] == 5) freq[i] = 2; /* B2 */
			}
			else
			{
				if (freq[i] == 3) freq[i] = 2; /* b1c B2a(C5I) */
			}
			if (freq[i] == 4) freq[i] = 3; /* B3 */
		}
		if (sys == SYS_GPS) {
			if (FREQMODE_CMP == 1) {//L1+L5 B1+B2a模式
				if (freq[i] == 2) freq[i] = 3;//调整GPS L2频freq值为3
				else if (freq[i] == 3) freq[i] = 2;//调整GPS L5频freq值为2
			}
		}
		if (sys == SYS_GAL) {
			if (FREQMODE_CMP == 1) {//L1+L5 E1+E5a模式
				if (freq[i] == 2) freq[i] = 3;//调整GAL L2频freq值为3
				else if (freq[i] == 3) freq[i] = 2;//调整GAL L5频freq值为2
			}
		}
		if (code[i] != CODE_NONE) {
			if (q) q += sprintf(q, "L%s%s", sig[i], i < h->nsig - 1 ? "," : "");
		}
		else {
			if (q) q += sprintf(q, "(%d)%s", h->sigs[i], i < h->nsig - 1 ? "," : "");

			trace(2, "rtcm3 %d: unknown signal id=%2d\r\n", type, h->sigs[i]);
		}
		//trace(3,"final i=%2d sys=%2d sigs=%d freq=%d sig=%s\r\n",i,sys,h->sigs[i],freq[i],sig[i]);
	}

	/* get signal index */
	sigindex(sys, code, freq, h->nsig, rtcm->opt, ind);

	for (i = j = 0; i < h->nsat; i++) {
		prn = h->sats[i];
		if (sys == SYS_QZS) prn += MINPRNQZS - 1;
		else if (sys == SYS_SBS) prn += MINPRNSBS - 1;

		if ((sat = satno(sys, prn))) {
			tt = timediff(rtcm->obs.data[0].time, rtcm->time);
			if (rtcm->obsflag || fabs(tt) > 1E-9) {
				rtcm->obs.n = rtcm->obsflag = 0;
			}
			index = obsindex(&rtcm->obs, rtcm->time, sat);
		}
		else {
			trace(2, "rtcm3 %d satellite error: prn=%d\r\n", type, prn);
		}
		for (k = 0; k < h->nsig; k++) {
			//trace(3,"sys=%2d k=%d/%d freq=%2d  cellmask=%d,sat=%2d index=%d ind[k]=%d\r\n",sys,k,h->nsig,freq[k],h->cellmask[k+i*h->nsig],sat,index,ind[k]);
			if (!h->cellmask[k + i * h->nsig]) continue;

			if (sat && index >= 0 && ind[k] >= 0) {
				/* satellite carrier wave length */
				wl = satwavelen(sat, freq[k] - 1, &rtcm->nav);
				// trace(3,"sys=%2d k=%d freq=%2d wl=%f\r\n",sys,k,freq[k],wl);
				 /* glonass wave length by extended info */
				if (sys == SYS_GLO && ex && ex[i] <= 13) {
					fn = ex[i] - 7;
					wl = CLIGHT / ((freq[k] == 2 ? FREQ2_GLO : FREQ1_GLO) +
						(freq[k] == 2 ? DFRQ2_GLO : DFRQ1_GLO) * fn);
				}
				/* pseudorange (m) */
				if (r[i] != 0.0 && pr[j] > -1E12) {
					rtcm->obs.data[index].P[ind[k]] = r[i] + pr[j];
				}
				/* carrier-phase (cycle) */
				if (r[i] != 0.0 && cp[j] > -1E12 && wl > 0.0) {
					rtcm->obs.data[index].L[ind[k]] = (r[i] + cp[j]) / wl;
				}
				/* doppler (hz) */
				if (rr && rrf && rrf[j] > -1E12 && wl > 0.0) {
					rtcm->obs.data[index].D[ind[k]] = (float)(-(rr[i] + rrf[j]) / wl);
				}
				rtcm->obs.data[index].lli[ind[k]] =
					lossoflock(rtcm, sat, index, ind[k], lock[j]) + (half[j] ? 2 : 0);
				rtcm->obs.data[index].snr[ind[k]] = (unsigned char)(cnr[j] * 4.0);
				rtcm->obs.data[index].code[ind[k]] = code[k];
				//trace(3,"traceobs:sys=%2d k=%3d j=%3d index=%3d sigs=%2d ind[k]=%3d sat=%3d P=%13.3f L=%13.3f wl=%5.3f D=%f code[ind[k]]=%d\n",sys,k,j,index,h->sigs[k],ind[k],sat, rtcm->obs.data[index].P[ind[k]],rtcm->obs.data[index].L[ind[k]],wl,rtcm->obs.data[index].D[ind[k]],rtcm->obs.data[index].code[ind[k]]);
			}
			j++;
		}
	}
}
/* decode type msm message header --------------------------------------------*/
static int decode_msm_head(rtcm_t* rtcm, int sys, int* sync, int* iod,
	msm_h_t* h, int* hsize)
{
	msm_h_t h0 = { 0 };
	double tow = 0, tod = 0;
	char* msg = "";
	int i = 24, j = 0, mask = 0, staid = 0, type = 0, ncell = 0;

	type = getbitu(rtcm->buff, i, 12); i += 12;

	*h = h0;
	if (i + 157 <= rtcm->len * 8) {
		staid = getbitu(rtcm->buff, i, 12);       i += 12;

		if (sys == SYS_GLO) {
			//dow   =getbitu(rtcm->buff,i, 3);       i+= 3;
			tod = getbitu(rtcm->buff, i, 27) * 0.001; i += 27;
			adjday_glot3(rtcm, tod);
		}
		else if (sys == SYS_CMP) {
			tow = getbitu(rtcm->buff, i, 30) * 0.001; i += 30;
			tow += 14.0; /* BDT -> GPST */
			adjweek3(rtcm, tow);
		}
		else {
			tow = getbitu(rtcm->buff, i, 30) * 0.001; i += 30;
			adjweek3(rtcm, tow);
		}
		*sync = getbitu(rtcm->buff, i, 1);       i += 1;
		*iod = getbitu(rtcm->buff, i, 3);       i += 3;
		h->time_s = getbitu(rtcm->buff, i, 7);       i += 7;
		h->clk_str = getbitu(rtcm->buff, i, 2);       i += 2;
		h->clk_ext = getbitu(rtcm->buff, i, 2);       i += 2;
		h->smooth = getbitu(rtcm->buff, i, 1);       i += 1;
		h->tint_s = getbitu(rtcm->buff, i, 3);       i += 3;
		for (j = 1; j <= 64; j++) {
			mask = getbitu(rtcm->buff, i, 1); i += 1;
			if (mask) h->sats[h->nsat++] = j;
		}
		for (j = 1; j <= 32; j++) {
			mask = getbitu(rtcm->buff, i, 1); i += 1;
			if (mask) h->sigs[h->nsig++] = j;
		}
	}
	else {
		trace(2, "rtcm3 %d length error: len=%d\r\n", type, rtcm->len);
		return -1;
	}
	/* test station id */
	if (!test_staid(rtcm, staid)) return -1;

	if (h->nsat * h->nsig > 64) {
		trace(2, "rtcm3 %d number of sats and sigs error: nsat=%d nsig=%d\r\n",
			type, h->nsat, h->nsig);
		return -1;
	}
	if (i + h->nsat * h->nsig > rtcm->len * 8) {
		trace(2, "rtcm3 %d length error: len=%d nsat=%d nsig=%d\r\n", type,
			rtcm->len, h->nsat, h->nsig);
		return -1;
	}
	for (j = 0; j < h->nsat * h->nsig; j++) {
		h->cellmask[j] = getbitu(rtcm->buff, i, 1); i += 1;
		if (h->cellmask[j]) ncell++;
	}
	*hsize = i;

	return ncell;
}

/* decode msm 4: full pseudorange and phaserange plus cnr --------------------*/
static int decode_msm4(rtcm_t* rtcm, int sys)
{
	msm_h_t h = { 0 };
	double r[64] = { 0 }, pr[64] = { 0 }, cp[64] = { 0 }, cnr[64] = { 0 };
	int i = 0, j = 0, type = 0, sync = 0, iod = 0, ncell = 0, rng = 0, rng_m = 0, prv = 0, cpv = 0, lock[64] = { 0 }, half[64] = { 0 };

	type = getbitu(rtcm->buff, 24, 12);
	if (g_rtcm_week == 0 && g_gps_toe == 0)
	{
		trace(2, "decode_msm4: decode failed by no eph week\n");
		return -1;
	}

	/* decode msm header */
	if ((ncell = decode_msm_head(rtcm, sys, &sync, &iod, &h, &i)) < 0) return -1;

	if (i + h.nsat * 18 + ncell * 48 > rtcm->len * 8) {
		trace(2, "rtcm3 %d length error: nsat=%d ncell=%d len=%d\r\n", type, h.nsat,
			ncell, rtcm->len);
		return -1;
	}
	for (j = 0; j < h.nsat; j++) r[j] = 0.0;
	for (j = 0; j < ncell; j++) pr[j] = cp[j] = -1E16;

	/* decode satellite data */
	for (j = 0; j < h.nsat; j++) { /* range */
		rng = getbitu(rtcm->buff, i, 8); i += 8;
		if (rng != 255) r[j] = rng * RANGE_MS;
	}
	for (j = 0; j < h.nsat; j++) {
		rng_m = getbitu(rtcm->buff, i, 10); i += 10;
		if (r[j] != 0.0) r[j] += rng_m * P2_10 * RANGE_MS;
	}
	/* decode signal data */
	for (j = 0; j < ncell; j++) { /* pseudorange */
		prv = getbits(rtcm->buff, i, 15); i += 15;
		if (prv != -16384) pr[j] = prv * P2_24 * RANGE_MS;
	}
	for (j = 0; j < ncell; j++) { /* phaserange */
		cpv = getbits(rtcm->buff, i, 22); i += 22;
		if (cpv != -2097152) cp[j] = cpv * P2_29 * RANGE_MS;
	}
	for (j = 0; j < ncell; j++) { /* lock time */
		lock[j] = getbitu(rtcm->buff, i, 4); i += 4;
	}
	for (j = 0; j < ncell; j++) { /* half-cycle ambiguity */
		half[j] = getbitu(rtcm->buff, i, 1); i += 1;
	}
	for (j = 0; j < ncell; j++) { /* cnr */
		cnr[j] = getbitu(rtcm->buff, i, 6) * 1.0; i += 6;
	}
	/* save obs data in msm message */
	save_msm_obs(rtcm, sys, &h, r, pr, cp, NULL, NULL, cnr, lock, NULL, half);

	//rtcm->obsflag=!sync;
	return 1;
}
/* decode msm 5: full pseudorange, phaserange, phaserangerate and cnr --------*/
static int decode_msm5(rtcm_t* rtcm, int sys)
{
	msm_h_t h = { 0 };
	double r[64] = { 0 }, rr[64] = { 0 }, pr[64] = { 0 }, cp[64] = { 0 }, rrf[64] = { 0 }, cnr[64] = { 0 };
	int i = 0, j = 0, type = 0, sync = 0, iod = 0, ncell = 0, rng = 0, rng_m = 0, rate = 0, prv = 0, cpv = 0, rrv = 0, lock[64] = { 0 };
	int ex[64] = { 0 }, half[64] = { 0 };

	type = getbitu(rtcm->buff, 24, 12);

	if (g_rtcm_week == 0 && g_gps_toe == 0)
	{
		trace(2, "decode_msm4: decode failed by no eph week");
		return -1;
	}

	/* decode msm header */
	if ((ncell = decode_msm_head(rtcm, sys, &sync, &iod, &h, &i)) < 0) return -1;

	if (i + h.nsat * 36 + ncell * 63 > rtcm->len * 8) {
		trace(2, "rtcm3 %d length error: nsat=%d ncell=%d len=%d\r\n", type, h.nsat,
			ncell, rtcm->len);
		return -1;
	}
	for (j = 0; j < h.nsat; j++) {
		r[j] = rr[j] = 0.0; ex[j] = 15;
	}
	for (j = 0; j < ncell; j++) pr[j] = cp[j] = rrf[j] = -1E16;

	/* decode satellite data */
	for (j = 0; j < h.nsat; j++) { /* range */
		rng = getbitu(rtcm->buff, i, 8); i += 8;
		if (rng != 255) r[j] = rng * RANGE_MS;
	}
	for (j = 0; j < h.nsat; j++) { /* extended info */
		ex[j] = getbitu(rtcm->buff, i, 4); i += 4;
	}
	for (j = 0; j < h.nsat; j++) {
		rng_m = getbitu(rtcm->buff, i, 10); i += 10;
		if (r[j] != 0.0) r[j] += rng_m * P2_10 * RANGE_MS;
	}
	for (j = 0; j < h.nsat; j++) { /* phaserangerate */
		rate = getbits(rtcm->buff, i, 14); i += 14;
		if (rate != -8192) rr[j] = rate * 1.0;
	}
	/* decode signal data */
	for (j = 0; j < ncell; j++) { /* pseudorange */
		prv = getbits(rtcm->buff, i, 15); i += 15;
		if (prv != -16384) pr[j] = prv * P2_24 * RANGE_MS;
	}
	for (j = 0; j < ncell; j++) { /* phaserange */
		cpv = getbits(rtcm->buff, i, 22); i += 22;
		if (cpv != -2097152) cp[j] = cpv * P2_29 * RANGE_MS;
	}
	for (j = 0; j < ncell; j++) { /* lock time */
		lock[j] = getbitu(rtcm->buff, i, 4); i += 4;
	}
	for (j = 0; j < ncell; j++) { /* half-cycle ambiguity */
		half[j] = getbitu(rtcm->buff, i, 1); i += 1;
	}
	for (j = 0; j < ncell; j++) { /* cnr */
		cnr[j] = getbitu(rtcm->buff, i, 6) * 1.0; i += 6;
	}
	for (j = 0; j < ncell; j++) { /* phaserangerate */
		rrv = getbits(rtcm->buff, i, 15); i += 15;
		if (rrv != -16384) rrf[j] = rrv * 0.0001;
	}
	/* save obs data in msm message */
	save_msm_obs(rtcm, sys, &h, r, pr, cp, rr, rrf, cnr, lock, ex, half);

	//rtcm->obsflag=!sync;
	return 1;
}
/* decode msm 6: full pseudorange and phaserange plus cnr (high-res) ---------*/
static int decode_msm6(rtcm_t* rtcm, int sys)
{
	msm_h_t h = { 0 };
	double r[64] = { 0 }, pr[64] = { 0 }, cp[64] = { 0 }, cnr[64] = { 0 };
	int i = 0, j = 0, type = 0, sync = 0, iod = 0, ncell = 0, rng = 0, rng_m = 0, prv = 0, cpv = 0, lock[64] = { 0 }, half[64] = { 0 };

	type = getbitu(rtcm->buff, 24, 12);

	/* decode msm header */
	if ((ncell = decode_msm_head(rtcm, sys, &sync, &iod, &h, &i)) < 0) return -1;

	if (i + h.nsat * 18 + ncell * 65 > rtcm->len * 8) {
		trace(2, "rtcm3 %d length error: nsat=%d ncell=%d len=%d\r\n", type, h.nsat,
			ncell, rtcm->len);
		return -1;
	}
	for (j = 0; j < h.nsat; j++) r[j] = 0.0;
	for (j = 0; j < ncell; j++) pr[j] = cp[j] = -1E16;

	/* decode satellite data */
	for (j = 0; j < h.nsat; j++) { /* range */
		rng = getbitu(rtcm->buff, i, 8); i += 8;
		if (rng != 255) r[j] = rng * RANGE_MS;
	}
	for (j = 0; j < h.nsat; j++) {
		rng_m = getbitu(rtcm->buff, i, 10); i += 10;
		if (r[j] != 0.0) r[j] += rng_m * P2_10 * RANGE_MS;
	}
	/* decode signal data */
	for (j = 0; j < ncell; j++) { /* pseudorange */
		prv = getbits(rtcm->buff, i, 20); i += 20;
		if (prv != -524288) pr[j] = prv * P2_29 * RANGE_MS;
	}
	for (j = 0; j < ncell; j++) { /* phaserange */
		cpv = getbits(rtcm->buff, i, 24); i += 24;
		if (cpv != -8388608) cp[j] = cpv * P2_31 * RANGE_MS;
	}
	for (j = 0; j < ncell; j++) { /* lock time */
		lock[j] = getbitu(rtcm->buff, i, 10); i += 10;
	}
	for (j = 0; j < ncell; j++) { /* half-cycle ambiguity */
		half[j] = getbitu(rtcm->buff, i, 1); i += 1;
	}
	for (j = 0; j < ncell; j++) { /* cnr */
		cnr[j] = getbitu(rtcm->buff, i, 10) * 0.0625; i += 10;
	}
	/* save obs data in msm message */
	save_msm_obs(rtcm, sys, &h, r, pr, cp, NULL, NULL, cnr, lock, NULL, half);

	//rtcm->obsflag=!sync;
	return 1;
}
/* decode msm 7: full pseudorange, phaserange, phaserangerate and cnr (h-res) */
static int decode_msm7(rtcm_t* rtcm, int sys)
{
	msm_h_t h = { 0 };
	double r[64] = { 0 }, rr[64] = { 0 }, pr[64] = { 0 }, cp[64] = { 0 }, rrf[64] = { 0 }, cnr[64] = { 0 };
	int i = 0, j = 0, type = 0, sync = 0, iod = 0, ncell = 0, rng = 0, rng_m = 0, rate = 0, prv = 0, cpv = 0, rrv = 0, lock[64] = { 0 };
	int ex[64] = { 0 }, half[64] = { 0 };

	type = getbitu(rtcm->buff, 24, 12);

	/* decode msm header */
	if ((ncell = decode_msm_head(rtcm, sys, &sync, &iod, &h, &i)) < 0) return -1;

	if (i + h.nsat * 36 + ncell * 80 > rtcm->len * 8) {
		trace(2, "rtcm3 %d length error: nsat=%d ncell=%d len=%d\r\n", type, h.nsat,
			ncell, rtcm->len);
		return -1;
	}
	for (j = 0; j < h.nsat; j++) {
		r[j] = rr[j] = 0.0; ex[j] = 15;
	}
	for (j = 0; j < ncell; j++) pr[j] = cp[j] = rrf[j] = -1E16;

	/* decode satellite data */
	for (j = 0; j < h.nsat; j++) { /* range */
		rng = getbitu(rtcm->buff, i, 8); i += 8;
		if (rng != 255) r[j] = rng * RANGE_MS;
	}
	for (j = 0; j < h.nsat; j++) { /* extended info */
		ex[j] = getbitu(rtcm->buff, i, 4); i += 4;
	}
	for (j = 0; j < h.nsat; j++) {
		rng_m = getbitu(rtcm->buff, i, 10); i += 10;
		if (r[j] != 0.0) r[j] += rng_m * P2_10 * RANGE_MS;
	}
	for (j = 0; j < h.nsat; j++) { /* phaserangerate */
		rate = getbits(rtcm->buff, i, 14); i += 14;
		if (rate != -8192) rr[j] = rate * 1.0;
	}
	/* decode signal data */
	for (j = 0; j < ncell; j++) { /* pseudorange */
		prv = getbits(rtcm->buff, i, 20); i += 20;
		if (prv != -524288) pr[j] = prv * P2_29 * RANGE_MS;
	}
	for (j = 0; j < ncell; j++) { /* phaserange */
		cpv = getbits(rtcm->buff, i, 24); i += 24;
		if (cpv != -8388608) cp[j] = cpv * P2_31 * RANGE_MS;
	}
	for (j = 0; j < ncell; j++) { /* lock time */
		lock[j] = getbitu(rtcm->buff, i, 10); i += 10;
	}
	for (j = 0; j < ncell; j++) { /* half-cycle amiguity */
		half[j] = getbitu(rtcm->buff, i, 1); i += 1;
	}
	for (j = 0; j < ncell; j++) { /* cnr */
		cnr[j] = getbitu(rtcm->buff, i, 10) * 0.0625; i += 10;
	}
	for (j = 0; j < ncell; j++) { /* phaserangerate */
		rrv = getbits(rtcm->buff, i, 15); i += 15;
		if (rrv != -16384) rrf[j] = rrv * 0.0001;
	}
	/* save obs data in msm message */
	save_msm_obs(rtcm, sys, &h, r, pr, cp, rr, rrf, cnr, lock, ex, half);

	//rtcm->obsflag=!sync;
	return 1;
}

static int decode_rtcm3_lg69t_pvt(rtcm_t* rtcm)
{
	unsigned int sub = 0, sta = 0, qual = 0, nsat_u = 0, nsat_v = 0, dsta = 0, eptime = 0;
	int    age = 0;
	double lat = 0, lon = 0, ght = 0;
	double vh = 0, vv = 0;
	double dir = 0;
	double cbias = 0, cdrift = 0;
	unsigned int year = 0;
	double dop[3] = { 0 }, h = 0;
	unsigned int week = 0;
	int fixin = 0;
	int dastatus = 0;
	double a = 0, b = 0, c = 0;
	int i = 24 + 12;

	sub = getbitu(rtcm->buff, i, 8);     i += 8;
	//trace(2,"decode_rtcm3_lg69t_pvt subTypeID:%d\r\n",sub);

	if (sub != 21) return -1;
	sta = getbitu(rtcm->buff, i, 12);            i += 12;
	year = getbitu(rtcm->buff, i, 6);            i += 6;
	qual = getbitu(rtcm->buff, i, 4);            i += 4;
	dastatus = getbits(rtcm->buff, i, 1);        i += 1; i += 1;
	fixin = getbits(rtcm->buff, i, 1);        i += 1; i += 1;
	nsat_u = getbitu(rtcm->buff, i, 8);            i += 8;
	nsat_v = getbitu(rtcm->buff, i, 8);            i += 8;
	dop[0] = getbitu(rtcm->buff, i, 8) * 0.1;        i += 8;
	dop[1] = getbitu(rtcm->buff, i, 8) * 0.1;        i += 8;
	dop[2] = getbitu(rtcm->buff, i, 8) * 0.1;        i += 8;
	h = getbits(rtcm->buff, i, 15) * 0.01;      i += 15;
	age = getbits(rtcm->buff, i, 24);            i += 24;
	dsta = getbitu(rtcm->buff, i, 12);      i += 12; i += 8;
	eptime = getbitu(rtcm->buff, i, 30);            i += 30;
	week = getbitu(rtcm->buff, i, 16);      i += 16; i += 8;
	lat = getbits(rtcm->buff, i, 32) * 0.001 / 3600; i += 32;
	lon = getbits(rtcm->buff, i, 32) * 0.001 / 3600; i += 32;
	ght = getbits(rtcm->buff, i, 20) * 0.1;		 i += 20;
	vh = getbits(rtcm->buff, i, 20) * 0.01;		 i += 20;
	vv = getbits(rtcm->buff, i, 20) * 0.01;		 i += 20;
	dir = getbits(rtcm->buff, i, 16) * 0.1;		 i += 16;
	a = getbitu(rtcm->buff, i, 16);		 	 i += 16;
	b = getbitu(rtcm->buff, i, 16);		 	 i += 16;
	c = getbitu(rtcm->buff, i, 16);		 	 i += 16;
	cbias = getbits(rtcm->buff, i, 32) * 0.001;		 i += 32;
	cdrift = getbits(rtcm->buff, i, 32) * 0.01;		 i += 32;

	double eptime_d = ROUND(((double)eptime) / 1000);
	trace(1, "G_pvt,year=%d,week=%d,%.1f,Qual=%d,fixin=%d,dastatus=%d,nsat_u=%d,nsat_v=%d,dop=%f %f %f,lat=%f,lon=%f,ght=%.1f,vv=%f,vh=%f,dir=%f,a=%f,b=%f,c=%f,cbias=%f,cdrift=%f\r\n",
		year, week, eptime_d, qual, fixin, dastatus, nsat_u, nsat_v, dop[0], dop[1], dop[2], lat, lon, ght, vv, vh, dir, a, b, c, cbias, cdrift);

	if (!qual || dop[1] > 25.4 || dop[2] > 25.4 || nsat_u < 10)
	{
		trace(1, "G_pvt invild\r\n");
		return -1;
	}

	rtcm->lgpvt.sta = sta;
	rtcm->lgpvt.qual = qual;
	rtcm->lgpvt.nsat_u = nsat_u;
	rtcm->lgpvt.nsat_v = nsat_v;
	rtcm->lgpvt.dop[0] = dop[0];
	rtcm->lgpvt.dop[1] = dop[1];
	rtcm->lgpvt.dop[2] = dop[2];
	rtcm->lgpvt.h = h;
	rtcm->lgpvt.age = age;
	rtcm->lgpvt.dsta = dsta;
	rtcm->lgpvt.time = gpst2time((int)week, eptime_d);
	rtcm->lgpvt.lat = lat;
	rtcm->lgpvt.lon = lon;
	rtcm->lgpvt.ght = ght;
	rtcm->lgpvt.vv = vv;
	rtcm->lgpvt.vh = vh;
	rtcm->lgpvt.dir = dir;
	rtcm->lgpvt.cbias = cbias;
	rtcm->lgpvt.cdrift = cdrift;

	return 9;
}
//VRS私有电文返回6，星历电文返回2 观测电文返回 0或者1.坐标电文返回5
/* decode rtcm ver.3 message -------------------------------------------------*/
static int decode_rtcm3_VRS(rtcm_t* rtcm)
{
	double rr[3] = { 0 };
	int i = 24 + 12, j = 0, k = 0, m = 0, n = 0, type = 0, staid = 0, nf = NFREQ;
	int satid[128] = { 0 }, nsat = 0, ns = 0, frq = 0, half = 0, lock[128] = { 0 };
	double r = 0.0, pr = 0, cp = 0, rng = 0, rng_m = 0, prv = 0, cpv = 0, wl = 0;
	pr = cp = -1E16;
	int s[3] = { 0 }, f[3] = { 0 }, ind[3][3] = { {0} };//GRC三系统及其频率和卫星数 频率索引

	//type = getbitu(rtcm->buff, i, 12); i += 12;
	staid = getbitu(rtcm->buff, i, 12); i += 12;
	rr[0] = getbits_38(rtcm->buff, i); i += 38;
	rr[1] = getbits_38(rtcm->buff, i); i += 38;
	rr[2] = getbits_38(rtcm->buff, i); i += 38;

	/* test station id */
	rtcm->staid = staid;
	rtcm->sta.deltype = 0; /* xyz */
	for (j = 0; j < 3; j++) {
		rtcm->sta.pos[j] = rr[j] * 0.0001;
		rtcm->sta.del[j] = 0.0;
	}
	rtcm->sta.hgt = 0.0;
	gtime_t time = { 0 };
	time.time = getbitu(rtcm->buff, i, 32); i += 32;
	time.sec = getbitu(rtcm->buff, i, 10); i += 10;

	for (nsat = 0, j = 0; j < 128; j++) {
		k = getbitu(rtcm->buff, i, 1); i++;
		if (k == 1) {
			rtcm->obs.data[nsat].sat = j + 1;
			rtcm->obs.data[nsat].time = time;
			satid[nsat++] = j + 1;
			if (satsys(j + 1, NULL) == SYS_GPS)s[0]++;
			else if (satsys(j + 1, NULL) == SYS_GLO)s[1]++;
			else if (satsys(j + 1, NULL) == SYS_CMP)s[2]++;
		}
	}

	rtcm->obs.n = nsat;

	//得到各系统的频率数及其索引值
	for (k = 0, j = 0; j < 3; j++) {
		frq = getbitu(rtcm->buff, i, 1); i++;
		if (frq == 1) {
			ind[0][k++] = j;
			f[0]++;
		}
	}
	for (k = 0, j = 0; j < 2; j++) {
		frq = getbitu(rtcm->buff, i, 1); i++;
		if (frq == 1) {
			ind[1][k++] = j;
			f[1]++;
		}
	}
	for (k = 0, j = 0; j < 3; j++) {
		frq = getbitu(rtcm->buff, i, 1); i++;
		if (frq == 1) {
			ind[2][k++] = j;
			f[2]++;
		}
	}

	i += 10;

	//DF397
	for (m = 0, ns = 0, j = 0; j < 3; j++) {
		for (n = 0; n < s[j]; n++) {
			rng = getbitu(rtcm->buff, i, 8); i += 8;
			if (rng != 255) r = rng * RANGE_MS;
			//trace(3, "DF397 nsat =%2d j=%3d rng  =%8.3f r=%10.3f \n", nsat, j, rng, r);
			for (k = 0; k < f[j] && k < nf; k++) {
				rtcm->obs.data[ns].P[k] = r;
				rtcm->obs.data[ns].L[k] = r;
			}
			ns++;
		}
	}

	//DF398
	for (m = 0, ns = 0, j = 0; j < 3; j++) {
		for (n = 0; n < s[j]; n++) {
			rng_m = getbitu(rtcm->buff, i, 10); i += 10;
			r = rtcm->obs.data[ns].P[0];
			if (r != 0.0) r = rng_m * P2_10 * RANGE_MS;
			//trace(3, "DF398 nsat =%2d j=%3d rng_m=%8.3f r=%10.3f \n", nsat, j, rng_m, r);
			for (k = 0; k < f[j] && k < nf; k++) {
				rtcm->obs.data[ns].P[k] += r;
				rtcm->obs.data[ns].L[k] += r;
			}
			ns++;
		}
	}

	/* pseudorange */  //DF400
	for (ns = 0, j = 0; j < 3; j++) {
		for (n = 0; n < s[j]; n++) {
			for (k = 0; k < f[j]; k++) {
				prv = getbits(rtcm->buff, i, 15); i += 15;
				if (prv != -16384) pr = prv * P2_24 * RANGE_MS;
				if (ind[j][k] < nf) {
					if (prv != -16384) rtcm->obs.data[ns].P[ind[j][k]] += pr;
					else {
						rtcm->obs.data[ns].P[ind[j][k]] = 0;
						rtcm->obs.data[ns].L[ind[j][k]] = 0;
					}
				}
				//trace(3, "DF400  j=%3d n=%3d k=%3d  prv  =%12.3f pr=%10.4f \n",  j,n,k, prv,pr );
			}
			ns++;
		}
	}

	/* carrier-phase (cycle) */ //DF401
	for (ns = 0, j = 0; j < 3; j++) {
		for (n = 0; n < s[j]; n++) {
			for (k = 0; k < f[j]; k++) {
				cpv = getbits(rtcm->buff, i, 22); i += 22;
				if (cpv != -2097152) cp = cpv * P2_29 * RANGE_MS;
				wl = satwavelen(satid[ns], ind[j][k], &rtcm->nav); //satellite carrier wave length
				if (ind[j][k] < nf) {
					r = rtcm->obs.data[ns].L[ind[j][k]];
					if (r != 0.0 && cpv != -2097152 && wl > 0.0)
						rtcm->obs.data[ns].L[ind[j][k]] = (r + cp) / wl;
					else rtcm->obs.data[ns].L[ind[j][k]] = 0;
				}
			}
			ns++;
		}
	}
	// traceobs(2, rtcm->obs.data, rtcm->obs.n);

	 /*lock time*/
	for (m = 0, ns = 0, j = 0; j < 3; j++) {//DF402
		for (n = 0; n < s[j]; n++) {
			for (k = 0; k < f[j]; k++) {
				lock[m++] = getbitu(rtcm->buff, i, 4); i += 4;
				//trace(3, "DF402  j=%3d n=%3d k=%3d m==%3d  lock=%3d \n", j, n, k,m, lock[m-1]);
			}
			ns++;
		}
	}

	/*lli*/
	for (m = 0, ns = 0, j = 0; j < 3; j++) {//DF420
		for (n = 0; n < s[j]; n++) {
			for (k = 0; k < f[j]; k++) {
				half = getbitu(rtcm->buff, i, 1); i += 1;
				rtcm->obs.data[ns].lli[ind[j][k]] =
					lossoflock(rtcm, satid[ns], ns, ind[j][k], lock[m++]) + (half ? 3 : 0);
				//if (ind[j][k] < nf) rtcm->obs.data[ns].lli[ind[j][k]] = half;
				//设置多普勒值和信噪比为0；
				rtcm->obs.data[ns].D[ind[j][k]] = 0;
				rtcm->obs.data[ns].snr[ind[j][k]] = 0;
				//code赋值
				if (j == 0 && k == 0) rtcm->obs.data[ns].code[ind[j][k]] = 1;//GPS L1C
				else if (j == 0 && k == 1) rtcm->obs.data[ns].code[ind[j][k]] = 14;//GPS L2c
				else if (j == 0 && k == 2) rtcm->obs.data[ns].code[ind[j][k]] = 24;//GPS L5I

				else if (j == 2 && k == 0) rtcm->obs.data[ns].code[ind[j][k]] = 47;//BDS B1I
				else if (j == 2 && k == 1) rtcm->obs.data[ns].code[ind[j][k]] = 27;//BDS B2I
				else if (j == 2 && k == 2) rtcm->obs.data[ns].code[ind[j][k]] = 42;//BDS B2I
			}
			ns++;
		}
	}
	return 6;//用于和观测电文和星历电文区分，星历电文返回2 观测电文返回 0或者1.坐标电文返回5
}

extern int decode_rtcm3(rtcm_t* rtcm)
{
	double tow = 0;
	int ret = 0, type = getbitu(rtcm->buff, 24, 12), week = 0;

	trace(3, "decode_rtcm3: len=%3d type=%d\r\n", rtcm->len, type);

	if (rtcm->outtype) {
		sprintf(rtcm->msgtype, "RTCM %4d (%4d):", type, rtcm->len);
	}
	/* real-time input option */
	if (strstr(rtcm->opt, "-RT_INP")) {
		tow = time2gpst(utc2gpst(timeget()), &week);
		rtcm->time = gpst2time(week, floor(tow));
	}
	rtcm->type = type;
	switch (type) {
	case 999: ret = decode_rtcm3_lg69t_pvt(rtcm); break;
	case 1300:ret = decode_rtcm3_VRS(rtcm); break;
	case 1005: ret = decode_type1005(rtcm); break;
	case 1006: ret = decode_type1006(rtcm); break;
	case 1019: ret = decode_type1019(rtcm); break;
	case 1046: ret = decode_type1046(rtcm); break; /* extension for IGS MGEX */
	case 1047: ret = decode_type1047(rtcm); break; /* beidou ephemeris (tentative mt) */
	case 1042: ret = decode_type63(rtcm); break; /* beidou ephemeris (rtcm draft) */
	case   63: ret = decode_type63(rtcm); break; /* beidou ephemeris (rtcm draft) */
	case 1074: ret = decode_msm4(rtcm, SYS_GPS); break;
	case 1075: ret = decode_msm5(rtcm, SYS_GPS); break;
	case 1077: ret = decode_msm7(rtcm, SYS_GPS); break;
		//        case 1094: ret=decode_msm4(rtcm,SYS_GAL); break;
		//        case 1095: ret=decode_msm5(rtcm,SYS_GAL); break;
		//        case 1097: ret=decode_msm7(rtcm,SYS_GAL); break;
	case 1124: ret = decode_msm4(rtcm, SYS_CMP); break;
	case 1125: ret = decode_msm5(rtcm, SYS_CMP); break;
	case 1127: ret = decode_msm7(rtcm, SYS_CMP); break;
	}
	if (ret >= 0) {
		type -= 1000;
		if (1 <= type && type <= 299) rtcm->nmsg3[type]++; /* 1001-1299 */
		else if (1000 <= type && type <= 1099) rtcm->nmsg3[type - 700]++; /* 2000-2099 */
		else rtcm->nmsg3[0]++;
	}
	return ret;
}