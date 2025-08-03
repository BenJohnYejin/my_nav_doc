#include "rtklib.h"

static int floatnum = 0;

midmsg_t midmsg_[MAXDIFOBS * 2];

/* single-differenced observable ---------------------------------------------*/
static double sdobs(const obsd_t* obs, int i, int j, int f)
{
	double pi = f < NFREQ ? obs[i].L[f] : obs[i].P[f - NFREQ];
	double pj = f < NFREQ ? obs[j].L[f] : obs[j].P[f - NFREQ];
	return pi == 0.0 || pj == 0.0 ? 0.0 : pi - pj;
}

/* baseline length -----------------------------------------------------------*/
extern double baseline(const double* ru, const double* rb, double* dr)
{
	int i = 0;
	for (i = 0; i < 3; i++) dr[i] = ru[i] - rb[i];
	return norm(dr, 3);
}
/* initialize state and covariance -------------------------------------------*/
static void initx(rtk_t* rtk, double xi, double var, int i)
{
	int j = 0;
	rtk->x[i] = xi;
	for (j = 0; j < rtk->nx; j++) {
		rtk->P[i + j * rtk->nx] = rtk->P[j + i * rtk->nx] = i == j ? var : 0.0;
	}
}

void udposvar(double en_var, double u_var, rtk_t* rtk)
{
	double Q[9] = { 0.0 }, qv[9] = { 0.0 }, pos[3] = { 0.0 };
	Q[0] = Q[4] = SQR(en_var);
	Q[8] = SQR(u_var);
	ecef2pos(rtk->x, pos);
	covecef(pos, Q, qv);
	trace(4, "udposvar en_var=%f u_var=%f Qv=\n", en_var, u_var);
	//trace.tracemat(5,Qv,3,3,6,3);
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			rtk->P[i + j * rtk->nx] += qv[i + j * 3];
		}
	}
}

static void udpos(rtk_t* rtk, double tt, double st)
{
	/*稳健性判断*/
	if (rtk == NULL) return;
	/*变量初始化*/
	double var = 0.0;
	int i = 0;
	int flag = 0;//1为卫星不可用率太高
	double errpro = 0.0;
	double dx[3] = { 0 };
	rtk->sol_rtk.resetflag = 0;
#if 0
	
	if (rtk->nsat){
		errpro = (double)rtk->n_errsat / rtk->nsat;//不可用卫星率(信噪比<25判断)
		if (errpro > 0.8) flag = 1;
	}else {
		flag = 1;
	}
#endif
	trace(3, "udpos:: errsat(SNR<25)=%2d presat=%2d err/all=%6.3f resetflag=%2d\n", rtk->n_errsat, rtk->presat, errpro, flag);
	/* check variance of estimated postion */
	for (i = 0; i < 3; i++) var += SQR(rtk->P[i + i * rtk->nx]);
	var = sqrt(var);
	trace(3, "udpos::tt(any)=%6.3f st(RTK)=%6.3f norm(x)=%f posvar=%f vtat=%d\n", tt, st, norm(rtk->x, 3), var, rtk->sol.vtat);
	/*! 位置初始化 */
	if (norm(rtk->x, 3) <= 0.0 || var > VAR_POS || tt > 5.0 || st > 5.0 || !rtk->sol.vtat) {
		memset(rtk->x, 0, rtk->nx * sizeof(double)); 
		rtk->sol_rtk.resetflag = 1;
		for (i = 0; i < 3; i++) initx(rtk, rtk->sol.rr[i], VAR_POS, i);
		return;
	}
	if (!rtk->opt.dynamics) return;
	for (i = 0; i < 3; i++) {
		rtk->x[i] = rtk->l_x[i];//位置赋上次更新后的位置
		rtk->x[i] += rtk->x_d[i + 3] * tt;
		//rtk->x[i] += (rtk->l_x_d[i + 3] + rtk->x_d[i + 3])* 0.5 * tt;  //RTD的速度x_d，tt为前后历元的时间差,RTK未计算速度  TODO 已test
	}
	trace(3, "before udpos rtk l_x:%12.3f %12.3f %12.3f %8.3f %8.3f %8.3f\n", rtk->l_x[0], rtk->l_x[1], rtk->l_x[2], rtk->l_x_d[3], rtk->l_x_d[4], rtk->l_x_d[5]);
	trace(3, "before udpos rtk xd:%12.3f %12.3f %12.3f %8.3f %8.3f %8.3f\n", rtk->x_d[0], rtk->x_d[1], rtk->x_d[2], rtk->x_d[3], rtk->x_d[4], rtk->x_d[5]);
	if (rtk->sol_last.stat < 3) {//todo固定/浮点/SBAS,放大方差而不是放小
		if (flag)
			udposvar(0.8 * tt, 0.8 * tt, rtk);
		else
			udposvar(0.2 * tt, 0.2 * tt, rtk);
		return;
	}
	else if (rtk->sol_last.stat > 3) {//单点/RTD
		if (flag)
			udposvar(0.5 * tt, 0.5 * tt, rtk);
		else
			udposvar(0.2 * tt, 0.2 * tt, rtk);
		return;
	}
}

/* temporal update of receiver h/w biases ------------------------------------*/
static void udrcvbias(rtk_t* rtk, double tt)
{
	int i = 0, j = 0;

	trace(3, "udrcvbias: tt=%.3f\r\n", tt);

	for (i = 0; i < NFREQGLO; i++) {
		j = IL(i, &rtk->opt);

		if (rtk->x[j] == 0.0) {
			initx(rtk, 1E-6, VAR_HWBIAS, j);
		}
		/* hold to fixed solution */
		else if (rtk->nfix >= rtk->opt.minfix && rtk->sol.ratio > rtk->opt.thresar[0]) {
			initx(rtk, rtk->xa[j], rtk->pa[j + j * rtk->na], j);
		}
		else {
			rtk->P[j + j * rtk->nx] += SQR(PRN_HWBIAS) * fabs(tt);
		}
	}
}
void detslp_vrs(rtk_t* rtk, const obsd_t* obs, int i, int j, const nav_t* nav) {
	/*初始化变量*/
	int f = 0, sat = obs[i].sat;
	double delta_vrs = 0.0;
	double lam = 0.0;
	double ct = difftime(obs[i].time.time, rtk->ssat[j].change_time.time);
	for (f = 0; f < rtk->opt.nf; f++) {
		lam = nav->lam[sat - 1][f];
		if (lam <= 0)continue;
		if (rtk->basechange) {//VRS换站判断
			rtk->ssat[j].change_time = obs[i].time;
			rtk->ssat[j].change_flag[f] = 0;
			continue;
		}
		if (rtk->ssat[j].pp[1][f] == 0.0 || rtk->ssat[j].ph[1][f] == 0.0 || obs[i].L[f] == 0.0 || obs[i].P[f] == 0.0 || fabs(ct) < 0.05)
			continue;
		if (rtk->ssat[j].change_flag[f] < 2) {
			rtk->ssat[j].change_flag[f]++;
			continue;
		}
		/*接收机钟跳探测*/
		delta_vrs = (rtk->ssat[j].ph[1][f] * lam - rtk->ssat[j].pp[1][f]) - (obs[i].L[f] * lam - obs[i].P[f]); //前后历元的伪距载波探测接收机钟跳
		if (delta_vrs > 0.5) {
			trace(4, "detslp_vrs : sat=%d L0=%16.3f P0=%16.3f L1=%16.3f P1=%16.3f delta_vrs = %8.3f In vrs cycle slip happen!\n", sat,
				rtk->ssat[j].ph[1][f], rtk->ssat[j].pp[1][f], obs[i].L[f], obs[i].P[f], delta_vrs);
			rtk->ssat[j].slip[f] |= 1;
			rtk->vrsbadnum++;
		}
	}
}
/* detect cycle slip by LLI --------------------------------------------------*/
void detslp_ll(rtk_t* rtk, const obsd_t* obs, int i, int j, int rcv) {
	unsigned char slip = 0, lli1 = 0, lli2 = 0, lli = 0;
	int f = 0;
	for (f = 0; f < rtk->opt.nf; f++) {
		/* restore previous lli LLI????LLI1与LLI2始终为0？？？？ */
		lli1 = (rtk->ssat[j].slip[f] >> 6) & 3;
		lli2 = (rtk->ssat[j].slip[f] >> 4) & 3;
		lli = rcv == 1 ? lli1 : lli2;
		/* detect slip by cycle slip flag */
		slip = (rtk->ssat[j].slip[f] | obs[i].lli[f]) & 3;
		if (obs[i].lli[f] & 1) {//先前与当前观测值之间失锁，可能发生了周跳
			trace(5, "detslp_ll:slip detected (sat=%2d rcv=%d lli%d=%x)\n",
				obs[i].sat, rcv, f + 1, obs[i].lli[f]);
			slip |= 1;
		}
		if ((obs[i].lli[f] & 2) || ((lli & 1) && (obs[i].lli[f] & 2)) || ((lli & 1) && !(obs[i].lli[f] & 1))) {
			trace(5, "detslp_ll:slip detected (sat=%2d rcv=%d lli%d=%x->%x)\n",
				obs[i].sat, rcv, f + 1, lli, obs[i].lli[f]);
			slip |= 2;
		}
		if (rcv == 1)
			rtk->ssat[j].slip[f] = (obs[i].lli[f] << 6) | (lli2 << 4) | slip;
		else
			rtk->ssat[j].slip[f] = (obs[i].lli[f] << 4) | (lli1 << 6) | slip;
	}
}

int detslp_dop(rtk_t* rtk, const obsd_t* obs, int ns, const int* iu, int rcv, const nav_t* nav, double* tt, double* difdopres, int* vflg, midmsg_t* midmsg) {
	int num = 0, i = 0, f = 0, sat = 0;
	double dph = 0, dpt = 0, lam = 0;
	for (i = 0; i < ns; i++) {
		f = 0; sat = obs[iu[i]].sat;dph = 0;dpt = 0;lam = 0;
		for (f = 0; f < rtk->opt.nf; f++) {
			/*稳健性判断*/
			if (obs[iu[i]].L[f] == 0.0 || obs[iu[i]].D[f] == 0.0f || rtk->ssat[i].ph[rcv - 1][f] == 0.0 || rtk->ssat[i].pd[rcv - 1][f] == 0.0) {
				trace(4,"sat=%2d detslp_dop:f=%2d L=%16.3f D=%6.3f ph=%6.3f\r\n",sat,f,obs[iu[i]].L[f],obs[iu[i]].D[f],rtk->ssat[i].ph[rcv - 1][f]);
				rtk->ssat[i].slip[f] |= 1;
				continue;
			}
			if (iu[i] < MAXDIFOBS * 2 && midmsg[iu[i]].ddop2[f]) { rtk->ssat[i].slip[f] |= 1;continue;} 
			if (rtk->ssat[i].slip[f] & 1) { trace(4,"detslp_dop:slip\r\n");continue;}                   //TODO  待调整测试
			if (fabs((*tt) = timediff(obs[iu[i]].time, rtk->ssat[i].pt[rcv - 1][f])) < DTTOL) continue; 
			if ((*tt) > 5.0) {trace(4,"detslp_dop::time too long tt=%2f\r\n",*tt);continue;}
			if ((lam = nav->lam[sat - 1][f]) <= 0.0) continue;
			
			/* phase difference and doppler x time (cycle) */
			dph = obs[iu[i]].L[f] - rtk->ssat[i].ph[rcv - 1][f];
			dpt = -(obs[iu[i]].D[f] + rtk->ssat[i].pd[rcv - 1][f]) * 0.5 * (*tt); 
			difdopres[num] = fabs(dph - dpt);
			trace(4, "sat=%2d rcv=%d L%d diffdopres=%.3f slip=%2d\n", sat, rcv, f + 1, difdopres[num], rtk->ssat[i].slip[f]);
			vflg[num++] = sat << 8 | f << 4;
		}
	}
	return num;
}

/*从大到小排序*/
extern int orderdopdif(int n, double* diftem) {
	if (diftem == NULL) return -1;
	int i = 0, j = 0, k = 0;
	double temp = 0, t_ = 0;
	double l[MAXPNTOBS * 2 * DIF_L_IN_EVL] = { 0.0 };

	for (i = 0; i < n; i++)
		l[i] = diftem[i];
	for (i = 0; i < n - 1; i++) {
		k = i;
		for (j = i + 1; j < n; j++) {
			if (l[k] < l[j]) {
				temp = l[i];
				t_ = diftem[i];
				l[i] = l[j];
				diftem[i] = diftem[j];
				l[j] = temp;
				diftem[j] = t_;
			}
		}
	}
	return 1;
}

void slipmark(rtk_t* rtk, const int* vflg, const int sys, const int n, const int dopf)
{
	int i = 0, j = 0, sat = 0, f = 0;
	int navsys = 0;
	for (i = 0; i < n; i++) {
		navsys = 0;
		sat = (vflg[i] >> 8) & 0xFF;f = (vflg[i] >> 4) & 0xF;
		navsys = satsys(sat, NULL);

		if (!(navsys & sys)) continue;
		if (f != dopf) continue;
		//根据sat找索引
		j = ix(sat, rtk->satid, rtk->presat);
		trace(4, "no enough dops to judge slip:sat=%d f=%d n=%d j=%d\n", sat, f, n, j);

		if (j >= 0 && f < rtk->opt.nf && j < MAXDIFOBS) {   //增加容错机制
			rtk->ssat[j].slip[f] |= 1;
		}else {
			trace(2, "error j<0 sat=%4d f=%4d j=%4d\r\n", sat, f, j);
		}
	}
}

void dopmarkslip(int sys, int num, int* vflg, double* difdopres, double tt, rtk_t* rtk, int dopf)
{
	if (vflg == NULL || difdopres == NULL || rtk == NULL) return;
	int j = 0, n = 0, slip_num = 0;
	double  mid_dif = 0;
	double  dif_tem[MAXDIFOBS] = { 0.0 };
	int sliplag[MAXDIFOBS * NFREQ] = { 0 };
	int i = 0;
	int navsys = 0, sat = 0, f = 0;
	double thres = 0.75;
	double half = 0.0;

	trace(2, "dopmarkslip:sys=%2d\r\n", sys);
	for (i = 0; i < num; i++){
		navsys = 0;
		sat = (vflg[i] >> 8) & 0xFF;f = (vflg[i] >> 4) & 0xF;
		navsys = satsys(sat, NULL);
		j = ix(sat, rtk->satid, rtk->presat);

		if (rtk->ssat[j].slip[f] & 3) continue;
		if (!(navsys & sys)) continue;
		if (f != dopf) continue;

		dif_tem[n++] = difdopres[i];
	}
	if (n < 3 || n >= MAXDIFOBS) {
		slipmark(rtk, vflg, sys, num, dopf);
		return;
	}
	orderdopdif(n, dif_tem);
	if (!(n % 2)) {
		mid_dif = (dif_tem[n / 2] + dif_tem[n / 2 - 1]) / 2;
		if (fabs(dif_tem[n / 2] - dif_tem[n / 2 - 1]) > 0.5) return;
	}else {
		mid_dif = dif_tem[(n - 1) / 2];
	}
	trace(2, "mid_dif=%f  n=%d\n", mid_dif, n);
	n = 0;
	for (i = 0; i < num; i++) {
		navsys = 0;
		sat = (vflg[i] >> 8) & 0xFF;f = (vflg[i] >> 4) & 0xF;
		navsys = satsys(sat, NULL);

		if (!(navsys & sys)) continue;
		if (f != dopf) continue;
		j = ix(sat, rtk->satid, rtk->presat);
		trace(5, "detected slip sat=%2d test=%9.3f thre=%9.3f slip=%2d\n", sat, fabs(mid_dif - difdopres[i]), thres, rtk->ssat[j].slip[f]);

		n++;
		if (fabs(mid_dif - difdopres[i]) < thres) continue;
		
		sliplag[i] = 1;
		slip_num++;
	}
	half = (double)n / 2;
	if (slip_num < half) {
		for (i = 0; i < num; i++) {
			if (sliplag[i]) {
				sat = (vflg[i] >> 8) & 0xFF;
				f = (vflg[i] >> 4) & 0xF;
				j = ix(sat, rtk->satid, rtk->presat);

				if (j >= 0 && f < rtk->opt.nf) {
					rtk->ssat[j].slip[f] |= 1;
				}else {
					trace(2, "error j<0 sat=%4d f=%4d j=%4d\r\n", sat, f, j);
				}
				trace(4, "slip sat=%2d L%d test=%.3f thre=%.3f\n", sat, f + 1,fabs(mid_dif - difdopres[i]), thres);
			}
		}
	}else {
		//slipmark(rtk,vflg,sys, num,dopf);
		trace(4, "too much slip  slip_num=%d  half = %f\n", slip_num, half);
	}
}

void desilpbydop(rtk_t* rtk, const obsd_t* obs, int ns, const int* iu, int rcv, const nav_t* nav, midmsg_t* midmsg)
{
	if (rtk == NULL || obs == NULL || iu == NULL || nav == NULL) {
		trace(3, "RtkposDynamic::desilpbydop, operate ,null pointer.\n");
		return;
	}
	int nf = rtk->opt.nf;
	int f = 0;
	int num = 0;
	double tt = 0;
	double difdopres[MAXDIFOBS * NFREQ] = { 0.0 };
	int vflg[MAXDIFOBS * NFREQ] = { 0 };
	
	trace(2, "dop cycle slip detct: rcv=%2d\r\n", rcv);
	num = detslp_dop(rtk, obs, ns, iu, rcv, nav, &tt, difdopres, vflg, midmsg);
	if (num < 3) return;
	
	for (f = 0; f < nf; f++){
		dopmarkslip(SYS_GPS | SYS_GAL | SYS_CMP | SYS_GLO, num, vflg, difdopres, tt, rtk, f);
	}
}
extern int ix(int sat, const int* sats, int nsat) {//?sats???sat???,?????-1;
	int i = 0;
	for (i = 0; i < nsat; i++) if (sat == sats[i]) break;
	if (i < nsat) return i;
	else return -1;
}

extern int ij(int i, int j) {
	return (i >= j) ? i * (i + 1) / 2 + j : j * (j + 1) / 2 + i;
}

void udbias(rtk_t* rtk, double tt, const obsd_t* obs, const int* sat, const int* iu, const int* ir, int ns, const nav_t* nav) {
	double cp = 0, pr = 0, cp1 = 0, cp2 = 0, pr1 = 0, pr2 = 0, * bias = NULL, offset = 0, lami = 0, lam1 = 0, lam2 = 0, c1 = 0, c2 = 0;
	int i = 0, j, f = 0, slip = 0, reset = 0, nf = NF(&rtk->opt);
	int na = rtk->na;
	nf = 1;

	trace(3, "udbias  : tt=%.1f ns=%d\n", tt, ns);
	for (f = 0; f < nf; f++) {
		/* reset phase-bias if instantaneous AR or expire obs outage counter */
		for (i = 0; i < ns; i++) {
			reset = ++rtk->ssat[i].outc[f] > (unsigned int)rtk->opt.maxout;//outc=0，outc=1;then 0

			if (rtk->opt.modear == ARMODE_INST && rtk->x[IB2(i, f, &rtk->opt, rtk->nsat)] != 0.0) {
				initx(rtk, 0.0, 0.0, IB2(i, f, &rtk->opt, rtk->nsat));
			}
			else if (reset && rtk->x[IB2(i, f, &rtk->opt, rtk->nsat)] != 0.0) {
				initx(rtk, 0.0, 0.0, IB2(i, f, &rtk->opt, rtk->nsat));
				trace(3, "udbias : obs outage counter overflow (sat=%3d L%d n=%d)\n",
					rtk->satid[i], f + 1, rtk->ssat[i].outc[f]);
				rtk->ssat[i].outc[f] = 0;  //20191014 add
			}
			if (rtk->opt.modear != ARMODE_INST && reset) {
				rtk->ssat[i].lock[f] = -rtk->opt.minlock;
			}
		}
		/* reset phase-bias if detecting cycle slip */
		for (i = 0; i < ns; i++) {
			rtk->P[IB2(i, f, &rtk->opt, rtk->nsat) + (IB2(i, f, &rtk->opt, rtk->nsat)) * rtk->nx] += rtk->opt.prn[0] * rtk->opt.prn[0] * tt;
			slip = rtk->ssat[i].slip[f];
			if (rtk->opt.modear == ARMODE_INST || !(slip & 1))
				continue;
			rtk->x[IB2(i, f, &rtk->opt, rtk->nsat)] = 0.0;
			rtk->ssat[i].lock[f] = -rtk->opt.minlock;
			trace(3, "udbias reset by slip:(sat=%3d L%d n=%d)\n", sat[i], f + 1,
				rtk->ssat[i].lock[f]);
		}
		bias = zeros(ns, 1);
		/* estimate approximate phase-bias by phase - code */
		for (i = j = 0, offset = 0.0; i < ns; i++) {
			if (rtk->opt.ionoopt != IONOOPT_IFLC) {
				cp = sdobs(obs, iu[i], ir[i], f); /* cycle */
				pr = sdobs(obs, iu[i], ir[i], f + NFREQ);
				lami = nav->lam[sat[i] - 1][f];
				if (cp == 0.0 || pr == 0.0 || lami <= 0.0)
					continue;
				bias[i] = cp - pr / lami;
			}
			else {
				cp1 = sdobs(obs, iu[i], ir[i], 0);
				cp2 = sdobs(obs, iu[i], ir[i], 1);
				pr1 = sdobs(obs, iu[i], ir[i], NFREQ);
				pr2 = sdobs(obs, iu[i], ir[i], NFREQ + 1);
				lam1 = nav->lam[sat[i] - 1][0];
				lam2 = nav->lam[sat[i] - 1][1];
				if (cp1 == 0.0 || cp2 == 0.0 || pr1 == 0.0 || pr2 == 0.0 || lam1 <= 0.0 || lam2 <= 0.0)
					continue;
				c1 = SQR(lam2) / (SQR(lam2) - SQR(lam1));
				c2 = -SQR(lam1) / (SQR(lam2) - SQR(lam1));
				bias[i] = (c1 * lam1 * cp1 + c2 * lam2 * cp2) - (c1 * pr1 + c2 * pr2);
			}
			if (rtk->x[IB2(i, f, &rtk->opt, rtk->nsat)] != 0.0) {
				offset += bias[i] - rtk->x[IB2(i, f, &rtk->opt, rtk->nsat)];
				j++;
				trace(4, "udbias:+offset sat=%3d bias=%9.3f N=%9.3lf offset=%9.3lf IB=%3d\n", sat[i], bias[i], rtk->x[i + na], bias[i] - rtk->x[i + na], i);
			}
		}
		trace(4, "udbias:average_offset=%6.3lf\n", offset / j);
		/* correct phase-bias offset to enssure phase-code coherency */
		if (j > 0) {
			for (i = 0; i < ns; i++) {
				if (rtk->x[IB2(i, f, &rtk->opt, rtk->nsat)] != 0.0)
					rtk->x[IB2(i, f, &rtk->opt, rtk->nsat)] += offset / j;
			}
		}
		/* set initial states of phase-bias */
		for (i = 0; i < ns; i++) {
			rtk->ssat[i].bias[f] = bias[i];
			rtk->ssat[i].rest[f] = 0;
			if (bias[i] == 0.0 || rtk->x[IB2(i, f, &rtk->opt, rtk->nsat)] != 0.0) {
				continue;
			}
			rtk->ssat[i].rest[f] = 1;
			initx(rtk, bias[i], SQR(rtk->opt.std[0]), i + na);//
			trace(4, "sat %2d amb by rest，bias=%6.3lf\n", sat[i], bias[i]);
		}

		xy_free(bias);
	}
}
/* temporal update of phase biases -方差的设置不同-------------------------------------------*/
void udbias2(rtk_t* rtk, double tt, const obsd_t* obs, const int* sat, const int* iu, const int* ir, int ns, const nav_t* nav, midmsg_t* midmsg) {
	if (rtk == NULL || obs == NULL || sat == NULL || iu == NULL || ir == NULL || nav == NULL) {
		trace(3, "RtkposDynamic::udbias2, operate null pointer,\n");
		return;
	}
	double cp = 0, pr = 0, cp1 = 0, cp2 = 0, pr1 = 0, pr2 = 0, offset = 0, lami = 0, lam1 = 0, lam2 = 0, c1 = 0, c2 = 0;
	int i = 0, j = 0, f = 0, slip = 0, reset = 0, nf = NF(&rtk->opt);
	double dx = 0.0;
	double bias[MAXDIFOBS] = { 0 };

	trace(2, "udbias  : tt=%.1f ns=%d\n", tt, ns);
	for (f = 0; f < nf; f++)
	{
		memset(bias, 0, sizeof(double) * MAXDIFOBS);
		/*利用载波伪距差估计载波相位偏差bias */
		for (i = j = 0, offset = 0.0; i < ns; i++) {
			if (rtk->opt.ionoopt != IONOOPT_IFLC) {
				cp = sdobs(obs, iu[i], ir[i], f); /* cycle 流动站与基站的载波相位差*/
				pr = sdobs(obs, iu[i], ir[i], f + NFREQ);
				lami = nav->lam[sat[i] - 1][f];
				if (cp == 0.0 || pr == 0.0 || lami <= 0.0)
					continue;
				if (midmsg[iu[i]].dpsr2[f] && iu[i] < MAXDIFOBS * 2)  //如有粗差，是否需要用该伪距计算的模糊度初值进行初始化，同时进行钟差修复
					continue;
				bias[i] = cp - pr / lami;
				if (rtk->x[IB2(i, f, &rtk->opt, rtk->nsat)] == 0.0){
					rtk->ssat[i].dxflag[f] = 30;
					continue;
				}

				dx = fabs(bias[i] - rtk->x[IB2(i, f, &rtk->opt, rtk->nsat)]);//求本次模糊度概略值与上次模糊度估计值，相差较大，则代表噪声越多就设置较大的方差？？？
				if (dx <= 10) rtk->ssat[i].dxflag[f] = 1;
				else if (dx <= 30) rtk->ssat[i].dxflag[f] = 5;
				else if (dx <= 100) rtk->ssat[i].dxflag[f] = 10;
				else rtk->ssat[i].dxflag[f] = 30;
			}
			else {
				cp1 = sdobs(obs, iu[i], ir[i], 0);
				cp2 = sdobs(obs, iu[i], ir[i], 1);
				pr1 = sdobs(obs, iu[i], ir[i], NFREQ);
				pr2 = sdobs(obs, iu[i], ir[i], NFREQ + 1);
				lam1 = nav->lam[sat[i] - 1][0];
				lam2 = nav->lam[sat[i] - 1][1];
				if (cp1 == 0.0 || cp2 == 0.0 || pr1 == 0.0 || pr2 == 0.0 || lam1 <= 0.0 || lam2 <= 0.0)
					continue;
				c1 = SQR(lam2) / (SQR(lam2) - SQR(lam1));
				c2 = -SQR(lam1) / (SQR(lam2) - SQR(lam1));
				bias[i] = (c1 * lam1 * cp1 + c2 * lam2 * cp2) - (c1 * pr1 + c2 * pr2);
			}
		}
		/* reset phase-bias if instantaneous AR or expire obs outage counter */
		for (i = 0; i < ns; i++) {
			reset = ++rtk->ssat[i].outc[f] > (unsigned int)rtk->opt.maxout;//TODO 1019 outc有问题 //此处影响明显 0728
# if  0
			if (rtk->tt > 2) {
				trace(5, "data break off.tt=%f\n", rtk->tt);
				reset = 1;
			}
			if (rtk->opt.modear == ARMODE_INST && rtk->x[i + na] != 0.0) {
				initx(rtk, 0.0, 0.0, i + na);
			}
#endif
			if (rtk->opt.modear == ARMODE_INST && rtk->x[IB2(i, f, &rtk->opt, rtk->nsat)] != 0.0) {
				initx(rtk, 0.0, 0.0, IB2(i, f, &rtk->opt, rtk->nsat));
			}
			else if (reset && rtk->x[IB2(i, f, &rtk->opt, rtk->nsat)] != 0.0) {
				initx(rtk, 0.0, 0.0, IB2(i, f, &rtk->opt, rtk->nsat));
				trace(4, "udbias : obs outage counter overflow (sat=%3d L%d n=%d)\r\n",
					rtk->satid[i], f + 1, rtk->ssat[i].outc[f]);
				rtk->ssat[i].outc[f] = 0;//todo 0729add
			}
		}
		/* reset phase-bias if detecting cycle slip */
		for (i = 0; i < ns; i++) {
			rtk->P[IB2(i, f, &rtk->opt, rtk->nsat) + IB2(i, f, &rtk->opt, rtk->nsat) * rtk->nx] += rtk->opt.prn[0] * rtk->opt.prn[0] * tt;
			slip = rtk->ssat[i].slip[f];

			if (rtk->opt.modear == ARMODE_INST || !(slip & 1)) continue;
			rtk->x[IB2(i, f, &rtk->opt, rtk->nsat)] = 0.0;
			rtk->ssat[i].lock[f] = -rtk->opt.minlock;
			trace(3, "udbias reset by slip:(sat=%3d L%d n=%d)\r\n", sat[i], f + 1, rtk->ssat[i].lock[f]);
		}
		/*计算每个卫星模糊度公共分量*/
		for (i = j = 0, offset = 0.0; i < ns; i++) {
			if (rtk->x[IB2(i, f, &rtk->opt, rtk->nsat)] != 0.0) {
				if (midmsg[iu[i]].dpsr2[f])  //如有粗差，是否需要用该伪距计算的模糊度初值进行初始化，同时进行钟差修复
					continue;
				offset += bias[i] - rtk->x[IB2(i, f, &rtk->opt, rtk->nsat)];
				j++;

				trace(4, "udbias:+offset sat=%3d bias=%9.3f N=%9.3lf offset=%9.3lf IB=%3d\r\n", sat[i], bias[i], rtk->x[IB2(i, f, &rtk->opt, rtk->nsat)], bias[i] - rtk->x[IB2(i, f, &rtk->opt, rtk->nsat)], IB2(i, f, &rtk->opt, rtk->nsat));
			}
		}

		/* correct phase-bias offset to enssure phase-code coherency */
		if (j > 0) {
			trace(4, "udbias:average_offset=%6.3lf\r\n", offset / j);
			for (i = 0; i < ns; i++) {
				if (rtk->x[IB2(i, f, &rtk->opt, rtk->nsat)] != 0.0)
					rtk->x[IB2(i, f, &rtk->opt, rtk->nsat)] += offset / j;
			}
		}
		/* set initial states of phase-bias */
		for (i = 0; i < ns; i++) {
			rtk->ssat[i].bias[f] = bias[i];
			rtk->ssat[i].rest[f] = 0;
			rtk->ssat[i].ambresetfail[f] = 0;
			trace(3, "i=%d f=%d ns=%d bias[i]=%f ib=%d x=%f\n", i, f, ns, bias[i], IB2(i, f, &rtk->opt, rtk->nsat), rtk->x[IB2(i, f, &rtk->opt, rtk->nsat)]);
			if (bias[i] == 0.0 || rtk->x[IB2(i, f, &rtk->opt, rtk->nsat)] != 0.0) {
				if (bias[i] == 0.0)
				{
					rtk->ssat[i].rest[f] = 1;
					rtk->ssat[i].lock[f] = -rtk->opt.minlock;//lock -3

					rtk->ssat[i].ambresetfail[f] = 1;

					initx(rtk, 0, (double)30 * (rtk->ssat[i].dxflag[f]), IB2(i, f, &rtk->opt, rtk->nsat));
				}
				continue;
			}
			rtk->ssat[i].rest[f] = 1;
			rtk->ssat[i].lock[f] = -rtk->opt.minlock;//lock -3

			initx(rtk, bias[i], (double)30 * (rtk->ssat[i].dxflag[f]), IB2(i, f, &rtk->opt, rtk->nsat));
			trace(4, "sat %2d amb by rest  bias=%6.3lf\n", sat[i], bias[i]);
		}
	}
}

/* single-differenced geometry-free linear combination of phase --------------*/
static double gfobs_ff(const obsd_t* obs, int i, int j, const double* lam, int f1, int f2)
{
	double pi = sdobs(obs, i, j, f1) * lam[f1], pj = sdobs(obs, i, j, f2) * lam[f2];
	return pi == 0.0 || pj == 0.0 ? 0.0 : pi - pj;
}
/* detect cycle slip by L1-L2 geometry free phase jump -----------------------*/
static void detslp_gf(rtk_t* rtk, const obsd_t* obs, int i, int j, int m,
	const nav_t* nav, int f1, int f2)
{
	int sat = obs[i].sat;
	double g0, g1;

	trace(3, "detslp_gf_l1l2: i=%d j=%d\r\n", i, j);

	if (rtk->opt.nf <= 1 || (g1 = gfobs_ff(obs, i, j, nav->lam[sat - 1], f1, f2)) == 0.0) return;

	g0 = rtk->ssat[m].gf; rtk->ssat[m].gf = g1;

	if (g0 != 0.0 && fabs(g1 - g0) > rtk->opt.thresslip) {
		rtk->ssat[m].slip[f1] |= 1;
		rtk->ssat[m].slip[f2] |= 1;

		trace(3, "slip detected GF2_jump (sat=%2d GF=%.3f %.3f f1=%d f2=%d)\r\n", sat, g0, g1, f1, f2);
	}
}
/*可能未完善*/
void ambfixbybasechange(rtk_t* rtk, const obsd_t* obs, const int* sat,const int* ir, const int ns, const nav_t* nav, const double* y)
{
	int i = 0, j = 0, f = 0;
	int nf = NF(&rtk->opt);
	double dx = 0.0;
	double lami = 0.0;

	for (i = 0; i < ns; i++)
	{
		for (f = 0; f < nf; f++){
			/*稳健性判断*/
			lami = 0.0;
			if (rtk->ssat[i].slip[f] & 1) continue;
			if (fabs(rtk->ssat[i].ph[1][f]) < ZEROS_MIN || fabs(obs[ir[i]].L[f]) < ZEROS_MIN) continue;
			if (fabs(rtk->ssat[i].pp[1][f]) < ZEROS_MIN || fabs(obs[ir[i]].P[f]) < ZEROS_MIN) continue;
			
			j = IB2(i, f, &rtk->opt, rtk->nsat);
			if (j < 0 || j >= rtk->nx) continue;//防止异常情况下j索引越界
			if (fabs(rtk->x[j]) < ZEROS_MIN) continue;
			
			lami = nav->lam[sat[i] - 1][f];
			if (lami == 0.0) continue;
			dx = (y[f + ir[i] * nf * 2] - rtk->ssat[i].vrs_phy[f]) / lami;
			trace(4, "ambfixbybasechange:sat=%d dx=%f rtkx=%f\n", sat[i], dx, rtk->x[j]);
			rtk->x[j] -= dx;
		}
	}
	rtk->basechange = 0;
}

void udstate(rtk_t* rtk, const obsd_t* obs, const int* sat,
	const int* iu, const int* ir, int ns, const nav_t* nav, const double* azel, const double* y, midmsg_t* midmsg) {
	/*稳健性判断*/
	if (rtk == NULL || obs == NULL || sat == NULL || iu == NULL || ir == NULL || nav == NULL || y == NULL) {
		return;
	}
	/*变量初始化*/
	double tt = fabs(rtk->tt);//tt为有任意解的龄期
	double st = 0.0;
	int i = 0, f = 0;
	/*计算RTK解的零期*/
	if (rtk->sol_rtk.time.time) st = timediff(obs[0].time, rtk->sol_rtk.time);//RTK解的零期
	trace(2, "udstate : ns=%d,tt=%f\n", ns, rtk->tt);
	/*位置的时间更新*/
	udpos(rtk, tt, st);
	rtk->vrsbadnum = 0;
	/*周跳探测*/
	for (i = 0; i < ns; i++) {
		for (f = 0; f < rtk->opt.nf; f++) rtk->ssat[i].slip[f] &= 0;
		detslp_ll(rtk, obs, iu[i], i, 1);  
		detslp_ll(rtk, obs, ir[i], i, 2); 
		detslp_gf(rtk, obs, iu[i], ir[i], i, nav, 0, 1);
		detslp_vrs(rtk, obs, ir[i], i, nav); 
	}
	desilpbydop(rtk, obs, ns, iu, 1, nav, midmsg);
#if 1
	/*vrs切换站模糊度处理*/
	if (rtk->basechange) ambfixbybasechange(rtk, obs, sat, ir, ns, nav, y);
#endif
# if 0
	desilpbydop_add(rtk, obs, ns, iu, 1, nav, azel);
	for (int i = 0; i < ns; i++) {
		if (rtk->ssat[i].slip[0]) {
			rtk->ssat[i].amc[0].ambcount = 0;   //检测出有周跳，参考模糊度重新计数
		}
	}
	/* temporal update of ionospheric parameters */
	if (rtk->opt.ionoopt >= IONOOPT_EST) {
		bl = baseline(rtk->x, rtk->rb, dr);
		udion(rtk, tt, bl, sat, ns);
	}
	/* temporal update of tropospheric parameters */
	if (rtk->opt.tropopt >= TROPOPT_EST) {
		udtrop(rtk, tt, bl);
	}
	/* temporal update of eceiver h/w bias */
	if (rtk->opt.glomodear == 2 && (rtk->opt.navsys & SYS_GLO)) {
		udrcvbias(rtk, tt);
	}
#endif
	/* temporal update of phase-bias */
	if (rtk->opt.mode > PMODE_DGPS) {
		//udbias(rtk, tt, obs, sat, iu, ir, ns, nav);//TODO12W
		udbias2(rtk, tt, obs, sat, iu, ir, ns, nav, midmsg);
	}
}

/* undifferenced phase/code residual for satellite ---------------------------*/
static void zdres_sat(int base, double r, const obsd_t* obs, const nav_t* nav,
	const double* azel, const double* dant,
	const prcopt_t* opt, double* y)
{
	const double* lam = nav->lam[obs->sat - 1];
	int i, nf = NF(opt);
	int sys = 0;
	if (!(sys = satsys(obs->sat, NULL))) {
		return;
	}
	if (opt->ionoopt == IONOOPT_IFLC) { /* iono-free linear combination */
	#if 0
		if (lam[0] == 0.0 || lam[1] == 0.0) return;
		if (testsnr(base, 0, azel[1], obs->snr[0] * 0.25, &opt->snrmask) ||
			testsnr(base, 1, azel[1], obs->snr[1] * 0.25, &opt->snrmask)) return;
		f1 = CLIGHT / lam[0];
		f2 = CLIGHT / lam[1];
		C1 = SQR(f1) / (SQR(f1) - SQR(f2));
		C2 = -SQR(f2) / (SQR(f1) - SQR(f2));
		dant_if = C1 * dant[0] + C2 * dant[1];
		if (obs->L[0] != 0.0 && obs->L[1] != 0.0) {
			y[0] = C1 * obs->L[0] * lam[0] + C2 * obs->L[1] * lam[1] - r - dant_if;
		}
		if (obs->P[0] != 0.0 && obs->P[1] != 0.0) {
			y[1] = C1 * obs->P[0] + C2 * obs->P[1] - r - dant_if;
		}
	# endif
	}else {
		for (i = 0; i < nf; i++) {
			if (lam[i] == 0.0) continue;
			/* check snr mask */
			if (testsnr(base, i, azel[1], obs->snr[i] * 0.25, opt->snroff, opt->snrthres)) {
				continue;
			}
			if (base == 0 && obs->D[i] == 0.0f) 
				continue;
			/* residuals = observable - pseudorange ??(y)=???-???*/
			if (obs->L[i] != 0.0) y[i] = obs->L[i] * lam[i] - r - dant[i];
			if (obs->P[i] != 0.0) y[i + nf] = obs->P[i] - r - dant[i];
			trace(4, "base=%d(0 is rover) f=%d zdres:sat=%d l:y[%d]=%6.3f p:y[%d]=%6.3f\r\n", base, i, obs->sat, i, y[i], i + nf, y[i + nf]);
		}
	}
}

extern int zdres(int base, const obsd_t* obs, int n, const double* rs, const double* dts,
	const int* svh, const nav_t* nav, const double* rr, const prcopt_t* opt, int index,
	double* y, double* e, double* azel, midmsg_t* midmsg)
{
	double r = 0.0, rr_[3] = { 0 }, pos[3] = { 0 }, dant[NFREQ] = { 0 };
	double zhd = 0.0, zazel[] = { 0.0,90.0 * D2R };
	int i = 0, nf = NF(opt);
	/*稳健性判断*/
	if (norm(rr, 3) <= 0.0) {
		trace(3, "Rtdpos::zdres, no receiver position\r\n");
		return 0; /* no receiver position */
	}
	for (i = 0; i < 3; i++) rr_[i] = rr[i];
	ecef2pos(rr_, pos);
	for (i = 0; i < n && i < MAXDIFOBS * 2; i++) {
		/* compute geometric-range and azimuth/elevation angle */
		if ((r = geodist(rs + i * 6, rr_, e + i * 3)) <= 0.0) {
			trace(4,"zdres_geometric_range_continue base=%d sat=%d r=%f\n",base, midmsg[i].sat,r);
			continue;
		}
		/* excluded satellite? */
		if (satexclude(obs[i].sat, svh[i], opt))
			continue;
# if 0
		if (base == 0) {
			if (obsdect_out(obs + i, opt)) {
				trace(4, "Pntpos::zdres sat obs error.sat:%d\n", obs[i].sat);
				continue;
			}
		}
#endif
		if (satazel(pos, e + i * 3, azel + i * 2) < opt->elmin) {
			trace(4,"zdres_satazel_continue base=%d sat=%d\n", base, midmsg[i].sat);
			continue;
		}
		/* satellite clock-bias */
		r += -CLIGHT * dts[i * 2];
		/* troposphere delay model (hydrostatic) */
		zhd= tropmodel(obs[0].time, pos, zazel, 0.0);
		r += tropmapf(obs[i].time, pos, azel + i * 2, NULL) * zhd;
		/* receiver antenna phase center correction */
#if  0
		antmodel(opt->pcvr + index, opt->antdel[index], azel + i * 2, opt->posopt[1], dant);
# endif
		/* undifferenced phase/code residual for satellite */
		zdres_sat(base, r, obs + i, nav, azel + i * 2, dant, opt, y + i * nf * 2);
	}
	return 1;
}
/* test valid observation data -----------------------------------------------*/
static int validobs(int i, int j, int f, int nf, double* y)
{
	/* if no phase observable, psudorange is also unusable */
	return y[f + i * nf * 2] != 0.0 && y[f + j * nf * 2] != 0.0 &&
		(f < nf || (y[f - nf + i * nf * 2] != 0.0 && y[f - nf + j * nf * 2] != 0.0));
}
/* double-differenced measurement error covariance ---------------------------*/
extern void ddcov(const int* nb, int n, const double* Ri, const double* Rj,
	int nv, double* R)
{
	int i = 0, j = 0, k = 0, b = 0;

	trace(3, "ddcov   : n=%d\r\n", n);

	for (i = 0; i < nv * nv; i++) R[i] = 0.0;
	for (b = 0; b < n; k += nb[b++]) {
		for (i = 0; i < nb[b]; i++) for (j = 0; j < nb[b]; j++) {
			R[k + i + (k + j) * nv] = Ri[k + i] + (i == j ? Rj[k + i] : 0.0);
		}
	}
	//trace(5,"R=\r\n"); tracemat(5,R,nv,nv,8,6);
}

int dynamic_selrefsat_rtk(rtk_t* rtk, const obsd_t* obs, const int* sat, double* y,
	double* azel, const int* iu, const int* ir, int ns, int m, midmsg_t* midmsg, int f)
{
	if (rtk == NULL || obs == NULL || sat == NULL || y == NULL || azel == NULL || iu == NULL || ir == NULL || midmsg == NULL) {
		trace(3, "RtkposDynamic:: dynamic_selrefsat_rtk, operate null pointer.\n");
		return -1;
	}
	int i = 0, j = 0;
	int nf = rtk->opt.nf;
	int sysi = 0;//系统编号，m=0:gps,m=1:glo,m=2:gal,m=3:bds
	int ff = f % nf;
	for (i = -1, j = 0; j < ns; j++) {
		sysi = limitcmpgeo(obs[iu[j]].sat);
		if (sysi <= SYS_NONE || !test_sys(sysi, m)) continue;

		if (y[f + iu[j] * nf * 2] == 0.0 || y[f + ir[j] * nf * 2] == 0.0)
			continue;
		if ((iu[j] < 2 * MAXDIFOBS) && (midmsg[iu[j]].dpsr1 == 1 || midmsg[iu[j]].ddop1))
			continue;
		if (obs[iu[j]].snr[ff] * 0.25 < rtk->opt.snravg[m][ff])
			rtk->ssat[j].snr_flag[ff] = 1;//add0817
		if ((rtk->ssat[j].ambresetfail[ff]))
			continue;
		if ((rtk->ssat[j].rest[ff]))
			continue;
		if ((rtk->ssat[j].slip[ff] & 3))
			continue;
		if ((i < 0 || azel[1 + iu[j] * 2] >= azel[1 + iu[i] * 2]) && obs[iu[j]].snr[ff] * 0.25 >= rtk->opt.snravg[m][ff] * 0.99 && azel[1 + iu[j] * 2] * R2D > MINREFSTARELEV)
			i = j;
	}
	if(i>-1) trace(3," dynamic_selrefsat_rtk::NO.1 success select ref sat! system is  %d, satnumber in sat list  is %d ,rank is %d\n",m,sat[i],i);
	if (i < 0) {
		for (i = -1, j = 0; j < ns; j++) {
			sysi = limitcmpgeo(obs[iu[j]].sat);  //从vrs数据来看，北斗GEO卫星信噪比相对其他卫星来说偏低，且当前单点未对北斗GEO卫星进行处理，当北斗GEO卫星存在粗差且被选为参考星时会导致整个系统双差残差都较大
			if (sysi <= SYS_NONE || !test_sys(sysi, m)) continue;

			if (y[f + iu[j] * nf * 2] == 0.0 || y[f + ir[j] * nf * 2] == 0.0)
				continue;
			//trace(5, "dynamic_selrefsat_rtk::rtk sel ref sat:%d snr=%6.1f elev=%6.1f sys_snr_ave=%6.1f\n", obs[iu[j]].sat, obs[iu[j]].snr[ff] * 0.25, azel[1 + iu[j] * 2] * R2D, rtk->opt.snravg[m][ff]);
			if (midmsg[iu[j]].dpsr1 == 1 || midmsg[iu[j]].ddop1)
				continue;
			if (obs[iu[j]].snr[ff] * 0.25 < rtk->opt.snravg[m][ff])
				rtk->ssat[j].snr_flag[ff] = 1;//add0817
			if ((rtk->ssat[j].ambresetfail[ff]))
				continue;
			if ((i < 0 || azel[1 + iu[j] * 2] >= azel[1 + iu[i] * 2]) && obs[iu[j]].snr[ff] * 0.25 >= rtk->opt.snravg[m][ff] * 0.99 && azel[1 + iu[j] * 2] * R2D > MINREFSTARELEV)
				i = j;
		}
		if(i>-1)  trace(3," dynamic_selrefsat_rtk::NO.2 success select ref sat by other! system is  %d, satnumber in sat list  is %d ,rank is %d\n",m,sat[i],i);
	}
	if (i < 0) trace(4, "dynamic_selrefsat_rtk::rtk select ref sat failed\n");
	return i;
}

/* single-differenced measurement error variance -----------------------------*/
double rtkvarerr2(int sat, int sys, double el, double bl, double dt, int f,
	const prcopt_t* opt)
{
	double a = 0.0, b = 0.0, c = opt->err[3] * bl / 1E4, d = CLIGHT * opt->sclkstab * dt, fact = 1.0;
	double sinel = sin(el);
	int i = sys == SYS_GLO ? 1 : (sys == SYS_GAL ? 2 : 0), nf = NF(opt);

	if (f >= nf) fact = opt->eratio[f - nf];
	if (fact <= 0.0)  fact = opt->eratio[0];
	fact *= sys == SYS_GLO ? EFACT_GLO : (sys == SYS_SBS ? EFACT_SBS : EFACT_GPS);
	a = fact * opt->err[1];
	b = fact * opt->err[2];

	return 2.0 * (opt->ionoopt == IONOOPT_IFLC ? 3.0 : 1.0) * (a * a + b * b / sinel / sinel + c * c) + d * d;
}

/* sinfle-differenced measurement error variance for phase and code (rtk) 没有对系统定权*/
double rtkvarerr(double snr, int sys, double bl, double el, double dt, int f, int nf) {
	double k = 30, a = 10, b = -0.06, phase = 0.001, code = 15;
	double sinel = sin(el);
	double c = bl / 100000;
	double fact = 1.0;
	snr = MIN(snr, 42);

	if (sys == SYS_GPS || sys == SYS_CMP || sys == SYS_GAL) {
		if (sys == SYS_GAL)fact *= 1.0;

		if (f < nf) return fact * phase * (k * pow(a, b * snr) / (sinel * sinel) + c * c + dt * dt * 0.0015);
		else if (f >= nf) return fact * code * (k * pow(a, b * snr) / (sinel * sinel) + c * c + dt * dt * 0.0015);
	}else {
		trace(5, "this may be a bug! -from RtkposDynamic.c rtkvarerr\n");
		return 10000.0;
	}
	return 1;
}
/*新版新增*/
void ddres_adjust_r(rtk_t* rtk, const obsd_t* obs, int f, int nf, int i, int j, double v, double* r1, double* r2)
{
	int ff = f % nf;
	double ri = 0, rj = 0;
	int a = 5;
	int b = 10;
	int c = 50;
	int e = 1000 * 45;   //降权降太狠了，相位双差残差较难收敛
	ri = *r1;
	rj = *r2;
#if 1
	if (f >= nf) { //伪距R阵根据残差大小调整
		if (fabs(v) > 3.0 && fabs(v) <= 5.0)
			rj *= a;
		if (fabs(v) > 5.0 && fabs(v) <= 10.0)
			rj *= b;
		else if (fabs(v) > 10.0)
			rj *= c;
	}
	else {     //调整相位R阵
		if ((rtk->ssat[i].rest[ff])) { //参考星重置了模糊度
			ri *= e;//b;
			rj *= e;//b;
		}
		else if (!(rtk->ssat[i].rest[ff]))  {//非参考星重置了模糊度
			if ((rtk->ssat[j].slip[ff] & 2) && !(rtk->ssat[j].rest[ff]))//1怎么办
				rj *= a;
			if ((rtk->ssat[j].rest[ff]))//1怎么办
				rj *= e;//b;
		}
		if (!(rtk->ssat[j].rest[ff]) && !(rtk->ssat[i].rest[ff]))  {//模糊度未重置时
			if (fabs(v) > 0.3) {
				rj *= b;
			}
		}
	}
	*r1 = ri;
	*r2 = rj;
# endif
}

/* double-differenced phase/code residuals -----------------------------------*/
int ddres(int rflag, rtk_t* rtk, const obsd_t* obs, const nav_t* nav, double dt, double* x, const double* P,
	const int* sat, double* y, double* e, double* azel, const int* iu, const int* ir, int ns, double* v, double* H, double* R, int* vflg, midmsg_t* midmsg) {
	prcopt_t* opt = &rtk->opt;
	double bl = 0, dr[3] = { 0 }, posu[3] = { 0 }, posr[3] = { 0 };
	double* ri = NULL, * rj = NULL, lami = 0, lamj = 0, * hi = NULL;
	int i = 0, j = 0, k = 0, m = 0, f = 0, ff = 0, nv = 0, nb[NFREQ * 4 * 2 + 2] = { 0 }, b = 0, sysi = 0, sysj = 0, nf = NF(opt);
	double prefixdt = 0.0;//nb频点*系统*观测类型
	int nsat = 0;
	int n_sat_errl = 0;
	int n_sat_slip = 0;
	double dop[4] = { 0 };
	double* azel_ = NULL;
	int na = NP(opt);
	int nx = NP(opt) + ns * nf;
	int  numflag[3] = { 0 };//三个阶梯的各自的个数
	double  numflagratio[3] = { 0.0 };//三个阶梯的各自的比例

	if (!rtk->prefixtime.time) prefixdt = timediff(rtk->prefixtime, obs[0].time);
	trace(3, "ddres   : dt=%.1f nx=%d ns=%d\r\n", dt, rtk->nx, ns);//dt基准流动数据龄期
	bl = baseline(x, rtk->rb, dr);
	ecef2pos(x, posu);
	ecef2pos(rtk->rb, posr);
	azel_ = zeros(2 * nf, ns);
	ri = mat(ns * nf * 2 * 2 + 1, 1);
	rj = mat(ns * nf * 2 * 2 + 1, 1);

	for (m = 0; m < 4; m++) /* m=0:gps/qzs/sbs,1:glo,2:gal,3:bds */
		for (f = 0; f < nf * 2; f++) {
			ff = f % nf;
			/*选参考星 */
			i = dynamic_selrefsat_rtk(rtk, obs, sat, y, azel, iu, ir, ns, m, midmsg, f);
			if (i < 0) continue;
			/* make double difference */
			for (j = 0; j < ns; j++) {
				if (i == j) continue;
				/*做双差残差前数据粗差剔除*/
				sysi = rtk->ssat[i].sys;
				sysj = rtk->ssat[j].sys;
				lami = nav->lam[sat[i] - 1][ff];
				lamj = nav->lam[sat[j] - 1][ff];
				if (lami <= 0.0 || lamj <= 0.0) continue;
				if (!test_sys(sysj, m) || !validobs(iu[j], ir[j], f, nf, y)) continue;//trace(2,"ddres: test_sys failed: sat=%3d m=%2d\n",sat[j],m);
				if (f < nf && rtk->ssat[j].ambresetfail[f]) {trace(4, "the sat amb reset failed:sat=%d\n", sat[j]);continue;}
				if (midmsg[iu[j]].dpsr2[ff] == 1){
					if (f >= nf) { //当伪距发生粗差时直接将伪距观测值剔除
						continue;//add//trace(2,"ddres:midmsg failed: sat=%3d i=%4d iu=%4d msat=%3d DSnr1=%2d\n",sat[j],j,iu[j],midmsg[iu[j]].sat,midmsg[iu[j]].DSnr1);
					}
					if (f < nf && rtk->ssat[j].lock[ff] <= 0) {//TODO 当伪距发生粗差时，相位模糊度只要未初始化可继续组双差方程
						continue;//add//trace(2,"ddres:midmsg failed: sat=%3d i=%4d iu=%4d msat=%3d DSnr1=%2d\n",sat[j],j,iu[j],midmsg[iu[j]].sat,midmsg[iu[j]].DSnr1);
					}
				}
				if (f < nf && midmsg[iu[j]].dlsr2[ff] == 1) continue;//TODO//add//trace(2,"ddres:midmsg failed: sat=%3d i=%4d iu=%4d msat=%3d DSnr1=%2d\n",sat[j],j,iu[j],midmsg[iu[j]].sat,midmsg[iu[j]].DSnr1);
				/* double-differenced residual */
				v[nv] = (y[f + iu[i] * nf * 2] - y[f + ir[i] * nf * 2]) - (y[f + iu[j] * nf * 2] - y[f + ir[j] * nf * 2]);
				trace(3," nv=%2d f=%2d i=%2d j=%2d nf =%2d iu[i]=%3d iu[j]=%3d ir[i]=%3d ir[j]=%3d v[nv] = %13.4f (y[f + iu[i] * nf * 2]  = %13.4f  y[f + ir[i] * nf * 2])  = %13.4f (y[f + iu[j] * nf * 2]  = %13.4f y[f + ir[j] * nf * 2])= = %13.4f\n",nv,f,i,j,nf , iu[i],iu[j], ir[i], ir[j],v[nv],y[f + iu[i] * nf * 2] , y[f + ir[i] * nf * 2] , y[f + iu[j] * nf * 2] , y[f + ir[j] * nf * 2]);
				/* partial derivatives by rover position */
				if (H) {
					hi = H + nv * (na + ns * nf);
					for (k = 0; k < 3; k++) hi[k] = -e[k + iu[i] * 3] + e[k + iu[j] * 3];
				}
				/* double-differenced phase-bias term */
				if (f < nf) {
					if (opt->ionoopt != IONOOPT_IFLC) {
						v[nv] -= lami * x[IB2(i, f, opt, rtk->nsat)] - lamj * x[IB2(j, f, opt, rtk->nsat)];
						if (H) {
							hi[IB2(i, f, opt, rtk->nsat)] =  lami;
							hi[IB2(j, f, opt, rtk->nsat)] = -lamj;
						}
					}
				}
# if  0
				/* glonass receiver h/w bias term */
				if (rtk->opt.glomodear == 2 && sysi == SYS_GLO && sysj == SYS_GLO && ff < NFREQGLO) {
					df = (CLIGHT / lami - CLIGHT / lamj) / 1E6; /* freq-difference (MHz) */
					v[nv] -= df * x[IL(ff, opt)];
					if (H) Hi[IL(ff, opt)] = df;
				}
				/* glonass interchannel bias correction */
				else if (sysi == SYS_GLO && sysj == SYS_GLO) {
					v[nv] -= gloicbcorr(sat[i], sat[j], &rtk->opt, lami, lamj, f);
				}
#endif
				if (opt->maxinno > 0.0 && fabs(v[nv]) > 30) {//缩小双差残差为固定阈值 50->30
					if (f < nf){
						midmsg[iu[j]].dlsr2[ff] = 1;
						trace(2, "outlier rejected sat = % 3d - % 3d %s%d v = %13.3f xi = %9.3f xj = %9.3f dxij = %9.3f\r\n", sat[i],sat[j], "L", f % nf + 1, v[nv], x[IB2(i, f, opt, rtk->nsat)], x[IB2(j, f, opt, rtk->nsat)], x[IB2(j, f, opt, rtk->nsat)] - x[IB2(i, f, opt, rtk->nsat)]);
					}else{
						midmsg[iu[j]].dpsr2[ff] = 1;
						trace(2, "outlier rejected sat = % 3d - % 3d %s%d v = %13.3f\r\n", sat[i], sat[j], "P", f % nf + 1, v[nv]);
					}
					continue;
				}
				/* single-differenced measurement error variances 没有对系统定权*/
				ri[nv] = rtkvarerr(obs[iu[i]].snr[ff] * 0.25, sysi, bl, azel[1 + iu[i] * 2], dt, f, nf);
				rj[nv] = rtkvarerr(obs[iu[j]].snr[ff] * 0.25, sysj, bl, azel[1 + iu[j] * 2], dt, f, nf);
#if 0
                if ( fabs(v[nv]) > 3.0 && fabs(v[nv]) <= 5.0){
                    if(rtk->vel>=0.5 ) rj[nv] *= 5.0;
                     numflag[0]++;
                     trace(5,"3-5 R\n");
                }
                if ( fabs(v[nv]) > 5.0 && fabs(v[nv]) <= 10.0){
                    if(rtk->vel>=0.5 ) rj[nv] *= 10.0;
                     numflag[1]++;
                     trace(5,"5-10 R\n");
                }
                else if (fabs(v[nv]) > 10.0){
                    if(rtk->vel>=0.5 ) rj[nv] *= 30.0;
                     numflag[2]++;
                     trace(5,"10+ R\n");
                }
#endif

				if (f < nf && fabs(v[nv])>1)
					n_sat_slip++;
				if (f < nf && fabs(v[nv])>0.5 && !(rtk->ssat[j].rest[ff])) 
				/* set valid data flags */
				if (opt->mode > PMODE_DGPS) {
					if (f < nf){
						rtk->ssat[i].vsat[f] = 1;
						rtk->ssat[j].vsat[f] = 1;
					}
				}else {
					rtk->ssat[i].vsat[f - nf] = 1;
					rtk->ssat[j].vsat[f - nf] = 1;
				}
				trace(3, "nv=%d i=%d j=%d sat=%3d-%3d %s%d f=%2d nsat=%d  opt=%2d v=%8.3f R=%10.6f %10.6f   Nr[%2d]=%8.3f   Ns[%2d]=%8.3f   float_N=%8.3lf lock=%d refi=%4d refj=%4d\r\n", nv, i, j, sat[i],
					sat[j], f < nf ? "L" : "P", f % nf + 1, f, rtk->nsat, IB2(j, ff, opt, rtk->nsat), v[nv], ri[nv], rj[nv], sat[i], f < nf ? x[IB2(i, ff, opt, rtk->nsat)] : 0.0,
					sat[j], f < nf ? x[IB2(j, ff, opt, rtk->nsat)] : 0.0, f < nf ? (x[IB2(i, ff, opt, rtk->nsat)] - x[IB2(j, ff, opt, rtk->nsat)]) : 0.0, rtk->ssat[j].lock[ff], rtk->ssat[i].ref[ff], rtk->ssat[j].ref[ff]);
				if (f < nf) {
					azel_[2 * nsat] = azel[iu[j] * 2];
					azel_[2 * nsat + 1] = azel[1 + iu[j] * 2];
					nsat++;
				}
				vflg[nv++] = (iu[i] << 16) | (iu[j] << 8) | ((f < nf ? 0 : 1) << 4) | (f % nf);
				nb[b]++;
			}
			b++;
		}
	if (nsat > 0){
		rtk->n_lerr_ratio = (double)n_sat_errl / nsat;//没有重置然后超限0.5比例
		rtk->n_slip_ratio = (double)n_sat_slip / nsat;//双差残差超限1比例
		trace(4, "rtkerro: nsat=%4d n_sat_errL=%4d n_Lerr_ratio=%6.3f n_slip_ratio=%6.3f\r\n", nsat, n_sat_errl, rtk->n_lerr_ratio, rtk->n_slip_ratio);
	}
	if (rtk->sol_rtk.resetflag){//时间更新初始化后测量更新对dop有约束
		dops_dif(nsat, azel_, rtk->opt.elmin, dop, NULL);
		trace(4, "RTK:the dops of RTK:nsat=%d gdop=%.1f pdop=%.1f\r\n", nsat, dop[0], dop[1]);
		if (dop[0] <= 0.0 || dop[1] > 4.0 || dop[0] > rtk->opt.maxgdop){
			trace(4, "RTK:gdop error nv=%d gdop=%.1f pdop=%.1f\r\n", nv, dop[0], dop[1]);
			nv = 0;
		}
	}
	ddcov(nb, b, ri, rj, nv, R);
	if (nv > 0) rtk_igg3test(rtk, rflag, obs, x, P, H, v, R, sat, nx, nv, vflg, midmsg);
	xy_free(ri);
	xy_free(rj);
	xy_free(azel_);
	return nv;
}

extern void age_init(rtk_t* rtk)
{
	if (rtk == NULL) {
		trace(3, "AbstractDoubleDiffPreHandle::AgeInit, operate null pointer.\n");
		return;
	}
	ssat_t ssat0 = { 0 };
	int i = 0;
	trace(3, "AbstractDoubleDiffPreHandle::AgeInit, rtkinit :\n");
	trace(3, "rtkinit :\n");
	memset(rtk->x, 0, sizeof(double) * rtk->nx);
	memset(rtk->P, 0, sizeof(double) * rtk->nx * rtk->nx);
	memset(rtk->xa, 0, sizeof(double) * rtk->na);
	memset(rtk->pa, 0, sizeof(double) * rtk->na * rtk->na);
	rtk->nfix = 0;
	for (i = 0; i < MAXDIFOBS; i++) {
		rtk->ssat[i] = ssat0;
		rtk->satid[i] = 0;
	}
}

void copysol(sol_t* sol, sol_t* tem_sol)
{
	if (sol == NULL || tem_sol == NULL) {
		trace(3, "AbstractDifferenceSolution::copysol, operate null pointer.\n");
		return;
	}
	for (int i = 0; i < 6; i++)
		sol->rr[i] = tem_sol->rr[i];
	for (int i = 0; i < 6; i++)
		sol->qr[i] = tem_sol->qr[i];
	sol->stat = tem_sol->stat;
	sol->ns = tem_sol->ns;
	sol->age = tem_sol->age;
}

int selectsatellite(const obsd_t* obs, double* azel, int nu, int nr,
	const prcopt_t* opt, int* sat, int* iu, int* ir) {
	if (obs == NULL || azel == NULL || opt == NULL || sat == NULL || iu == NULL || ir == NULL) {
		trace(3, "DefaultDoubleDifPreHandle::SelectSatellite, operate null pointer.\n");
		return -1;
	}
	int i = 0, j = 0, k = 0;
	int sys;
	if (!(sys = satsys(obs->sat, NULL))) {
		return 0;
	}
	trace(3, "select common satellite  : nu=%d nr=%d\n", nu, nr);
	for (i = 0, j = nu; i < nu && j < nu + nr && k < MAXDIFOBS; i++, j++) { 
		if (obs[i].sat < obs[j].sat) {
			j--;
		}else if (obs[i].sat > obs[j].sat) {
			i--;
		}else if (azel[1 + j * 2] >= opt->elmin) { /* elevation at base station */
			if (obs[i].snr[0] * 0.25 > opt->snrthres)
			{
				sat[k] = obs[i].sat; iu[k] = i; ir[k++] = j;
				trace(4, "(%2d) sat=%3d iu=%2d ir=%2d el=%4.1lf snr = %4.1lf\r\n", k - 1, obs[i].sat, i, j, azel[1 + j * 2] * R2D, obs[i].snr[0] * 0.25);
			}
		}
	}
	return k;
}
/*双差选共视星，基准站非差残差*/
int reldatapre(rtk_t* rtk, const obsd_t* obs, int nu, int nr, const nav_t* nav, midmsg_t* midmsg,
	double* rs, double* dts, int* svh, double* y, double* e, double* azel, int* sat, int* iu, int* ir) {
	if (rtk == NULL || obs == NULL || nav == NULL || midmsg == NULL || rs == NULL || dts == NULL ||
		svh == NULL || y == NULL || e == NULL || azel == NULL || sat == NULL || iu == NULL || ir == NULL) {
		return -1;
	}
	prcopt_t* opt = &rtk->opt;
	gtime_t time = obs[0].time;
	double snr_avg = 0, sns, this_snr = 0;
	int i = 0, j = 0, m = 0, n = nu + nr, ns = 0, nf = opt->nf;
	int sys = 0;
	int f = 0;
	double var[MAXDIFOBS * 2] = { 0.0 };
	for (i = 0; i < MAXDIFOBS; i++) {
		for (j = 0; j < NFREQ; j++) {
			rtk->ssat[i].rest[j] = 0;
			rtk->ssat[i].vsat[j] = rtk->ssat[i].snr[j] = 0;
		}
	}
	/* satellite positions/clocks */
	satposs(time, obs, n, nav, opt->sateph, rs, dts, var, svh, 2 * MAXDIFOBS);
	/* undifferenced residuals for base station */
	if (!zdres(1, obs + nu, nr, rs + nu * 6, dts + nu * 2, svh + nu, nav, rtk->rb,
		opt, 1, y + nu * nf * 2, e + nu * 3, azel + nu * 2, midmsg + nu)) {
		trace(4, "undifferenced residuals for base station error\r\n");
		return -1;
	}
	/* select common satellites between rover and base-station共视星 */
	if ((ns = selectsatellite(obs, azel, nu, nr, opt, sat, iu, ir)) <= 0) {
		trace(1, "no common satellite\r\n");
		return -1;
	}
	for (i = 0; i < ns; i++) {
		for (j = 0; j < nf; j++) {
			rtk->ssat[i].snr[j] = obs[iu[i]].snr[j];
			rtk->ssat[i].sys = satsys(obs[iu[i]].sat, NULL);
# if 0
			if (obs[iu[i]].SNR[j] > 1 && obs[iu[i]].snr[j] * 0.25 <= 25)  //避免统计了信噪比为零的情况
				rtk->n_errsat++;
			if (obs[iu[i]].SNR[j] > 1 && obs[iu[i]].snr[j] * 0.25 <= 30)  //避免统计了信噪比为零的情况
				rtk->n_errsat_30++;
#endif
		}
	}
	/* 选参考星的判断1：计算所有高度角大于35的信噪比均值（最小是40） */ //test  未用
# if 1
	for (m = 0; m < 4; m++) {
		for (f = 0; f < opt->nf; f++)
		{
			snr_avg = 0.0;
			sns = 0;
			for (i = 0; i < ns; i++) {
				sys = limitcmpgeo(obs[iu[j]].sat);  //从vrs数据来看，北斗GEO卫星信噪比相对其他卫星来说偏低，且当前单点未对北斗GEO卫星进行处理，当北斗GEO卫星存在粗差且被选为参考星时会导致整个系统双差残差都较大
				if (sys <= SYS_NONE || !test_sys(sys, m))
					continue;
				if (azel[ir[i] * 2 + 1] * R2D <= MINREFSTARELEV || obs[iu[i]].snr[f] * 0.25 <= (double)opt->snrthres)
					continue;
				snr_avg += obs[iu[i]].snr[f] * 0.25;
				sns++;
			}
			if (sns > 0) {
				opt->snravg[m][f] = MIN(snr_avg / sns, 40);
			}else {
				opt->snravg[m][f] = 0;
			}
		}
	}
#endif
	return ns;
}

void savesol(rtk_t* rtk, int stat_rtk, int stat_rtd)
{
	if (rtk == NULL) {
		trace(3, "RtkDynamicDifferenceSolution::savesol, operate null pointer.\r\n");
		return;
	}
	trace(3, "savesol stat_rtk=%d stat_rtd=%d\r\n", stat_rtk, stat_rtd);
	double qr_rtk[3] = { 0.0 };
	double qr_rtd[3] = { 0.0 };
	double qr_spp[3] = { 0.0 };
	double diff[3] = { 0.0 };
	double qr_sol[3] = { 0.0 };
	for (int i = 0; i < 3; i++) {
		qr_rtk[i] = rtk->sol_rtk.qr[i];
		qr_rtd[i] = rtk->sol_rtd.qr[i];
		qr_spp[i] = rtk->sol_spp.qr[i];
		diff[i] = rtk->sol_rtk.rr[i] - rtk->sol_rtd.rr[i];
		qr_sol[i] = rtk->sol.qr[i];
	}
	trace(4, "norm rtk qr = %6.3lf  stat =%d ", norm(qr_rtk, 3), rtk->sol_rtk.stat);
	trace(4, "norm rtd qr = %6.3lf  stat =%d ", norm(qr_rtd, 3), rtk->sol_rtd.stat);
	trace(4, "norm spp qr = %6.3lf  stat =%d ", norm(qr_spp, 3), rtk->sol.stat);

	if (norm(qr_sol, 3) > 30)  //主要是清除原存在sol里的最小二乘单点结果，主要考虑输出kalman单点的结果，最小二乘单点容易导致轨迹不够平滑
		rtk->sol.stat = 0;

	if (rtk->basechange) {
		floatnum = 0;
	}
	if (rtk->opt.mode == PMODE_DGPS) {
		copysol(&rtk->sol, &rtk->sol_rtd);
	}
	else if (rtk->opt.mode == PMODE_KINEMA || rtk->opt.mode == PMODE_STATIC) {
		{
			if (norm(qr_rtk, 3) < 1.0) {
				floatnum++;
			}else {
				floatnum = 0;
			}
			trace(4, "floatnum=%d\r\n", floatnum);
			/*if (floatnum > 5) */
			trace(5, "rtkstat=%d\r\n", rtk->sol_rtk.stat);
			if (stat_rtk >= 0 && norm(qr_rtk, 3) < 0.1 && rtk->sol_rtk.stat == 1 && floatnum > 0)//只出  由5改成0  1
			{
				copysol(&rtk->sol, &rtk->sol_rtk);
				trace(4, "solution is RTK floatnum=%d\r\n", floatnum);
			}
			else if (stat_rtk >= 0 && norm(qr_rtk, 3) < 1.0 && rtk->sol_rtk.stat == 2)//只出 2
			{
				copysol(&rtk->sol, &rtk->sol_rtk);
				trace(4, "solution is float  RTK floatnum=%d\r\n", floatnum);
			}
			else if (stat_rtd >= 0 && norm(qr_rtd, 3) < 5 && rtk->sol_rtd.stat == 4)//只出 rtd成功
			{
				copysol(&rtk->sol, &rtk->sol_rtd);
				trace(4, "solution is RTd floatnum=%d\r\n", floatnum);
			}
			else if (stat_rtd == -1 && stat_rtk == -1 && rtk->sol_spp.stat == 5 && norm(qr_spp, 3) < 20)//  4 rtd  rtk 均失败
			{
				if (norm(qr_sol, 3) > 15 && rtk->sol.spp_flag == 1)//spp fail 超限 不传递下一历元
				{
					rtk->sol.spp_flag = 2;
					trace(4, "solution is spp but the spp pail \r\n");
				}
				copysol(&rtk->sol, &rtk->sol_spp);
				trace(4, "solution is spp floatnum=%d   norm(qr_spp, 3) =%f\r\n", floatnum, norm(qr_spp, 3)); //仅仅spp
			}
			else if (norm(qr_sol, 3) > 20 && rtk->sol.spp_flag == 1)//spp fail 超限 不save
			{
				rtk->sol.stat = 0;
				trace(4, "norm(qr_sol, 3) >15 \r\n");
			}else {
				trace(4, "solution is  not  all and save the sol.rr  norm(qr_sol, 3) is ok rtk->sol.stat=%d  \r\n", rtk->sol.stat);
				if ((rtk->sol.stat == 4 && norm(qr_sol, 3) > 5) && rtk->sol.spp_flag == 1)
				{
					rtk->sol.spp_flag = 2;
					trace(4, "solution is rtd but the spp fail \r\n");
					trace(5, "lasttime =%s currenttime=%s  \n", time_str(rtk->sol_last.time, 3), time_str(rtk->sol.time, 3));
				}
				if ((rtk->sol.stat == 5 && norm(qr_sol, 3) > 10) && rtk->sol.spp_flag == 1)//上面条件都不满足，spp fail 但是进行了kalman 此时存的是spp的话，不传递下一历元
				{
					rtk->sol.spp_flag = 2;
					trace(4, "solution is spp but the spp fail \r\n");
				}
			}
		}
	}
}
void rebuild_p(rtk_t* rtk, int m, double* preP, int n, int nx)
{
	int i = 0, j = 0;

	for (i = 0; i < nx; i++)
	{
		rtk->P[m + i * rtk->nx] = preP[n + i * nx];
		rtk->P[m * rtk->nx + i] = preP[n * nx + i];
	}
}

void addp_one_ranks(rtk_t* rtk, int k, int f)
{
	int i = 0, j = 0;
	int nf = rtk->opt.nf - 1;

	for (i = IB2(rtk->nsat, nf, &rtk->opt, rtk->nsat) + f; i > (k - 1); i--)  
	{
		//trace(2, "addp: i=%4d nsat=%4d k=%4d\r\n", i,rtk->nsat,k);
		for (j = 0; j < i; j++)
		{
			if (j < k)
				rtk->P[i + 1 + j * rtk->nx] = rtk->P[i + j * rtk->nx];

			else /*if(j>k)*/
				rtk->P[i + 1 + (j + 1) * rtk->nx] = rtk->P[i + j * rtk->nx];
		}
		rtk->P[i + 1 + k * rtk->nx] = 0;
	}

	for (j = IB2(rtk->nsat, nf, &rtk->opt, rtk->nsat) + f; j > (k - 1); j--)
	{
		for (i = 0; i < j; i++)      //左下方下移[(0,k),(k,k),(0,presat),(k,presat)]
		{
			if (i < k)
				rtk->P[i + (j + 1) * rtk->nx] = rtk->P[i + j * rtk->nx];
			else /*if(i>k)*/
				rtk->P[i + 1 + (j + 1) * rtk->nx] = rtk->P[i + j * rtk->nx];
		}
		rtk->P[k + (j + 1) * rtk->nx] = 0.0;
	}
	for (i = IB2(rtk->nsat, nf, &rtk->opt, rtk->nsat) + 1 + f; i > k; i--)
		rtk->P[i + i * rtk->nx] = rtk->P[i - 1 + (i - 1) * rtk->nx];  
}

void delp_one_ranks1(rtk_t* rtk, int k)
{
	int i = 0, j = 0;
	int na = rtk->na;
	for (i = k; i < rtk->nsat + na; i++)   //右上方前移[(k,0),(k,k),(presat,0),(presat,k)]
	{
		for (j = 0; j < i; j++)
		{
			if (j < k)
			{
				trace(3, "i=%3d j=%3d p(i,j)=%7.3f\n", i, j, rtk->P[i + 1 + j * rtk->nx]);
				rtk->P[i + j * rtk->nx] = rtk->P[i + 1 + j * rtk->nx];
			}
			else
				rtk->P[i + (j - 1) * rtk->nx] = rtk->P[i + 1 + j * rtk->nx];
		}
	}
	trace(3, "P(0)=\n"); tracemat(5, rtk->P, rtk->nx, rtk->nx, 7, 4);

	for (j = k; j < rtk->nsat + na; j++)
	{
		for (i = 0; i < j; i++)      //左下方上移[(0,k),(k,k),(0,presat),(k,presat)]
		{
			if (j < k)
				rtk->P[i + j * rtk->nx] = rtk->P[i + (j + 1) * rtk->nx];
			else
				rtk->P[i - 1 + j * rtk->nx] = rtk->P[i + (j + 1) * rtk->nx];
		}
	}

	for (i = k; i < rtk->nsat + na; i++)
		rtk->P[i + i * rtk->nx] = rtk->P[i + 1 + (i + 1) * rtk->nx];   //更新对角线上的值
}

void delp_one_ranks(rtk_t* rtk, int k, int f)
{
	int i = 0, j = 0;
	int nf = rtk->opt.nf - 1;

	for (i = k; i < IB2(rtk->nsat, nf, &rtk->opt, rtk->nsat) - f; i++)
	{
		for (j = 0; j < i; j++)
		{
			if (j < k)
			{
				rtk->P[i + j * rtk->nx] = rtk->P[i + 1 + j * rtk->nx];
				rtk->P[j + i * rtk->nx] = rtk->P[j + (i + 1) * rtk->nx];
			}
		}
		//右下角原始先上移
		for (j = k; j < IB2(rtk->nsat, nf, &rtk->opt, rtk->nsat) - f; j++) {
			rtk->P[i + j * rtk->nx] = rtk->P[i + 1 + j * rtk->nx];
		}
		for (j = k; j < IB2(rtk->nsat, nf, &rtk->opt, rtk->nsat) - f; j++) {
			rtk->P[i + j * rtk->nx] = rtk->P[i + (j + 1) * rtk->nx];
		}
	}
}
void recode_fall_sat(rtk_t* rtk, int sat, int k)
{
	/*变量初始化*/
	int i = 0, j = 0;
	int nf = NF(&rtk->opt);
	ssat_t ssat0 = { 0 };
	for (i = 0; i < rtk->falsatnum; i++)
	{
		if (timediff(rtk->time, rtk->f_ssat[i].pt[0][0]) > 5)  
		{
			for (j = i; j < rtk->falsatnum - 1; j++)
				rtk->f_ssat[j] = rtk->f_ssat[j + 1];
			rtk->f_ssat[rtk->falsatnum - 1] = ssat0;
			rtk->falsatnum--;
		}
	}
	for (i = 0; i < rtk->falsatnum; i++)
	{
		if (sat == rtk->f_ssat[i].sat)
		{
			rtk->f_ssat[i] = rtk->ssat[k];
			for (j = 0; j < nf; j++)
				rtk->f_ssat[i].x[j] = rtk->x[IB2(k, j, &rtk->opt, rtk->presat)];//记录降星时的模糊度
			trace(5, "old fall star:sat=%4d x=%6.3f\r\n", rtk->ssat[k].sat, rtk->ssat[k].x[0]);
			return;
		}
	}
	trace(5, "new fall star:sat=%4d falsatnum=%4d\r\n", rtk->ssat[k].sat, rtk->falsatnum);
	if (rtk->falsatnum >= MAXFALSTA)   
		return;
	rtk->f_ssat[i] = rtk->ssat[k];
	for (j = 0; j < nf; j++)
		rtk->f_ssat[i].x[j] = rtk->x[IB2(k, j, &rtk->opt, rtk->presat)];
	rtk->falsatnum++;
	trace(5, "new fall star:sat=%4d ssat.x[0]=%6.3f rtk.x[0]=%6.3f\r\n", rtk->ssat[k].sat, rtk->ssat[k].x[0], rtk->x[IB2(k, 0, &rtk->opt, rtk->presat)]);
}
/*写法可能有问题*/
void recode_nocom_sat(rtk_t* rtk, obsd_t obs)
{
	int i = 0, j = 0, nf = rtk->opt.nf;
	ssat_t ssat0 = { 0 };
	trace(5, "recode_nocom_sat::recode no commom sat: %4d\r\n", obs.sat);
	/*之前储存过的降星信息覆盖*/
	for (i = 0; i < rtk->falsatnum; i++){
		if (obs.sat == rtk->f_ssat[i].sat){
			for (j = 0; j < nf; j++){
				rtk->f_ssat[i].pt[obs.rcv - 1][j] = obs.time;
				rtk->f_ssat[i].ph[obs.rcv - 1][j] = obs.L[j];
				rtk->f_ssat[i].pd[obs.rcv - 1][j] = obs.D[j];
				rtk->f_ssat[i].pp[obs.rcv - 1][j] = obs.P[j];
			}
		}
	}

	if (rtk->falsatnum >= MAXFALSTA) return;//超出阈值则不在继续存储

	rtk->f_ssat[i] = ssat0;
	rtk->f_ssat[i].sat = obs.sat;
	for (j = 0; j < nf; j++){
		rtk->f_ssat[i].pt[obs.rcv - 1][j] = obs.time;
		rtk->f_ssat[i].ph[obs.rcv - 1][j] = obs.L[j];
		rtk->f_ssat[i].pd[obs.rcv - 1][j] = obs.D[j];
		rtk->f_ssat[i].pp[obs.rcv - 1][j] = obs.P[j];
	}
	rtk->falsatnum++;
}

void adjust_p(rtk_t* rtk, const int* sat, int ns)
{
	/*初始化*/
	int i = 0, j = 0, k = 0;
	int satid[MAXDIFOBS] = { 0 };
	int na = rtk->na;
	int nsat = rtk->nsat;
	int nf = NF(&rtk->opt);
	for (i = 0; i < rtk->nsat; i++) satid[i] = rtk->satid[i];
	/*先判断是否存在减星，避免内存过大*/
	for (k = 0, i = 0; i < nsat; i++, k++)
	{
		if (rtk->nsat <= 0) break;
		if (ix(rtk->satid[i], sat, ns) < 0) {
			
			recode_fall_sat(rtk, rtk->satid[i], i);
			for (j = k; j < rtk->nsat - 1; j++)//将降星号在satid中抽除
				satid[j] = satid[j + 1];
			//P阵中分别删去一行 一列
			trace(2, "decline sat:%d i=%d k+na=%d %d\r\n", rtk->satid[i], i, k, k + na);
			for (j = 0; j < nf; j++)
				delp_one_ranks(rtk, IB2(k, j, &rtk->opt, rtk->nsat) - j, j); 
			k--;
			rtk->nsat--;
		}
	}

	for (k = 0, i = 0; i < ns; i++, k++)
	{
		if (rtk->nsat <= 0)
			break;
		if (ix(sat[i], satid, rtk->nsat) < 0) {
			for (j = rtk->nsat + 1; j > k; j--)  //增星时插入卫星号，p值插入相应的零
				satid[j] = satid[j - 1];
			trace(2, "rise sat:%d i=%d j=%2d satidj=%2d  k=%d nsat=%4d\r\n", sat[i], i, j, satid[j], k, rtk->nsat);
			satid[k] = sat[i];
			//rtk->P阵中分别插入一行 一列
			for (j = 0; j < nf; j++)
				addp_one_ranks(rtk, IB2(k, j, &rtk->opt, rtk->nsat) + j, j);  //k的索引应与i一致
			rtk->nsat++;
		}
	}
}
int find_in_falsat(rtk_t* rtk, int sat, int k, int j)
{
	int i = 0;
	double dt = 0.0;
	
	for (i = 0; i < rtk->falsatnum; i++)//遍历降星标记rtk->f_ssat列表，看本星是否在之前列表存在过
	{
		//trace(5,"falsatinfo: i=%4d sat=%4d dt=%6.3f",i,rtk->f_ssat[i].sat,timediff(rtk->time,rtk->f_ssat[i].pt[0][0]));
		if (sat == rtk->f_ssat[i].sat)
		{
			/*稳健性判断*/
			if ((rtk->f_ssat[i].pt[0][j].time == 0 && rtk->f_ssat[i].pt[0][j].sec == 0.0) || rtk->f_ssat[i].ph[0][j] == 0.0)
				return 0;
			
			dt = timediff(rtk->time, rtk->f_ssat[i].pt[0][j]);
			trace(5, "sat fined in 5 sec:sat=%4d dt=%6.3f x=%6.3f\r\n", sat, dt, rtk->f_ssat[i].x[j]);
			if (dt > 2) {//目前outlock阈值为1,存上上个历元的x值仅作为模糊度重置标识
				rtk->x[IB2(k, j, &rtk->opt, rtk->nsat)] = 0.0;
			}else{
				rtk->x[IB2(k, j, &rtk->opt, rtk->nsat)] = rtk->f_ssat[i].x[j];
			}
			rtk->ssat[k] = rtk->f_ssat[i];
			rtk->f_ssat[i].outc[j] = 1;   //避免f=1的时候覆盖了f=0的时候outc的值
			rtk->ssat[k].outc[j] = 1;
			return 1;
		}
	}
	return 0;
}
void transfer(rtk_t* rtk, const obsd_t* obs, const int* sat, int* iu, int ns, double* azel)
{
	/*变量初始化*/
	int i = 0, j = 0, m = 0;
	double* pre_x = NULL;
	prcopt_t opt = rtk->opt;
	ssat_t ssat[MAXDIFOBS] = { {0} }, ssat0 = { 0 };
	int satid[MAXDIFOBS] = { 0 };
	int na = rtk->na;
	int nf = NF(&opt);
	//nx=rtk->presat*nf +na;   //上一个历元估计的位置参数+共视星数
	pre_x = zeros(rtk->nx, 1);
	/*先转存上一个历元的信息*/
	matcpy(pre_x, rtk->x, rtk->nx, 1);
	for (i = 0; i < rtk->presat; i++) ssat[i] = rtk->ssat[i];
	for (i = 0; i < rtk->presat; i++) satid[i] = rtk->satid[i];
	for (i = 0; i < na; i++) rtk->x[i] = pre_x[i];//位置初始化，套娃有什么用???
	rtk->nsat = rtk->presat;
	adjust_p(rtk, sat, ns);
	for (i = 0; i < ns; i++)
	{
		for (j = 0; j < rtk->presat; j++)
		{
			if (sat[i] == satid[j])// 遍历前历元卫星是否与当前sat[i]一致。satid[j]为 前历元卫星ID，sat[i]为当前历元
			{
				rtk->satid[i] = sat[i];
				rtk->ssat[i].sat = sat[i];
				rtk->ssat[i] = ssat[j];
				rtk->ssat[i].sys = satsys(sat[i], NULL);
				rtk->ssat[i].azel[0] = azel[iu[i] * 2];
				rtk->ssat[i].azel[1] = azel[1 + iu[i] * 2];

				for (m = 0; m < nf; m++){
					rtk->x[IB2(i, m, &rtk->opt, rtk->nsat)] = pre_x[IB2(j, m, &rtk->opt, rtk->presat)];
					rtk->ssat[i].x[m] = pre_x[IB2(j, m, &rtk->opt, rtk->presat)];
				}

				trace(5, "in sat: %4d i=%4d j=%4d sys=%d ssat_sys=%d x=%6.3f x=%6.3f\r\n", satid[j], i, j, rtk->ssat[i].sys, ssat[j].sys, rtk->x[i + na], rtk->x[IB2(i, 0, &rtk->opt, rtk->nsat)]);
				break;
			}
		}
		if (j == rtk->presat)
		{
			rtk->satid[i] = sat[i];
			rtk->ssat[i] = ssat0;

			for (m = 0; m < nf; m++) {
				if (!find_in_falsat(rtk, sat[i], i, m)){
					initx(rtk, 0, 0, IB2(i, m, &rtk->opt, rtk->nsat));
					if (rtk->presat > 0) {
						rtk->ssat[i].outc[m] = 1;  //中断计数  
					}
				}
			}
			rtk->ssat[i].sat = sat[i];
			rtk->ssat[i].sys = satsys(sat[i], NULL);
			rtk->ssat[i].azel[0] = azel[iu[i] * 2];
			rtk->ssat[i].azel[1] = azel[1 + iu[i] * 2];
			trace(5, "new sat: %4d i=%4d j=%4d x=%6.3f\r\n", sat[i], i, j, rtk->x[IB2(i, 0, &rtk->opt, rtk->nsat)]);
		}
	}
	/*rtk->ssat空余位置补0，并且相应的x和p清零*/
	for (i = ns; i < MAXDIFOBS; i++){
		rtk->satid[i] = 0;
		rtk->ssat[i] = ssat0;
	}
	m = nf - 1;
	for (i = IB2(ns, m, &rtk->opt, rtk->nsat); i < rtk->nx; i++) initx(rtk, 0, 0, i);  //其他清零，P值在rtk->x更新时更新
	/*更新前历元和当前历元差分星数*/
	rtk->presat = ns; 
	rtk->nsat = ns;
	xy_free(pre_x);
}

void set_p(rtk_t* rtk, double* Pp, int nx)
{
	int i = 0, j = 0;
	for (i = 0; i < nx; i++) for (j = 0; j < nx; j++) Pp[i * nx + j] = rtk->P[i * rtk->nx + j];
}
/* validation of solution ----------------------------------------------------*/
static int valpos(rtk_t* rtk, const int* sat, const double* v, const double* R, const int* vflg,
	int nv, double thres)
{
#if 0
	prcopt_t* opt = &rtk->opt;
	double vv = 0.0;
#endif
	double fact = thres * thres;
	int i = 0, stat = 1, sat1 = 0, sat2 = 0, type = 0, freq = 0, j = 0;
	char* stype = NULL;
	trace(3, "valpos  : nv=%d thres=%.1f\n", nv, thres);
	/* post-fit residual test */
	for (i = 0; i < nv; i++) {
		if (v[i] * v[i] <= fact * R[i + i * nv]) continue;//主要公式

		sat1 = (vflg[i] >> 16) & 0xFF;
		sat2 = (vflg[i] >> 8) & 0xFF;
		type = (vflg[i] >> 4) & 0xF;
		freq = vflg[i] & 0xF;
		stype = type == 0 ? "L" : (type == 1 ? "P" : "C");
		trace(5, "large residual (sat=%2d-%2d %s%d v=%6.3f sig=%.3f)\n",sat[sat1], sat[sat2], stype, freq + 1, v[i], SQRT(R[i + i * nv]));
		if (type == 0){
			j = ix(sat[sat2], rtk->satid, rtk->presat);
			rtk->ssat[j].vsat[freq] = 0; //影响
		}
	}
#if 0 
	if (stat && nv > NP(opt)) {
		/* chi-square validation */
		for (i = 0; i < nv; i++) vv += v[i] * v[i] / R[i + i * nv];

		if (vv > chisqr[nv - NP(opt) - 1]) {
			trace(3, "residuals validation failed (nv=%d np=%d vv=%.2f cs=%.2f)\n",
				nv, NP(opt), vv, chisqr[nv - NP(opt) - 1]);
			stat = 0;
		}
		else {
			trace(3, "valpos : validation ok (%s nv=%d np=%d vv=%.2f cs=%.2f)\n",
				time_str(rtk->sol.time, 2), nv, NP(opt), vv, chisqr[nv - NP(opt) - 1]);
		}
	}
#endif
	return stat;
}

int fixsoltest(rtk_t* rtk, double* xa)
{
	int i = 0;
	double difx[3] = { 0 };
	for (i = 0; i < 3; i++)
		difx[i] = xa[i] - rtk->x[i];
	if (rtk->sol.ratio > 3)
		return 1;
	if (!rtk->sol_last.stat) 
		return -1;
	if (rtk->sol_last.stat > 2) 
		return -1;
	if (norm(difx, 3) >= 2) { 
		trace(4, "fix test:the difx is too big %f\n", norm(difx, 3));
		return -1;
	}
# if 0
	if (rtk->sol.ratio < 2 && norm(difx, 3) >= 1)
		return -1;
	
	if (rtk->sol_rtk.nsat[0] == 0 || rtk->sol_rtk.nsat[3] == 0)
		return -1;
#endif
	return 1;
}

int rtk_position(rtk_t* rtk, const obsd_t* obs, int nu, int nr,
	const nav_t* nav, midmsg_t* midmsg, double* rs, double* dts,
	int* svh, double* y, double* e, double* azel, int ns, int* sat, int* iu, int* ir)
{
	/*稳健性判断*/
	if (rtk == NULL || obs == NULL || nav == NULL || midmsg == NULL || rs == NULL ||
 		dts == NULL || svh == NULL || y == NULL || e == NULL || azel == NULL || sat == NULL || iu == NULL || ir == NULL) {
		return -1;
	}
	/*初始化变量*/
	prcopt_t* opt = &rtk->opt;
	gtime_t time = obs[0].time;
	int i = 0, j = 0, f = 0, nx = 0, ny = 0, nv = 0, info = 0, nf = opt->nf;
	int stat = SOLQ_FLOAT, vflg[MAXDIFOBS * NFREQ * 2 + 1] = { 0 };
	int k = 0;
	int nofix = 0;
	double* pp = NULL, * H = NULL, * R = NULL;
	double qra[3] = { 0 }, qrf[3] = { 0 };
	double dt = timediff(time, obs[nu].time);
	double xp[MAXDIFOBS * NFREQ + 6] = { 0 }, xa[MAXDIFOBS * NFREQ + 6] = { 0 }, v[MAXDIFOBS * NFREQ * 2 + 1] = { 0 }, bias[MAXDIFOBS * NFREQ + 6] = { 0 };
	nx = NP(opt) + ns * nf;//优化空间只是位置速度+模糊度
	ny = ns * nf * 2 + 1;
	pp = zeros(nx, nx);
	H = zeros(nx, ny); R = mat(ny, ny);
	for (i = 0; i < MAXDIFOBS; i++) {
		for (j = 0; j < NFREQ; j++) {
			rtk->ssat[i].snr_flag[j] = 0; //TODO
			rtk->ssat[i].vsat[j] = 0;
			rtk->ssat[i].snr[j] = 0;
		}
	}
	trace(2, "start rtk position.\r\n");
	transfer(rtk, obs, sat, iu, ns, azel);
	udstate(rtk, obs, sat, iu, ir, ns, nav, azel, y, midmsg);
	trace(3, "x(0)="); tracemat(4, rtk->x, 1, NR(opt), 13, 4);//trace(3,"P(0)=\r\n");//tracemat(5,rtk->P,rtk->nx,rtk->nx,7,4);
	matcpy(xp, rtk->x, nx, 1);
	for (i = 0; i < nx; i++) for(j = 0; j < nx; j++) pp[i * nx + j] = rtk->P[i * rtk->nx + j];
	for (i = 0; i < 2; i++){
		if (!zdres(0, obs, nu, rs, dts, svh, nav, xp, opt, 0, y, e, azel, midmsg)){
			stat = SOLQ_NONE;
			break;
		}
		/* double-differenced*/
		if ((nv = ddres(0, rtk, obs, nav, dt, xp, pp, sat, y, e, azel, iu, ir, ns, v, H, R,vflg, midmsg)) < 4){
			trace(5, "no double-differenced residual\r\n");
			stat = SOLQ_NONE;
			break;
		}
		set_p(rtk, pp, nx);
		if ((info = filter_leador(xp, pp, H, v, R, nx, nv, 5, 10))){//(5,10)
			trace(4, "filter error (info=%d)\r\n", info);
			stat = SOLQ_NONE;
			break;
		}
		trace(3, "after filter rtk: %s pos[0]=%12.6lf pos[1]=%12.6lf pos[2]=%12.6lf\r\n", time_str(obs[0].time, 3), xp[0], xp[1], xp[2]);
		/**/
		if (rtk->n_lerr_ratio > 0.6 || rtk->n_slip_ratio > 0.7) {   //TODO  参数有待调整，该参数为载波相位双差残差大于0.3的观测方程比例，当该比例较高时该kalman的结果视为异常，不保存，且下一个历元所有的载波相位模糊度全部重置
			trace(2, "rtk filter fail by too many bad data:eratio=%6.3f slip_ratio=%6.3f\r\n", rtk->n_lerr_ratio, rtk->n_slip_ratio);
			//stat = SOLQ_NONE; //此处影响明显 0926 不能直接去掉
			//break;
		}
	}
	if (rtk->vrsbadnum > (rtk->nsat / 2))  stat = SOLQ_NONE;
	
	if (stat != SOLQ_NONE && zdres(0, obs, nu, rs, dts, svh, nav, xp, opt, 0, y, e, azel, midmsg)) {
		nv = ddres(1, rtk, obs, nav, dt, xp, pp, sat, y, e, azel, iu, ir, ns, v, H, R, vflg, midmsg);//为什么还要用ddres呢？？？
		/* update state and covariance matrix */
		if (valpos(rtk, sat, v, R, vflg, nv, 4.0)){
			matcpy(rtk->x, xp, nx, 1);
			matcpy(xa, xp, nx, 1);
			/*储存P阵*/
			for (i = 0; i < nx; i++) for (j = 0; j < nx; j++) rtk->P[i * rtk->nx + j] = pp[i * nx + j];
			/* update ambiguity control struct 储存双差卫星数、更新lock、outc、储存系统双差卫星数*/
			int sysnum[4] = { 0 };
			rtk->sol_rtk.float_ns = 0;
			for (i = 0; i < ns; i++) for (f = 0; f < nf; f++) {	
				if (!rtk->ssat[i].vsat[f]) continue;
				rtk->ssat[i].lock[f]++;
				rtk->ssat[i].outc[f] = 0;
				/* valid satellite count by L1 (store double difference num) */
				//if (f == 0) //改成以频点数统计最终输出的卫星数
				rtk->sol_rtk.float_ns++;
				if (!f && satsys(sat[i] - 1, NULL) == SYS_GPS) sysnum[0]++;
				if (!f && satsys(sat[i] - 1, NULL) == SYS_CMP) sysnum[1]++;
				//trace(4, "sat=%2d nf=%2d lock=%4d \n", sat[i],f,rtk->ssat[sat[i] - 1].lock[f]);
			}
			if (rtk->sol_rtk.float_ns >= 2 && sysnum[0] >= 2 && sysnum[1] >= 2) rtk->sol_rtk.float_ns = rtk->sol_rtk.float_ns - 2;
			else if (rtk->sol_rtk.float_ns >= 1) rtk->sol_rtk.float_ns = rtk->sol_rtk.float_ns - 1;
			if (rtk->sol_rtk.float_ns < 4) stat = SOLQ_NONE;  
			trace(2, "sol_rtk.ns=%d stat=%d\r\n", rtk->sol_rtk.float_ns, stat);
		}else {
			stat = SOLQ_NONE;
		}
		//        if(stat==SOLQ_NONE)
		//        {
		//            for (i = 0; i < MAXDIFOBS; i++) for (f = 0; f < nf; f++) {
		//                rtk->ssat[i].outc[f] = 1;
		//            }
		//        }
		//        else {
		//            /* update state and covariance matrix */
		//            matcpy(rtk->x, xp, nx, 1);
		//            matcpy(xa, xp, nx, 1);
		//            for (i = 0; i < nx; i++) for (j = 0; j < nx; j++)
		//                rtk->P[i * rtk->nx + j] = pp[i * nx + j];
		//        }
	}
	xy_free(pp);
	xy_free(H);
	xy_free(R);
	if (fabs(rtk->sol.age) > 15 && rtk->vel < 1.0)
		nofix = 1;
	/* 模糊度固定 */
	if (!nofix && stat != SOLQ_NONE && manage_resamb(rtk, bias, xa) > 1)
	{
		if (zdres(0, obs, nu, rs, dts, svh, nav, xa, opt, 0, y, e, azel, midmsg)){
			//post-fit reisiduals for fixed solution
			trace(3, "hold amb:nifx=%4d minfix=%4d\r\n", rtk->nfix, rtk->opt.minfix);
			if (fixsoltest(rtk, xa) > 0){
				if (++rtk->nfix >= rtk->opt.minfix && rtk->opt.modear == ARMODE_FIXHOLD) {
					holdamb_t(rtk, xa, vflg, ns);
				}
				stat = SOLQ_FIX;
			}
			else {
				rtk->nfix = 0;
			}
		}
	}
	/* save solution status */
	if (stat == SOLQ_FIX)
	{
		for (i = 0; i < 3; i++){
			rtk->sol_rtk.rr[i] = rtk->xa[i];
			rtk->sol_rtk.rr[i + 3] = rtk->sol.rr[i + 3];
			rtk->sol_rtk.qr[i] = rtk->pa[i + i * rtk->na];
			qra[i] = rtk->pa[i + i * rtk->na];
			qrf[i] = rtk->P[i + i * rtk->nx];
		}
		rtk->sol_rtk.qr[3] = rtk->pa[1];
		rtk->sol_rtk.qr[4] = rtk->pa[1 + 2 * rtk->na];
		rtk->sol_rtk.qr[5] = rtk->pa[2];
		trace(4, "after amb rtk:    %s pos[0]=%12.6lf pos[1]=%12.6lf pos[2]=%12.6lf vel[0]=%6.6lf vel[1]=%6.6lf vel[2]=%6.6lf vel=%6.6lf\n", time_str(rtk->sol_rtk.time, 3), rtk->sol_rtk.rr[0], rtk->sol_rtk.rr[1], rtk->sol_rtk.rr[2], rtk->xa[3],rtk->sol_rtk.rr[4], rtk->sol_rtk.rr[5], norm(rtk->sol_rtk.rr + 3, 3));
	}
	else if (stat == SOLQ_FLOAT) {
		for (i = 0; i < 3; i++) {
			rtk->sol_rtk.rr[i] = rtk->x[i];
			rtk->sol_rtk.rr[i + 3] = rtk->sol.rr[i + 3];
			rtk->sol_rtk.qr[i] = rtk->P[i + i * rtk->nx];
		}

		rtk->sol_rtk.qr[3] = rtk->P[1];
		rtk->sol_rtk.qr[4] = rtk->P[1 + 2 * rtk->nx];
		rtk->sol_rtk.qr[5] = rtk->P[2];
		rtk->sol_rtk.ratio = 0.0;  //20200422add
		rtk->sol_rtk.ns = rtk->sol_rtk.float_ns;
		rtk->nfix = 0;
	}
	trace(2, "rtk qra=%6.3lf qrf=%6.3lf\n", norm(qra, 3), norm(qrf, 3));
	if (stat == SOLQ_FIX) {
		if (norm(qra, 3) > norm(qrf, 3) || norm(qra, 3) > 0.1) { 
			for (i = 0; i < 3; i++) {
				rtk->sol_rtk.rr[i] = rtk->x[i];
				rtk->sol_rtk.rr[i + 3] = rtk->sol.rr[i + 3];
				rtk->sol_rtk.qr[i] = rtk->P[i + i * rtk->nx];
			}

			rtk->sol_rtk.qr[3] = rtk->P[1];
			rtk->sol_rtk.qr[4] = rtk->P[1 + 2 * rtk->nx];
			rtk->sol_rtk.qr[5] = rtk->P[2];
			rtk->nfix = 0;
			rtk->sol_rtk.ratio = 0.0;  //20200422add
			rtk->sol_rtk.ns = rtk->sol_rtk.float_ns;
			stat = SOLQ_FLOAT;
		}//add
# if 0
		if (rtk->sol.ratio < 2){
			rtk->sol.ratio = 0.0;
			stat = SOLQ_FLOAT;
		}else {
			rtk->prefixtime = obs[0].time;
		}
#endif
		rtk->prefixtime = obs[0].time;
	}
	for (i = 0; i < ns; i++) for (j = 0; j < nf; j++) 
	{
		if (obs[iu[i]].L[j] != 0.0){//流动站:时间、载波、伪距、doppler信息
			rtk->ssat[i].pt[obs[iu[i]].rcv - 1][j] = obs[iu[i]].time;
			rtk->ssat[i].ph[obs[iu[i]].rcv - 1][j] = obs[iu[i]].L[j];
			rtk->ssat[i].pd[obs[iu[i]].rcv - 1][j] = (double)obs[iu[i]].D[j];
			rtk->ssat[i].pp[obs[iu[i]].rcv - 1][j] = obs[iu[i]].P[j];
		}
		if (obs[ir[i]].L[j] != 0.0){
			k = obs[ir[i]].rcv;//基准站:...
			//if(!rtk->up_vrs_flag) continue; //避免10hz时vrs为1hz，前后存的历元实际为同一个的情况
			rtk->ssat[i].pt[k - 1][j] = obs[ir[i]].time;
			rtk->ssat[i].ph[k - 1][j] = obs[ir[i]].L[j];
			rtk->ssat[i].pp[k - 1][j] = obs[ir[i]].P[j];
			rtk->ssat[i].vrs_phy[j] = y[j + ir[i] * nf * 2];
		}
	}
	rtk->up_vrs_flag = 0;
#  if 0
	for (i = 0, j = 0; i < nu; i++){
		if (j < ns && obs[i].sat == obs[iu[j]].sat){
			j++;
			continue;
		}
		recode_nocom_sat(rtk, obs[i]);
	}
# endif
	/* output snr of rover receiver */
	for (i = 0; i < ns; i++) for (j = 0; j < nf; j++) {
		rtk->ssat[i].snr[j] = obs[iu[i]].snr[j];
	}

	rtk->sol_rtk.stat = stat;
	syncsolinfo(rtk->sol, &rtk->sol_rtk);

	if (stat != SOLQ_NONE) {
		rtk->sol_rtk.time = obs[0].time;
		if (stat == SOLQ_FIX){
			rtk->sol.pre_ratio = rtk->sol.ratio;
			for (i = 0; i < 3; i++) rtk->x[i] = rtk->xa[i];
		}
	}
	return stat != SOLQ_NONE;
}

int relpos(rtk_t* rtk, const obsd_t* obs, int nu, int nr, const nav_t* nav,midmsg_t* midmsg, gtime_t g_time)
{
	/*稳健性判断*/
	if (rtk == NULL || obs == NULL || nav == NULL) {
		trace(3, "RtkDynamicDifferenceSolution::Filter, operate null pointer.\r\n");return -1;
	}
	if (nr == 0) { savesol(rtk, -1, -1); return 0; }
	/*初始化变量*/
	double qr_rtd[3] = { 0.0 };//TODO
	double qr_spp[3] = { 0.0 };
	int i = 0;
	int nf = rtk->opt.nf;
	prcopt_t* opt = &rtk->opt;
	double* y = NULL, * y_d = NULL;
	int n = nu + nr, ns = 0, sat[MAXDIFOBS] = { 0 }, iu[MAXDIFOBS] = { 0 }, ir[MAXDIFOBS] = { 0 }, svh[MAXDIFOBS * 2] = { 0 };
	int stat_rtk = -1, stat_rtd = -1;
	double rs[MAXDIFOBS * 12] = { 0 }, dts[MAXDIFOBS * 4] = { 0 }, e[MAXDIFOBS * 6] = { 0.0 }, azel[MAXDIFOBS * 4] = { 0.0 };//azel[流动：方位角仰角，基准：方位角仰角]
	y = zeros(2, n * nf);
	y_d = mat(2, n * nf);
	rtk->sol.age = (float)timediff(obs[0].time, obs[nu].time);//差分龄期
	trace(3, "rtd,time is %s,age is %f\r\n", time_str(obs[0].time, 2), rtk->sol.age);
	if (obs[0].time.time != 0 && rtk->time_k.time != 0)
		rtk->tt = timediff(obs[0].time, rtk->time_k);
	//   trace(5,"3data break off.tt=%f\n",rtk->tt);
	if (obs[0].time.time != 0 && rtk->time_d.time != 0)
		rtk->tt_d = timediff(obs[0].time, rtk->time_d);//上次RTD开始解算与本次的龄期
	if ((ns = reldatapre(rtk, obs, nu, nr, nav, midmsg, rs, dts, svh, y, e, azel, sat, iu, ir)) <= 5) {
		trace(3, "RtkDynamicDifferenceSolution::Filter, reldatapre error: satnum less 5\r\n");
		savesol(rtk, stat_rtk, stat_rtd);
		xy_free(y); xy_free(y_d);
		return 0;
	}
	trace(3, "rtd,time is %s,age is %f maxdiffrtd:%f solstat=%d\r\n", time_str(obs[0].time, 2), rtk->sol.age, opt->maxtdiffrtd, rtk->sol.stat);
	if (opt->mode >= PMODE_DGPS) {
		if (fabs(rtk->sol.age) <= opt->maxtdiffrtd)//差分龄期判断
		{
			memcpy(y_d, y, sizeof(double) * n * nf * 2);
			stat_rtd = rtd_position(rtk, obs, nu, nr, nav, midmsg, rs, dts, svh,y_d, e, azel, ns, sat, iu, ir);
			trace(4, "after rtd:        %s pos[0]=%12.6lf pos[1]=%12.6lf pos[2]=%12.6lf vel[0]=%6.6lf vel[1]=%6.6lf vel[2]=%6.6lf vel=%6.6lf\r\n",time_str(rtk->sol_rtd.time, 3), rtk->sol_rtd.rr[0], rtk->sol_rtd.rr[1], rtk->sol_rtd.rr[2], rtk->sol_rtd.rr[3], rtk->sol_rtd.rr[4], rtk->sol_rtd.rr[5], norm(rtk->sol_rtd.rr + 3, 3));
		}else {
			memset(rtk->x_d, 0, sizeof(double) * 3);
			memset(rtk->p_d, 0, sizeof(double) * 3 * 3);
		}
	}
	for (i = 0; i < 3; i++) {
		qr_rtd[i] = rtk->sol_rtd.qr[i];
		qr_spp[i] = rtk->sol_sppk.qr[i];
	}
	xy_free(y_d);
	if (rtk->sol_rtd.stat == SOLQ_DGPS && norm(qr_rtd, 3) < 100) {
		memcpy(&rtk->sol, &rtk->sol_rtd, sizeof(sol_t) * 1);
	}
		/*打印rtd解的速度信息、上次成功RTD解的时间*/
	rtk->vel = norm(rtk->sol.rr + 3, 3);
	trace(5, "after rtd currenttime=%s vel = %f \n", time_str(rtk->sol.time, 3), rtk->vel);
	trace(5, "after rtd lasttime =%s ", time_str(rtk->sol_last.time, 3));
	if (opt->mode == PMODE_KINEMA) {
		trace(3, "rtk_age:%f  maxdiifrtk:%f solstat=%d\r\n", rtk->sol.age, opt->maxtdiffrtk, rtk->sol.stat);
		rtk->ratiomodel = opt->ratiomodel; //add  控制ratio是动态阈值还是经验值
		rtk->n_lerr_ratio = 0.0;
		trace(4, "ratio model(0动态 1经验)%d\n", rtk->ratiomodel);
	/*RTK开始*/
		if (fabs(rtk->sol.age) <= opt->maxtdiffrtk) {
			stat_rtk = rtk_position(rtk, obs, nu, nr, nav, midmsg, rs, dts, svh,
				y, e, azel, ns, sat, iu, ir);
			if (stat_rtk >= 0){
				rtk->time_k = obs[0].time;
				for (i = 0; i < 3; i++) rtk->l_x[i] = rtk->x[i];
			}
		}else {
			age_init(rtk);
		}
	}

	savesol(rtk, stat_rtk, stat_rtd);
	xy_free(y);
	if (rtk->sol.stat == SOLQ_FIX) { //固定解计数 标识为0
		rtk->fix_con_num++;
		rtk->statchange = 0;
	}else {
		if (rtk->fix_con_num > 5) {
			rtk->statchange = 1;
			trace(5, "statchange =1 and next  no used  float in rtdpos \n");//rtdpos中用的固定解或者浮点解 这里卡浮点解 所以其他解的时候stachange =1or0 不影响
		}else {
			rtk->statchange = 0;
		}
		rtk->fix_con_num = 0;
	}
	trace(5, "rtk->fix_con_num =%d statchange=%d \n", rtk->fix_con_num, rtk->statchange);
	return 1;
}

/* initialize rtk control ------------------------------------------------------
* initialize rtk control struct
* args   : rtk_t    *rtk    IO  rtk control/result struct
*          prcopt_t *opt    I   positioning options (see rtklib.h)
* return : none
*-----------------------------------------------------------------------------*/
extern void rtkinit(rtk_t* rtk, const prcopt_t* opt)
{
	ssat_t ssat0 = { 0 };
	int i = 0;

	trace(3, "rtkinit :\r\n");

	rtk->nx = NR(opt) + MAXDIFOBS * opt->nf; //NX(opt)MAXDIFOBS*opt->nf为优化后的控制的模糊度
	rtk->na = NR(opt); //NR(opt)3
	rtk->tt = 0.0;
	rtk->tt_d = 0.0;

	memset(&rtk->time, 0, sizeof(gtime_t));
	memset(&rtk->time_d, 0, sizeof(gtime_t));
	memset(&rtk->time_s, 0, sizeof(gtime_t));
	memset(&rtk->time_k, 0, sizeof(gtime_t));
	memset(&rtk->sol, 0, sizeof(sol_t));
	memset(&rtk->sol_spp, 0, sizeof(sol_t));
	memset(&rtk->sol_rtd, 0, sizeof(sol_t));
	memset(&rtk->sol_rtk, 0, sizeof(sol_t));
	memset(&rtk->sol_sppk, 0, sizeof(sol_t));
	memset(&rtk->sol_last, 0, sizeof(sol_t));
	memset(&rtk->rb, 0, sizeof(double) * 6);

	memset(&rtk->x_s, 0, sizeof(double) * 10);
	memset(&rtk->p_s, 0, sizeof(double) * 10 * 10);
	memset(&rtk->x_d, 0, sizeof(double) * 6);
	memset(&rtk->p_d, 0, sizeof(double) * 6 * 6);
	memset(&rtk->l_x_d, 0, sizeof(double) * 6);
	memset(&rtk->l_x, 0, sizeof(double) * 3);
	memset(&rtk->lgpvt, 0, sizeof(lg69t_pvt_t));

	rtk->x = zeros(rtk->nx, 1);
	rtk->P = zeros(rtk->nx, rtk->nx);
	rtk->xa = zeros(rtk->na, 1);
	rtk->pa = zeros(rtk->na, rtk->na);
	rtk->nfix =/*rtk->neb=*/0;
	for (i = 0; i < MAXDIFOBS; i++) {
		rtk->ssat[i] = ssat0;
		rtk->satid[i] = 0;
	}
	for (i = 0; i < MAXFALSTA; i++) {
		rtk->f_ssat[i] = ssat0;
	}
	rtk->opt = *opt;
	rtk->staid = 0;

	rtk->presat = 0;
	rtk->falsatnum = 0;
	rtk->up_vrs_flag = 0;
	rtk->vrsbadnum = 0;
	rtk->fix_con_num = 0;

	for (i = 0; i < 4; i++)
		rtk->vrs_sys_flag[i] = 0;
}
/* free rtk control ------------------------------------------------------------
* free memory for rtk control struct
* args   : rtk_t    *rtk    IO  rtk control/result struct
* return : none
*-----------------------------------------------------------------------------*/
extern void rtkfree(rtk_t* rtk)
{
	trace(3, "rtkfree :\r\n");

	rtk->nx = rtk->na = 0;
	xy_free(rtk->x);
	xy_free(rtk->P);
	xy_free(rtk->xa);
	xy_free(rtk->pa);
}
/* precise positioning ---------------------------------------------------------
* input observation data and navigation message, compute rover position by
* precise positioning
* args   : rtk_t *rtk       IO  rtk control/result struct
*            rtk->sol       IO  solution
*                .time      O   solution time
*                .rr[]      IO  rover position/velocity
*                               (I:fixed mode,O:single mode)
*                .dtr[0]    O   receiver clock bias (s)
*                .dtr[1]    O   receiver glonass-gps time offset (s)
*                .Qr[]      O   rover position covarinace
*                .stat      O   solution status (SOLQ_???)
*                .ns        O   number of valid satellites
*                .age       O   age of differential (s)
*                .ratio     O   ratio factor for ambiguity validation
*            rtk->rb[]      IO  base station position/velocity
*                               (I:relative mode,O:moving-base mode)
*            rtk->nx        I   number of all states
*            rtk->na        I   number of integer states
*            rtk->ns        O   number of valid satellite
*            rtk->tt        O   time difference between current and previous (s)
*            rtk->x[]       IO  float states pre-filter and post-filter
*            rtk->P[]       IO  float covariance pre-filter and post-filter
*            rtk->xa[]      O   fixed states after AR
*            rtk->Pa[]      O   fixed covariance after AR
*            rtk->ssat[s]   IO  sat(s+1) status
*                .sys       O   system (SYS_???)
*                .az   [r]  O   azimuth angle   (rad) (r=0:rover,1:base)
*                .el   [r]  O   elevation angle (rad) (r=0:rover,1:base)
*                .vs   [r]  O   data valid single     (r=0:rover,1:base)
*                .resp [f]  O   freq(f+1) pseudorange residual (m)
*                .resc [f]  O   freq(f+1) carrier-phase residual (m)
*                .vsat [f]  O   freq(f+1) data vaild (0:invalid,1:valid)
*                .fix  [f]  O   freq(f+1) ambiguity flag
*                               (0:nodata,1:float,2:fix,3:hold)
*                .slip [f]  O   freq(f+1) slip flag
*                               (bit8-7:rcv1 LLI, bit6-5:rcv2 LLI,
*                                bit2:parity unknown, bit1:slip)
*                .lock [f]  IO  freq(f+1) carrier lock count
*                .outc [f]  IO  freq(f+1) carrier outage count
*                .slipc[f]  IO  freq(f+1) cycle slip count
*                .rejc [f]  IO  freq(f+1) data reject count
*                .gf        IO  geometry-free phase (L1-L2) (m)
*                .gf2       IO  geometry-free phase (L1-L5) (m)
*            rtk->nfix      IO  number of continuous fixes of ambiguity
*            rtk->neb       IO  bytes of error message buffer
*            rtk->errbuf    IO  error message buffer
*            rtk->tstr      O   time string for debug
*            rtk->opt       I   processing options
*          obsd_t *obs      I   observation data for an epoch
*                               obs[i].rcv=1:rover,2:reference
*                               sorted by receiver and satellte
*          int    n         I   number of observation data
*          nav_t  *nav      I   navigation messages
* return : status (-1: system problem.0:sloution failed.1:valid solution)
* notes  : before calling function, base station position rtk->sol.rb[] should
*          be properly set for relative mode except for moving-baseline
*-----------------------------------------------------------------------------*/
extern int rtkpos(rtk_t* rtk, const obsd_t* obsd, int n, const nav_t* navs)
{
	/*稳健性判断*/
	if (rtk == NULL || obsd == NULL || navs == NULL)
		return -1;
	/*设置变量*/
	int i = 0, nu = 0, nr = 0;
	prcopt_t* opt = &rtk->opt;
	gtime_t time = { 0 };
	for (i = 0; i < 3; i++){
		rtk->rb[i] = opt->rb[i];
	}
	for (nu = 0; nu < n && obsd[nu].rcv == 1; nu++);/* count rover/base station observations */
	for (nr = 0; nu + nr < n && obsd[nu + nr].rcv == 2; nr++);
	/*观测值trace日志打开*/
	trace(4, "rtkpos  : time=%s %ld\r\n", time_str(obsd[0].time, 3), obsd[0].time.time);
	trace(4, "obs=\r\n"); traceobs(rtk, 4, obsd, n);
	/* 如果流动站卫星数小于5颗，不进行下面的计算 */
	if (nu < 5) {
		trace(4, "nu_ satnum less than 5\r\n");
		return -1;
	}
	/*单点定位*/
	if (pnt_pos_solution3(rtk, obsd, n, nu, nr, navs) < 0){
		trace(2, "pntpos fail \r\n");
		return -1;
	}
	/*RTD/RTK定位*/
	return relpos(rtk, obsd, nu, nr, navs, midmsg_, time);
}