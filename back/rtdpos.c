#include "rtklib.h"

#define VAR_POS_RTD     SQR(500.0) /* initial variance of receiver pos (m^2) */
#define VAR_VEL_RTD     SQR(100.0) /* initial variance of receiver vel ((m/s)^2) */
#define VAR_ACC     SQR(10.0) /* initial variance of receiver acc ((m/ss)^2) */

#define NX_D        4
#define MAXITR      10

double variance(double snr, int sys, double bl, double el, double dt) {
	double k1 = 8, a1 = 10, b1 = -0.05;
	double k2 = 5, a2 = 10, b2 = -0.05;
	double k3 =15, a3 = 10, b3 = -0.05;
	double sinel = sin(el);
	double c = bl / 100000;
	snr = MIN(snr, 42);

	if (sys == SYS_GPS || sys == SYS_GAL) {
		return k1 * pow(a1, b1 * snr) / (sinel * sinel) + c * c + dt * dt * 0.0015;
	}
	else if (sys == SYS_CMP) {
		return k2 * pow(a2, b2 * snr) / (sinel * sinel) + c * c + dt * dt * 0.0015;
	}
	else if (sys == SYS_GLO) {
		return k3 * pow(a3, b3 * snr) / (sinel * sinel) + c * c + dt * dt * 0.0015;
	}
	else {
		return 1000.0;
	}
}

double var_update_rtd(double* rrj, double* v, int nv, int dopflag, int* vpos, int* vflg, midmsg_t* midmsg) {
	if (rrj == NULL || v == NULL || vpos == NULL) {
		trace(3, "Rtdpos::var_update, operate null pointer.\n");
		return -1.0;
	}
	int i = 0, j = 0, pr_num = 0, dop_num = 0, vflag = 0, rtflag = 1;
	double vpr[MAXDIFOBS * NFREQ] = { 0.0 }, vdop[MAXDIFOBS * NFREQ] = { 0.0 };
	double prvvmid = 0.0, prvstd = 0.0, prvmin = 3.0, prvmax = 15.0, prvthre = 0.0;
	double dopvvmid = 0.0, dopvstd = 0.0, dopmin = 0.3, dopvmax = 3.0, dopvthre = 0.0;
	int flag = 0;
	int sat = 0, sys = 0, prn = 0;
	//分别把伪距残差和多普勒残差值绝对值分类放在vpr和vdop数组里
	if (dopflag) {
		for (i = 0; i < nv; i++) {
			flag = (vflg[i] >> 4) & 0xF;
			sat = (vflg[i] >> 16) & 0xFF;

			sys = satsys(sat, &prn);
			if (sys == SYS_CMP && prn < 5)  //由于单点时未将BDS geo卫星纳入解算及粗差判断，在统计rtd双差残差序列的中位值和方差时不考虑BDS geo卫星
				continue;

			if (flag == 1) {
				vpr[pr_num++] = v[i];
			}
			else {
				vdop[dop_num++] = v[i];
			}
		}
	}
	else {
		for (i = 0; i < nv; i++) {
			vpr[pr_num++] = v[i];
		}
	}
	trace(4, "var_update_rtd:pr_num=%4d dop_num=%4d\n", pr_num, dop_num);

	if (pr_num >= 5) {
		orderdopdif(pr_num, vpr);
		prvvmid = pr_num % 2 == 1 ? vpr[pr_num / 2] : vpr[pr_num / 2 - 1];

		if (prvvmid <= 5) {
			prvmax = 10.0;
		}
		else if (prvvmid <= 10) {
			prvmax = 15.0;
		}
		else if (prvvmid <= 15) {
			prvmax = 25.0;
		}
		else {
			prvmax = 25.0;
			prvvmid = 15.0;
		}
		j = 0;
		for (i = 1; i < pr_num - 1; i++) {
			if (fabs(vpr[i] - prvvmid) < 1.5 * prvmax) {//滤除离群点的伪距双差残差
				prvstd = prvstd + (vpr[i] - prvvmid) * (vpr[i] - prvvmid);
				j++;
			}
		}
		if (j > 1e-6)   //j非零
			prvstd = sqrt(prvstd / j);
		prvthre = prvstd;
		if (prvthre < prvmin)prvthre = prvmin;
	}

	if (dop_num >= 5) {
		orderdopdif(dop_num, vdop);
		dopvvmid = dop_num % 2 == 1 ? vdop[dop_num / 2] : vdop[dop_num / 2 - 1];//取中位数

		j = 0;
		for (i = 1; i < dop_num - 1; i++) {
			if (fabs(vdop[i] - dopvvmid) < 3 * dopvmax) {
				dopvstd = dopvstd + (vdop[i] - dopvvmid) * (vdop[i] - dopvvmid);
				j++;
			}
		}
		if (j > 1e-6)   //j非零
			dopvstd = sqrt(dopvstd / j);
		dopvthre = dopvstd;
		if (dopvthre < dopmin)dopvthre = dopmin;
	}

	trace(5, "Rtdpos::var_update, var_update2: prmid=%8.3f prvvstd=%8.3f  prvvthre=%8.3f dopvmid=%8.3f dopvvstd=%6.3f dopvvthre=%6.3f\n", prvvmid, prvstd, prvthre, dopvvmid, dopvstd, dopvthre);
	trace(5, "var_update2: prmid=%8.3f prvvstd=%8.3f  prvvthre=%8.3f dopvmid=%8.3f dopvvstd=%6.3f dopvvthre=%6.3f\n", prvvmid, prvstd, prvthre, dopvvmid, dopvstd, dopvthre);
	vflag = 0;
	if (dopflag) {
		for (i = 0; i < nv; i++) {
			flag = (vflg[i] >> 4) & 0xF;
			if (flag == 1) {
				if (fabs(v[i] - prvvmid) > 3 * prvthre && prvthre != 0.0) {
					rrj[i] = fabs(v[i]) * 1e6;

					midmsg[vpos[i]].dpsr1 = 1;
					trace(5, "Rtdpos::var_update, varupdate2:No.%d pr out, vv = %f, threshold = %f\n", i, v[i], prvthre);
					trace(5, "varupdate2:No.%d pr out, vv = %f, threshold = %f\n", i, v[i], prvthre);
				}
			}
			else {
				if (fabs(v[i] - dopvvmid) > 3 * dopvthre && dopvthre != 0.0) {
					rrj[i] = fabs(v[i]) * 1e6;
					midmsg[vpos[i]].ddop1 = 1;
					trace(5, "Rtdpos::var_update, varupdate2:No.%d dop out, vv = %f, threshold = %f\n", i, v[i], dopvthre);
					trace(5, "varupdate2:No.%d dop out, vv = %f, threshold = %f\n", i, v[i], dopvthre);
				}
			}
		}
	}
	else {
		for (i = 0; i < nv; i++) {
			if (fabs(v[i] - prvvmid) > 3 * prvthre && prvthre != 0.0) {
				rrj[i] = fabs(v[i]) * 1e6;

				midmsg[vpos[i]].dpsr1 = 1;

				trace(5, "Rtdpos::var_update, varupdate2:No.%d pr out, vv = %f, threshold = %f\n", i, v[i], prvthre);
				trace(5, "varupdate2:No.%d pr out, vv = %f, threshold = %f\n", i, v[i], prvthre);
			}
		}
	}

	return rtflag;
}
int  dynamic_udpos_rtd(rtk_t* rtk, double tt, double st, double lt, int flag) {
	if (rtk == NULL) {
		trace(3, "RtkDynamicRtdpos::dynamic_udpos_rtd, operate null pointer.\n");
		return -1;
	}
	double pos[3] = { 0 }, q[9] = { 0 }, qv[9] = { 0 }, var = 0.0;
	double xp[6] = { 0.0 }, ff[36] = { 0.0 }, fp[36] = { 0.0 };
	int i = 0, j = 0;
	double dd[3] = { 0 };
	double dt = 0.0;
	rtk->sol_rtd.resetflag = 0;
	trace(3, "before udpos rtd rr:%12.3f %12.3f %12.3f %8.3f %8.3f %8.3f\n", rtk->sol.rr[0], rtk->sol.rr[1], rtk->sol.rr[2], rtk->sol.rr[3], rtk->sol.rr[4], rtk->sol.rr[5]);
	trace(3, "before udpos rtd xd:%12.3f %12.3f %12.3f %8.3f %8.3f %8.3f\n", rtk->x_d[0], rtk->x_d[1], rtk->x_d[2], rtk->x_d[3], rtk->x_d[4], rtk->x_d[5]);
	for (i = 0; i < 3; i++) dd[i] = rtk->sol.rr[i] - rtk->x_d[i];
	/* check variance of estimated postion */
	for (i = 0; i < 3; i++) var += rtk->p_d[i + i * 6];
	var /= 3.0;
	trace(3, "before udpos rtd:tt=%6.3f st=%6.3f lt=%6.3f var=%6.3f dd(the norm of rr-xd)=%6.3f\n", tt, st, lt, var, norm(dd, 3));
	/* initialize position for first epoch */
	if (flag || norm(rtk->x_d, 3) <= 0.0 || var > VAR_POS_RTD || tt > 5.0 || st > 10.0 || lt > 5.0 || !rtk->sol.vtat) //时间更新初始化？为什么也需要多普勒测速成功？？？
	{
		rtk->sol_rtd.resetflag = 1;
		if (rtk->sol.stat)//如果单点定位有解
		{
			for (i = 0; i < 3; i++) {
				if (rtk->sol.rr[i + 3] == 0.0)
					rtk->sol.rr[i + 3] = 0.000001;//初始化接收机单点定位速度
				rtk->x_d[i] = rtk->sol.rr[i];//赋值单点定位
				if (rtk->opt.dynamics)
					rtk->x_d[i + 3] = rtk->sol.rr[i + 3];//赋值速度
			}
		}else{
			trace(3, "dynamic_udpos_rtd::initialize->sol.stat=0,tt=%f vtat=%d\n", tt, rtk->sol.vtat);
			if (tt > 10 || !rtk->sol.vtat) return 0;
			for (i = 0; i < 3; i++) {
				if (rtk->sol.rr[i + 3] == 0.0)
					rtk->sol.rr[i + 3] = 0.000001;
				rtk->x_d[i] = rtk->x_d[i] + rtk->x_d[i + 3] * tt;//有上次预测位置加速度推算本次位置
				if (rtk->opt.dynamics)
					rtk->x_d[i + 3] = rtk->sol.rr[i + 3];
			}
		}
		/*初始化方差*/
		for (i = 0; i < 3; i++) {
			for (j = 0; j < 3; j++) {
				if (i == j){
					rtk->p_d[i + j * 6] = VAR_POS_RTD;
					rtk->p_d[j + i * 6] = VAR_POS_RTD;
				}else{
					rtk->p_d[i + j * 6] = 0.0;
					rtk->p_d[j + i * 6] = 0.0;
				}
				if (rtk->opt.dynamics){
					if (i == j){
						rtk->p_d[i + 3 + (j + 3) * 6] = VAR_VEL_RTD;
						rtk->p_d[j + 3 + (i + 3) * 6] = VAR_VEL_RTD;
					}else {
						rtk->p_d[i + 3 + (j + 3) * 6] = 0.0;
						rtk->p_d[j + 3 + (i + 3) * 6] = 0.0;
					}
				}
			}
		}
		if (var > VAR_POS_RTD) {
			trace(3, "reset rtd position due to large variance: var=%.3f\n", var);
		}
		return 1;
	}
	if (!rtk->opt.dynamics)	return 1;
	memcpy(rtk->l_x_d, rtk->x_d, sizeof(double) * 6);
	if (rtk->sol.last_stat && rtk->sol.last_stat <= 2 && rtk->opt.mode == PMODE_KINEMA && !rtk->basechange) {  //TODO rtk->sol.last_stat在单点开始时初始化了没有进入
		trace(5, "rtk->rtd rtk: %12.3f %12.3f %12.3f\n", rtk->l_x[0], rtk->l_x[1], rtk->l_x[2]);
		trace(5, "rtk->rtd rtd: %12.3f %12.3f %12.3f\n", rtk->x_d[0], rtk->x_d[1], rtk->x_d[2]);
		for (i = 0; i < 3; i++) rtk->x_d[i] = rtk->l_x[i];
	}
	dt = timediff(rtk->time, rtk->sol_last.time);
	if (rtk->opt.mode > PMODE_DGPS && dt < 3 && (rtk->sol_last.stat == SOLQ_FIX || rtk->sol_last.stat == SOLQ_FLOAT)) {
		for (i = 0; i < 6; i++) {
			rtk->x_d[i] = rtk->sol_last.rr[i];
		}
		tt = timediff(rtk->time, rtk->sol_last.time);
	}
	if (rtk->sol.vtat > 0) {  //TODO
		for (i = 3; i < 6; i++) rtk->x_d[i] = rtk->sol.rr[i];
	}
	/* state transition of position/velocity/acceleration */
	ff[0] = ff[7] = ff[14] = ff[21] = ff[28] = ff[35] = 1.0;
	for (i = 0; i < 3; i++) ff[i + (i + 3) * 6] = tt;//todoF阵的设置
	/** x=F*x, P=F*P*F+Q */
	matmul("NN", 6, 1, 6, 1.0, ff, rtk->x_d, 0.0, xp);
	matmul("NN", 6, 6, 6, 1.0, ff, rtk->p_d, 0.0, fp);
	matmul("NT", 6, 6, 6, 1.0, fp, ff, 0.0, rtk->p_d);
	matcpy(rtk->x_d, xp, 6, 1);
	/* process noise added to only acceleration */
	q[0] = SQR(0.1 * tt);//水平0.1tt
	q[4] = SQR(0.1 * tt);
	q[8] = SQR(0.01 * tt);//垂直0.01tt
	ecef2pos(rtk->x_d, pos);/*Q阵坐标转换*/
	covecef(pos, q, qv);
	for (i = 0; i < 3; i++) {/*Q阵的添加*/
		for (j = 0; j < 3; j++) {
			rtk->p_d[i + 3 + (j + 3) * 6] += qv[i + j * 3];
		}
	}
	return 1;
}

int dynamic_selrefsat_rtd(rtk_t* rtk, const obsd_t* obs, const int* sat, double* y,
	double* azel, const int* iu, const int* ir, int ns, int m, midmsg_t* midmsg, int f)
{
	if (rtk == NULL || obs == NULL || sat == NULL || y == NULL || azel == NULL || iu == NULL || ir == NULL || midmsg == NULL) {
		trace(3, "RtkDynamicRtdpos:: dynamic_selrefsat_rtd , operate null pointer.\n");
		return -1;
	}
	int i = 0, j = 0;
	int index = -1;
	int nf = rtk->opt.nf;//1;
	int sysi = 0;
	trace(5, "dynamic_selrefsat_rtd:: RTD Select reference satellite\n");
	
	if (index > 0) {
		return index;
	}else { /*如果未指定参考星则选择默认的参考星方案*/
		for (i = -1, j = 0; j < ns; j++) {
			sysi = limitcmpgeo(obs[iu[j]].sat);  
			if (sysi <= SYS_NONE || !test_sys(sysi, m))
				continue;
			if (y[f + nf + iu[j] * nf * 2] == 0.0 || y[f + nf + ir[j] * nf * 2] == 0.0) {
				trace(4, "y=0 continue.the sat number is %2d\n", sat[j]);
				continue;
			}
			if (midmsg[iu[j]].ddop1 == 1 || midmsg[iu[j]].dpsr1 == 1) {
				trace(4, "DDOP1=%d DPsr=%d.the sat number is %2d\n", midmsg[iu[j]].ddop1, midmsg[iu[j]].dpsr1, sat[j]);
				continue;}
			/*虽然RTD选参考星不需要考虑周跳，但尽量与rtk选参考星保持一致*/
			if (obs[iu[j]].lli[f] & 3) 
				continue;
			if (azel[1 + iu[j] * 2] * R2D < MINREFSTARELEV) continue;
		    trace(5,"eligible ref sat=%d snr = %f  avg=%f \n",sat[j],obs[iu[j]].snr[0] * 0.25,rtk->opt.snravg[m]);
			if ((i < 0 || azel[1 + iu[j] * 2] >= azel[1 + iu[i] * 2]) && obs[iu[j]].snr[f] * 0.25 >= rtk->opt.snravg[m][f] * 0.99)
				i = j;
		}
		if (i < 0){
			for (i = -1, j = 0; j < ns; j++) {
				sysi = satsys(obs[iu[j]].sat, NULL);
				if (!test_sys(sysi, m))
					continue;
				/*检测当前星是否为北斗GEO卫星*/
				sysi = limitcmpgeo(obs[iu[j]].sat);  //检测当前星是否为北斗GEO卫星
				if (sysi <= SYS_NONE || !test_sys(sysi, m))
					continue;
				if (y[f + nf + iu[j] * nf * 2] == 0.0 || y[f + nf + ir[j] * nf * 2] == 0.0) {
					trace(4, "y=0 continue.the sat number is %2d\n", sat[j]);
					continue;
				}
				if (midmsg[iu[j]].ddop1 == 1 || midmsg[iu[j]].dpsr1 == 1) {
					trace(4, "DDOP1=%d DPsr=%d.the sat number is %2d\n", midmsg[iu[j]].ddop1, midmsg[iu[j]].dpsr1, sat[j]);
					continue;
				}
				if (azel[1 + iu[j] * 2] * R2D < MINREFSTARELEV) continue;
				trace(5,"eligible ref sat=%d snr = %f  avg=%f\n",sat[j],obs[iu[j]].snr[0] * 0.25,rtk->opt.snravg[m]);
				if ((i < 0 || azel[1 + iu[j] * 2] >= azel[1 + iu[i] * 2]) && obs[iu[j]].snr[f] * 0.25 >= rtk->opt.snravg[m][f] * 0.99)
					i = j;
			}
		}
		/*打印最终的选参考星信息*/
		if (i > -1) {
			trace(3, "Rtdpos::selrefsatID,  success select RTD ref sat! sys=%d, sat=%d ,rank=%d\n", m, sat[i], i);
		}
		return i;
	}
}
int dynamic_ddres_rtd(rtk_t* rtk, const obsd_t* obs, const nav_t* nav, double* dts,
	double* rs, double dt, const double* x, const double* P, const int* sat, double* y,
	double* e, double* azel, const int* iu, const int* ir, int ns, double* v, double* H, double* R, int* vflg, midmsg_t* midmsg, int itr)
{
	if (rtk == NULL || obs == NULL || nav == NULL || dts == NULL || rs == NULL || x == NULL || sat == NULL || y == NULL ||
		e == NULL || azel == NULL || iu == NULL || ir == NULL || v == NULL || H == NULL || R == NULL || vflg == NULL || midmsg == NULL) {
		trace(3, "RtkDynamicRtdpos::dynamic_ddres_rtd, operate null pointer.\r\n");
		return -1;
	}
	double bl = 0, dr[3] = { 0 };
	double lami = 0, lamj = 0, dpvi = 0, dpvj = 0, vs[3] = { 0 }, * hi = NULL;
	int i = 0, j = 0, k = 0, m = 0, nv = 0, nb[NFREQ * 4 * NFREQ * DPS_IN_RTD + 2] = { 0 }, b = 0, sysi = 0, sysj = 0, nf = rtk->opt.nf, dop_flag = 0, nsat = 0;
	double dop[4] = { 0 };
	double azel_[MAXDIFOBS * 4] = { 0.0 };
	double rri[MAXDIFOBS * NFREQ * DPS_IN_RTD] = { 0.0 }, rrj[MAXDIFOBS * NFREQ * DPS_IN_RTD] = { 0.0 };
	int vpos[MAXDIFOBS * NFREQ * DPS_IN_RTD] = { 0.0 };
	rtk->n_errsat = 0;
	trace(3, "ddres   : dt(diffage)=%.1f nx=%d ns=%d\r\n", dt, rtk->nx, ns);
	bl = baseline(x, rtk->rb, dr);

	for (m = 0; m < 4; m++) {
		for (int f = 0; f < nf; f++) {
			i = dynamic_selrefsat_rtd(rtk, obs, sat, y, azel, iu, ir, ns, m, midmsg, f);
			if (i < 0) continue;
			lami = nav->lam[sat[i] - 1][f];
			for (k = 0; k < 3; k++) vs[k] = rs[k + 3 + iu[i] * 6] - rtk->x_d[k + 3];
			dpvi = -lami * obs[iu[i]].D[f] - (dot(vs, e + iu[i] * 3, 3) - CLIGHT * dts[1 + iu[i] * 2]);//多普勒残差，理论为0
			/*伪距载波:双差、H阵设置、方差阵设置 */
			for (j = 0; j < ns; j++) {
				if (i == j) continue;
				/*判断是否为同系统*/
				sysi = satsys(obs[iu[i]].sat, NULL);sysj = satsys(obs[iu[j]].sat, NULL);
				if (!test_sys(sysj, m)) continue;
				/*有效非差残差判断*/
				if (y[f + nf + iu[j] * nf * 2] == 0.0 || y[f + nf + ir[j] * nf * 2] == 0.0) {
					trace(3, "ddres continue: sat%d !y[1 + iu[j] * nf * 2]=%10.3lf y[1 + ir[j] * nf * 2]=%10.3lf\r\n", sat[j], y[1 + iu[j] * nf * 2], y[1 + ir[j] * nf * 2]);
					continue;
				}
				/*有效波长判断*/
				lami = nav->lam[sat[i] - 1][f];lamj = nav->lam[sat[j] - 1][f];
				if (lami <= 0.0 || lamj <= 0.0) continue;
				/*利用hi初始化H阵为0*/
				if (H) {
					hi = H + nv * 6;
					for (k = 0; k < 6; k++) hi[k] = 0.0;
				}
				if (midmsg[iu[j]].dpsr2[f] == 1) continue;//TODO
				/*做双差double - differenced residual y的 + nf代表伪距 不加nf代表载波*/
				v[nv] = (y[f + nf + iu[i] * nf * 2] - y[f + nf + ir[i] * nf * 2]) - (y[f + nf + iu[j] * nf * 2] - y[f + nf + ir[j] * nf * 2]);
				/*设置H阵partial derivatives by rover position */
				if (H) {
					for (k = 0; k < 3; k++) hi[k] = -e[k + iu[i] * 3] + e[k + iu[j] * 3];//i是基准星数据，j是差分星观测数据
				}
				rri[nv] = 15 * variance(obs[iu[i]].snr[f] * 0.25, sysi, bl, azel[1 + iu[i] * 2], dt);//方差阵的设置
				rrj[nv] = 15 * variance(obs[iu[j]].snr[f] * 0.25, sysj, bl, azel[1 + iu[j] * 2], dt);
				if (fabs(v[nv]) > 200.0) {
					trace(3, "psr outage v=%f\r\n", v[nv]);
					midmsg[iu[j]].dpsr2[f] = 1;
					continue;
				}
				/*储存差分星的可用状态位、高度角等信息，打印*/
				rtk->ssat[i].vsat[f] = rtk->ssat[j].vsat[f] = 1;
				vpos[nv] = iu[j];
				azel_[2 * nsat] = azel[iu[j] * 2];
				azel_[2 * nsat + 1] = azel[1 + iu[j] * 2];
				nsat++;
				trace(4, "sat=%3d-%3d %s%d v=%8.3f R=%10.6f %10.6f\r\n", sat[i], sat[j], "P", f + 1, v[nv], rri[nv], rrj[nv]);
				vflg[nv++] = (iu[i] << 16) | (iu[j] << 8) | (1 << 4) | (f);//将卫星i至高位（8个2进制）、卫星j至中位（8个二进制）、vflg至低（4个二进制）、f最低（4），方便储存
				nb[b]++; // nb[频点1GPS有k个卫星的双差为距载波，频点1GPS有k个卫星的双差多普勒，频点2 * ***，频点n * ***，频点1GLO有k个双差为距载波，***]
			}
			b++;

			if (obs[iu[i]].D[f] != 0.0f && norm(rs + 3 + iu[i] * 6, 3) > 0.0) // &&rtk->opt.dynamics
			{
				dop_flag = 1;
				for (j = 0; j < ns; j++) {
					if (i == j) continue;
					sysi = satsys(obs[iu[i]].sat, NULL);//基准卫星系统
					sysj = satsys(obs[iu[j]].sat, NULL);//差分卫星系统
					if (!test_sys(sysj, m)) continue;//系统筛选
					if (obs[iu[j]].D[f] == 0.0f || norm(rs + 3 + iu[j] * 6, 3) <= 0.0) continue;//接收机多普勒值与卫星位置不能为0
					if (y[f + nf + iu[j] * nf * 2] == 0.0 || y[f + nf + ir[j] * nf * 2] == 0.0) continue;//流动站基准站非差残差不能为0
					if (midmsg[iu[j]].dpsr2[f] == 1 || midmsg[iu[j]].ddop2[f] == 1)continue;//多普勒异常判定
					lami = nav->lam[sat[i] - 1][f];
					lamj = nav->lam[sat[j] - 1][f];
					if (lami <= 0.0 || lamj <= 0.0)continue;
					/*H阵初始化*/ 
					if (H) {
						hi = H + nv * 6;
						for (k = 0; k < 6; k++) hi[k] = 0.0;
					}
					/* partial derivatives by rover position 设置H阵*/
					if (H) {  
						for (k = 0; k < 3; k++) { hi[k + 3] = -e[k + iu[i] * 3] + e[k + iu[j] * 3];}
					}
					/*求多普勒残差*/
					for (k = 0; k < 3; k++) {vs[k] = rs[k + 3 + iu[j] * 6] - x[k + 3];}//rtk->x[k + 3]卫星速度-接收机速度（接收机相对差分卫星的运动速度）
					
					dpvj = -lamj * obs[iu[j]].D[f] - (dot(vs, e + iu[j] * 3, 3) - CLIGHT * dts[1 + iu[j] * 2]);//多普勒频移残差理论为0
					v[nv] = dpvi - dpvj;//多普勒双差残差
					rri[nv] = 3.0 * variance(obs[iu[i]].snr[f] * 0.25, sysi, 0, azel[1 + iu[i] * 2], dt);
					rrj[nv] = 3.0 * variance(obs[iu[j]].snr[f] * 0.25, sysj, 0, azel[1 + iu[j] * 2], dt);
					/*多普勒残差异常判断*/
					if (fabs(v[nv]) > 20.0){
						midmsg[iu[j]].ddop2[f] = 1;
						continue;
					}
					vpos[nv] = iu[j];
					trace(4, "sat=%3d-%3d %s%d v=%8.3f R=%10.6f %10.6f\r\n", sat[i], sat[j], "D", f + 1, v[nv], rri[nv], rrj[nv]);
					vflg[nv++] = (iu[i] << 16) | (iu[j] << 8) | (0 << 4) | (f);
					nb[b]++;
				}
				b++;
			}
		}
	}	
# if  0
	/* epoch diff pos constraint */
	velcons_rtd(rtk, x, P, v, H, rri, rrj, &nv, &b, vflg, nb, itr);
	vsat = izeros(nv, 1);
#endif
#  if 1
	/*根据dop判断量测是否成功*/
	if (rtk->sol_rtd.resetflag)
	{
		dops_dif(nsat, azel_, rtk->opt.elmin, dop, NULL);
		trace(4, "RTD:the dops of RTD:nv=%d gdop=%.1f pdop=%.1f\r\n", nsat, dop[0], dop[1]);

		if (dop[0] <= 0.0 || dop[1] > 5 || dop[0] > rtk->opt.maxgdop){
			trace(4, "RTD:gdop error nv=%d gdop=%.1f pdop=%.1f\n", nv, dop[0], dop[1]);
			nv = 0;
		}
	}
#endif
	/*抗差*/
	//var_update_rtd(rrj, v,nv,dop_flag,vpos,vflg,midmsg);
# if 0
	var_update_grubs(rrj, nb, b, v, nv, dop_flag, vsat);
	trace(5, "H=\n");  tracemat(5, H, rtk->nx, nv, 7, 4);
	rtk->nsat = nsat;
	if (nsat < 5) nv = 0;
#endif
	if (R)
		ddcov(nb, b, rri, rrj, nv, R);

	return nv;
}

extern int rtd_position(rtk_t* rtk, const obsd_t* obs, int nu, int nr,
	const nav_t* nav, midmsg_t* midmsg, double* rs, double* dts,
	int* svh, double* y, double* e, double* azel, int ns, int* sat, int* iu, int* ir)
{
	/*稳健性判断*/
	if (rtk == NULL || obs == NULL || nav == NULL || midmsg == NULL || rs == NULL || dts == NULL ||
		svh == NULL || y == NULL || e == NULL || azel == NULL || sat == NULL || iu == NULL || ir == NULL) {
		return -1;
	}
	/*变量初始化*/
	prcopt_t* opt = &rtk->opt;
	double* R = NULL;
	double xp[6] = { 0.0 }, pp[36] = { 0.0 };
	double v[MAXDIFOBS * NFREQ * DPS_IN_RTD] = { 0.0 }, H[MAXDIFOBS * 6 * NFREQ * DPS_IN_RTD] = { 0.0 };
	int i = 0, f = 0, nv = 0, info = 0, nf = opt->nf;
	int stat = SOLQ_DGPS, vflg[MAXDIFOBS * NFREQ * DPS_IN_RTD + 1] = { 0 };
	int sysnum[4] = { 0 };
	double dif[3] = { 0 };
	double vv = 0.0;
	double kvv = 0.0;
	gtime_t time = obs[0].time;
	double dt = 0.0, tt = 0.0, st = 0.0, lt = 0.0;
	int updateflag = 1;
	trace(5, "RtkDynamicRtdpos::rtd_position\r\n");
	/*计算解的龄期等*/
	dt = timediff(time, obs[nu].time);/*差分龄期*/
	if (rtk->time_d.time)
		tt = timediff(obs[0].time, rtk->time_d);
	if (rtk->sol_rtd.time.time)
		st = timediff(obs[0].time, rtk->sol_rtd.time);
	if (rtk->sol_last.time.time != 0)
		lt = fabs(timediff(obs[0].time, rtk->sol_last.time));
	/*RTD时间更新*/
	updateflag = dynamic_udpos_rtd(rtk, tt, st, lt, 0);
	if (updateflag == 0) {
		trace(3, "rtd error\n");
		rtk->sol_rtd.stat = SOLQ_NONE;
		stat = SOLQ_NONE;
		return stat != SOLQ_NONE;
	}
	/*打印时间更细的待估参数*/
	trace(3, "x(0)="); tracemat(4, rtk->x_d, 1, 6, 13, 4);
	/*量测初始化*/
	rtk->time_d = obs[0].time;
	for (i = 0; i < MAXDIFOBS; i++){
		for (f = 0; f < NFREQ; f++){
			rtk->ssat[i].snr_flag[f] = 0;  //TODO
			rtk->ssat[i].vsat[f] = 0;
			rtk->ssat[i].snr[f] = 0;
		}
	}
	R = mat(ns * NFREQ * DPS_IN_RTD, ns * NFREQ * DPS_IN_RTD);
	matcpy(xp, rtk->x_d, 6, 1);
	matcpy(pp, rtk->p_d, 6, 6);
	for (i = 0; i < 2; i++) {//迭代两次，RTKLIB迭代一次
		if (!zdres(0, obs, nu, rs, dts, svh, nav, xp, opt, 0, y, e, azel, midmsg)) {
			trace(3, "rover initial position error\r\n");
			stat = SOLQ_NONE;
			break;
		}
		if ((nv = dynamic_ddres_rtd(rtk, obs, nav, dts, rs, dt, xp, pp, sat, y, e, azel, iu, ir, ns, v, H, R, vflg, midmsg, i)) < 4) {
			trace(3, "rtderror: double-differenced residual too less.nv=%d\r\n", nv);
			stat = SOLQ_NONE;
			break;
		}
		matcpy(pp, rtk->p_d, 6, 6);
		rtd_igg3test(rtk, obs, xp, pp, H, v, R, sat, 6, nv, vflg, midmsg);
		/*滤波*/
		if ((info = filter_leador(xp, pp, H, v, R, 6, nv, 2, 5))) {
			trace(3, "filter error (info=%d)\r\n", info);
			stat = SOLQ_NONE;
			break;
		}
		trace(3, "x(%d)=", i + 1); tracemat(4, xp, 1, 6, 13, 4);
	}
	/*检验*/
	if (stat != SOLQ_NONE)
	{
		if (!zdres(0, obs, nu, rs, dts, svh, nav, xp, opt, 0, y, e, azel, midmsg)) {
			trace(3, "rover initial position error\r\n");
			stat = SOLQ_NONE;
		}
		if ((nv = dynamic_ddres_rtd(rtk, obs, nav, dts, rs, dt, xp, pp, sat, y, e, azel, iu, ir, ns, v, H, R, vflg, midmsg, i)) < 4) {
			trace(3, "rtderror: double-differenced residual too less.nv=%d\r\n", nv);
			stat = SOLQ_NONE;
		}else {
			vv = sqrt(dot(v, v, nv) / nv);
			for (i = 0; i < nv; i++) v[i] /= sqrt(R[i + i * nv]);
			kvv = sqrt(dot(v, v, nv) / nv);
			trace(4, "rtd:kvv=%16.3f vv=%16.3f\n", kvv, vv);
		}
		if (kvv > 3 && vv > 10)
			stat = SOLQ_NONE;
	}
	/*保存sol相关参数update ambiguity control struct*/
	if (stat != SOLQ_NONE && zdres(0, obs, nu, rs, dts, svh, nav, xp, opt, 0, y, e, azel, midmsg)) {
		rtk->sol_rtd.ns = 0;
		/*统计每个系统的可用卫星*/
		for (i = 0; i < ns; i++) {
			for (f = 0; f < nf; f++) {
				if (!rtk->ssat[i].vsat[f]) continue;
				/* valid satellite count by L1 (store double difference num) */
				rtk->sol_rtd.ns++;
				if (!f && satsys(sat[i] - 1, NULL) == SYS_GPS) sysnum[0]++;
				if (!f && satsys(sat[i] - 1, NULL) == SYS_CMP) sysnum[1]++;
			}
		}
		/* 可用卫星统计判断*/
		if (rtk->sol_rtd.ns >= 2 && sysnum[0] >= 2 && sysnum[1] >= 2) {
			rtk->sol_rtd.ns = rtk->sol_rtd.ns - 2;
		}else if (rtk->sol_rtd.ns >= 1){
			rtk->sol_rtd.ns = rtk->sol_rtd.ns - 1;
		}
		if (rtk->sol_rtd.ns < 5) stat = SOLQ_NONE;
		trace(4, "rtk->sol_rtd.ns=%d  n=%d stat=%d\r\n", rtk->sol_rtd.ns, nv, stat);
		/* 保存解的状态 */
		if (stat != SOLQ_NONE) {
			matcpy(rtk->x_d, xp, 6, 1);
			matcpy(rtk->p_d, pp, 6, 6);
			for (i = 0; i < 3; i++) {
				rtk->sol_rtd.rr[i] = rtk->x_d[i];
				rtk->sol_rtd.qr[i] = rtk->p_d[i + i * 6];
			}
			for (i = 3; i < 6; i++) rtk->sol_rtd.rr[i] = rtk->x_d[i];
			rtk->sol_rtd.qr[3] = rtk->p_d[1];
			rtk->sol_rtd.qr[4] = rtk->p_d[1 + 2 * 3];
			rtk->sol_rtd.qr[5] = rtk->p_d[2];
			rtk->time_ld = time;
			rtk->sol_rtd.time = obs[0].time;
		}
	}
	rtk->sol_rtd.stat = stat;
	/*单点关键信息同步至rtk->sol_rtd*/
	syncsolinfo(rtk->sol, &rtk->sol_rtd);

	trace(2, "rtd stat:%2d\r\n", stat);

	xy_free(R);
	return stat != SOLQ_NONE;
}