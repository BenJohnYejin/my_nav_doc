#include "rtklib.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern prcopt_t popt;
/* 自定义格式输出文件的全局变量 */
static FILE* fp_custom = NULL;
static char custom_file[1024] = "";
/* 输出文件路径 */
char outfile[1024] = "";
//!************************************RINEX文件位置补充
#define MAXOBS_1S_1DAY 86400
#define MAXOBS      96                  /* max number of obs in an epoch */
#define NINCOBS     262144
static obs_t obss = { 0 };
static nav_t navs = { 0 };
static sta_t stas[MAXRCV];
static int nepoch = 0;
static int iobsu = 0;
static int iobsr = 0;
static int reverse = 0;
static int aborts = 0;
//!************************************输出
#define MAXSOLMSG   8191
#define NMEA_TID   "GP"
static const int nmea_solq[] = {
	SOLQ_NONE ,SOLQ_SINGLE, SOLQ_DGPS, SOLQ_PPP , SOLQ_FIX,
	SOLQ_FLOAT,SOLQ_DR    , SOLQ_NONE, SOLQ_NONE, SOLQ_NONE
};

extern int expath(const char* path, char* paths[], int nmax)
{
	int i, j, n = 0;
	char tmp[1024];
#ifdef WIN32
	WIN32_FIND_DATA file;
	HANDLE h;
	char dir[1024] = "", * p;

	trace(3, "expath  : path=%s nmax=%d\n", path, nmax);

	if ((p = strrchr(path, '\\'))) {
		strncpy(dir, path, p - path + 1); dir[p - path + 1] = '\0';
	}
	if ((h = FindFirstFile((LPCTSTR)path, &file)) == INVALID_HANDLE_VALUE) {
		strcpy(paths[0], path);
		return 1;
	}
	sprintf(paths[n++], "%s%s", dir, file.cFileName);
	while (FindNextFile(h, &file) && n < nmax) {
		if (file.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) continue;
		sprintf(paths[n++], "%s%s", dir, file.cFileName);
	}
	FindClose(h);
#else
	struct dirent* d;
	DIR* dp;
	const char* file = path;
	char dir[1024] = "", s1[1024], s2[1024], * p, * q, * r;

	trace(3, "expath  : path=%s nmax=%d\n", path, nmax);

	if ((p = strrchr(path, '/')) || (p = strrchr(path, '\\'))) {
		file = p + 1; strncpy(dir, path, p - path + 1); dir[p - path + 1] = '\0';
	}
	if (!(dp = opendir(*dir ? dir : "."))) return 0;
	while ((d = readdir(dp))) {
		if (*(d->d_name) == '.') continue;
		sprintf(s1, "^%s$", d->d_name);
		sprintf(s2, "^%s$", file);
		for (p = s1; *p; p++) *p = (char)tolower((int)*p);
		for (p = s2; *p; p++) *p = (char)tolower((int)*p);

		for (p = s1, q = strtok_r(s2, "*", &r); q; q = strtok_r(NULL, "*", &r)) {
			if ((p = strstr(p, q))) p += strlen(q); else break;
		}
		if (p && n < nmax) sprintf(paths[n++], "%s%s", dir, d->d_name);
	}
	closedir(dp);
#endif
	/* sort paths in alphabetical order */
	for (i = 0; i < n - 1; i++) {
		for (j = i + 1; j < n; j++) {
			if (strcmp(paths[i], paths[j]) > 0) {
				strcpy(tmp, paths[i]);
				strcpy(paths[i], paths[j]);
				strcpy(paths[j], tmp);
			}
		}
	}
	for (i = 0; i < n; i++) trace(3, "expath  : file=%s\n", paths[i]);

	return n;
}

extern void settspan(gtime_t ts, gtime_t te) {}

static void freeobsnav(obs_t* obs, nav_t* nav)
{
	trace(3, "freeobsnav:\n");

	free(obs->data); obs->data = NULL; obs->n = obs->nmax = 0;
	free(nav->eph); nav->eph = NULL; nav->n = nav->nmax = 0;
	free(nav->geph); nav->geph = NULL; nav->ng = nav->ngmax = 0;
}

static int readrnxfile(const char* file, gtime_t ts, gtime_t te, double tint,
	const char* opt, int flag, int index, char* type,
	obs_t* obs, nav_t* nav)
{
	FILE* fp;
	int  stat;
	char tmpfile[1024];

	trace(3, "readrnxfile: file=%s flag=%d index=%d\n", file, flag, index);

	if (!(fp = fopen(file, "r"))) {
		trace(2, "rinex file open error: %s\n", file);
		return 0;
	}
	stat = readrnxfp(fp, ts, te, tint, opt, flag, index, type, obs, nav);

	fclose(fp);

	return stat;
}

extern int readrnxt(const char* file, int rcv, gtime_t ts, gtime_t te,
	double tint, const char* opt, obs_t* obs, nav_t* nav)
{
	int i, n, stat = 0;
	const char* p;
	char type = ' ', * files[MAXEXFILE] = { 0 };

	trace(3, "readrnxt: file=%s rcv=%d\n", file, rcv);

	if (!*file) {
		return readrnxfp(stdin, ts, te, tint, opt, 0, 1, &type, obs, nav);
	}
	for (i = 0; i < MAXEXFILE; i++) {
		if (!(files[i] = (char*)malloc(1024))) {
			for (i--; i >= 0; i--) free(files[i]);
			return -1;
		}
	}
	//if ((n = expath(file, files, MAXEXFILE)) <= 0) {
	//	for (i = 0; i < MAXEXFILE; i++) free(files[i]);
	//	return 0;
	//}
	stat = readrnxfile(file, ts, te, tint, opt, 0, rcv, &type, obs, nav);
	//if (type == 'O') {
	//	if (!(p = strrchr(file, FILEPATHSEP))) p = file - 1;
	//	if (!*sta->name) setstr(sta->name, p + 1, 4);
	//}
	for (i = 0; i < MAXEXFILE; i++) free(files[i]);

	return stat;
}

static int readobsnav(gtime_t ts, gtime_t te, double ti, char** infile,
	int n, const char* opt, obs_t* obs, nav_t* nav)
{
	int i, j, ind = 0, nobs = 0, rcv = 1;

	trace(3, "readobsnav: ts=%s n=%d\n", time_str(ts, 0), n);

	obs->data = NULL; obs->n = obs->nmax = 0;
	nav->eph = NULL; nav->n = nav->nmax = 0;
	nav->geph = NULL; nav->ng = nav->ngmax = 0;
	nepoch = 0;

	for (i = 0; i < n; i++) {
		if (checkbrk("")) return 0;

		if (obs->n > nobs)
			rcv++;//rcv=1为流动站
		nobs = obs->n;
		trace(1, "readnavobs num=%d...\n", i);
		if (readrnxt(infile[i], rcv, ts, te, ti, opt, obs, nav) < 0) {
			checkbrk("error : insufficient memory");
			trace(1, "insufficient memory\n");
			return 0;
		}
	}
	if (obs->n <= 0) {
		checkbrk("error : no obs data");
		trace(1, "\n");
		return 0;
	}
	if (nav->n <= 0 && nav->ng <= 0) {
		checkbrk("error : no nav data");
		trace(1, "\n");
		return 0;
	}
	nepoch = sortobs(obs);

	uniqnav(nav);

	if (ts.time == 0 || te.time == 0) {
		for (i = 0; i < obs->n; i++)
			if (obs->data[i].rcv == 1)
				break;
		for (j = obs->n - 1; j >= 0; j--)
			if (obs->data[j].rcv == 1)
				break;
		if (i < j) {
			if (ts.time == 0) ts = obs->data[i].time;
			if (te.time == 0) te = obs->data[j].time;
			settspan(ts, te);
		}
	}
	return 1;
}
//!**********************************输出增加
int outnmea_gga(unsigned char* buff, const sol_t* sol);

extern int outsols(unsigned char* buff, const sol_t* sol, const double* rb)
{
	gtime_t time, ts = { 0 };
	double gpst;
	int week, timeu;

	char s[64];
	unsigned char* p = buff;
	trace(4, "outsols :\n");

	//p += outnmea_rmc(p, sol);
	p += outnmea_gga(p, sol); 
		

	return (int)(p - buff);
}
void insert_posout(char* FileName, const char* forword, char* newFileName) {
	char* dot = strrchr(FileName, '.'); // 找到最后一个点的位置
	if (dot) {
		// 复制文件名的点前部分到新文件名中
		strncpy(newFileName, FileName, dot - FileName);
		newFileName[dot - FileName] = '\0'; // 确保字符串结尾

		// 拼接forword字符串和原始文件后缀
		strcat(newFileName, forword);
		strcat(newFileName, dot);
	}
	else {
		// 如果没有找到点，直接将FileName与forword合并
		strcpy(newFileName, FileName);
		strcat(newFileName, forword);
	}
}

extern void outsol(FILE* fp, const sol_t* sol, const double* rb)
{
	unsigned char buff[MAXSOLMSG + 1];
	int n;

	trace(4, "outsol  :\n");

	if ((n = outsols(buff, sol, rb)) > 0) {
		fwrite(buff, n, 1, fp);
		fputc('\n', fp);  // 单独写入换行符
	}
}
//!**********************************平滑滤波位置补充
static sol_t* solf;
static sol_t* solb;
static double* rbf;
static double* rbb;
static int nitm = 0;
static int iitm = 0;
static int isolf = 0;
static int isolb = 0;
static gtime_t invalidtm[100] = { {0} };

static sol_t fillsoltm(const sol_t solold, const sol_t solnew, const gtime_t tm)
{
	gtime_t t1 = { 0 }, t2 = { 0 };
	sol_t sol = solold;
	int i = 0;

	if (solold.stat == 0 || solnew.stat == 0) {
		sol.stat = 0;
	}
	else {
		sol.stat = (solold.stat > solnew.stat) ? solold.stat : solnew.stat;
	}
	sol.ns = (solold.ns < solnew.ns) ? solold.ns : solnew.ns;
	sol.ratio = (solold.ratio < solnew.ratio) ? solold.ratio : solnew.ratio;

	t1 = solold.time;
	t2 = solnew.time;
	sol.time = tm;

	for (i = 0; i < 6; i++)
	{
		sol.rr[i] = solold.rr[i] + timediff(tm, t1) / timediff(t2, t1) * (solnew.rr[i] - solold.rr[i]);
	}

	return sol;
}

extern int smoother(const double* xf, const double* Qf, const double* xb,
	const double* Qb, int n, double* xs, double* Qs)
{
	double* invQf = mat(n, n), * invQb = mat(n, n), * xx = mat(n, 1);
	int i, info = -1;

	matcpy(invQf, Qf, n, n);
	matcpy(invQb, Qb, n, n);
	if (!matinv(invQf, n) && !matinv(invQb, n)) {
		for (i = 0; i < n * n; i++) Qs[i] = invQf[i] + invQb[i];
		if (!(info = matinv(Qs, n))) {
			// /Xs平滑后解的协方差矩阵$Qs$是前后向协方差矩阵的加权和的逆。这表示平滑后的不确定性由前后向的贡献共同决定。
			//Qs最终平滑解$xs$是对前向和后向加权解的加权平均值，权重由它们各自的协方差矩阵决定。
			matmul("NN", n, 1, n, 1.0, invQf, xf, 0.0, xx);
			matmul("NN", n, 1, n, 1.0, invQb, xb, 1.0, xx);
			matmul("NN", n, 1, n, 1.0, Qs, xx, 0.0, xs);//
		}
	}
	free(invQf); free(invQb); free(xx);
	return info;
}

static int valcomb(const sol_t* solf, const sol_t* solb, double* rbf,
	double* rbb, const prcopt_t* popt)
{
	double dr[3], var[3];
	int i;
	char tstr[32];

	trace(4, "valcomb :\n");

	for (i = 0; i < 3; i++) {
		dr[i] = solf->rr[i] - solb->rr[i];
		var[i] = (double)solf->qr[i] + (double)solb->qr[i];
	}
	for (i = 0; i < 3; i++) {
		if (dr[i] * dr[i] <= 16.0 * var[i]) continue;

		time2str(solf->time, tstr, 2);
		trace(2, "degrade fix to float: %s dr=%.3f %.3f %.3f std=%.3f %.3f %.3f\n",
			tstr + 11, dr[0], dr[1], dr[2], SQRT(var[0]), SQRT(var[1]), SQRT(var[2]));
		return 0;
	}
	return 1;
}

static void combres(FILE* fp, const prcopt_t* popt)
{
	gtime_t time = { 0 };
	sol_t sols = { {0} }, sol = { {0} }, oldsol = { {0} }, newsol = { {0} };
	double tt, Qf[9], Qb[9], Qs[9], rbs[3] = { 0 }, rb[3] = { 0 }, rr_f[3], rr_b[3], rr_s[3];
	int i, j, k, solstatic, num = 0, pri[] = { 7,1,2,3,4,5,1,6 };

	trace(3, "combres : isolf=%d isolb=%d\n", isolf, isolb);

	for (i = 0, j = isolb - 1; i < isolf && j >= 0; i++, j--) {
		if ((tt = timediff(solf[i].time, solb[j].time)) < -DTTOL) {
			sols = solf[i];
			for (k = 0; k < 3; k++) rbs[k] = rbf[k + i * 3];
			j++;
		}
		else if (tt > DTTOL) {
			sols = solb[j];
			for (k = 0; k < 3; k++) rbs[k] = rbb[k + j * 3];
			i--;
		}
		else if (pri[solf[i].stat] < pri[solb[j].stat]) {
			sols = solf[i];
			for (k = 0; k < 3; k++) rbs[k] = rbf[k + i * 3];
		}
		else if (pri[solf[i].stat] > pri[solb[j].stat]) {
			sols = solb[j];
			for (k = 0; k < 3; k++) rbs[k] = rbb[k + j * 3];
		}
		else {//如果时间能对上，两个的定位解状态还相等
			sols = solf[i];
			sols.time = timeadd(sols.time, -tt / 2.0);
			//!固定解的稳健性判断：根据前后向位置差和滤波方差判断是否判定为固定解
			if (popt->mode == PMODE_KINEMA && sols.stat == SOLQ_FIX) {
				if (!valcomb(solf + i, solb + j, rbf + i * 3, rbb + j * 3, popt))
					sols.stat = SOLQ_FLOAT;
			}
			//!都是固定解的话进行位置平滑
			for (k = 0; k < 3; k++) {//方差赋值
				Qf[k + k * 3] = solf[i].qr[k];
				Qb[k + k * 3] = solb[j].qr[k];
			}
			Qf[1] = Qf[3] = solf[i].qr[3];//协方差赋值
			Qf[5] = Qf[7] = solf[i].qr[4];
			Qf[2] = Qf[6] = solf[i].qr[5];
			Qb[1] = Qb[3] = solb[j].qr[3];
			Qb[5] = Qb[7] = solb[j].qr[4];
			Qb[2] = Qb[6] = solb[j].qr[5];
			if (smoother(solf[i].rr, Qf, solb[j].rr, Qb, 3, sols.rr, Qs))//根据前后向方差和位置进行加权平均
				continue;
			sols.qr[0] = (float)Qs[0];
			sols.qr[1] = (float)Qs[4];
			sols.qr[2] = (float)Qs[8];
			sols.qr[3] = (float)Qs[1];
			sols.qr[4] = (float)Qs[5];
			sols.qr[5] = (float)Qs[2];
			//todo 都是固定解的话进行速度平滑 目前速度没有平滑，因为没存qv
		//	if (popt->dynamics) {
		//		for (k = 0; k < 3; k++) {
		//			Qf[k + k * 3] = solf[i].qv[k];
		//			Qb[k + k * 3] = solb[j].qv[k];
		//		}
		//		Qf[1] = Qf[3] = solf[i].qv[3];
		//		Qf[5] = Qf[7] = solf[i].qv[4];
		//		Qf[2] = Qf[6] = solf[i].qv[5];
		//		Qb[1] = Qb[3] = solb[j].qv[3];
		//		Qb[5] = Qb[7] = solb[j].qv[4];
		//		Qb[2] = Qb[6] = solb[j].qv[5];
		//		if (smoother(solf[i].rr + 3, Qf, solb[j].rr + 3, Qb, 3, sols.rr + 3, Qs)) continue;
		//	}
		//}
		//todo 需更改输出
			outsol(fp, &sols, rbs);
		}
	}
}
//!**********************************处理函数增加
extern void settime(gtime_t time) {}
static int nextobsf(const obs_t* obs, int* i, int rcv)
{
	double tt;
	int n;

	for (; *i < obs->n; (*i)++) if (obs->data[*i].rcv == rcv) break;
	for (n = 0; *i + n < obs->n; n++) {
		tt = timediff(obs->data[*i + n].time, obs->data[*i].time);
		if (obs->data[*i + n].rcv != rcv || tt > DTTOL) break;
	}
	return n;
}

static int nextobsb(const obs_t* obs, int* i, int rcv)
{
	double tt;
	int n;

	for (; *i >= 0; (*i)--) if (obs->data[*i].rcv == rcv) break;
	for (n = 0; *i - n >= 0; n++) {
		tt = timediff(obs->data[*i - n].time, obs->data[*i].time);
		if (obs->data[*i - n].rcv != rcv || tt < -DTTOL) break;
	}
	return n;
}

static int inputobs(obsd_t* obs, int solq, const prcopt_t* popt)
{
	gtime_t time = { 0 };
	int i, nu, nr, n = 0;
	double dt, dt_next;

	//if (0 <= iobsu && iobsu < obss.n) {
	//	settime((time = obss.data[iobsu].time));
	//	if (checkbrk("processing : %s Q=%d", time_str(time, 0), solq)) {
	//		aborts = 1; showmsg("aborted"); return -1;
	//	}
	//}
	if (!reverse) {
		if ((nu = nextobsf(&obss, &iobsu, 1)) <= 0) return -1;

			dt = timediff(obss.data[iobsr].time, obss.data[iobsu].time);
			for (i = iobsr; (nr = nextobsf(&obss, &i, 2)) > 0; iobsr = i, i += nr) {
				dt_next = timediff(obss.data[i].time, obss.data[iobsu].time);
				if (fabs(dt_next) > fabs(dt)) break;
				dt = dt_next;
			}
		nr = nextobsf(&obss, &iobsr, 2);
		if (nr <= 0) {
			nr = nextobsf(&obss, &iobsr, 2);
		}
		for (i = 0; i < nu && n < MAXOBS * 2; i++) obs[n++] = obss.data[iobsu + i];

		for (i = 0; i < nr && n < MAXOBS * 2; i++) obs[n++] = obss.data[iobsr + i];

		iobsu += nu;
	}
	else {
		if ((nu = nextobsb(&obss, &iobsu, 1)) <= 0) return -1;

			dt = iobsr >= 0 ? timediff(obss.data[iobsr].time, obss.data[iobsu].time) : 0;
			for (i = iobsr; (nr = nextobsb(&obss, &i, 2)) > 0; iobsr = i, i -= nr) {
				dt_next = timediff(obss.data[i].time, obss.data[iobsu].time);
				if (fabs(dt_next) > fabs(dt)) break;
				dt = dt_next;
			}

		nr = nextobsb(&obss, &iobsr, 2);
		for (i = 0; i < nu && n < MAXOBS * 2; i++) obs[n++] = obss.data[iobsu - nu + 1 + i];
		for (i = 0; i < nr && n < MAXOBS * 2; i++) obs[n++] = obss.data[iobsr - nr + 1 + i];
		iobsu -= nu;

	}
	return n;
}

static void procpos( const prcopt_t* popt, rtk_t* rtk, FILE* fp)
{
	gtime_t time = { 0 };
	sol_t sol = { {0} }, oldsol = { {0} }, newsol = { {0} };
	obsd_t* obs_ptr = (obsd_t*)malloc(sizeof(obsd_t) * MAXOBS * 2);
	double rb[3] = { 0 };
	int i, nobs, n, solstatic, num = 0, pri[] = { 6,1,2,3,4,5,1,6 };
	int epoch = 0;
	char* obver_time;
	double time_result = 0.0;
	int year, month, day, hour, minute, second, millisecond;
	nmea_t result;
	int nu = 0, nr = 0;
	obsd_t obs[MAXRECOBS * 2];
	obs_t obsu;
	obsu.data = (obsd_t*)xy_malloc(sizeof(obsd_t) * MAXRECOBS);
	obs_t obsr;
	obsr.data = (obsd_t*)xy_malloc(sizeof(obsd_t) * MAXRECOBS * 2);
    
	rtkinit(rtk, popt);

	while ((nobs = inputobs(obs_ptr, rtk->sol.stat, popt)) >= 0) {
		obver_time = time_str(obs_ptr[0].time, 3);

		if (sscanf(obver_time, "%d/%d/%d %d:%d:%d.%3d", &year, &month, &day, &hour, &minute, &second, &millisecond) != 7) {
			printf("日期时间解析失败\n");
			return 1;
		}
		time_result = hour * 10000000.0 + minute * 100000.0 + second * 1000.0 + millisecond;
		epoch++;

		for (nu = 0; nu < nobs && obs_ptr[nu].rcv == 1; nu++);/* count rover/base station observations */
		for (nr = 0; nu + nr < nobs && obs_ptr[nu + nr].rcv == 2; nr++);
		n = 0;
		for (i = 0; i < nu; i++) {
			obsu.data[n++] = obs_ptr[i];
		}
		obsu.n = n;
		n = 0;
		for (i = nu; i < nu + nr; i++) {
			obsr.data[n++] = obs_ptr[i];
		}
		obsr.n = n;
		if (obsu.n < 5) {
			trace(4, "nu_ satnum less than 4\r\n");
			continue;
		}

		n = select_sat(obsu, obsr, &navs, *popt, obs, rtk->opt.rb);
		
		for (i = 0; i < n; i++) obsu.data[i] = obs[i];
		obsu.n = n;
		for (i = 0; i < nr; i++) {
			for (int j = 0; j < obsu.n; j++) {
				if (obsr.data[i].sat == obsu.data[j].sat) {
					obs[n++] = obsr.data[i];
				}
			}
		}

		if (rtkpos(rtk, obs, n, &navs) < 0) continue;

		rtk->sol.last_stat = rtk->sol.stat;

		rtkoutnmea(rtk, &result);
		fprintf(fp, "%s\n%s\n%s\n%s\n", result.zda, result.gga, result.rmc, result.vtg);
		
		/* 生成自定义格式的.txt结果文件 */
		{
			/* 首次使用时打开文件 */
			if (fp_custom == NULL) {
				strcpy(custom_file, outfile);  // 直接赋值指针
				fp_custom = fopen(custom_file, "w");
				if (fp_custom == NULL) {
					trace(1, "自定义格式结果文件打开失败: %s\n", custom_file);
				}
				else {
					trace(2, "自定义格式结果文件打开成功: %s\n", custom_file);
				}
			}
			
			/* 输出自定义格式的结果 */
			if (fp_custom != NULL) {
				rtkoutcustom(rtk, fp_custom);
				/* 确保文件实时写入 */
				fflush(fp_custom);
			}
		}

		checkbrk("processing : %s Q=%d\n", time_str(rtk->sol.time, 3), rtk->sol.stat);

		if (!reverse) {
			if (isolf >= nepoch) {
				free(obs_ptr);
				return;
			}
			solf[isolf] = rtk->sol;
			for (i = 0; i < 3; i++) rbf[i + isolf * 3] = rtk->rb[i];
			isolf++;
		}
		else {
			if (isolb >= nepoch) {
				free(obs_ptr);
				return;
			}
			solb[isolb] = rtk->sol;
			for (i = 0; i < 3; i++) rbb[i + isolb * 3] = rtk->rb[i];
			isolb++;
		}
	}
	free(obs_ptr);
	
	/* 关闭自定义格式输出文件 */
	if (fp_custom != NULL) {
		fclose(fp_custom);
		fp_custom = NULL;
		trace(2, "自定义格式结果文件已关闭\n");
	}
}
// 经纬高转XYZ坐标函数
void blh2xyz(double B, double L, double H, double* x, double* y, double* z) {
	double t = RE_WGS84 * (1.0 - FE_WGS84);
	double e = sqrt((RE_WGS84 * RE_WGS84 - t * t) / (RE_WGS84 * RE_WGS84));  // 第一偏心率
	double Br = B * D2R;  // 纬度转弧度
	double Lr = L * D2R;  // 经度转弧度
	double N = RE_WGS84 / sqrt(1.0 - e * e * sin(Br) * sin(Br));  // 卯酉圈曲率半径

	*x = (N + H) * cos(Br) * cos(Lr);
	*y = (N + H) * cos(Br) * sin(Lr);
	*z = (N * (1.0 - e * e) + H) * sin(Br);
}
//!**********************************主函数
#if defined(WIN32)
void postcombine(char** infile,int pntmode, int use_blh_input, double* base) {
#define NUMSYS      7                   /* number of systems */

	strcpy(outfile, infile[1]);
	char* dot = strrchr(outfile, '.');
	if (dot) strcpy(dot, ".txt");

	int start_end_time_limit = 0;//0为不限制起止时间 1为限制
	char* buff1 = "2022 03 17 06 00 38.0";
	char* buff2 = "2022 03 17 06 49 35.0";

	int flagtrace = 1, tracel = 5;

	FILE* fp[4]; FILE* fpoutforword; FILE* fpoutbackword = NULL;
	double tint = 0.0;	int n = 0;
	gtime_t startime = { 0 }, endtime = { 0 };
	int i, j;
	int n1 = 1, n2 = 1;
	int sys, tsys = TSYS_GPS;
	int nr = 0;
	int flag1 = 1, flag2 = 1, flag = 0;

	char opt[2] = "\0", type[2] = "\0";
	char tracefile[100];
	double ver;
	nmea_t result;
	prcopt_t prcopt = popt;
	obs_t obsu, obsr;
	obsd_t data1[MAXRECOBS], data2[MAXRECOBS * 2], obs[MAXRECOBS * 2];
	nav_t nav = { 0 };
	rtk_t rtk;
	char tobs1[NUMSYS][MAXOBSTYPE][4] = { {""} };
	char tobs2[NUMSYS][MAXOBSTYPE][4] = { {""} };
	obsu.data = (obsd_t*)xy_malloc(sizeof(obsd_t) * MAXRECOBS);  
	obsr.data = (obsd_t*)xy_malloc(sizeof(obsd_t) * MAXRECOBS * 2); 
	str2time(buff1, 0, 26, &startime);
	str2time(buff2, 0, 26, &endtime);
	char newfilename[256] = "";

	if (pntmode == 0) {

		n = 2; 
		prcopt.mode = PMODE_SINGLE;
	} else {
		// RTK模式
		n = 3; 
		prcopt.mode = PMODE_KINEMA;
		/*指定参考站位置*/
		if (use_blh_input==1) {
			// 如果使用经纬高坐标输入，则将经纬高转换为XYZ坐标
			double x, y, z;
			blh2xyz(base[0], base[1], base[2], &x, &y, &z);
			prcopt.rb[0] = x;
			prcopt.rb[1] = y;
			prcopt.rb[2] = z;
		}else if(use_blh_input==0){
			prcopt.rb[0] = base[0];
			prcopt.rb[1] = base[1];
			prcopt.rb[2] = base[2];
		}
	}

	/*打开文件*/
	for (i = 0; i < n; i++) fp[i] = fopen(infile[i], "r");
	fp[3] = fopen(outfile, "w");
	for (i = 0; i < 4; i++) {
		if (!fp[i]) {
			for (j = 0; j < 4; j++) if (fp[j]) fclose(fp[j]);
			perror("原始或输出文件打开错误");
			return;
		}
	}
	/* open trace file */
	if (flagtrace && tracel > 0) {
		strcpy(tracefile, outfile);
		strcat(tracefile, ".trace");
		tracelevel(tracel);
		traceopen(tracefile);
	}
	//限制时间
	if (start_end_time_limit == 0) {
		startime = (gtime_t){ 0 };
		endtime = (gtime_t){ 0 };
		tint = 0;
	}
	if (!readobsnav(startime, endtime, tint, infile, n, &prcopt, &obss, &navs)) {
		freeobsnav(&obss, &navs);

		return 0;
	}
	iobsu = iobsr = reverse = aborts = 0;
	solf = (sol_t*)malloc(sizeof(sol_t) * nepoch);
	solb = (sol_t*)malloc(sizeof(sol_t) * nepoch);
	rbf = (double*)malloc(sizeof(double) * nepoch * 3);
	rbb = (double*)malloc(sizeof(double) * nepoch * 3);
	
	if (solf && solb) {
		isolf = isolb = 0;
			char* forword = "_forword";
			insert_posout(outfile, forword, newfilename);
			if (!(fpoutforword = fopen(newfilename, "w"))) {
				trace(1, "open forword.pos error %s\n", newfilename);
				return;
			}

		procpos(&prcopt, &rtk, fpoutforword);
		//free(fpoutforword);
#if 0//todo反向滤波暂时取消，滤波异常待处理
		reverse = 1; iobsu = iobsr = obss.n - 1;
		forword = "_backword";
		insert_posout(outfile, forword, newfilename);
		if (!(fpoutbackword= fopen(newfilename, "w"))) {
			trace(1, "open backword.pos error %s\n", newfilename);
			return;
		}
		procpos(&prcopt, &rtk, fpoutbackword);
		//free(fpoutbackword);

		if (!aborts) {
			combres(fp[3], &prcopt);
		}
#endif
	}
	else showmsg("error : memory allocation");
	
	free(solf);
	free(solb);
	free(rbf);
	free(rbb);

	rtkfree(&rtk);
	/* 关闭已打开的文件 */
	for (i = 0; i < n; i++) {
		if (fp[i]) fclose(fp[i]);
	}
	/* 单独关闭输出文件 */
	if (fp[3]) fclose(fp[3]);
	traceclose();
	return;
}
#endif