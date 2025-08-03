#include "rtklib.h"

/* constants/macros ----------------------------------------------------------*/

#define NUMSYS      7                   /* number of systems */
#define MAXPOSHEAD  1024                /* max head line position */
#define MINFREQ_GLO -7                  /* min frequency number glonass */
#define MAXFREQ_GLO 13                  /* max frequency number glonass */
#define NINCOBS     262144              /* inclimental number of obs data */

static const int navsys[] = {             /* satellite systems */
	SYS_GPS,SYS_GLO,SYS_GAL,SYS_QZS,SYS_CMP,0
};
static const char syscodes[] = "GREJSCI"; /* satellite system codes */

static const char obscodes[] = "CLDS";    /* obs type codes */

static const char frqcodes[] = "1256789"; /* frequency codes */

static const double ura_eph[] = {         /* ura values (ref [3] 20.3.3.3.1.1) */
	2.4,3.4,4.85,6.85,9.65,13.65,24.0,48.0,96.0,192.0,384.0,768.0,1536.0,
	3072.0,6144.0,0.0
};
static const double ura_nominal[] = {     /* ura nominal values */
	2.0,2.8,4.0,5.7,8.0,11.3,16.0,32.0,64.0,128.0,256.0,512.0,1024.0,
	2048.0,4096.0,8192.0
};
/* type definition -----------------------------------------------------------*/
typedef struct {                        /* signal index type */
	int n;                              /* number of index */
	int frq[MAXOBSTYPE];                /* signal frequency (1:L1,2:L2,...) */
	int pos[MAXOBSTYPE];                /* signal index in obs data (-1:no) */
	unsigned char pri[MAXOBSTYPE];     /* signal priority (15-0) */
	unsigned char type[MAXOBSTYPE];     /* type (0:C,1:L,2:D,3:S) */
	unsigned char code[MAXOBSTYPE];     /* obs code (CODE_L??) */
	double shift[MAXOBSTYPE];           /* phase shift (cycle) */
} sigind_t;

/* set string without tail space ---------------------------------------------*/
static void setstr(char* dst, const char* src, int n)
{
	char* p = dst;
	const char* q = src;
	while (*q && q < src + n) *p++ = *q++;
	*p-- = '\0';
	while (p >= dst && *p == ' ') *p-- = '\0';
}
/* adjust time considering week handover -------------------------------------*/
static gtime_t adjweek(gtime_t t, gtime_t t0)
{
	double tt = timediff(t, t0);
	if (tt < -302400.0) return timeadd(t, 604800.0);
	if (tt > 302400.0) return timeadd(t, -604800.0);
	return t;
}
/* adjust time considering week handover -------------------------------------*/
static gtime_t adjday(gtime_t t, gtime_t t0)
{
	double tt = timediff(t, t0);
	if (tt < -43200.0) return timeadd(t, 86400.0);
	if (tt > 43200.0) return timeadd(t, -86400.0);
	return t;
}
/* time string for ver.3 (yyyymmdd hhmmss UTC) -------------------------------*/
static void timestr_rnx(char* str)
{
	gtime_t time;
	double ep[6];
	time = timeget();
	time.sec = 0.0;
	time2epoch(time, ep);
	sprintf(str, "%04.0f%02.0f%02.0f %02.0f%02.0f%02.0f UTC", ep[0], ep[1], ep[2],
		ep[3], ep[4], ep[5]);
}
/* satellite to satellite code -----------------------------------------------*/
static int sat2code(int sat, char* code)
{
	int prn;
	switch (satsys(sat, &prn)) {
	case SYS_GPS: sprintf(code, "G%2d", prn - MINPRNGPS + 1); break;
	case SYS_GLO: sprintf(code, "R%2d", prn - MINPRNGLO + 1); break;
	case SYS_GAL: sprintf(code, "E%2d", prn - MINPRNGAL + 1); break;
	case SYS_QZS: sprintf(code, "J%2d", prn - MINPRNQZS + 1); break;
	case SYS_CMP: sprintf(code, "C%2d", prn - MINPRNCMP + 1); break;
	default: return 0;
	}
	return 1;
}
/* ura index to ura nominal value (m) ----------------------------------------*/
static double uravalue(int sva)
{
	return 0 <= sva && sva < 15 ? ura_nominal[sva] : 8192.0;
}
/* ura value (m) to ura index ------------------------------------------------*/
static int uraindex(double value)
{
	int i;
	for (i = 0; i < 15; i++) if (ura_eph[i] >= value) break;
	return i;
}

/* convert rinex obs type ver.2 -> ver.3 -------------------------------------*/
static void convcode(double ver, int sys, const char* str, char* type)
{
	strcpy(type, "   ");

	if (!strcmp(str, "P1")) { /* ver.2.11 GPS L1PY,GLO L2P */
		if (sys == SYS_GPS) sprintf(type, "%c1W", 'C');
		else if (sys == SYS_GLO) sprintf(type, "%c1P", 'C');
	}
	else if (!strcmp(str, "P2")) { /* ver.2.11 GPS L2PY,GLO L2P */
		if (sys == SYS_GPS) sprintf(type, "%c2W", 'C');
		else if (sys == SYS_GLO) sprintf(type, "%c2P", 'C');
	}
	else if (!strcmp(str, "C1")) { /* ver.2.11 GPS L1C,GLO L1C/A */
		if (ver >= 2.12); /* reject C1 for 2.12 */
		else if (sys == SYS_GPS) sprintf(type, "%c1C", 'C');
		else if (sys == SYS_GLO) sprintf(type, "%c1C", 'C');
		else if (sys == SYS_GAL) sprintf(type, "%c1X", 'C'); /* ver.2.12 */
		else if (sys == SYS_QZS) sprintf(type, "%c1C", 'C');
	}
	else if (!strcmp(str, "C2")) {
		if (sys == SYS_GPS) {
			if (ver >= 2.12) sprintf(type, "%c2W", 'C'); /* L2P(Y) */
			else           sprintf(type, "%c2X", 'C'); /* L2C */
		}
		else if (sys == SYS_GLO) sprintf(type, "%c2C", 'C');
		else if (sys == SYS_QZS) sprintf(type, "%c2X", 'C');
		else if (sys == SYS_CMP) sprintf(type, "%c1X", 'C'); /* ver.2.12 B1 */
	}
	else if (ver >= 2.12 && str[1] == 'A') { /* ver.2.12 L1C/A */
		if (sys == SYS_GPS) sprintf(type, "%c1C", str[0]);
		else if (sys == SYS_GLO) sprintf(type, "%c1C", str[0]);
		else if (sys == SYS_QZS) sprintf(type, "%c1C", str[0]);
	}
	else if (ver >= 2.12 && str[1] == 'B') { /* ver.2.12 GPS L1C */
		if (sys == SYS_GPS) sprintf(type, "%c1X", str[0]);
		else if (sys == SYS_QZS) sprintf(type, "%c1X", str[0]);
	}
	else if (ver >= 2.12 && str[1] == 'C') { /* ver.2.12 GPS L2C */
		if (sys == SYS_GPS) sprintf(type, "%c2X", str[0]);
		else if (sys == SYS_QZS) sprintf(type, "%c2X", str[0]);
	}
	else if (ver >= 2.12 && str[1] == 'D') { /* ver.2.12 GLO L2C/A */
		if (sys == SYS_GLO) sprintf(type, "%c2C", str[0]);
	}
	else if (ver >= 2.12 && str[1] == '1') { /* ver.2.12 GPS L1PY,GLO L1P */
		if (sys == SYS_GPS) sprintf(type, "%c1W", str[0]);
		else if (sys == SYS_GLO) sprintf(type, "%c1P", str[0]);
		else if (sys == SYS_GAL) sprintf(type, "%c1X", str[0]); /* tentative */
		else if (sys == SYS_CMP) sprintf(type, "%c1X", str[0]); /* extension */
	}
	else if (ver < 2.12 && str[1] == '1') {
		if (sys == SYS_GPS) sprintf(type, "%c1C", str[0]);
		else if (sys == SYS_GLO) sprintf(type, "%c1C", str[0]);
		else if (sys == SYS_GAL) sprintf(type, "%c1X", str[0]); /* tentative */
		else if (sys == SYS_QZS) sprintf(type, "%c1C", str[0]);
	}
	else if (str[1] == '2') {
		if (sys == SYS_GPS) sprintf(type, "%c2W", str[0]);
		else if (sys == SYS_GLO) sprintf(type, "%c2P", str[0]);
		else if (sys == SYS_QZS) sprintf(type, "%c2X", str[0]);
		else if (sys == SYS_CMP) sprintf(type, "%c1X", str[0]); /* ver.2.12 B1 */
	}
	else if (str[1] == '5') {
		if (sys == SYS_GPS) sprintf(type, "%c5X", str[0]);
		else if (sys == SYS_GAL) sprintf(type, "%c5X", str[0]);
		else if (sys == SYS_QZS) sprintf(type, "%c5X", str[0]);
	}
	else if (str[1] == '6') {
		if (sys == SYS_GAL) sprintf(type, "%c6X", str[0]);
		else if (sys == SYS_QZS) sprintf(type, "%c6X", str[0]);
		else if (sys == SYS_CMP) sprintf(type, "%c6X", str[0]); /* ver.2.12 B3 */
	}
	else if (str[1] == '7') {
		if (sys == SYS_GAL) sprintf(type, "%c7X", str[0]);
		else if (sys == SYS_CMP) sprintf(type, "%c7X", str[0]); /* ver.2.12 B2 */
	}
	else if (str[1] == '8') {
		if (sys == SYS_GAL) sprintf(type, "%c8X", str[0]);
	}
	trace(3, "convcode: ver=%.2f sys=%2d type= %s -> %s\n", ver, sys, str, type);
}
/* decode obs header ---------------------------------------------------------*/
static void decode_obsh(FILE* fp, char* buff, double ver, int* tsys,
	char tobs[][MAXOBSTYPE][4], nav_t* nav/*, sta_t *sta*/)
{
	/* default codes for unknown code */
	const char* defcodes[] = {
		"CWX    ",  /* GPS: L125____ */
		"CC     ",  /* GLO: L12_____ */
		"X XXXX ",  /* GAL: L1_5678_ */
		"CXXX   ",  /* QZS: L1256___ */
		"C X    ",  /* SBS: L1_5____ */
		"X  XX  ",  /* BDS: L1__67__ */
		"  A   A"   /* IRN: L__5___9 */
	};
	int i, j, k, n, nt, prn, fcn;
	const char* p;
	char* label = buff + 60, str[4];

	trace(4, "decode_obsh: ver=%.2f\n", ver);

	if (strstr(label, "MARKER NAME")) {
	}
	else if (strstr(label, "MARKER NUMBER")) { /* opt */
	}
	else if (strstr(label, "MARKER TYPE")); /* ver.3 */
	else if (strstr(label, "OBSERVER / AGENCY"));
	else if (strstr(label, "REC # / TYPE / VERS")) {
	}
	else if (strstr(label, "ANT # / TYPE")) {
	}
	else if (strstr(label, "APPROX POSITION XYZ")) {
	}
	else if (strstr(label, "ANTENNA: DELTA H/E/N")) {
	}
	else if (strstr(label, "ANTENNA: DELTA X/Y/Z")); /* opt ver.3 */
	else if (strstr(label, "ANTENNA: PHASECENTER")); /* opt ver.3 */
	else if (strstr(label, "ANTENNA: B.SIGHT XYZ")); /* opt ver.3 */
	else if (strstr(label, "ANTENNA: ZERODIR AZI")); /* opt ver.3 */
	else if (strstr(label, "ANTENNA: ZERODIR XYZ")); /* opt ver.3 */
	else if (strstr(label, "CENTER OF MASS: XYZ")); /* opt ver.3 */
	else if (strstr(label, "SYS / # / OBS TYPES")) { /* ver.3 */
		if (!(p = strchr(syscodes, buff[0]))) {
			trace(2, "invalid system code: sys=%c\n", buff[0]);
			return;
		}
		i = (int)(p - syscodes);
		n = (int)str2num(buff, 3, 3);
		for (j = nt = 0, k = 7; j < n; j++, k += 4) {
			if (k > 58) {
				if (!fgets(buff, MAXRNXLEN, fp)) break;
				k = 7;
			}
			if (nt < MAXOBSTYPE - 1) setstr(tobs[i][nt++], buff + k, 3);
		}
		*tobs[i][nt] = '\0';

		/* change beidou B1 code: 3.02 draft -> 3.02/3.03 */
		if (i == 5) {
			for (j = 0; j < nt; j++) if (tobs[i][j][1] == '2') tobs[i][j][1] = '1';
		}
		/* if unknown code in ver.3, set default code */
		for (j = 0; j < nt; j++) {
			if (tobs[i][j][2]) continue;
			if (!(p = strchr(frqcodes, tobs[i][j][1]))) continue;
			tobs[i][j][2] = defcodes[i][(int)(p - frqcodes)];
			trace(2, "set default for unknown code: sys=%c code=%s\n", buff[0],
				tobs[i][j]);
		}
	}
	else if (strstr(label, "WAVELENGTH FACT L1/2")); /* opt ver.2 */
	else if (strstr(label, "# / TYPES OF OBSERV")) { /* ver.2 */
		n = (int)str2num(buff, 0, 6);
		for (i = nt = 0, j = 10; i < n; i++, j += 6) {
			if (j > 58) {
				if (!fgets(buff, MAXRNXLEN, fp)) break;
				j = 10;
			}
			if (nt >= MAXOBSTYPE - 1) continue;
			if (ver <= 2.99) {
				setstr(str, buff + j, 2);
				convcode(ver, SYS_GPS, str, tobs[0][nt]);
				convcode(ver, SYS_GLO, str, tobs[1][nt]);
				convcode(ver, SYS_GAL, str, tobs[2][nt]);
				convcode(ver, SYS_QZS, str, tobs[3][nt]);
				convcode(ver, SYS_CMP, str, tobs[5][nt]);
			}
			nt++;
		}
		*tobs[0][nt] = '\0';
	}
	else if (strstr(label, "SIGNAL STRENGTH UNIT")); /* opt ver.3 */
	else if (strstr(label, "INTERVAL")); /* opt */
	else if (strstr(label, "TIME OF FIRST OBS")) {
		if (!strncmp(buff + 48, "GPS", 3)) *tsys = TSYS_GPS;
		else if (!strncmp(buff + 48, "GLO", 3)) *tsys = TSYS_UTC;
		else if (!strncmp(buff + 48, "GAL", 3)) *tsys = TSYS_GAL;
		//else if (!strncmp(buff+48,"QZS",3)) *tsys=TSYS_QZS; /* ver.3.02 */
		else if (!strncmp(buff + 48, "BDT", 3)) *tsys = TSYS_CMP; /* ver.3.02 */
		//else if (!strncmp(buff+48,"IRN",3)) *tsys=TSYS_IRN; /* ver.3.03 */
	}
	else if (strstr(label, "TIME OF LAST OBS")); /* opt */
	else if (strstr(label, "RCV CLOCK OFFS APPL")); /* opt */
	else if (strstr(label, "SYS / DCBS APPLIED")); /* opt ver.3 */
	else if (strstr(label, "SYS / PCVS APPLIED")); /* opt ver.3 */
	else if (strstr(label, "SYS / SCALE FACTOR")); /* opt ver.3 */
	else if (strstr(label, "SYS / PHASE SHIFTS")); /* ver.3.01 */
	else if (strstr(label, "GLONASS SLOT / FRQ #")) { /* ver.3.02 */
		if (nav) {
			for (i = 0, p = buff + 4; i < 8; i++, p += 8) {
				if (sscanf(p, "R%2d %2d", &prn, &fcn) < 2) continue;
				if (1 <= prn && prn <= MAXPRNGLO) nav->glo_fcn[prn - 1] = fcn + 8;
			}
		}
	}
	else if (strstr(label, "GLONASS COD/PHS/BIS")) { /* ver.3.02 */
		if (nav) {
			for (i = 0, p = buff; i < 4; i++, p += 13) {
				if (strncmp(p + 1, "C1C", 3)) nav->glo_cpbias[0] = str2num(p, 5, 8);
				else if (strncmp(p + 1, "C1P", 3)) nav->glo_cpbias[1] = str2num(p, 5, 8);
				else if (strncmp(p + 1, "C2C", 3)) nav->glo_cpbias[2] = str2num(p, 5, 8);
				else if (strncmp(p + 1, "C2P", 3)) nav->glo_cpbias[3] = str2num(p, 5, 8);
			}
		}
	}
	else if (strstr(label, "LEAP SECONDS")) { /* opt */
		if (nav) nav->leaps = (int)str2num(buff, 0, 6);
	}
	else if (strstr(label, "# OF SALTELLITES")) { /* opt */
		/* skip */;
	}
	else if (strstr(label, "PRN / # OF OBS")) { /* opt */
		/* skip */;
	}
}
/* decode nav header ---------------------------------------------------------*/
static void decode_navh(char* buff, nav_t* nav)
{
	int i, j;
	char* label = buff + 60;

	trace(4, "decode_navh:\n");

	if (strstr(label, "ION ALPHA")) { /* opt ver.2 */
		if (nav) {
			for (i = 0, j = 2; i < 4; i++, j += 12) nav->ion_gps[i] = str2num(buff, j, 12);
		}
	}
	else if (strstr(label, "ION BETA")) { /* opt ver.2 */
		if (nav) {
			for (i = 0, j = 2; i < 4; i++, j += 12) nav->ion_gps[i + 4] = str2num(buff, j, 12);
		}
	}
	else if (strstr(label, "DELTA-UTC: A0,A1,T,W")) { /* opt ver.2 */
		if (nav) {
			for (i = 0, j = 3; i < 2; i++, j += 19) nav->utc_gps[i] = str2num(buff, j, 19);
			for (; i < 4; i++, j += 9) nav->utc_gps[i] = str2num(buff, j, 9);
		}
	}
	else if (strstr(label, "IONOSPHERIC CORR")) { /* opt ver.3 */
		if (nav) {
			if (!strncmp(buff, "GPSA", 4)) {
				for (i = 0, j = 5; i < 4; i++, j += 12) nav->ion_gps[i] = str2num(buff, j, 12);
			}
			else if (!strncmp(buff, "GPSB", 4)) {
				for (i = 0, j = 5; i < 4; i++, j += 12) nav->ion_gps[i + 4] = str2num(buff, j, 12);
			}
			else if (!strncmp(buff, "GAL", 3)) {
				for (i = 0, j = 5; i < 4; i++, j += 12) nav->ion_gal[i] = str2num(buff, j, 12);
			}
			else if (!strncmp(buff, "QZSA", 4)) { /* v.3.02 */
				for (i = 0, j = 5; i < 4; i++, j += 12) nav->ion_qzs[i] = str2num(buff, j, 12);
			}
			else if (!strncmp(buff, "QZSB", 4)) { /* v.3.02 */
				for (i = 0, j = 5; i < 4; i++, j += 12) nav->ion_qzs[i + 4] = str2num(buff, j, 12);
			}
			else if (!strncmp(buff, "BDSA", 4)) { /* v.3.02 */
				for (i = 0, j = 5; i < 4; i++, j += 12) nav->ion_cmp[i] = str2num(buff, j, 12);
			}
			else if (!strncmp(buff, "BDSB", 4)) { /* v.3.02 */
				for (i = 0, j = 5; i < 4; i++, j += 12) nav->ion_cmp[i + 4] = str2num(buff, j, 12);
			}
		}
	}
	else if (strstr(label, "TIME SYSTEM CORR")) { /* opt ver.3 */
		if (nav) {
			if (!strncmp(buff, "GPUT", 4)) {
				nav->utc_gps[0] = str2num(buff, 5, 17);
				nav->utc_gps[1] = str2num(buff, 22, 16);
				nav->utc_gps[2] = str2num(buff, 38, 7);
				nav->utc_gps[3] = str2num(buff, 45, 5);
			}
			else if (!strncmp(buff, "GLUT", 4)) {
				nav->utc_glo[0] = str2num(buff, 5, 17);
				nav->utc_glo[1] = str2num(buff, 22, 16);
			}
			else if (!strncmp(buff, "GAUT", 4)) { /* v.3.02 */
				nav->utc_gal[0] = str2num(buff, 5, 17);
				nav->utc_gal[1] = str2num(buff, 22, 16);
				nav->utc_gal[2] = str2num(buff, 38, 7);
				nav->utc_gal[3] = str2num(buff, 45, 5);
			}
			else if (!strncmp(buff, "QZUT", 4)) { /* v.3.02 */
				nav->utc_qzs[0] = str2num(buff, 5, 17);
				nav->utc_qzs[1] = str2num(buff, 22, 16);
				nav->utc_qzs[2] = str2num(buff, 38, 7);
				nav->utc_qzs[3] = str2num(buff, 45, 5);
			}
			else if (!strncmp(buff, "BDUT", 4)) { /* v.3.02 */
				nav->utc_cmp[0] = str2num(buff, 5, 17);
				nav->utc_cmp[1] = str2num(buff, 22, 16);
				nav->utc_cmp[2] = str2num(buff, 38, 7);
				nav->utc_cmp[3] = str2num(buff, 45, 5);
			}
			else if (!strncmp(buff, "SBUT", 4)) { /* v.3.02 */
				nav->utc_cmp[0] = str2num(buff, 5, 17);
				nav->utc_cmp[1] = str2num(buff, 22, 16);
				nav->utc_cmp[2] = str2num(buff, 38, 7);
				nav->utc_cmp[3] = str2num(buff, 45, 5);
			}
		}
	}
	else if (strstr(label, "LEAP SECONDS")) { /* opt */
		if (nav) nav->leaps = (int)str2num(buff, 0, 6);
	}
}
/* decode gnav header --------------------------------------------------------*/
static void decode_gnavh(char* buff, nav_t* nav)
{
	char* label = buff + 60;

	trace(4, "decode_gnavh:\n");

	if (strstr(label, "CORR TO SYTEM TIME")); /* opt */
	else if (strstr(label, "LEAP SECONDS")) { /* opt */
		if (nav) nav->leaps = (int)str2num(buff, 0, 6);
	}
}
/* decode geo nav header -----------------------------------------------------*/
static void decode_hnavh(char* buff, nav_t* nav)
{
	char* label = buff + 60;

	trace(4, "decode_hnavh:\n");

	if (strstr(label, "CORR TO SYTEM TIME")); /* opt */
	else if (strstr(label, "D-UTC A0,A1,T,W,S,U")); /* opt */
	else if (strstr(label, "LEAP SECONDS")) { /* opt */
		if (nav) nav->leaps = (int)str2num(buff, 0, 6);
	}
}
/* read rinex header ---------------------------------------------------------*/
extern int readrnxh(FILE* fp, double* ver, char* type, int* sys, int* tsys,
	char tobs[][MAXOBSTYPE][4], nav_t* nav/*, sta_t *sta*/)
{
	double bias;
	char buff[MAXRNXLEN], * label = buff + 60;
	int i = 0, block = 0, sat;

	trace(3, "readrnxh:\n");

	*ver = 2.10; *type = ' '; *sys = SYS_GPS;

	while (fgets(buff, MAXRNXLEN, fp)) {
		if (strlen(buff) <= 60) continue;

		else if (strstr(label, "RINEX VERSION / TYPE")) {
			*ver = str2num(buff, 0, 9);
			*type = *(buff + 20);

			/* satellite system */
			switch (*(buff + 40)) {
			case ' ':
			case 'G': *sys = SYS_GPS;  *tsys = TSYS_GPS; break;
			case 'R': *sys = SYS_GLO;  *tsys = TSYS_UTC; break;
			case 'E': *sys = SYS_GAL;  *tsys = TSYS_GAL; break; /* v.2.12 */
			case 'C': *sys = SYS_CMP;  *tsys = TSYS_CMP; break; /* v.2.12 */
			case 'M': *sys = SYS_NONE; *tsys = TSYS_GPS; break; /* mixed */
			default:
				trace(2, "not supported satellite system: %c\n", *(buff + 40));
				break;
			}
			continue;
		}
		else if (strstr(label, "PGM / RUN BY / DATE")) continue;
		else if (strstr(label, "COMMENT")) { /* opt */

			/* read cnes wl satellite fractional bias */
			if (strstr(buff, "WIDELANE SATELLITE FRACTIONAL BIASES") ||
				strstr(buff, "WIDELANE SATELLITE FRACTIONNAL BIASES")) {
				block = 1;
			}
			else if (block) {
				/* cnes/cls grg clock */
				if (!strncmp(buff, "WL", 2) && (sat = satid2no(buff + 3)) &&
					sscanf(buff + 40, "%lf", &bias) == 1) {
				}
				/* cnes ppp-wizard clock */
				else if ((sat = satid2no(buff + 1)) && sscanf(buff + 6, "%lf", &bias) == 1) {
				}
			}
			continue;
		}
		/* file type */
		switch (*type) {
		case 'O': decode_obsh(fp, buff, *ver, tsys, tobs, nav/*,sta*/); break;
		case 'N': decode_navh(buff, nav); break;
		case 'G': decode_gnavh(buff, nav); break;
		case 'H': decode_hnavh(buff, nav); break;
		case 'J': decode_navh(buff, nav); break; /* extension */
		case 'L': decode_navh(buff, nav); break; /* extension */
		}
		if (strstr(label, "END OF HEADER")) return 1;

		if (++i >= MAXPOSHEAD && *type == ' ') break; /* no rinex file */
	}
	return 0;
}
/* decode obs epoch ----------------------------------------------------------*/
static int decode_obsepoch(FILE* fp, char* buff, double ver, gtime_t* time,
	int* flag, int* sats)
{
	int i, j, n;
	char satid[8] = "";

	//trace(4,"decode_obsepoch: ver=%.2f\n",ver);

	if (ver <= 2.99) { /* ver.2 */
		if ((n = (int)str2num(buff, 29, 3)) <= 0) return 0;

		/* epoch flag: 3:new site,4:header info,5:external event */
		*flag = (int)str2num(buff, 28, 1);

		if (3 <= *flag && *flag <= 5) return n;

		if (str2time(buff, 0, 26, time)) {
			trace(2, "rinex obs invalid epoch: epoch=%26.26s\n", buff);
			return 0;
		}
		for (i = 0, j = 32; i < n; i++, j += 3) {
			if (j >= 68) {
				if (!fgets(buff, MAXRNXLEN, fp)) break;
				j = 32;
			}
			if (i < MAXRECOBS) {
				strncpy(satid, buff + j, 3);
				sats[i] = satid2no(satid);
			}
		}
	}
	else { /* ver.3 */
		if ((n = (int)str2num(buff, 32, 3)) <= 0) return 0;

		*flag = (int)str2num(buff, 31, 1);

		if (3 <= *flag && *flag <= 5) return n;

		if (buff[0] != '>' || str2time(buff, 1, 28, time)) {
			trace(2, "rinex obs invalid epoch: epoch=%29.29s\n", buff);
			return 0;
		}
	}
	//trace(4,"decode_obsepoch: time=%s flag=%d\n",time_str(*time,3),*flag);
	return n;
}
/* decode obs data -----------------------------------------------------------*/
static int decode_obsdata(FILE* fp, char* buff, double ver, int mask,
	sigind_t* index, obsd_t* obs)
{
	sigind_t* ind;
	double val[MAXOBSTYPE] = { 0 };
	unsigned char lli[MAXOBSTYPE] = { 0 };
	char satid[8] = "";
	int i, j, n, m, stat = 1, p[MAXOBSTYPE], k[16], l[16];

	//trace(4,"decode_obsdata: ver=%.2f\n",ver);

	if (ver > 2.99) { /* ver.3 */
		strncpy(satid, buff, 3);
		obs->sat = (unsigned char)satid2no(satid);
	}
	if (!obs->sat) {
		//trace(4,"decode_obsdata: unsupported sat sat=%s\n",satid);
		stat = 0;
	}
	else if (!(satsys(obs->sat, NULL) & mask)) {
		stat = 0;
	}

	/* read obs data fields */
	switch (satsys(obs->sat, NULL)) {
	case SYS_GLO: ind = index + 1; break;
	case SYS_GAL: ind = index + 2; break;
	case SYS_QZS: ind = index + 3; break;

	case SYS_CMP: ind = index + 5; break;
	default:      ind = index; break;
	}
	for (i = 0, j = ver <= 2.99 ? 0 : 3; i < ind->n; i++, j += 16) {
		if (ver <= 2.99 && j >= 80) { /* ver.2 */
			if (!fgets(buff, MAXRNXLEN, fp)) break;
			j = 0;
		}
		if (stat) {
			val[i] = str2num(buff, j, 14) + ind->shift[i];
			lli[i] = (unsigned char)str2num(buff, j + 14, 1) & 7;

			if (lli[i] == 3)lli[i] = 2;
			if (lli[i] == 4)lli[i] = 3;
		}
	}
	if (!stat) return 0;

	for (i = 0; i < NFREQ + NEXOBS; i++) {
		obs->P[i] = obs->L[i] = 0.0; obs->D[i] = 0.0f;
		obs->snr[i] = obs->lli[i] = obs->code[i] = 0;
	}
	/* assign position in obs data */
	for (i = n = m = 0; i < ind->n; i++) {
		p[i] = ver <= 2.11 ? ind->frq[i] - 1 : ind->pos[i];

		if (ind->type[i] == 0 && p[i] == 0) k[n++] = i; /* C1? index */
		if (ind->type[i] == 0 && p[i] == 1) l[m++] = i; /* C2? index */
	}
	if (ver <= 2.11) {
		/* if multiple codes (C1/P1,C2/P2), select higher priority */
		if (n >= 2) {
			if (val[k[0]] == 0.0 && val[k[1]] == 0.0) {
				p[k[0]] = -1; p[k[1]] = -1;
			}
			else if (val[k[0]] != 0.0 && val[k[1]] == 0.0) {
				p[k[0]] = 0; p[k[1]] = -1;
			}
			else if (val[k[0]] == 0.0 && val[k[1]] != 0.0) {
				p[k[0]] = -1; p[k[1]] = 0;
			}
			else if (ind->pri[k[1]] > ind->pri[k[0]]) {
				p[k[1]] = 0; p[k[0]] = NEXOBS < 1 ? -1 : NFREQ;
			}
			else {
				p[k[0]] = 0; p[k[1]] = NEXOBS < 1 ? -1 : NFREQ;
			}
		}
		if (m >= 2) {
			if (val[l[0]] == 0.0 && val[l[1]] == 0.0) {
				p[l[0]] = -1; p[l[1]] = -1;
			}
			else if (val[l[0]] != 0.0 && val[l[1]] == 0.0) {
				p[l[0]] = 1; p[l[1]] = -1;
			}
			else if (val[l[0]] == 0.0 && val[l[1]] != 0.0) {
				p[l[0]] = -1; p[l[1]] = 1;
			}
			else if (ind->pri[l[1]] > ind->pri[l[0]]) {
				p[l[1]] = 1; p[l[0]] = NEXOBS < 2 ? -1 : NFREQ + 1;
			}
			else {
				p[l[0]] = 1; p[l[1]] = NEXOBS < 2 ? -1 : NFREQ + 1;
			}
		}
	}
	/* save obs data */
	for (i = 0; i < ind->n; i++) {
		if (p[i] < 0 || val[i] == 0.0) continue;
		switch (ind->type[i]) {
		case 0: obs->P[p[i]] = val[i]; obs->code[p[i]] = ind->code[i]; break;
		case 1: obs->L[p[i]] = val[i]; obs->lli[p[i]] = lli[i];       break;
		case 2: obs->D[p[i]] = (float)val[i];                        break;
		case 3: obs->snr[p[i]] = (unsigned char)(val[i] * 4.0 + 0.5);    break;
		}
	}
	//trace(4,"decode_obsdata: time=%s sat=%2d\n",time_str(obs->time,0),obs->sat);
	return 1;
}
/* save slips ----------------------------------------------------------------*/
static void saveslips(unsigned char slips[][NFREQ], obsd_t* data)
{
	int i;
	for (i = 0; i < NFREQ; i++) {
		if (data->lli[i] & 1) slips[data->sat - 1][i] |= LLI_SLIP;
	}
}
/* restore slips -------------------------------------------------------------*/
static void restslips(unsigned char slips[][NFREQ], obsd_t* data)
{
	int i;
	for (i = 0; i < NFREQ; i++) {
		if (slips[data->sat - 1][i] & 1) data->lli[i] |= LLI_SLIP;
		slips[data->sat - 1][i] = 0;
	}
}
/* add obs data --------------------------------------------------------------*/
static int addobsdata(obs_t* obs, const obsd_t* data)
{
	obsd_t* obs_data;

	if (obs->nmax <= obs->n) {
		if (obs->nmax <= 0) obs->nmax = NINCOBS; else obs->nmax *= 2;
		if (!(obs_data = (obsd_t*)realloc(obs->data, sizeof(obsd_t) * obs->nmax))) {
			trace(1, "addobsdata: memalloc error n=%dx%d\n", sizeof(obsd_t), obs->nmax);
			xy_free(obs->data); obs->data = NULL; obs->n = obs->nmax = 0;
			return -1;
		}
		obs->data = obs_data;
	}
	obs->data[obs->n++] = *data;
	return 1;
}
/* set system mask -----------------------------------------------------------*/
static int set_sysmask(const char* opt)
{
	const char* p;
	int mask = SYS_NONE;

	if (!(p = strstr(opt, "-SYS="))) return SYS_ALL;

	for (p += 5; *p && *p != ' '; p++) {
		switch (*p) {
		case 'G': mask |= SYS_GPS; break;
		case 'R': mask |= SYS_GLO; break;
		case 'E': mask |= SYS_GAL; break;
		case 'J': mask |= SYS_QZS; break;
		case 'C': mask |= SYS_CMP; break;
		}
	}
	return mask;
}
/* set signal index ----------------------------------------------------------*/
static void set_index(double ver, int sys, const char* opt,
	char tobs[MAXOBSTYPE][4], sigind_t* ind)
{
	const char* p = "";
	char str[8] = "", * optstr = "";
	double shift = 0;;
	int i = 0, j = 0, k = 0, n = 0;

	for (i = n = 0; *tobs[i]; i++, n++) {
		ind->code[i] = obs2code(tobs[i] + 1, ind->frq + i);
		ind->type[i] = (p = strchr(obscodes, tobs[i][0])) ? (int)(p - obscodes) : 0;
		ind->pri[i] = getcodepri(sys, ind->code[i], opt);
		ind->pos[i] = -1;

		/* frequency index for beidou */
		if (sys == SYS_CMP) {
			if (FREQMODE_CMP == 0)//B1 B2  B3模式
			{
				if (ind->frq[i] == 5) ind->frq[i] = 2; /* B2 */
			}
			else
			{
				if (ind->frq[i] == 3) ind->frq[i] = 2; /* b1c B2a(C5I) */
			}
			if (ind->frq[i] == 4) ind->frq[i] = 3; /* B3 */
		}
		if (sys == SYS_GPS) {
			if (FREQMODE_CMP == 1) {//L1+L5 B1+B2a模式
				if (ind->frq[i] == 2) ind->frq[i] = 3;     //调整GPS L2频freq值为3
				else if (ind->frq[i] == 3) ind->frq[i] = 2;//调整GPS L5频freq值为2
			}
		}
		if (sys == SYS_GAL) {
			if (FREQMODE_CMP == 1) {//L1+L5 E1+E5a模式
				if (ind->frq[i] == 2) ind->frq[i] = 3;//调整GAL L2频freq值为3
				else if (ind->frq[i] == 3) ind->frq[i] = 2;//调整GAL L5频freq值为2
			}
		}
	}
	/* parse phase shift options */
	switch (sys) {
	case SYS_GPS: optstr = "-GL%2s=%lf"; break;
	case SYS_GLO: optstr = "-RL%2s=%lf"; break;
	case SYS_GAL: optstr = "-EL%2s=%lf"; break;
	case SYS_QZS: optstr = "-JL%2s=%lf"; break;
	case SYS_CMP: optstr = "-CL%2s=%lf"; break;
	}
	for (p = opt; p && (p = strchr(p, '-')); p++) {
		if (sscanf(p, optstr, str, &shift) < 2) continue;
		for (i = 0; i < n; i++) {
			if (strcmp(code2obs(ind->code[i], NULL), str)) continue;
			ind->shift[i] = shift;
			trace(2, "phase shift: sys=%2d tobs=%s shift=%.3f\n", sys,
				tobs[i], shift);
		}
	}
	/* assign index for highest priority code */
	for (i = 0; i < NFREQ; i++) {
		for (j = 0, k = -1; j < n; j++) {
			if (ind->frq[j] == i + 1 && ind->pri[j] && (k<0 || ind->pri[j]>ind->pri[k])) {
				k = j;
			}
		}
		if (k < 0) continue;

		for (j = 0; j < n; j++) {
			if (ind->code[j] == ind->code[k]) ind->pos[j] = i;
		}
	}
	/* assign index of extended obs data */
	for (i = 0; i < NEXOBS; i++) {
		for (j = 0; j < n; j++) {
			if (ind->code[j] && ind->pri[j] && ind->pos[j] < 0) break;
		}
		if (j >= n) break;

		for (k = 0; k < n; k++) {
			if (ind->code[k] == ind->code[j]) ind->pos[k] = NFREQ + i;
		}
	}
	for (i = 0; i < n; i++) {
		if (!ind->code[i] || !ind->pri[i] || ind->pos[i] >= 0) continue;
	}
	ind->n = n;

#if 0 /* for debug */
	for (i = 0; i < n; i++) {
		trace(2, "set_index: sys=%2d,tobs=%s code=%2d pri=%2d frq=%d pos=%d shift=%5.2f\n",
			sys, tobs[i], ind->code[i], ind->pri[i], ind->frq[i], ind->pos[i],
			ind->shift[i]);
	}
#endif
}
/* read rinex obs data body --------------------------------------------------*/
extern int readrnxobsb(FILE* fp, const char* opt, double ver, int* tsys,
	char tobs[][MAXOBSTYPE][4], int* flag, obsd_t* data, int rcv)
{
	gtime_t time = { 0 };
	sigind_t index[7] = { {0} };
	char buff[MAXRNXLEN] = "";
	int i = 0, n = 0, nsat = 0, sats[MAXRECOBS] = { 0 }, mask = 0;

	/* set system mask */
	mask = set_sysmask(opt);

	/* set signal index */
	set_index(ver, SYS_GPS, opt, tobs[0], index);
	set_index(ver, SYS_GLO, opt, tobs[1], index + 1);
	set_index(ver, SYS_GAL, opt, tobs[2], index + 2);
	set_index(ver, SYS_QZS, opt, tobs[3], index + 3);
	set_index(ver, SYS_CMP, opt, tobs[5], index + 5);

	/* read record */
	while (fgets(buff, MAXRNXLEN, fp)) {
		/* decode obs epoch */
		if (i == 0) {
			if ((nsat = decode_obsepoch(fp, buff, ver, &time, flag, sats)) <= 0) {
				continue;
			}
		}
		else if (*flag <= 2 || *flag == 6) {
			data[n].time = time;
			/* decode obs data */
			if (decode_obsdata(fp, buff, ver, mask, index, data + n) && n < (MAXRECOBS * rcv)) n++;
		}
		else if (*flag == 3 || *flag == 4) { /* new site or header info follows */

			/* decode obs header */
			decode_obsh(fp, buff, ver, tsys, tobs, NULL/*,sta*/);
		}
		if (++i > nsat) return n;
	}
	return -1;
}
/* read rinex obs ------------------------------------------------------------*/
static int readrnxobs(FILE* fp, gtime_t ts, gtime_t te, double tint,
	const char* opt, int rcv, double ver, int* tsys,
	char tobs[][MAXOBSTYPE][4], obs_t* obs)
{
	obsd_t* data = { 0 };
	unsigned char slips[MAXSAT][NFREQ] = { {0} };
	int i = 0, n = 0, flag = 0, stat = 0;

	trace(4, "readrnxobs: rcv=%d ver=%.2f tsys=%d\n", rcv, ver, tsys);

	if (!obs || rcv > MAXRCV) return 0;

	if (!(data = (obsd_t*)calloc(sizeof(obsd_t), MAXRECOBS))) return 0;

	/* read rinex obs data body */
	while ((n = readrnxobsb(fp, opt, ver, tsys, tobs, &flag, data, rcv)) >= 0 && stat >= 0) {
		for (i = 0; i < n; i++) {
			/* utc -> gpst */
			if (*tsys == TSYS_UTC) data[i].time = utc2gpst(data[i].time);

			/* save cycle-slip */
			saveslips(slips, data + i);
		}
		/* screen data by time */
		if (n > 0 && !screent(data[0].time, ts, te, tint)) continue;

		for (i = 0; i < n; i++) {
			/* restore cycle-slip */
			restslips(slips, data + i);

			data[i].rcv = (unsigned char)rcv;

			/* save obs data */
			if ((stat = addobsdata(obs, data + i)) < 0) break;
		}
	}
	trace(4, "readrnxobs: nobs=%d stat=%d\n", obs->n, stat);

	xy_free(data);

	return stat;
}
/* decode ephemeris ----------------------------------------------------------*/
static int decode_eph(double ver, int sat, gtime_t toc, const double* data,
	eph_t* eph)
{
	eph_t eph0 = { 0 };
	int sys = 0;

	trace(4, "decode_eph: ver=%.2f sat=%2d\n", ver, sat);

	sys = satsys(sat, NULL);

	if (!(sys & (SYS_GPS | SYS_GAL | SYS_QZS | SYS_CMP))) {
		trace(3, "ephemeris error: invalid satellite sat=%2d\n", sat);
		return 0;
	}
	*eph = eph0;

	eph->sat = sat;
	eph->toc = toc;

	eph->f0 = data[0];
	eph->f1 = data[1];
	eph->f2 = data[2];

	eph->A = SQR(data[10]); eph->e = data[8]; eph->i0 = data[15]; eph->omg0 = data[13];
	eph->omg = data[17]; eph->m0 = data[6]; eph->deln = data[5]; eph->omgd = data[18];
	eph->idot = data[19]; eph->crc = data[16]; eph->crs = data[4]; eph->cuc = data[7];
	eph->cus = data[9]; eph->cic = data[12]; eph->cis = data[14];

	if (sys == SYS_GPS || sys == SYS_QZS) {
		eph->iode = (int)data[3];      /* IODE */
		eph->iodc = (int)data[26];      /* IODC */
		eph->toes = data[11];      /* toe (s) in gps week */
		eph->week = (int)data[21];      /* gps week */
		eph->toe = adjweek(gpst2time(eph->week, data[11]), toc);
		eph->ttr = adjweek(gpst2time(eph->week, data[27]), toc);

		eph->code = (int)data[20];      /* GPS: codes on L2 ch */
		eph->svh = (int)data[24];      /* sv health */
		eph->sva = uraindex(data[23]);  /* ura (m->index) */
		eph->flag = (int)data[22];      /* GPS: L2 P data flag */

		eph->tgd = data[25];      /* TGD */
		if (sys == SYS_GPS) {
			eph->fit = data[28];        /* fit interval (h) */
		}
		else {
			eph->fit = data[28] == 0.0 ? 1.0 : 2.0; /* fit interval (0:1h,1:>2h) */
		}
	}
	else if (sys == SYS_GAL) { /* GAL ver.3 */
		eph->iode = (int)data[3];      /* IODnav */
		eph->toes = data[11];      /* toe (s) in galileo week */
		eph->week = (int)data[21];      /* gal week = gps week */
		eph->toe = adjweek(gpst2time(eph->week, data[11]), toc);
		eph->ttr = adjweek(gpst2time(eph->week, data[27]), toc);

		eph->code = (int)data[20];      /* data sources */
		/* bit 0 set: I/NAV E1-B */
		/* bit 1 set: F/NAV E5a-I */
		/* bit 2 set: F/NAV E5b-I */
		/* bit 8 set: af0-af2 toc are for E5a.E1 */
		/* bit 9 set: af0-af2 toc are for E5b.E1 */
		eph->svh = (int)data[24];      /* sv health */
		/* bit     0: E1B DVS */
		/* bit   1-2: E1B HS */
		/* bit     3: E5a DVS */
		/* bit   4-5: E5a HS */
		/* bit     6: E5b DVS */
		/* bit   7-8: E5b HS */
		eph->sva = uraindex(data[23]); /* ura (m->index) */

		eph->tgd = data[25];      /* BGD E5a/E1 */
		eph->tgdb = data[26];      /* BGD E5b/E1 */
	}
	else if (sys == SYS_CMP) { /* BeiDou v.3.02 */
		//eph->toc=bdt2gpst(eph->toc);  /* bdt -> gpst */
		eph->iode = (int)data[3];      /* AODE */
		eph->iodc = (int)data[28];      /* AODC */
		eph->toes = data[11];      /* toe (s) in bdt week */
		eph->week = (int)data[21];      /* bdt week */
		eph->toe = bdt2gpst(bdt2time(eph->week, data[11])); /* bdt -> gpst */
		eph->ttr = bdt2gpst(bdt2time(eph->week, data[27])); /* bdt -> gpst */
		eph->toe = adjweek(eph->toe, toc);
		eph->ttr = adjweek(eph->ttr, toc);

		eph->svh = (int)data[24];      /* satH1 */
		eph->sva = uraindex(data[23]);  /* ura (m->index) */

		eph->tgd = data[25];      /* TGD1 B1/B3 */
		eph->tgdb = data[26];      /* TGD2 B2/B3 */
	}

	if (eph->iode < 0 || 1023 < eph->iode) {
		trace(2, "rinex nav invalid: sat=%2d iode=%d\n", sat, eph->iode);
	}
	if (eph->iodc < 0 || 1023 < eph->iodc) {
		trace(2, "rinex nav invalid: sat=%2d iodc=%d\n", sat, eph->iodc);
	}
	return 1;
}
/* decode glonass ephemeris --------------------------------------------------*/
static int decode_geph(double ver, int sat, gtime_t toc, double* data,
	geph_t* geph)
{
	geph_t geph0 = { 0 };
	gtime_t tof = { 0 };
	double tow = 0, tod = 0;
	int week = 0, dow = 0;

	trace(4, "decode_geph: ver=%.2f sat=%2d\n", ver, sat);

	if (satsys(sat, NULL) != SYS_GLO) {
		trace(3, "glonass ephemeris error: invalid satellite sat=%2d\n", sat);
		return 0;
	}
	*geph = geph0;

	geph->sat = sat;

	/* toc rounded by 15 min in utc */
	tow = time2gpst(toc, &week);
	toc = gpst2time(week, floor((tow + 450.0) / 900.0) * 900);
	dow = (int)floor(tow / 86400.0);

	/* time of frame in utc */
	tod = ver <= 2.99 ? data[2] : fmod(data[2], 86400.0); /* tod (v.2), tow (v.3) in utc */
	tof = gpst2time(week, tod + dow * 86400.0);
	tof = adjday(tof, toc);

	geph->toe = utc2gpst(toc);   /* toc (gpst) */
	geph->tof = utc2gpst(tof);   /* tof (gpst) */

	/* iode = tb (7bit), tb =index of UTC+3H within current day */
	geph->iode = (int)(fmod(tow + 10800.0, 86400.0) / 900.0 + 0.5);

	geph->taun = -data[0];       /* -taun */
	geph->gamn = data[1];       /* +gamman */

	geph->pos[0] = data[3] * 1E3; geph->pos[1] = data[7] * 1E3; geph->pos[2] = data[11] * 1E3;
	geph->vel[0] = data[4] * 1E3; geph->vel[1] = data[8] * 1E3; geph->vel[2] = data[12] * 1E3;
	geph->acc[0] = data[5] * 1E3; geph->acc[1] = data[9] * 1E3; geph->acc[2] = data[13] * 1E3;

	geph->svh = (int)data[6];
	geph->frq = (int)data[10];
	geph->age = (int)data[14];

	/* some receiver output >128 for minus frequency number */
	if (geph->frq > 128) geph->frq -= 256;

	if (geph->frq < MINFREQ_GLO || MAXFREQ_GLO < geph->frq) {
		trace(2, "rinex gnav invalid freq: sat=%2d fn=%d\n", sat, geph->frq);
	}
	return 1;
}

/* read rinex navigation data body -------------------------------------------*/
static int readrnxnavb(FILE* fp, const char* opt, double ver, int sys,
	int* type, eph_t* eph, geph_t* geph/*, seph_t *seph*/)
{
	gtime_t toc = { 0 };
	double data[64] = { 0 };
	int i = 0, j = 0, prn = 0, sat = 0, sp = 3, mask = 0;
	char buff[MAXRNXLEN] = "", id[8] = "", * p = "";

	/* set system mask */
	mask = set_sysmask(opt);

	while (fgets(buff, MAXRNXLEN, fp)) {
		if (i == 0) {
			/* decode satellite field */
			if (ver >= 3.0 || sys == SYS_GAL || sys == SYS_QZS) { /* ver.3 or GAL/QZS */
				strncpy(id, buff, 3);
				sat = satid2no(id);
				sp = 4;
				if (ver >= 3.0)
					satid2sys(id, &sys);
			}
			else {
				prn = (int)str2num(buff, 0, 2);

				if (sys == SYS_GLO) {
					sat = satno(SYS_GLO, prn);
				}
				else if (93 <= prn && prn <= 97) { /* extension */
					sat = satno(SYS_QZS, prn + 100);
				}
				else sat = satno(SYS_GPS, prn);
			}
			/* decode toc field */
			if (str2time(buff + sp, 0, 19, &toc)) {
				trace(2, "rinex nav toc error: %23.23s\n", buff);
				return 0;
			}
			/* decode data fields */
			for (j = 0, p = buff + sp + 19; j < 3; j++, p += 19) {
				data[i++] = str2num(p, 0, 19);
			}
		}
		else {
			/* decode data fields */
			for (j = 0, p = buff + sp; j < 4; j++, p += 19) {
				data[i++] = str2num(p, 0, 19);
			}
			/* decode ephemeris */
			if (sys == SYS_GLO && i >= 15) {
				if (!(mask & sys)) return 0;
				*type = 1;
				return decode_geph(ver, sat, toc, data, geph);
			}

			else if (i >= 31) {
				if (!(mask & sys)) return 0;
				*type = 0;
				return decode_eph(ver, sat, toc, data, eph);
			}
		}
	}
	return -1;
}
/* add ephemeris to navigation data ------------------------------------------*/
static int add_eph(nav_t* nav, const eph_t* eph)
{
	eph_t* nav_eph = { 0 };

	if (nav->nmax <= nav->n) {
		nav->nmax += 1024;
		if (!(nav_eph = (eph_t*)realloc(nav->eph, sizeof(eph_t) * nav->nmax))) {
			trace(1, "decode_eph malloc error: n=%d\n", nav->nmax);
			xy_free(nav->eph); nav->eph = NULL; nav->n = nav->nmax = 0;
			return 0;
		}
		nav->eph = nav_eph;
	}
	nav->eph[nav->n++] = *eph;
	return 1;
}
static int add_geph(nav_t* nav, const geph_t* geph)
{
	geph_t* nav_geph = { 0 };

	if (nav->ngmax <= nav->ng) {
		nav->ngmax += 1024;
		if (!(nav_geph = (geph_t*)realloc(nav->geph, sizeof(geph_t) * nav->ngmax))) {
			trace(1, "decode_geph malloc error: n=%d\n", nav->ngmax);
			xy_free(nav->geph); nav->geph = NULL; nav->ng = nav->ngmax = 0;
			return 0;
		}
		nav->geph = nav_geph;
	}
	nav->geph[nav->ng++] = *geph;
	return 1;
}

/* read rinex nav/gnav/geo nav -----------------------------------------------*/
static int readrnxnav(FILE* fp, const char* opt, double ver, int sys,
	nav_t* nav)
{
	eph_t eph = { 0 };
	geph_t geph = { 0 };
	int stat = 0, type = 0;

	if (!nav) return 0;

	/* read rinex navigation data body */
	while ((stat = readrnxnavb(fp, opt, ver, sys, &type, &eph, &geph)) >= 0) {
		/* add ephemeris to navigation data */
		if (stat) {
			switch (type) {
			case 1: stat = add_geph(nav, &geph); break;
			default: stat = add_eph(nav, &eph); break;
			}
			if (!stat) return 0;
		}
	}
	return nav->n > 0 || nav->ng > 0/*||nav->ns>0*/;
}

/* read rinex file -----------------------------------------------------------*/
extern int readrnxfp(FILE* fp, gtime_t ts, gtime_t te, double tint,
	const char* opt, int flag, int index, char* type,
	obs_t* obs, nav_t* nav /*,sta_t *sta*/)
{
	double ver = 0;
	int sys = 0, tsys = TSYS_GPS;
	char tobs[NUMSYS][MAXOBSTYPE][4] = { {""} };

	/* read rinex header */
	if (!readrnxh(fp, &ver, type, &sys, &tsys, tobs, nav)) return 0;

	/* flag=0:except for clock,1:clock */
	if ((!flag && *type == 'C') || (flag && *type != 'C')) return 0;

	/* read rinex body */
	switch (*type) {
	case 'O': return readrnxobs(fp, ts, te, tint, opt, index, ver, &tsys, tobs, obs/*,sta*/);
	case 'N': return readrnxnav(fp, opt, ver, sys, nav);
	case 'G': return readrnxnav(fp, opt, ver, SYS_GLO, nav);
	}

	return 0;
}