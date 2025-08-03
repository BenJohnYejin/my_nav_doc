#include "rtklib.h"
#include "rtkport.h"

extern XYRTK_Callbacks* cb;

#define NINCOBSnew     144
#define TRACE
#if defined(WIN32)||defined(LINUX)
#if defined(TRACE)
static FILE* fp_trace = NULL;     /* file pointer of trace */
static char file_trace[1024];   /* trace file */

#endif
#endif
//* constants -----------------------------------------------------------------*/
static const double gpst0[] = { 1980,1, 6,0,0,0 }; /* gps time reference */
static const double gst0[] = { 1999,8,22,0,0,0 }; /* galileo system time reference */
static const double bdt0[] = { 2006,1, 1,0,0,0 }; /* beidou time reference */

static double leaps[MAXLEAPS + 1][7] = { /* leap seconds (y,m,d,h,m,s,utc-gpst) */  //当有新增跳秒时注意增大MAXLEAPS
	{2017,1,1,0,0,0,-18},
	{2015,7,1,0,0,0,-17},
	{2012,7,1,0,0,0,-16},
	{2009,1,1,0,0,0,-15},
	{2006,1,1,0,0,0,-14},
	{1999,1,1,0,0,0,-13},
	{1997,7,1,0,0,0,-12},
	{1996,1,1,0,0,0,-11},
	{1994,7,1,0,0,0,-10},
	{1993,7,1,0,0,0, -9},
	{1992,7,1,0,0,0, -8},
	{1991,1,1,0,0,0, -7},
	{1990,1,1,0,0,0, -6},
	{1988,1,1,0,0,0, -5},
	{1985,7,1,0,0,0, -4},
	{1983,7,1,0,0,0, -3},
	{1982,7,1,0,0,0, -2},
	{1981,7,1,0,0,0, -1},
	{0}
};
# if 0
const double chisqr[100] = {      /* chi-sqr(n) (alpha=0.001) */
	10.8,13.8,16.3,18.5,20.5,22.5,24.3,26.1,27.9,29.6,
   31.3,32.9,34.5,36.1,37.7,39.3,40.8,42.3,43.8,45.3,
	46.8,48.3,49.7,51.2,52.6,54.1,55.5,56.9,58.3,59.7,
	61.1,62.5,63.9,65.2,66.6,68.0,69.3,70.7,72.1,73.4,
	74.7,76.0,77.3,78.6,80.0,81.3,82.6,84.0,85.4,86.7,
   88.0,89.3,90.6,91.9,93.3,94.7,96.0,97.4,98.7,100 ,
	101 ,102 ,103 ,104 ,105 ,107 ,108 ,109 ,110 ,112 ,
	113 ,114 ,115 ,116 ,118 ,119 ,120 ,122 ,123 ,125 ,
	126 ,127 ,128 ,129 ,131 ,132 ,133 ,134 ,135 ,137 ,
	138 ,139 ,140 ,142 ,143 ,144 ,145 ,147 ,148 ,149
};
#endif
const double lam_carr[MAXFREQ] = { /* carrier wave length (m) */
	CLIGHT / FREQ1,CLIGHT / FREQ2,CLIGHT / FREQ5,CLIGHT / FREQ6,CLIGHT / FREQ7,
	CLIGHT / FREQ8,CLIGHT / FREQ9
};
static char* obscodes[] = {       /* observation code strings */

	""  ,"1C","1P","1W","1Y", "1M","1N","1S","1L","1E", /*  0- 9 */
	"1A","1B","1X","1Z","2C", "2D","2S","2L","2X","2P", /* 10-19 */
	"2W","2Y","2M","2N","5I", "5Q","5X","7I","7Q","7X", /* 20-29 */
	"6A","6B","6C","6X","6Z", "6S","6L","8L","8Q","8X", /* 30-39 */
	"2I","2Q","6I","6Q","3I", "3Q","3X","1I","1Q","5A", /* 40-49 */
	"5B","5C","9A","9B","9C", "9X",""  ,""  ,""  ,""    /* 50-59 */
};

static unsigned char obsfreqs[] = {
	/* 1:L1/E1, 2:L2/B1, 3:L5/E5a/L3/B2a, 4:L6/LEX/B3, 5:E5b/B2, 6:E5(a+b), 7:S */
	0, 1, 1, 1, 1,  1, 1, 1, 1, 1, /*  0- 9 */
	1, 1, 1, 1, 2,  2, 2, 2, 2, 2, /* 10-19 */
	2, 2, 2, 2, 3,  3, 3, 5, 5, 5, /* 20-29 */
	4, 4, 4, 4, 4,  4, 4, 6, 6, 6, /* 30-39 */
	2, 2, 4, 4, 3,  3, 3, 1, 1, 3, /* 40-49 */
	3, 3, 7, 7, 7,  7, 0, 0, 0, 0  /* 50-59 */
};
static char codepris[7][MAXFREQ][16] = {  /* code priority table */

	/* L1/E1      L2/B1     L5/E5a/L3/B2a   L6/LEX/B3  E5b/B2    E5(a+b)  S */
	 {"CPYWMNSL","PYWCMNDSLX","IQX"       ,""       ,""       ,""      ,""    }, /* GPS */
	 {"PC"      ,"PC"        ,"IQX"       ,""       ,""       ,""      ,""    }, /* GLO */
	 {"CABXZ"   ,""          ,"IQX"       ,"ABCXZ"  ,"IQX"    ,"IQX"   ,""    }, /* GAL */
	 {"CSLXZ"   ,"SLX"       ,"IQX"       ,"SLX"    ,""       ,""      ,""    }, /* QZS */
	 {"C"       ,""          ,"IQX"       ,""       ,""       ,""      ,""    }, /* SBS */
	 {"IBQX"    ,"IQX"       ,"IQX"       ,"IQX"    ,"IQX"    ,""      ,""    }, /* BDS */
	 {""        ,""          ,"ABCX"      ,""       ,""       ,""      ,"ABCX"}  /* IRN */
};

static const unsigned int tbl_CRC24Q[] = {
	0x000000,0x864CFB,0x8AD50D,0x0C99F6,0x93E6E1,0x15AA1A,0x1933EC,0x9F7F17,
	0xA18139,0x27CDC2,0x2B5434,0xAD18CF,0x3267D8,0xB42B23,0xB8B2D5,0x3EFE2E,
	0xC54E89,0x430272,0x4F9B84,0xC9D77F,0x56A868,0xD0E493,0xDC7D65,0x5A319E,
	0x64CFB0,0xE2834B,0xEE1ABD,0x685646,0xF72951,0x7165AA,0x7DFC5C,0xFBB0A7,
	0x0CD1E9,0x8A9D12,0x8604E4,0x00481F,0x9F3708,0x197BF3,0x15E205,0x93AEFE,
	0xAD50D0,0x2B1C2B,0x2785DD,0xA1C926,0x3EB631,0xB8FACA,0xB4633C,0x322FC7,
	0xC99F60,0x4FD39B,0x434A6D,0xC50696,0x5A7981,0xDC357A,0xD0AC8C,0x56E077,
	0x681E59,0xEE52A2,0xE2CB54,0x6487AF,0xFBF8B8,0x7DB443,0x712DB5,0xF7614E,
	0x19A3D2,0x9FEF29,0x9376DF,0x153A24,0x8A4533,0x0C09C8,0x00903E,0x86DCC5,
	0xB822EB,0x3E6E10,0x32F7E6,0xB4BB1D,0x2BC40A,0xAD88F1,0xA11107,0x275DFC,
	0xDCED5B,0x5AA1A0,0x563856,0xD074AD,0x4F0BBA,0xC94741,0xC5DEB7,0x43924C,
	0x7D6C62,0xFB2099,0xF7B96F,0x71F594,0xEE8A83,0x68C678,0x645F8E,0xE21375,
	0x15723B,0x933EC0,0x9FA736,0x19EBCD,0x8694DA,0x00D821,0x0C41D7,0x8A0D2C,
	0xB4F302,0x32BFF9,0x3E260F,0xB86AF4,0x2715E3,0xA15918,0xADC0EE,0x2B8C15,
	0xD03CB2,0x567049,0x5AE9BF,0xDCA544,0x43DA53,0xC596A8,0xC90F5E,0x4F43A5,
	0x71BD8B,0xF7F170,0xFB6886,0x7D247D,0xE25B6A,0x641791,0x688E67,0xEEC29C,
	0x3347A4,0xB50B5F,0xB992A9,0x3FDE52,0xA0A145,0x26EDBE,0x2A7448,0xAC38B3,
	0x92C69D,0x148A66,0x181390,0x9E5F6B,0x01207C,0x876C87,0x8BF571,0x0DB98A,
	0xF6092D,0x7045D6,0x7CDC20,0xFA90DB,0x65EFCC,0xE3A337,0xEF3AC1,0x69763A,
	0x578814,0xD1C4EF,0xDD5D19,0x5B11E2,0xC46EF5,0x42220E,0x4EBBF8,0xC8F703,
	0x3F964D,0xB9DAB6,0xB54340,0x330FBB,0xAC70AC,0x2A3C57,0x26A5A1,0xA0E95A,
	0x9E1774,0x185B8F,0x14C279,0x928E82,0x0DF195,0x8BBD6E,0x872498,0x016863,
	0xFAD8C4,0x7C943F,0x700DC9,0xF64132,0x693E25,0xEF72DE,0xE3EB28,0x65A7D3,
	0x5B59FD,0xDD1506,0xD18CF0,0x57C00B,0xC8BF1C,0x4EF3E7,0x426A11,0xC426EA,
	0x2AE476,0xACA88D,0xA0317B,0x267D80,0xB90297,0x3F4E6C,0x33D79A,0xB59B61,
	0x8B654F,0x0D29B4,0x01B042,0x87FCB9,0x1883AE,0x9ECF55,0x9256A3,0x141A58,
	0xEFAAFF,0x69E604,0x657FF2,0xE33309,0x7C4C1E,0xFA00E5,0xF69913,0x70D5E8,
	0x4E2BC6,0xC8673D,0xC4FECB,0x42B230,0xDDCD27,0x5B81DC,0x57182A,0xD154D1,
	0x26359F,0xA07964,0xACE092,0x2AAC69,0xB5D37E,0x339F85,0x3F0673,0xB94A88,
	0x87B4A6,0x01F85D,0x0D61AB,0x8B2D50,0x145247,0x921EBC,0x9E874A,0x18CBB1,
	0xE37B16,0x6537ED,0x69AE1B,0xEFE2E0,0x709DF7,0xF6D10C,0xFA48FA,0x7C0401,
	0x42FA2F,0xC4B6D4,0xC82F22,0x4E63D9,0xD11CCE,0x575035,0x5BC9C3,0xDD8538
};

/* satellite system+prn/slot number to satellite number ------------------------
* convert satellite system+prn/slot number to satellite number
* args   : int    sys       I   satellite system (SYS_GPS,SYS_GLO,...)
*          int    prn       I   satellite prn/slot number
* return : satellite number (0:error)
*-----------------------------------------------------------------------------*/
extern int satno(int sys, int prn)
{
	if (prn <= 0) return 0;
	switch (sys) {
	case SYS_GPS:
		if (prn < MINPRNGPS || MAXPRNGPS < prn) return 0;
		return prn - MINPRNGPS + 1;
	case SYS_GLO:
		if (prn < MINPRNGLO || MAXPRNGLO < prn) return 0;
		return NSATGPS + prn - MINPRNGLO + 1;
	case SYS_CMP://CMP
		if (prn < MINPRNCMP || MAXPRNCMP < prn) return 0;
		return NSATGPS + NSATGLO + prn - MINPRNCMP + 1;
	case SYS_GAL://GAL
		if (prn < MINPRNGAL || MAXPRNGAL < prn) return 0;
		return NSATGPS + NSATGLO + NSATCMP + prn - MINPRNGAL + 1;
	case SYS_QZS://QZSS
		if (prn < MINPRNQZS || MAXPRNQZS < prn) return 0;
		return NSATGPS + NSATGLO + NSATCMP + NSATGAL + prn - MINPRNQZS + 1;
	}
	return 0;
}
//原本的函数
extern int satno1(int sys, int prn)
{
	if (prn <= 0) return 0;
	switch (sys) {
	case SYS_GPS:
		if (prn < MINPRNGPS || MAXPRNGPS < prn) return 0;
		return prn - MINPRNGPS + 1;
	case SYS_GLO:
		if (prn < MINPRNGLO || MAXPRNGLO < prn) return 0;
		return NSATGPS + prn - MINPRNGLO + 1;
	case SYS_GAL:
		if (prn < MINPRNGAL || MAXPRNGAL < prn) return 0;
		return NSATGPS + NSATGLO + prn - MINPRNGAL + 1;
	case SYS_QZS:
		if (prn < MINPRNQZS || MAXPRNQZS < prn) return 0;
		return NSATGPS + NSATGLO + NSATGAL + prn - MINPRNQZS + 1;
	case SYS_CMP:
		if (prn < MINPRNCMP || MAXPRNCMP < prn) return 0;
		return NSATGPS + NSATGLO + NSATGAL + NSATQZS + prn - MINPRNCMP + 1;
	}
	return 0;
}
/* satellite number to satellite system ----------------------------------------
* convert satellite number to satellite system
* args   : int    sat       I   satellite number (1-MAXSAT)
*          int    *prn      IO  satellite prn/slot number (NULL: no output)
* return : satellite system (SYS_GPS,SYS_GLO,...)
*-----------------------------------------------------------------------------*/
extern int satsys(int sat, int* prn)
{
	//sysstem list: gps glo bds gal qzs
	int sys = SYS_NONE;
	if (sat <= 0 || MAXSAT < sat) sat = 0;
	else if (sat <= NSATGPS) {
		sys = SYS_GPS; sat += MINPRNGPS - 1;
	}
	else if ((sat -= NSATGPS) <= NSATGLO) {
		sys = SYS_GLO; sat += MINPRNGLO - 1;
	}
	else if ((sat -= NSATGLO) <= NSATCMP) {
		sys = SYS_CMP; sat += MINPRNCMP - 1;
	}
	else if ((sat -= NSATCMP) <= NSATGAL) {
		sys = SYS_GAL; sat += MINPRNGAL - 1;
	}

	else sat = 0;
	if (prn) *prn = sat;
	return sys;
}
//原本的函数
extern int satsys1(int sat, int* prn)
{
	int sys = SYS_NONE;
	if (sat <= 0 || MAXSAT < sat) sat = 0;
	else if (sat <= NSATGPS) {
		sys = SYS_GPS; sat += MINPRNGPS - 1;
	}
	else if ((sat -= NSATGPS) <= NSATGLO) {
		sys = SYS_GLO; sat += MINPRNGLO - 1;
	}
	else if ((sat -= NSATGLO) <= NSATGAL) {
		sys = SYS_GAL; sat += MINPRNGAL - 1;
	}
	else if ((sat -= NSATGAL) <= NSATQZS) {
		sys = SYS_QZS; sat += MINPRNQZS - 1;
	}
	else if ((sat -= NSATQZS) <= NSATCMP) {
		sys = SYS_CMP; sat += MINPRNCMP - 1;
	}
	else sat = 0;
	if (prn) *prn = sat;
	return sys;
}
/* satellite id to satellite number --------------------------------------------
* convert satellite id to satellite number
* args   : char   *id       I   satellite id (nn,Gnn,Rnn,Enn,Jnn,Cnn,Inn or Snn)
* return : satellite number (0: error)
* notes  : 120-142 and 193-199 are also recognized as sbas and qzss
*-----------------------------------------------------------------------------*/
extern int satid2no(const char* id)
{
	int sys = 0, prn = 0;
	char code = "";

	if (sscanf(id, "%d", &prn) == 1) {
		if (MINPRNGPS <= prn && prn <= MAXPRNGPS) sys = SYS_GPS;
		else return 0;
		return satno(sys, prn);
	}
	if (sscanf(id, "%c%d", &code, &prn) < 2) return 0;

	switch (code) {
	case 'G': sys = SYS_GPS; prn += MINPRNGPS - 1; break;
	case 'R': sys = SYS_GLO; prn += MINPRNGLO - 1; break;
	case 'C': sys = SYS_CMP; prn += MINPRNCMP - 1; break;
	case 'E': sys = SYS_GAL; prn += MINPRNGAL - 1; break;
	case 'J': sys = SYS_QZS; prn += MINPRNQZS - 1; break;

	default: return 0;
	}
	return satno(sys, prn);
}

extern void satid2sys(const char* id, int* sys)
{
	int prn = 0;
	char code = "";

	if (sscanf(id, "%d", &prn) == 1) {
		if (MINPRNGPS <= prn && prn <= MAXPRNGPS) *sys = SYS_GPS;
		else if (MINPRNSBS <= prn && prn <= MAXPRNSBS) *sys = SYS_SBS;
		else if (MINPRNQZS <= prn && prn <= MAXPRNQZS) *sys = SYS_QZS;
	}

	if (sscanf(id, "%c%d", &code, &prn) < 2) *sys = 0;
	switch (code) {
	case 'G': *sys = SYS_GPS; break;
	case 'R': *sys = SYS_GLO; break;
	case 'E': *sys = SYS_GAL; break;
	case 'J': *sys = SYS_QZS; break;
	case 'C': *sys = SYS_CMP; break;
	}
}
/* satellite number to satellite id --------------------------------------------
* convert satellite number to satellite id
* args   : int    sat       I   satellite number
*          char   *id       O   satellite id (Gnn,Rnn,Enn,Jnn,Cnn,Inn or nnn)
* return : none
*-----------------------------------------------------------------------------*/
extern void satno2id(int sat, char* id)
{
	int prn = 0;
	switch (satsys(sat, &prn)) {
	case SYS_GPS: sprintf(id, "G%02d", prn - MINPRNGPS + 1); return;
	case SYS_GLO: sprintf(id, "R%02d", prn - MINPRNGLO + 1); return;
	case SYS_GAL: sprintf(id, "E%02d", prn - MINPRNGAL + 1); return;
	case SYS_QZS: sprintf(id, "J%02d", prn - MINPRNQZS + 1); return;
	case SYS_CMP: sprintf(id, "C%02d", prn - MINPRNCMP + 1); return;
	}
	strcpy(id, "");
}
/* test excluded satellite -----------------------------------------------------
* test excluded satellite
* args   : int    sat       I   satellite number
*          int    svh       I   sv health flag
*          prcopt_t *opt    I   processing options (NULL: not used)
* return : status (1:excluded,0:not excluded)
*-----------------------------------------------------------------------------*/
extern int satexclude(int sat, int svh, const prcopt_t* opt)
{
	int sys = satsys(sat, NULL);
	if (svh < 0) return 1; /* ephemeris unavailable */
	if (opt) {
		if (!(sys & opt->navsys)) return 1; /* unselected sat sys */
	}
	if (sys == SYS_QZS) svh &= 0xFE; /* mask QZSS LEX health */
	if (svh) {
		trace(3, "unhealthy satellite: sat=%3d svh=%02X\r\n", sat, svh);
		return 1;
	}
	return 0;
}
/* test SNR mask ---------------------------------------------------------------
* test SNR mask
* args   : int    base      I   rover or base-station (0:rover,1:base station)
*          int    freq      I   frequency (0:L1,1:L2,2:L3,...)
*          double el        I   elevation angle (rad)
*          double snr       I   C/N0 (dBHz)
*          snrmask_t *mask  I   SNR mask
*		   
* return : status (1:masked,0:unmasked)
*-----------------------------------------------------------------------------*/
extern int testsnr(int base, int freq, double el, double snr, const int flag, const float snrthres)
{
	//不对基站做SNR筛选
	if (base == 1) return 0;
#if 0
	//根据高度角阶梯型设置信噪比阈值，5度带
	double minsnr, a;
	int i;

	if (!mask->ena[base] || freq < 0 || freq >= NFREQ) return 0;

	a = (el * R2D + 5.0) / 10.0;
	i = (int)floor(a); a -= i;
	if (i < 1) minsnr = mask->mask[freq][0];
	else if (i > 8) minsnr = mask->mask[freq][8];
	else minsnr = (1.0 - a) * mask->mask[freq][i - 1] + a * mask->mask[freq][i];

	return snr < minsnr;
#endif
//信噪比固定阈值
	if (!flag)
		return 0;
	else {
		return snr <= (double)snrthres;
	}
}
/* obs type string to obs code -------------------------------------------------
* convert obs code type string to obs code
* args   : char   *str   I      obs code string ("1C","1P","1Y",...)
*          int    *freq  IO     frequency (1:L1,2:L2,3:L5,4:L6,5:L7,6:L8,0:err)
*                               (NULL: no output)
* return : obs code (CODE_???)
* notes  : obs codes are based on reference [6] and qzss extension
*-----------------------------------------------------------------------------*/
extern unsigned char obs2code(const char* obs, int* freq)
{
	int i = 0;

	if (freq) *freq = 0;
	for (i = 1; *obscodes[i]; i++) {
		if (strcmp(obscodes[i], obs)) continue;
		if (freq) *freq = obsfreqs[i];
		return (unsigned char)i;
	}
	return CODE_NONE;
}
/* obs code to obs code string -------------------------------------------------
* convert obs code to obs code string
* args   : unsigned char code I obs code (CODE_???)
*          int    *freq  IO     frequency (NULL: no output)
*                               (1:L1/E1, 2:L2/B1, 3:L5/E5a/L3, 4:L6/LEX/B3,
								 5:E5b/B2, 6:E5(a+b), 7:S)
* return : obs code string ("1C","1P","1P",...)
* notes  : obs codes are based on reference [6] and qzss extension
*-----------------------------------------------------------------------------*/
extern char* code2obs(unsigned char code, int* freq)
{
	if (freq) *freq = 0;
	if (code <= CODE_NONE || MAXCODE < code) return "";
	if (freq) *freq = obsfreqs[code];
	return obscodes[code];
}

/* get code priority -----------------------------------------------------------
* get code priority for multiple codes in a frequency
* args   : int    sys     I     system (SYS_???)
*          unsigned char code I obs code (CODE_???)
*          char   *opt    I     code options (NULL:no option)
* return : priority (15:highest-1:lowest,0:error)
*-----------------------------------------------------------------------------*/
extern int getcodepri(int sys, unsigned char code, const char* opt)
{
	const char* p = NULL, * optstr = NULL;
	char* obs = NULL, str[8] = "";
	int i = 0, j = 0;

	switch (sys) {
	case SYS_GPS: i = 0; optstr = "-GL%2s"; break;
	case SYS_GLO: i = 1; optstr = "-RL%2s"; break;
	case SYS_CMP: i = 5; optstr = "-CL%2s"; break;
	case SYS_GAL: i = 2; optstr = "-EL%2s"; break;
	case SYS_QZS: i = 3; optstr = "-JL%2s"; break;
	default: return 0;
	}
	obs = code2obs(code, &j);

	for (p = opt; p && (p = strchr(p, '-')); p++) {
		if (sscanf(p, optstr, str) < 1 || str[0] != obs[0]) continue;
		return str[1] == obs[1] ? 15 : 0;
	}

	return (p = strchr(codepris[i][j - 1], obs[1])) ? 14 - (int)(p - codepris[i][j - 1]) : 0;
}
/* extract unsigned/signed bits ------------------------------------------------
* extract unsigned/signed bits from byte data
* args   : unsigned char *buff I byte data
*          int    pos    I      bit position from start of data (bits)
*          int    len    I      bit length (bits) (len<=32)
* return : extracted unsigned/signed bits
*-----------------------------------------------------------------------------*/
extern unsigned int getbitu(const unsigned char* buff, int pos, int len)
{
	unsigned int bits = 0;
	int i = 0;
	for (i = pos; i < pos + len; i++) bits = (bits << 1) + ((buff[i / 8] >> (7 - i % 8)) & 1u);
	return bits;
}
extern int getbits(const unsigned char* buff, int pos, int len)
{
	unsigned int bits = getbitu(buff, pos, len);
	if (len <= 0 || 32 <= len || !(bits & (1u << (len - 1)))) return (int)bits;
	return (int)(bits | (~0u << len)); /* extend sign */
}

/* new matrix ------------------------------------------------------------------
* allocate memory of matrix
* args   : int    n,m       I   number of rows and columns of matrix
* return : matrix pointer (if n<=0 or m<=0, return NULL)
*-----------------------------------------------------------------------------*/
extern double* mat(int n, int m)
{
	double* p = NULL;

#if defined(WIN32) || defined(LINUX)
	if (!(p = (double*)calloc(sizeof(double), n * m))) {
#else
	if (!(p = (double*)xy_malloc(sizeof(double) * n * m))) {
#endif
		trace(3, "matrix memory allocation error: n=%d,m=%d\r\n", n, m);
	}

	return p;
}
/* new integer matrix ----------------------------------------------------------
* allocate memory of integer matrix
* args   : int    n,m       I   number of rows and columns of matrix
* return : matrix pointer (if n<=0 or m<=0, return NULL)
*-----------------------------------------------------------------------------*/
extern int* imat(int n, int m)
{
	int* p = NULL;

	if (n <= 0 || m <= 0)return NULL;
#if defined(WIN32) || defined(LINUX)
	if (!(p = (int*)calloc(sizeof(int), n * m))) {
#else
	if (!(p = (int*)xy_malloc(sizeof(int) * n * m))) {
#endif
		trace(2, "integer matrix memory allocation error: n=%d,m=%d\r\n", n, m);
	}

	return p;
}
/* zero matrix -----------------------------------------------------------------
* generate new zero matrix
* args   : int    n,m       I   number of rows and columns of matrix
* return : matrix pointer (if n<=0 or m<=0, return NULL)
*-----------------------------------------------------------------------------*/
extern double* zeros(int n, int m)
{
	double* p = NULL;
#if defined(WIN32) || defined(LINUX)
	if (n <= 0 || m <= 0) return NULL;
	if (!(p = (double*)calloc(sizeof(double), n * m))) {
		//if (!(p = (double*)malloc(sizeof(double)*n * m))) {   用malloc需要注意初始化，calloc已经自动初始化
			//fatalerr("matrix memory allocation error: n=%d,m=%d\n",n,m);
	}
#else
	if ((p = mat(n, m))) for (n = n * m - 1; n >= 0; n--) p[n] = 0.0;
#endif
	return p;
}

/* identity matrix -------------------------------------------------------------
* generate new identity matrix
* args   : int    n         I   number of rows and columns of matrix
* return : matrix pointer (if n<=0, return NULL)
*-----------------------------------------------------------------------------*/
extern double* eye(int n)
{
	double* p = NULL;
	int i = 0;

	if ((p = zeros(n, n))) for (i = 0; i < n; i++) p[i + i * n] = 1.0;
	return p;
}

extern tri_mat_t* trimat(int n, int m)
{
	tri_mat_t* p = NULL;

	if (!(p = (tri_mat_t*)xy_malloc(sizeof(tri_mat_t) * n * m))) {
		trace(3, "matrix memory allocation error: n=%d,m=%d\r\n", n, m);
	}

	return p;
}

/* inner product ---------------------------------------------------------------
* inner product of vectors
* args   : double *a,*b     I   vector a,b (n x 1)
*          int    n         I   size of vector a,b
* return : a'*b
*-----------------------------------------------------------------------------*/
extern double dot(const double* a, const double* b, int n)
{
	double c = 0.0;

	while (--n >= 0) c += a[n] * b[n];
	return c;
}
/* euclid norm -----------------------------------------------------------------
* euclid norm of vector
* args   : double *a        I   vector a (n x 1)
*          int    n         I   size of vector a
* return : || a ||
*-----------------------------------------------------------------------------*/
extern double norm(const double* a, int n)
{
	return sqrt(dot(a, a, n));
}

/* copy matrix -----------------------------------------------------------------
* copy matrix
* args   : double *A        O   destination matrix A (n x m)
*          double *B        I   source matrix B (n x m)
*          int    n,m       I   number of rows and columns of matrix
* return : none
*-----------------------------------------------------------------------------*/
extern void matcpy(double* A, const double* B, int n, int m)
{
	memcpy(A, B, sizeof(double) * n * m);
}
extern void imatcpy(int* A, const int* B, int n, int m)
{
	memcpy(A, B, sizeof(int) * n * m);
}
/* matrix routines -----------------------------------------------------------*/

#ifdef LAPACK /* with LAPACK/BLAS or MKL */

/* multiply matrix (wrapper of blas dgemm) -------------------------------------
* multiply matrix by matrix (C=alpha*A*B+beta*C)
* args   : char   *tr       I  transpose flags ("N":normal,"T":transpose)
*          int    n,k,m     I  size of (transposed) matrix A,B
*          double alpha     I  alpha
*          double *A,*B     I  (transposed) matrix A (n x m), B (m x k)
*          double beta      I  beta
*          double *C        IO matrix C (n x k)
* return : none
*-----------------------------------------------------------------------------*/
extern void matmul(const char* tr, int n, int k, int m, double alpha,
	const double* A, const double* B, double beta, double* C)
{
	int lda = tr[0] == 'T' ? m : n, ldb = tr[1] == 'T' ? k : m;

	dgemm_((char*)tr, (char*)tr + 1, &n, &k, &m, &alpha, (double*)A, &lda, (double*)B,
		&ldb, &beta, C, &n);
}
/* inverse of matrix -----------------------------------------------------------
* inverse of matrix (A=A^-1)
* args   : double *A        IO  matrix (n x n)
*          int    n         I   size of matrix A
* return : status (0:ok,0>:error)
*-----------------------------------------------------------------------------*/
extern int matinv(double* A, int n)
{
	double* work;
	int info, lwork = n * 16, * ipiv = imat(n, 1);

	work = mat(lwork, 1);
	dgetrf_(&n, &n, A, &n, ipiv, &info);
	if (!info) dgetri_(&n, A, &n, ipiv, work, &lwork, &info);
	xy_free(ipiv); xy_free(work);
	return info;
}
/* solve linear equation -------------------------------------------------------
* solve linear equation (X=A\Y or X=A'\Y)
* args   : char   *tr       I   transpose flag ("N":normal,"T":transpose)
*          double *A        I   input matrix A (n x n)
*          double *Y        I   input matrix Y (n x m)
*          int    n,m       I   size of matrix A,Y
*          double *X        O   X=A\Y or X=A'\Y (n x m)
* return : status (0:ok,0>:error)
* notes  : matirix stored by column-major order (fortran convention)
*          X can be same as Y
*-----------------------------------------------------------------------------*/
extern int solve(const char* tr, const double* A, const double* Y, int n,
	int m, double* X)
{
	double* B = mat(n, n);
	int info, * ipiv = imat(n, 1);

	matcpy(B, A, n, n);
	matcpy(X, Y, n, m);
	dgetrf_(&n, &n, B, &n, ipiv, &info);
	if (!info) dgetrs_((char*)tr, &n, &m, B, &n, ipiv, X, &n, &info);
	xy_free(ipiv); xy_free(B);
	return info;
}

#else /* without LAPACK/BLAS or MKL */

/* multiply matrix -----------------------------------------------------------*/
//原矩阵相乘，头文件中未加定义  原始定义
extern void matmul2(const char* tr, int n, int k, int m, double alpha,
	const double* a, const double* b, double beta, double* c)
{
	double d = 0;
	int i = 0, j = 0, x = 0, f = tr[0] == 'N' ? (tr[1] == 'N' ? 1 : 2) : (tr[1] == 'N' ? 3 : 4);

	for (i = 0; i < n; i++) for (j = 0; j < k; j++) {
		d = 0.0;
		switch (f) {
		case 1: for (x = 0; x < m; x++) d += a[i + x * n] * b[x + j * m]; break;
		case 2: for (x = 0; x < m; x++) d += a[i + x * n] * b[j + x * k]; break;
		case 3: for (x = 0; x < m; x++) d += a[x + i * m] * b[x + j * m]; break;
		case 4: for (x = 0; x < m; x++) d += a[x + i * m] * b[j + x * k]; break;
		}
		if (beta == 0.0) c[i + j * n] = alpha * d; else c[i + j * n] = alpha * d + beta * c[i + j * n];
	}
}
/*
* 
* T为需要转置、N为不需要转置
* n I a矩阵的行
* k I b矩阵的列
* m I a矩阵的列=b矩阵的行
* alpha I a*b的系数
* beta I c阵的系数
* c=alpha*a*b+beta*c
*/
extern void matmul(const char* tr, int n, int k, int m, double alpha,
	const double* a, const double* b, double beta, double* c)
{
	double d = 0;
	int i = 0, j = 0, x = 0, f = tr[0] == 'N' ? (tr[1] == 'N' ? 1 : 2) : (tr[1] == 'N' ? 3 : 4);

	if (beta == 0.0)
	{
		switch (f)
		{
		case 1:
		{
			for (i = 0; i < n; i++) for (j = 0; j < k; j++) {
				d = 0.0;
				for (x = 0; x < m; x++)
					d += a[i + x * n] * b[x + j * m];
				c[i + j * n] = alpha * d;
			}
			break;
		}
		case 2:
		{
			for (i = 0; i < n; i++) for (j = 0; j < k; j++) {
				d = 0.0;
				for (x = 0; x < m; x++)
					d += a[i + x * n] * b[j + x * k];
				c[i + j * n] = alpha * d;
			}
			break;
		}
		case 3:
		{
			for (i = 0; i < n; i++) for (j = 0; j < k; j++) {
				d = 0.0;
				for (x = 0; x < m; x++)
					d += a[x + i * m] * b[x + j * m];
				c[i + j * n] = alpha * d;
			}
			break;
		}
		case 4:
		{
			for (i = 0; i < n; i++) for (j = 0; j < k; j++) {
				d = 0.0;
				for (x = 0; x < m; x++)
					d += a[x + i * m] * b[j + x * k];
				c[i + j * n] = alpha * d;
			}
			break;
		}
		}
	}
	else {
		switch (f) {
		case 1:
		{
			for (i = 0; i < n; i++) for (j = 0; j < k; j++) {
				d = 0.0;
				for (x = 0; x < m; x++)
					d += a[i + x * n] * b[x + j * m];
				c[i + j * n] = alpha * d + beta * c[i + j * n];
			}
			break;
		}
		case 2:
		{
			for (i = 0; i < n; i++) for (j = 0; j < k; j++) {
				d = 0.0;
				for (x = 0; x < m; x++)
					d += a[i + x * n] * b[j + x * k];
				c[i + j * n] = alpha * d + beta * c[i + j * n];
			}
			break;
		}
		case 3:
		{
			for (i = 0; i < n; i++) for (j = 0; j < k; j++) {
				d = 0.0;
				for (x = 0; x < m; x++)
					d += a[x + i * m] * b[x + j * m];
				c[i + j * n] = alpha * d + beta * c[i + j * n];
			}
			break;
		}
		case 4:
		{
			for (i = 0; i < n; i++) for (j = 0; j < k; j++) {
				d = 0.0;
				for (x = 0; x < m; x++)
					d += a[x + i * m] * b[j + x * k];
				c[i + j * n] = alpha * d + beta * c[i + j * n];
			}
			break;
		}
		}
	}
}

void trimat2(tri_mat_t2 * tri, int n, int m, int flag)
{
	tri->tri = trimat(n, m);
	if (flag == 0)
	{
		tri->ijnonzerosnums = izeros(n, 1);
		tri->ijnonzerosindex = izeros(n, 1);
	}
	else
	{
		tri->ijnonzerosnums = izeros(m, 1);
		tri->ijnonzerosindex = izeros(m, 1);
	}
}

void trimat2free(tri_mat_t2 * tri)
{
	xy_free(tri->tri);
	xy_free(tri->ijnonzerosnums);
	xy_free(tri->ijnonzerosindex);
}

int getnonzerovalue(const double* a, int n, int m, tri_mat_t * at)
{
	int i = 0, j = 0;
	int k = 0;

	for (j = 0; j < m; j++)
	{
		for (i = 0; i < n; i++)
		{
			if (fabs(a[i + j * n]) < ZEROS_MIN)
				continue;
			at[k].i = i;
			at[k].j = j;
			at[k].value = a[i + j * n];
			k++;
		}
	}

	return k;
}

int getnonzerovalue_trans(const double* a, int n, int m, tri_mat_t * at)
{
	int i = 0, j = 0;
	int k = 0;

	for (j = 0; j < m; j++)
		for (i = 0; i < n; i++)
		{
			if (fabs(a[i + j * n]) < ZEROS_MIN)
				continue;
			at[k].i = j;
			at[k].j = i;
			at[k].value = a[i + j * n];
			k++;
		}

	return k;
}

int igetnonzerovalue(const double* a, int m, int k, tri_mat_t2 * at)
{
	int i = 0, j = 0, x = 0;
	int num = 0;

	for (x = 0; x < m; x++)
	{
		num = 0;
		at->ijnonzerosindex[x] = i;
		for (j = 0; j < k; j++)
		{
			if (fabs(a[x + j * m]) < ZEROS_MIN)
				continue;
			at->tri[i].i = x;
			at->tri[i].j = j;
			at->tri[i].value = a[x + j * m];
			num++;
			i++;
		}
		at->ijnonzerosnums[x] = i;
	}

	return i;
}

//稀疏矩阵取非零元素,包括对角矩阵 每列的非零元素
int jgetnonzerovalue(const double* a, int m, int k, tri_mat_t2 * at)
{
	int i = 0, j = 0, x = 0;
	int num = 0;

	for (j = 0; j < k; j++)
	{
		num = 0;
		at->ijnonzerosindex[j] = i;
		for (x = 0; x < m; x++)
		{
			if (fabs(a[x + j * m]) < ZEROS_MIN)
				continue;
			at->tri[i].i = j;
			at->tri[i].j = x;
			at->tri[i].value = a[x + j * m];
			num++;
			i++;
		}
		at->ijnonzerosnums[j] = i;
	}
	return i;
}

void ijsparsematmul11(const tri_mat_t * at, const tri_mat_t2 * bt, int n, int anum, int k, double alpha, double* c)
{
	int i = 0, j = 0, x = 0;

	for (i = 0; i < anum; i++) 
	{
		j = at[i].j; 
		for (x = bt->ijnonzerosindex[j]; x < bt->ijnonzerosnums[j]; x++)  //提取b矩阵中第j行非零元素
		{
			c[at[i].i + bt->tri[x].j * n] += at[i].value * (bt->tri[x].value);
		}
	}

	if (alpha != 1.0)
	{
		for (j = 0; j < k; j++)
		{
			for (i = 0; i < n; i++)
			{
				c[i + j * n] *= alpha;
			}
		}
	}
}

void ijsparsematmul12(const tri_mat_t * at, const tri_mat_t2 * bt, int n, int anum, int k, double alpha, double beta, double* c)
{
	int i = 0, j = 0, x = 0;

	double* d = NULL;

	d = zeros(n, k);

	for (i = 0; i < anum; i++) 
	{
		j = at[i].j;  //找到那一行
		for (x = bt->ijnonzerosindex[j]; x < bt->ijnonzerosnums[j]; x++)  //提取b矩阵中第j行非零元素
		{
			d[at[i].i + bt->tri[x].j * n] += at[i].value * (bt->tri[x].value);
		}
	}

	for (j = 0; j < k; j++)
	{
		for (i = 0; i < n; i++)
		{
			c[i + j * n] = beta * c[i + j * n] + alpha * d[i + j * n];
		}
	}
	xy_free(d);
}

//只考虑被乘矩阵的稀疏性
void sparsematmul_new11(int n, int m, int k, double alpha,
	const double* a, const double* b, double beta, double* c)
{
	int i = 0, j = 0, x = 0;
	double d = 0;

	tri_mat_t2 at;
	trimat2(&at, n, m, 0);

	jgetnonzerovalue(a, n, m, &at);

	for (i = 0; i < n; i++) for (j = 0; j < k; j++)   
	{
		d = 0.0;
		for (x = at.ijnonzerosindex[i]; x < at.ijnonzerosnums[i]; x++) 
		{
			d += (at.tri[x].value) * b[at.tri[x].i + j * m];
		}
		c[i + j * n] = alpha * d + beta * c[i + j * n];
	}
	trimat2free(&at);
}

//只考虑被乘矩阵的稀疏性
void sparsematmul_new12(int n, int m, int k, double alpha,
	const double* a, const double* b, double beta, double* c)
{
	int i = 0, j = 0, x = 0;
	double d = 0;

	tri_mat_t2 bt;
	trimat2(&bt, m, k, 1);

	jgetnonzerovalue(b, m, k, &bt);

	if (beta == 0.0)
	{
		for (j = 0; j < k; j++) for (i = 0; i < n; i++)  
		{
			d = 0.0;
			for (x = bt.ijnonzerosindex[j]; x < bt.ijnonzerosnums[j]; x++)  
			{
				d += a[i + (bt.tri[x].j) * n] * (bt.tri[x].value);
			}
			c[i + j * n] = alpha * d;
		}
	}
	else {
		for (j = 0; j < k; j++) for (i = 0; i < n; i++)  
		{
			d = 0.0;
			for (x = bt.ijnonzerosindex[j]; x < bt.ijnonzerosnums[j]; x++) 
			{
				d += a[i + (bt.tri[x].j) * n] * (bt.tri[x].value);
			}
			c[i + j * n] = alpha * d + beta * c[i + j * n];
		}
	}

	trimat2free(&bt);
}

//稀疏矩阵取非零元素,包括对角矩阵
int jgetnonzerovalue22(const double* b, int m, int k, tri_mat_t2 * bt)
{
	int i = 0, j = 0, x = 0;
	int num = 0;

	for (j = 0; j < k; j++)
	{
		num = 0;
		bt->ijnonzerosindex[j] = i;
		for (x = 0; x < m; x++)
		{
			if (fabs(b[j + x * k]) < ZEROS_MIN)
				continue;
			bt->tri[i].i = j;
			bt->tri[i].j = x;
			bt->tri[i].value = b[j + x * k];
			num++;
			i++;
		}
		bt->ijnonzerosnums[j] = i;
	}
	return i;
}

//只考虑被乘矩阵的稀疏性
void sparsematmul_new22(int n, int m, int k, double alpha,
	const double* a, const double* b, double beta, double* c)
{
	int i = 0, j = 0, x = 0;
	double d = 0;

	tri_mat_t2 bt;
	trimat2(&bt, m, k, 1);

	jgetnonzerovalue22(b, m, k, &bt);

	if (beta == 0.0)
	{
		for (j = 0; j < k; j++) for (i = 0; i < n; i++)
		{
			d = 0.0;
			for (x = bt.ijnonzerosindex[j]; x < bt.ijnonzerosnums[j]; x++)
			{
				d += a[i + (bt.tri[x].j) * n] * (bt.tri[x].value);
			}
			c[i + j * n] = alpha * d;
		}
	}
	else {
		for (j = 0; j < k; j++) for (i = 0; i < n; i++)
		{
			d = 0.0;
			for (x = bt.ijnonzerosindex[j]; x < bt.ijnonzerosnums[j]; x++)  
			{
				d += a[i + (bt.tri[x].j) * n] * (bt.tri[x].value);
			}
			c[i + j * n] = alpha * d + beta * c[i + j * n];
		}
	}

	trimat2free(&bt);
}

//稀疏矩阵取非零元素,包括对角矩阵
int getnonzerovalue31(const double* a, int n, int m, tri_mat_t2 * at)
{
	int i = 0, j = 0, x = 0;

	for (i = 0; i < n; i++)
	{
		at->ijnonzerosindex[i] = j;
		for (x = 0; x < m; x++)
		{
			if (fabs(a[x + i * m]) < ZEROS_MIN)
				continue;
			at->tri[j].i = x;
			at->tri[j].j = i;
			at->tri[j].value = a[x + i * m];
			j++;
		}
		at->ijnonzerosnums[i] = j;
	}

	return j;
}

//只考虑被乘矩阵的稀疏性
void sparsematmul_new31(int n, int m, int k, double alpha,
	const double* a, const double* b, double beta, double* c)
{
	int i = 0, j = 0, x = 0;
	double d = 0;

	tri_mat_t2 at;
	trimat2(&at, n, m, 0);

	getnonzerovalue31(a, n, m, &at);

	if (beta == 0.0)
	{
		for (j = 0; j < k; j++) for (i = 0; i < n; i++)   
		{
			d = 0.0;
			for (x = at.ijnonzerosindex[i]; x < at.ijnonzerosnums[i]; x++)  
			{
				d += (at.tri[x].value) * b[at.tri[x].i + j * m];
			}
			c[i + j * n] = alpha * d;
		}
	}
	else {
		for (j = 0; j < k; j++) for (i = 0; i < n; i++)   
		{
			d = 0.0;
			for (x = at.ijnonzerosindex[i]; x < at.ijnonzerosnums[i]; x++) 
			{
				d += (at.tri[x].value) * b[at.tri[x].i + j * m];
			}
			c[i + j * n] = alpha * d + beta * c[i + j * n];
		}
	}

	trimat2free(&at);
}

void sparsematmul_new32(int n, int m, int k, double alpha,
	const double* a, const double* b, double beta, double* c)
{
	int i = 0, j = 0, x = 0;
	double d = 0;

	tri_mat_t2 bt;
	trimat2(&bt, m, k, 1);

	jgetnonzerovalue(b, m, k, &bt);

	for (j = 0; j < k; j++) for (i = 0; i < n; i++) 
	{
		d = 0.0;
		for (x = bt.ijnonzerosindex[j]; x < bt.ijnonzerosnums[j]; x++)  
		{
			d += a[i + (bt.tri[x].j) * n] * (bt.tri[x].value);
		}
		c[i + j * n] = alpha * d;
	}
	trimat2free(&bt);
}

void matmul_spar2(int flag, const char* tr, int n, int k, int m, double alpha,
	const double* a, const double* b, double beta, double* c)
{
	int f = tr[0] == 'N' ? (tr[1] == 'N' ? 1 : 2) : (tr[1] == 'N' ? 3 : 4);

	//矩阵转置及稀疏处理
	switch (f) {
	case 1:
		if (flag) sparsematmul_new11(n, m, k, alpha, a, b, beta, c);   //由于暂未用到，未验证,未完善
		else sparsematmul_new12(n, m, k, alpha, a, b, beta, c);  //1  已验证
		break;
	case 2:
		if (flag) sparsematmul_new11(n, m, k, alpha, a, b, beta, c);   //由于暂未用到未验证,未完善
		else sparsematmul_new22(n, m, k, alpha, a, b, beta, c);  //3    已验证
		break;
	case 3:
		if (flag) sparsematmul_new31(n, m, k, alpha, a, b, beta, c);  //2  已验证
		else sparsematmul_new32(n, m, k, alpha, a, b, beta, c);      //由于暂未用到，未验证,未完善
		break;
	case 4:

		break;
	}
}

//sparse multiply matrix  稀疏矩阵乘稀疏矩阵  考虑了两个矩阵的稀疏性
void matmul_spar(const char* tr, int n, int k, int m, double alpha,
	const double* a, const double* b, double beta, double* c)
{
	int f = tr[0] == 'N' ? (tr[1] == 'N' ? 1 : 2) : (tr[1] == 'N' ? 3 : 4);
	int anum = 0, bnum = 0;

	//将a、b矩阵转成三元组
	tri_mat_t* at;
	tri_mat_t2 bt;
	at = trimat(n, m);
	trimat2(&bt, m, k, 0);

	//矩阵转置及稀疏处理
	switch (f) {
	case 1:anum = getnonzerovalue(a, n, m, at);      bnum = igetnonzerovalue(b, m, k, &bt); break;
	case 2:anum = getnonzerovalue(a, n, m, at);      bnum = jgetnonzerovalue(b, k, m, &bt); break;
	case 3:anum = getnonzerovalue_trans(a, m, n, at); bnum = igetnonzerovalue(b, m, k, &bt); break;
	case 4:anum = getnonzerovalue_trans(a, m, n, at); bnum = jgetnonzerovalue(b, k, m, &bt); break;
	}

	//trace(5,"n=%d m=%d k=%d n*m=%d m*k=%d mkn=%d anum=%d bnum=%d\n",n,m,k,n*m,m*k,n*m*k,anum,bnum);
	//稀疏矩阵相乘
	if (beta == 0.0)
	{
		memset(c, 0, sizeof(double) * n * k);
		ijsparsematmul11(at, &bt, n, anum, k, alpha, c);
	}
	else {
		ijsparsematmul12(at, &bt, n, anum, k, alpha, beta, c);
	}

	xy_free(at);
	trimat2free(&bt);
}

extern void matmul_new(int flag, const char* tr, int n, int k, int m, double alpha,
	const double* a, const double* b, double beta, double* c)
{
	if (flag == 3)  //新的稀疏矩阵相乘 两个都是稀疏矩阵
		matmul_spar(tr, n, k, m, alpha, a, b, beta, c);
	else if (flag == 5)  //新的稀疏矩阵相乘    普通矩阵*稀疏矩阵   当前优先采用仅考虑一个矩阵的稀疏性，特殊情况可考虑两个矩阵的稀疏性
		matmul_spar2(0, tr, n, k, m, alpha, a, b, beta, c);
	else if (flag == 6)  //新的稀疏矩阵相乘    稀疏矩阵*普通矩阵
		matmul_spar2(1, tr, n, k, m, alpha, a, b, beta, c);
}

/* LU decomposition ----------------------------------------------------------*/
static int ludcmp(double* A, int n, int* indx, double* d)
{
	double big = 0, s = 0, tmp = 0, * vv = mat(n, 1);
	int i = 0, imax = 0, j = 0, k = 0;

	*d = 1.0;
	for (i = 0; i < n; i++) {
		big = 0.0; for (j = 0; j < n; j++) if ((tmp = fabs(A[i + j * n])) > big) big = tmp;
		if (big > 0.0) vv[i] = 1.0 / big; else { xy_free(vv); return -1; }
	}
	for (j = 0; j < n; j++) {
		for (i = 0; i < j; i++) {
			s = A[i + j * n]; for (k = 0; k < i; k++) s -= A[i + k * n] * A[k + j * n]; A[i + j * n] = s;
		}
		big = 0.0;
		for (i = j; i < n; i++) {
			s = A[i + j * n]; for (k = 0; k < j; k++) s -= A[i + k * n] * A[k + j * n]; A[i + j * n] = s;
			if ((tmp = vv[i] * fabs(s)) >= big) { big = tmp; imax = i; }
		}
		if (j != imax) {
			for (k = 0; k < n; k++) {
				tmp = A[imax + k * n]; A[imax + k * n] = A[j + k * n]; A[j + k * n] = tmp;
			}
			*d = -(*d); vv[imax] = vv[j];
		}
		indx[j] = imax;
		if (A[j + j * n] == 0.0) { xy_free(vv); return -1; }
		if (j != n - 1) {
			tmp = 1.0 / A[j + j * n]; for (i = j + 1; i < n; i++) A[i + j * n] *= tmp;
		}
	}
	xy_free(vv);
	return 0;
}
/* LU back-substitution ------------------------------------------------------*/
static void lubksb(const double* A, int n, const int* indx, double* b)
{
	double s = 0;
	int i = 0, ii = -1, ip = 0, j = 0;

	for (i = 0; i < n; i++) {
		ip = indx[i]; s = b[ip]; b[ip] = b[i];
		if (ii >= 0) for (j = ii; j < i; j++) s -= A[i + j * n] * b[j]; else if (s) ii = i;
		b[i] = s;
	}
	for (i = n - 1; i >= 0; i--) {
		s = b[i]; for (j = i + 1; j < n; j++) s -= A[i + j * n] * b[j]; b[i] = s / A[i + i * n];
	}
}
/* inverse of matrix ---------------------------------------------------------*/
extern int matinv(double* A, int n)
{
	double d = 0, * B = NULL;
	int i = 0, j = 0, * indx = NULL;
	indx = imat(n, 1); B = mat(n, n); matcpy(B, A, n, n);

	if (ludcmp(B, n, indx, &d)) { xy_free(indx); xy_free(B); return -1; }
	for (j = 0; j < n; j++) {
		for (i = 0; i < n; i++) A[i + j * n] = 0.0;
		A[j + j * n] = 1.0;
		lubksb(B, n, indx, A + j * n);
	}
	xy_free(indx); xy_free(B);
	return 0;
}
/* solve linear equation -----------------------------------------------------*/
extern int solve(const char* tr, const double* A, const double* Y, int n,
	int m, double* X)
{
	double* B = mat(n, n);
	int info = 0;

	matcpy(B, A, n, n);
	if (!(info = matinv(B, n))) matmul(tr[0] == 'N' ? "NN" : "TN", n, m, n, 1.0, B, Y, 0.0, X);
	xy_free(B);
	return info;
}
#endif
/* end of matrix routines ----------------------------------------------------*/

/* least square estimation -----------------------------------------------------
* least square estimation by solving normal equation (x=(A*A')^-1*A*y)
* args   : double *A        I   transpose of (weighted) design matrix (n x m)
*          double *y        I   (weighted) measurements (m x 1)
*          int    n,m       I   number of parameters and measurements (n<=m)
*          double *x        O   estmated parameters (n x 1)
*          double *Q        O   esimated parameters covariance matrix (n x n)
* return : status (0:ok,0>:error)
* notes  : for weighted least square, replace A and y by A*w and w*y (w=W^(1/2))
*          matirix stored by column-major order (fortran convention)
*-----------------------------------------------------------------------------*/
extern int lsq(const double* A, const double* y, int n, int m, double* x,
	double* Q)
{
	//double *Ay;//调整矩阵维数大小
	double Ay[MAXPNTOBS * DIF_L_IN_EVL] = { 0.0 };
	int info = 0;
	if (m < n) return -1;

	matmul("NN", n, 1, m, 1.0, A, y, 0.0, Ay); /* Ay=A*y */
	matmul("NT", n, n, m, 1.0, A, A, 0.0, Q);  /* Q=A*A' */
	if (!(info = matinv(Q, n))) matmul("NN", n, 1, n, 1.0, Q, Ay, 0.0, x); /* x=Q^-1 *Ay */
	return info;
}

int cal_vk(const double* R, const double* Q, const double* v, int nv, double* vk) {
	if (R == NULL || Q == NULL || v == NULL || vk == NULL) {
		return -1;
	}
	double* qr = NULL;
	qr = mat(nv, nv);
	matmul("NN", nv, nv, nv, 1, R, Q, 0, qr);
	matmul("NN", nv, 1, nv, 1, qr, v, 0, vk);
	for (int i = 0; i < nv; i++) {
		vk[i] = -vk[i];
	}
	xy_free(qr);
	return 0;
}

// pnt 2 5    rtd 5 10    rtk 2 5
void robust_rr(double* R, double* Q, const double* v, int nv, int k1, int k2) {
	if (R == NULL || Q == NULL || v == NULL) {
		return;
	}
	double* vk = mat(nv, 1);
	if (!matinv(Q, nv)) {
		cal_vk(R, Q, v, nv, vk);
		double f = 0;
		for (int i = 0; i < nv; i++) {
			f = fabs(vk[i]) / sqrt(R[i + i * nv]);
			if (f > 2 && f <= 4) {
				R[i + i * nv] *= k1;
			}
			else if (f > 4) {
				R[i + i * nv] *= k2;
			}
		}
	}
	xy_free(vk);
}

/* kalman filter ---------------------------------------------------------------
// kalman filter state update as follows:
//
//   K=P*H*(H'*P*H+R)^-1, xp=x+K*v, Pp=(I-K*H')*P
//
// args   : double *x        I   states vector (n x 1)
//          double *P        I   covariance matrix of states (n x n)
//          double *H        I   transpose of design matrix (n x m)
//          double *v        I   innovation (measurement - model) (m x 1)
//          double *R        I   covariance matrix of measurement error (m x m)
//          int    n,m       I   number of states and measurements
//          double *xp       O   states vector after update (n x 1)
//          double *Pp       O   covariance matrix of states after update (n x n)
// return : status (0:ok,<0:error)
// notes  : matirix stored by column-major order (fortran convention)
//          if state x[i]==0.0, not updates state x[i]/P[i+i*n]
-----------------------------------------------------------------------------*/
static int filter_(const double* x, const double* P, const double* H,
	const double* v, const double* R, int n, int m,
	double* xp, double* pp, int k1, int k2)
{
	double* F = mat(n, m), * Q = mat(m, m), * K = mat(n, m), * I = eye(n), * qq = mat(m, m);
	int info = 0;
	matcpy(Q, R, m, m);
	matcpy(xp, x, n, 1);
	matmul("NN", n, m, n, 1.0, P, H, 0.0, F);       /* Q=H'*P*H+R */
	matmul("TN", m, m, n, 1.0, H, F, 1.0, Q);

#if 1
	matcpy(qq, R, m, m);
	robust_rr(qq, Q, v, m, k1, k2);//根据V与R的关系适量进行抗差处理
	matmul("TN", m, m, n, 1.0, H, F, 1.0, qq);
#endif
	if (!(info = matinv(qq, m))) {
		matmul("NN", n, m, m, 1.0, F, qq, 0.0, K);   /* K=P*H*Q^-1 */
		matmul("NN", n, 1, m, 1.0, K, v, 1.0, xp);  /* xp=x+K*v */
		matmul("NT", n, n, m, -1.0, K, H, 1.0, I);  /* pp=(I-K*H')*P */
		matmul("NN", n, n, n, 1.0, I, P, 0.0, pp);
	}
	xy_free(F); xy_free(Q); xy_free(K); xy_free(I); xy_free(qq);
	return info;
}

static int filter_2(const double* x, const double* P, const double* H,
	const double* v, const double* R, int n, int m,
	double* xp, double* pp, int k1, int k2)
{
	double* F = mat(n, m), * Q = mat(m, m), * K = mat(n, m), * I = eye(n);
	double* p1 = mat(n, n), * p2 = mat(n, n), * r1 = mat(n, m);
	double* qq = mat(m, m);
	int info = 0;

	matcpy(Q, R, m, m);
	matcpy(xp, x, n, 1);
	/* Q=H'*P*H+R */
	matmul("NN", n, m, n, 1.0, P, H, 0.0, F);       //F=P*H
	//F(n,m),P(n,n),H(n,m)
	matmul("TN", m, m, n, 1.0, H, F, 1.0, Q);       //Q=H'*F+R
	//Q(m,m),H(n,m),F(n,m)
#if 1
	matcpy(qq, R, m, m);
	robust_rr(qq, Q, v, m, k1, k2);
	matmul("TN", m, m, n, 1.0, H, F, 1.0, qq);
#endif
	//edit by fzhou @ GFZ, 2017-05-13
	//Pp=(I-K*H')*P*(I-K*H')'+K*R*K' 公式比 Pp=(I-K*H')*P 更稳健
	if (!(info = matinv(qq, m))) {
		matmul("NN", n, m, m, 1.0, F, qq, 0.0, K);   /* K=P*H*Q^-1 */
		matmul("NN", n, 1, m, 1.0, K, v, 1.0, xp);  /* xp=x+K*v */
		matmul("NT", n, n, m, -1.0, K, H, 1.0, I);  /* I=(I-K*H') */
		matmul("NN", n, n, n, 1.0, I, P, 0.0, p1);  /* P1=(I-K*H')*P */
		matmul("NT", n, n, n, 1.0, p1, I, 0.0, p2); /* P2=(I-K*H')*P*(I-K*H')' */
		matcpy(pp, p2, n, n);
		matmul("NN", n, m, m, 1.0, K, R, 0.0, r1);  /* R1=K*R */
		matmul("NT", n, n, m, 1.0, r1, K, 1.0, pp); /* Pp=P2+K*R*K'=(I-K*H')*P*(I-K*H')'+K*R*K' */
	}

	xy_free(F); xy_free(Q); xy_free(K); xy_free(I);
	xy_free(p1); xy_free(p2); xy_free(r1); xy_free(qq);
	return info;
}

//kalman filter optimization
//x(n,1) P(n,n) H(n,m) R(m*,m) v(m,1)
extern int filter_opti(double* x, double* P, const double* H,
	const double* v, double* R, int n, int m)
{
	double* F = mat(n, m), * K = mat(n, m), * I = eye(n), * pp = mat(n, n);
	int info = 0;

	matmul("NN", n, m, n, 1.0, P, H, 0.0, F);       /* R=H'*P*H+R */
	matmul("TN", m, m, n, 1.0, H, F, 1.0, R);
	if (!(info = matinv(R, m))) {
		matmul("NN", n, m, m, 1.0, F, R, 0.0, K);   /* K=P*H*Q^-1 */
		matmul("NN", n, 1, m, 1.0, K, v, 1.0, x);  /* xp=x+K*v */
		matmul("NT", n, n, m, -1.0, K, H, 1.0, I);  /* Pp=(I-K*H')*P */
		matmul("NN", n, n, n, 1.0, I, P, 0.0, pp);
	}
	matcpy(P, pp, n, n);
	xy_free(F); xy_free(K); xy_free(I); xy_free(pp);
	return info;
}
/*1.matmul_new函数代替matmul:*/
static int filter_new(const double* x, const double* p, const double* h,
	const double* v, const double* r, int n, int m,
	double* xp, double* pp)
{
	double* ff = mat(n, m), * q = mat(m, m), * kk = mat(n, m), * ii = eye(n);
	int info = 0;

	matcpy(q, r, m, m);
	matcpy(xp, x, n, 1);
	matmul_new(5, "NN", n, m, n, 1.0, p, h, 0.0, ff);       /* Q=H'*P*H+R */
	matmul_new(6, "TN", m, m, n, 1.0, h, ff, 1.0, q);
	if (!(info = matinv(q, m))) {
		matmul("NN", n, m, m, 1.0, ff, q, 0.0, kk);   /* K=P*H*Q^-1 */
		matmul("NN", n, 1, m, 1.0, kk, v, 1.0, xp);  /* xp=x+K*v */
		matmul_new(5, "NT", n, n, m, -1.0, kk, h, 1.0, ii);  /* Pp=(I-K*H')*P */
		matmul("NN", n, n, n, 1.0, ii, p, 0.0, pp);
	}

	xy_free(ff); xy_free(q); xy_free(kk); xy_free(ii);
	return info;
}
/*改进点，优化计算空间：稀疏阵的处理*/
extern int filter_leador(double* x, double* P, const double* H, const double* v,
	const double* R, int n, int m, int k1, int k2)
{
	//优化计算空间
	//trace(3, "filter n=%3d m=%3d \r\n",n,m);
	double* x_ = NULL, * xp_ = NULL, * P_ = NULL, * pp_ = NULL, * H_ = NULL;
	int i = 0, j = 0, k = 0, info = 0, * ix = NULL;
	/*判断带估x参数维数是否小于设置的维数n*/
	ix = imat(n, 1); for(i = k = 0; i < n; i++) {if (x[i] != 0.0 && P[i + i * n] > 0.0) ix[k++] = i;}
	x_ = mat(k, 1); xp_ = mat(k, 1); P_ = mat(k, k); pp_ = mat(k, k); H_ = mat(k, m);
	for (i = 0; i < k; i++) {
		x_[i] = x[ix[i]];
		for (j = 0; j < k; j++) P_[i + j * k] = P[ix[i] + ix[j] * n];
		for (j = 0; j < m; j++) H_[i + j * k] = H[ix[i] + j * n];
	}
	//info=filter_(x_,P_,H_,v,R,k,m,xp_,pp_,k1,k2);//根据V与R的关系抗差
	//info=filter_3(x_,P_,H_,v,R,k,m,xp_,pp_);
	info = filter_new(x_, P_, H_, v, R, k, m, xp_, pp_);
	for (i = 0; i < k; i++) {
		x[ix[i]] = xp_[i];
		if (pp_[i + i * k] <= 0.0){
			trace(5, "filter fatal:i=%2d ix=%2d\r\n", i, ix[i]);
		}
		for (j = 0; j < k; j++) P[ix[i] + ix[j] * n] = pp_[i + j * k];
	}
	xy_free(ix); xy_free(x_); xy_free(xp_); xy_free(P_); xy_free(pp_); xy_free(H_);
	return info;
}

//* smoother --------------------------------------------------------------------
//* combine forward and backward filters by fixed-interval smoother as follows:
//*
//*   xs=Qs*(Qf^-1*xf+Qb^-1*xb), Qs=(Qf^-1+Qb^-1)^-1)
//*
//* args   : double *xf       I   forward solutions (n x 1)
//* args   : double *Qf       I   forward solutions covariance matrix (n x n)
//*          double *xb       I   backward solutions (n x 1)
//*          double *Qb       I   backward solutions covariance matrix (n x n)
//*          int    n         I   number of solutions
//*          double *xs       O   smoothed solutions (n x 1)
//*          double *Qs       O   smoothed solutions covariance matrix (n x n)
//* return : status (0:ok,0>:error)
//* notes  : see reference [4] 5.2
//*          matirix stored by column-major order (fortran convention)
//*-----------------------------------------------------------------------------*/
#  if 0
extern int smoother(const double* xf, const double* Qf, const double* xb,
	const double* Qb, int n, double* xs, double* Qs)
{
	double* invqf = mat(n, n), * invqb = mat(n, n), * xx = mat(n, 1);
	int i, info = -1;

	matcpy(invqf, Qf, n, n);
	matcpy(invqb, Qb, n, n);
	if (!matinv(invqf, n) && !matinv(invqb, n)) {
		for (i = 0; i < n * n; i++) Qs[i] = invqf[i] + invqb[i];
		if (!(info = matinv(Qs, n))) {
			matmul("NN", n, 1, n, 1.0, invqf, xf, 0.0, xx);
			matmul("NN", n, 1, n, 1.0, invqb, xb, 1.0, xx);
			matmul("NN", n, 1, n, 1.0, Qs, xx, 0.0, xs);
		}
	}
	xy_free(invqf); xy_free(invqb); xy_free(xx);
	return info;
}
#endif
/* print matrix ----------------------------------------------------------------
* print matrix to stdout
* args   : double *A        I   matrix A (n x m)
*          int    n,m       I   number of rows and columns of A
*          int    p,q       I   total columns, columns under decimal point
*         (FILE  *fp        I   output file pointer)
* return : none
* notes  : matirix stored by column-major order (fortran convention)
*-----------------------------------------------------------------------------*/

extern void matfprint(const double A[], int n, int m, int p, int q, FILE * fp)
{
	int i = 0, j = 0;

	for (i = 0; i < n; i++) {
#if defined(WIN32) || defined(LINUX)
		for (j = 0; j < m; j++) fprintf(fp, " %*.*f", p, q, A[i + j * n]);
		fprintf(fp, "\r\n");
#else
		//for (j=0;j<m;j++) trace(5," %*.*f",p,q,A[i+j*n]);
		//trace(5,"\r\n");
#endif
	}
}
# if 0
extern void matprint(const double A[], int n, int m, int p, int q)
{
	matfprint(A, n, m, p, q, stdout);
}
#endif
/* string to number ------------------------------------------------------------
* convert substring in string to number
* args   : char   *s        I   string ("... nnn.nnn ...")
*          int    i,n       I   substring position and width
* return : converted number (0.0:error)
*-----------------------------------------------------------------------------*/
extern double str2num(const char* s, int i, int n)
{
	double value = 0;
	char str[256] = "", * p = str;

	if (i < 0 || (int)strlen(s) < i || (int)sizeof(str) - 1 < n) return 0.0;
	for (s += i; *s && --n >= 0; s++) *p++ = *s == 'd' || *s == 'D' ? 'E' : *s;
	*p = '\0';
	return sscanf(str, "%lf", &value) == 1 ? value : 0.0;
}
/* string to time --------------------------------------------------------------
* convert substring in string to gtime_t struct
* args   : char   *s        I   string ("... yyyy mm dd hh mm ss ...")
*          int    i,n       I   substring position and width
*          gtime_t *t       O   gtime_t struct
* return : status (0:ok,0>:error)
*-----------------------------------------------------------------------------*/
extern int str2time(const char* s, int i, int n, gtime_t * t)
{
	double ep[6] = { 0 };
	char str[256] = "", * p = str;

	if (i < 0 || (int)strlen(s) < i || (int)sizeof(str) - 1 < i) return -1;
	for (s += i; *s && --n >= 0;) *p++ = *s++;
	*p = '\0';
	if (sscanf(str, "%lf %lf %lf %lf %lf %lf", ep, ep + 1, ep + 2, ep + 3, ep + 4, ep + 5) < 6)
		return -1;
	if (ep[0] < 100.0) ep[0] += ep[0] < 80.0 ? 2000.0 : 1900.0;
	*t = epoch2time(ep);
	return 0;
}
/* convert calendar day/time to time -------------------------------------------
* convert calendar day/time to gtime_t struct
* args   : double *ep       I   day/time {year,month,day,hour,min,sec}
* return : gtime_t struct
* notes  : proper in 1970-2037 or 1970-2099 (64bit time_t)
*-----------------------------------------------------------------------------*/
extern gtime_t epoch2time(const double* ep)
{
	const int doy[] = { 1,32,60,91,121,152,182,213,244,274,305,335 };
	gtime_t time = { 0 };
	int days = 0, sec = 0, year = (int)ep[0], mon = (int)ep[1], day = (int)ep[2];

	if (year < 1970 || 2099 < year || mon < 1 || 12 < mon) return time;

	/* leap year if year%4==0 in 1901-2099 */
	days = (year - 1970) * 365 + (year - 1969) / 4 + doy[mon - 1] + day - 2 + (year % 4 == 0 && mon >= 3 ? 1 : 0);
	sec = (int)floor(ep[5]);
	time.time = (time_t)days * 86400 + (int)ep[3] * 3600 + (int)ep[4] * 60 + sec;
	time.sec = ep[5] - sec;
	return time;
}
/* time to calendar day/time ---------------------------------------------------
* convert gtime_t struct to calendar day/time
* args   : gtime_t t        I   gtime_t struct
*          double *ep       O   day/time {year,month,day,hour,min,sec}
* return : none
* notes  : proper in 1970-2037 or 1970-2099 (64bit time_t)
*-----------------------------------------------------------------------------*/
extern void time2epoch(gtime_t t, double* ep)
{
	const int mday[] = { /* # of days in a month */
		31,28,31,30,31,30,31,31,30,31,30,31,31,28,31,30,31,30,31,31,30,31,30,31,
		31,29,31,30,31,30,31,31,30,31,30,31,31,28,31,30,31,30,31,31,30,31,30,31
	};
	int days = 0, sec = 0, mon = 0, day = 0;

	/* leap year if year%4==0 in 1901-2099 */
	days = (int)(t.time / 86400);
	sec = (int)(t.time - (time_t)days * 86400);
	for (day = days % 1461, mon = 0; mon < 48; mon++) {
		if (day >= mday[mon]) day -= mday[mon]; else break;
	}
	ep[0] = 1970 + days / 1461 * 4 + mon / 12; ep[1] = mon % 12 + 1; ep[2] = day + 1;
	ep[3] = sec / 3600; ep[4] = sec % 3600 / 60; ep[5] = sec % 60 + t.sec;
}
/* gps time to time ------------------------------------------------------------
* convert week and tow in gps time to gtime_t struct
* args   : int    week      I   week number in gps time
*          double sec       I   time of week in gps time (s)
* return : gtime_t struct
*-----------------------------------------------------------------------------*/
extern gtime_t gpst2time(int week, double sec)
{
	gtime_t t = epoch2time(gpst0);

	if (sec < -1E9 || 1E9 < sec) sec = 0.0;
	t.time += (time_t)86400 * 7 * week + (int)sec;
	t.sec = sec - (int)sec;
	return t;
}
/* time to gps time ------------------------------------------------------------
* convert gtime_t struct to week and tow in gps time
* args   : gtime_t t        I   gtime_t struct
*          int    *week     IO  week number in gps time (NULL: no output)
* return : time of week in gps time (s)
*-----------------------------------------------------------------------------*/
extern double time2gpst(gtime_t t, int* week)
{
	gtime_t t0 = epoch2time(gpst0);
	time_t sec = t.time - t0.time;
	int w = (int)(sec / (86400 * 7));

	if (week) *week = w;
	return (double)(sec - (double)w * 86400 * 7) + t.sec;
}
/* galileo system time to time -------------------------------------------------
* convert week and tow in galileo system time (gst) to gtime_t struct
* args   : int    week      I   week number in gst
*          double sec       I   time of week in gst (s)
* return : gtime_t struct
*-----------------------------------------------------------------------------*/
extern gtime_t gst2time(int week, double sec)
{
	gtime_t t = epoch2time(gst0);

	if (sec < -1E9 || 1E9 < sec) sec = 0.0;
	t.time += (time_t)86400 * 7 * week + (int)sec;
	t.sec = sec - (int)sec;
	return t;
}
/* time to galileo system time -------------------------------------------------
* convert gtime_t struct to week and tow in galileo system time (gst)
* args   : gtime_t t        I   gtime_t struct
*          int    *week     IO  week number in gst (NULL: no output)
* return : time of week in gst (s)
*-----------------------------------------------------------------------------*/
extern double time2gst(gtime_t t, int* week)
{
	gtime_t t0 = epoch2time(gst0);
	time_t sec = t.time - t0.time;
	int w = (int)(sec / (86400 * 7));

	if (week) *week = w;
	return (double)(sec - (double)w * 86400 * 7) + t.sec;
}
/* beidou time (bdt) to time ---------------------------------------------------
* convert week and tow in beidou time (bdt) to gtime_t struct
* args   : int    week      I   week number in bdt
*          double sec       I   time of week in bdt (s)
* return : gtime_t struct
*-----------------------------------------------------------------------------*/
extern gtime_t bdt2time(int week, double sec)
{
	gtime_t t = epoch2time(bdt0);

	if (sec < -1E9 || 1E9 < sec) sec = 0.0;
	t.time += (time_t)86400 * 7 * week + (int)sec;
	t.sec = sec - (int)sec;
	return t;
}
/* time to beidouo time (bdt) --------------------------------------------------
* convert gtime_t struct to week and tow in beidou time (bdt)
* args   : gtime_t t        I   gtime_t struct
*          int    *week     IO  week number in bdt (NULL: no output)
* return : time of week in bdt (s)
*-----------------------------------------------------------------------------*/
extern double time2bdt(gtime_t t, int* week)
{
	gtime_t t0 = epoch2time(bdt0);
	time_t sec = t.time - t0.time;
	int w = (int)(sec / (86400 * 7));

	if (week) *week = w;
	return (double)(sec - (double)w * 86400 * 7) + t.sec;
}
/* add time --------------------------------------------------------------------
* add time to gtime_t struct
* args   : gtime_t t        I   gtime_t struct
*          double sec       I   time to add (s)
* return : gtime_t struct (t+sec)
*-----------------------------------------------------------------------------*/
extern gtime_t timeadd(gtime_t t, double sec)
{
	double tt = 0;

	t.sec += sec; tt = floor(t.sec); t.time += (int)tt; t.sec -= tt;
	return t;
}
/* time difference -------------------------------------------------------------
* difference between gtime_t structs
* args   : gtime_t t1,t2    I   gtime_t structs
* return : time difference (t1-t2) (s)
*-----------------------------------------------------------------------------*/
extern double timediff(gtime_t t1, gtime_t t2)
{
	double dt = ((long)t1.time - (long)t2.time) + (t1.sec - t2.sec);    // becareful !!!!!!!  by mo.wh 20201126
	return dt;
}

/* get current time in utc -----------------------------------------------------
* get current time in utc
* args   : none
* return : current time in utc
*-----------------------------------------------------------------------------*/
//static double timeoffset_=0.0;        /* time offset (s) */

extern gtime_t timeget(void)
{
	gtime_t time = { 0 };
# if 0
	double ep[6] = { 0 };
#if defined(WIN32)
	SYSTEMTIME ts;

	GetSystemTime(&ts); /* utc */
	ep[0] = ts.wYear; ep[1] = ts.wMonth;  ep[2] = ts.wDay;
	ep[3] = ts.wHour; ep[4] = ts.wMinute; ep[5] = ts.wSecond + ts.wMilliseconds * 1E-3;
#elif defined(LINUX)
# if 0
	struct timeval tv;
	struct tm* tt;

	if (!gettimeofday(&tv, NULL) && (tt = gmtime(&tv.tv_sec))) {
		ep[0] = tt->tm_year + 1900; ep[1] = tt->tm_mon + 1; ep[2] = tt->tm_mday;
		ep[3] = tt->tm_hour; ep[4] = tt->tm_min; ep[5] = tt->tm_sec + tv.tv_usec * 1E-6;
	}
#endif
#elif defined(RTOS)

#endif

	time = epoch2time(ep);

#ifdef CPUTIME_IN_GPST /* cputime operated in gpst */
	time = gpst2utc(time);
#endif
	return timeadd(time, timeoffset_);
#endif

	return time;
}

extern gtime_t gpst2utc(gtime_t t)
{
	gtime_t tu = { 0 };
	int i = 0;

	for (i = 0; leaps[i][0] > 0; i++) {
		tu = timeadd(t, leaps[i][6]);
		if (timediff(tu, epoch2time(leaps[i])) >= 0.0) return tu;
	}
	return t;
}
/* utc to gpstime --------------------------------------------------------------
* convert utc to gpstime considering leap seconds
* args   : gtime_t t        I   time expressed in utc
* return : time expressed in gpstime
* notes  : ignore slight time offset under 100 ns
*-----------------------------------------------------------------------------*/
extern gtime_t utc2gpst(gtime_t t)
{
	int i = 0;

	for (i = 0; leaps[i][0] > 0; i++) {
		if (timediff(t, epoch2time(leaps[i])) >= 0.0) return timeadd(t, -leaps[i][6]);
	}
	return t;
}
/* gpstime to bdt --------------------------------------------------------------
* convert gpstime to bdt (beidou navigation satellite system time)
* args   : gtime_t t        I   time expressed in gpstime
* return : time expressed in bdt
* notes  : ref [8] 3.3, 2006/1/1 00:00 BDT = 2006/1/1 00:00 UTC
*          no leap seconds in BDT
*          ignore slight time offset under 100 ns
*-----------------------------------------------------------------------------*/
extern gtime_t gpst2bdt(gtime_t t)
{
	return timeadd(t, -14.0);
}
/* bdt to gpstime --------------------------------------------------------------
* convert bdt (beidou navigation satellite system time) to gpstime
* args   : gtime_t t        I   time expressed in bdt
* return : time expressed in gpstime
* notes  : see gpst2bdt()
*-----------------------------------------------------------------------------*/
extern gtime_t bdt2gpst(gtime_t t)
{
	return timeadd(t, 14.0);
}
/* time to day and sec -------------------------------------------------------*/
static double time2sec(gtime_t time, gtime_t * day)
{
	double ep[6] = { 0 }, sec = 0;
	time2epoch(time, ep);
	sec = ep[3] * 3600.0 + ep[4] * 60.0 + ep[5];
	ep[3] = ep[4] = ep[5] = 0.0;
	*day = epoch2time(ep);
	return sec;
}
/* utc to gmst -----------------------------------------------------------------
* convert utc to gmst (Greenwich mean sidereal time)
* args   : gtime_t t        I   time expressed in utc
*          double ut1_utc   I   UT1-UTC (s)
* return : gmst (rad)
*-----------------------------------------------------------------------------*/
extern double utc2gmst(gtime_t t, double ut1_utc)
{
	const double ep2000[] = { 2000,1,1,12,0,0 };
	gtime_t tut = { 0 }, tut0 = { 0 };
	double ut = 0, t1 = 0, t2 = 0, t3 = 0, gmst0 = 0, gmst = 0;

	tut = timeadd(t, ut1_utc);
	ut = time2sec(tut, &tut0);
	t1 = timediff(tut0, epoch2time(ep2000)) / 86400.0 / 36525.0;
	t2 = t1 * t1; t3 = t2 * t1;
	gmst0 = 24110.54841 + 8640184.812866 * t1 + 0.093104 * t2 - 6.2E-6 * t3;
	gmst = gmst0 + 1.002737909350795 * ut;

	return fmod(gmst, 86400.0) * PI / 43200.0; /* 0 <= gmst <= 2*PI */
}
/* time to string --------------------------------------------------------------
* convert gtime_t struct to string
* args   : gtime_t t        I   gtime_t struct
*          char   *s        O   string ("yyyy/mm/dd hh:mm:ss.ssss")
*          int    n         I   number of decimals
* return : none
*-----------------------------------------------------------------------------*/
extern void time2str(gtime_t t, char* s, int n)
{
	double ep[6] = { 0 };

	if (n < 0) n = 0; else if (n > 12) n = 12;
	if (1.0 - t.sec < 0.5 / pow(10.0, n)) { t.time++; t.sec = 0.0; };
	time2epoch(t, ep);
	sprintf(s, "%04.0f/%02.0f/%02.0f %02.0f:%02.0f:%0*.*f", ep[0], ep[1], ep[2],
		ep[3], ep[4], n <= 0 ? 2 : n + 3, n <= 0 ? 0 : n, ep[5]);
}
/* get time string -------------------------------------------------------------
* get time string
* args   : gtime_t t        I   gtime_t struct
*          int    n         I   number of decimals
* return : time string
* notes  : not reentrant, do not use multiple in a function
*-----------------------------------------------------------------------------*/
extern char* time_str(gtime_t t, int n)
{
	static char buff[64] = "";
	time2str(t, buff, n);
	return buff;
}
/* time to day of year ---------------------------------------------------------
* convert time to day of year
* args   : gtime_t t        I   gtime_t struct
* return : day of year (days)
*-----------------------------------------------------------------------------*/
extern double time2doy(gtime_t t)
{
	double ep[6] = { 0 };

	time2epoch(t, ep);
	ep[1] = ep[2] = 1.0; ep[3] = ep[4] = ep[5] = 0.0;
	return timediff(t, epoch2time(ep)) / 86400.0 + 1.0;
}
/* adjust gps week number ------------------------------------------------------
* adjust gps week number using cpu time
* args   : int   week       I   not-adjusted gps week number
* return : adjusted gps week number
*-----------------------------------------------------------------------------*/
extern int adjgpsweek(int week)
{
	int w;
	//(void)time2gpst(utc2gpst(timeget()),&w);
	//if (w<1560) w=1560; /* use 2009/12/1 if time is earlier than 2009/12/1 */
	w = 2116; //use 2020/07/30 if time is earlier than 2020/07/30 */
	return week + (w - week + 512) / 1024 * 1024;
}

/* convert degree to deg-min-sec -----------------------------------------------
* convert degree to degree-minute-second
* args   : double deg       I   degree
*          double *dms      O   degree-minute-second {deg,min,sec}
*          int    ndec      I   number of decimals of second
* return : none
*-----------------------------------------------------------------------------*/
extern void deg2dms(double deg, double* dms, int ndec)
{
	double sign = deg < 0.0 ? -1.0 : 1.0, a = fabs(deg);
	double unit = pow(0.1, ndec);
	dms[0] = floor(a); a = (a - dms[0]) * 60.0;
	dms[1] = floor(a); a = (a - dms[1]) * 60.0;
	dms[2] = floor(a / unit + 0.5) * unit;
	if (dms[2] >= 60.0) {
		dms[2] = 0.0;
		dms[1] += 1.0;
		if (dms[1] >= 60.0) {
			dms[1] = 0.0;
			dms[0] += 1.0;
		}
	}
	dms[0] *= sign;
}

/* transform ecef to geodetic postion ------------------------------------------
* transform ecef position to geodetic position
* args   : double *r        I   ecef position {x,y,z} (m)
*          double *pos      O   geodetic position {lat,lon,h} (rad,m)
* return : none
* notes  : WGS84, ellipsoidal height
*-----------------------------------------------------------------------------*/
extern void ecef2pos(const double* r, double* pos)
{
	double e2 = FE_WGS84 * (2.0 - FE_WGS84), r2 = dot(r, r, 2), z = 0, zk = 0, v = RE_WGS84, sinp = 0;

	for (z = r[2], zk = 0.0; fabs(z - zk) >= 1E-4;) {
		zk = z;
		sinp = z / sqrt(r2 + z * z);
		v = RE_WGS84 / sqrt(1.0 - e2 * sinp * sinp);
		z = r[2] + v * e2 * sinp;
	}

	if (r2 > ZEROS_MIN) {
		pos[0] = atan(z / sqrt(r2));
		pos[1] = atan2(r[1], r[0]);
	}
	else {
		pos[0] = r[2] > 0.0 ? PI / 2.0 : -PI / 2.0;
		pos[1] = 0.0;
	}
	pos[2] = sqrt(r2 + z * z) - v;
}
///* transform geodetic to ecef position -----------------------------------------
//* transform geodetic position to ecef position
//* args   : double *pos      I   geodetic position {lat,lon,h} (rad,m)
//*          double *r        O   ecef position {x,y,z} (m)
//* return : none
//* notes  : WGS84, ellipsoidal height
//*-----------------------------------------------------------------------------*/
extern void pos2ecef(const double* pos, double* r)
{
	double sinp = sin(pos[0]), cosp = cos(pos[0]), sinl = sin(pos[1]), cosl = cos(pos[1]);
	double e2 = FE_WGS84 * (2.0 - FE_WGS84), v = RE_WGS84 / sqrt(1.0 - e2 * sinp * sinp);

	r[0] = (v + pos[2]) * cosp * cosl;
	r[1] = (v + pos[2]) * cosp * sinl;
	r[2] = (v * (1.0 - e2) + pos[2]) * sinp;
}
/* ecef to local coordinate transfromation matrix ------------------------------
* compute ecef to local coordinate transfromation matrix
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *E        O   ecef to local coord transformation matrix (3x3)
* return : none
* notes  : matirix stored by column-major order (fortran convention)
*-----------------------------------------------------------------------------*/
extern void xyz2enu(const double* pos, double* E)
{
	double sinp = sin(pos[0]), cosp = cos(pos[0]), sinl = sin(pos[1]), cosl = cos(pos[1]);

	E[0] = -sinl;      E[3] = cosl;       E[6] = 0.0;
	E[1] = -sinp * cosl; E[4] = -sinp * sinl; E[7] = cosp;
	E[2] = cosp * cosl;  E[5] = cosp * sinl;  E[8] = sinp;
}
/* transform ecef vector to local tangental coordinate -------------------------
* transform ecef vector to local tangental coordinate
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *r        I   vector in ecef coordinate {x,y,z}
*          double *e        O   vector in local tangental coordinate {e,n,u}
* return : none
*-----------------------------------------------------------------------------*/
extern void ecef2enu(const double* pos, const double* r, double* e)
{
	double E[9] = { 0 };

	xyz2enu(pos, E);
	matmul("NN", 3, 1, 3, 1.0, E, r, 0.0, e);
}

extern void enu2ecef(const double* pos, const double* e, double* r)
{
	double E[9] = { 0 };

	xyz2enu(pos, E);
	matmul("TN", 3, 1, 3, 1.0, E, e, 0.0, r);
}

/* transform local enu coordinate covariance to xyz-ecef -----------------------
* transform local enu covariance to xyz-ecef coordinate
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *Q        I   covariance in local enu coordinate
*          double *P        O   covariance in xyz-ecef coordinate
* return : none
*-----------------------------------------------------------------------------*/
extern void covecef(const double* pos, const double* Q, double* P)
{
	double E[9] = { 0 }, eq[9] = { 0 };

	xyz2enu(pos, E);
	matmul("TN", 3, 3, 3, 1.0, E, Q, 0.0, eq);
	matmul("NN", 3, 3, 3, 1.0, eq, E, 0.0, P);
}

/* compare observation data -------------------------------------------------*/
static int cmpobs(const void* p1, const void* p2)
{
	obsd_t* q1 = (obsd_t*)p1, * q2 = (obsd_t*)p2;
	double tt = timediff(q1->time, q2->time);
	if (fabs(tt) > DTTOL) return tt < 0 ? -1 : 1;
	if (q1->rcv != q2->rcv) return (int)q1->rcv - (int)q2->rcv;
	return (int)q1->sat - (int)q2->sat;
}
/*检查不同系统的伪距观测值判读异常与否*/
int checkobs(obsd_t * obs) {
	if (obs == NULL) {
		trace(3, "Rtkcmn::checkobs, operate null pointer.\n");
		return -1;
	}
	int sys = satsys(obs->sat, NULL);
	//trace->trace(4,"obs data P=%f sys =%d\n",obs->P[0],sys);
	if (sys == SYS_CMP) {
		if (obs->P[0] > MinPseudorange_BDS && obs->P[0] < MaxPseudorange_BDS)
			return 1;
		else
			trace(3, "Rtkcmn::checkobs,BDS:obs->sat=%4d  P[0]=%12.3f\n", obs->sat, obs->P[0]);
			return 0;
	}
	if (sys == SYS_GPS) {
		if (obs->P[0] > MinPseudorange_GPS && obs->P[0] < MaxPseudorange_GPS)
			return 1;
		else
			trace(3, "Rtkcmn::checkobs,GPS:obs->sat=%4d  P[0]=%12.3f\n", obs->sat, obs->P[0]);
			return 0;
	}
	if (sys == SYS_GLO) {
		if (obs->P[0] > MinPseudorange_GLO && obs->P[0] < MaxPseudorange_GLO)
			return 1;
		else 
			trace(3, "Rtkcmn::checkobs,GLO:obs->sat=%4d  P[0]=%12.3f\n", obs->sat, obs->P[0]);
			return 0;
	}
	if (sys == SYS_GAL) {
		if (obs->P[0] > MinPseudorange_GAL && obs->P[0] < MaxPseudorange_GAL)
			return 1;
		else 
			trace(3, "Rtkcmn::checkobs,GAL:obs->sat=%4d  P[0]=%12.3f\n", obs->sat, obs->P[0]);
			return 0;
	}
	return 1;
}
/* sort and unique observation data --------------------------------------------
* sort and unique observation data by time, rcv, sat
* args   : obs_t *obs    IO     observation data
* return : number of epochs
*-----------------------------------------------------------------------------*/
extern int sortobs(obs_t * obs)
{
	int i = 0, j = 0, n = 0;

	//  trace(3,"sortobs: nobs=%d\r\n",obs->n);

	if (obs->n <= 0) return 0;

	qsort(obs->data, (size_t)obs->n, sizeof(obsd_t), cmpobs);

	/* delete duplicated data */
	for (i = j = 0; i < obs->n; i++) {
		if (obs->data[i].sat != obs->data[j].sat ||//缺乏对第一颗卫星的checkobs检测
			obs->data[i].rcv != obs->data[j].rcv ||
			timediff(obs->data[i].time, obs->data[j].time) != 0.0) {
			if (checkobs(obs->data + i))//有异常的伪距
				obs->data[++j] = obs->data[i];
		}
	}
	obs->n = j + 1;

	/*判断第一个数据是否异常，在这里补充第一颗卫星的检测*/
	if (!checkobs(obs->data)) {
		for (i = 0; i < obs->n - 1; i++) {
			obs->data[i] = obs->data[i + 1];
		}
		obs->n = obs->n - 1;
	}

	for (i = n = 0; i < obs->n; i = j, n++) {
		for (j = i + 1; j < obs->n; j++) {
			if (timediff(obs->data[j].time, obs->data[i].time) > DTTOL) break;
		}
	}
	return n;
}

/* screen by time --------------------------------------------------------------
* screening by time start, time end, and time interval
* args   : gtime_t time  I      time
*          gtime_t ts    I      time start (ts.time==0:no screening by ts)
*          gtime_t te    I      time end   (te.time==0:no screening by te)
*          double  tint  I      time interval (s) (0.0:no screen by tint)
* return : 1:on condition, 0:not on condition
*-----------------------------------------------------------------------------*/
extern int screent(gtime_t time, gtime_t ts, gtime_t te, double tint)
{
	return (tint <= 0.0 || fmod(time2gpst(time, NULL) + DTTOL, tint) <= DTTOL * 2.0) &&
		(ts.time == 0 || timediff(time, ts) >= -DTTOL) &&
		(te.time == 0 || timediff(time, te) < DTTOL);
}

/* free observation data -------------------------------------------------------
* free memory for observation data
* args   : obs_t *obs    IO     observation data
* return : none
*-----------------------------------------------------------------------------*/
extern void freeobs(obs_t * obs)
{
	if (obs->data)
	{
		xy_free(obs->data); obs->data = NULL; obs->n = obs->nmax = 0;
	}
}
/* debug trace functions -----------------------------------------------------*/

static int level_trace = 0;       /* level of trace */

#if defined(WIN32) || defined(LINUX)
extern void traceopen(const char* file)
{
	//gtime_t time=utc2gpst(timeget());
	char path[1024] = "";
	strcpy(path, file);
	if (!*path || !(fp_trace = fopen(path, "w"))) fp_trace = stderr;
	strcpy(file_trace, file);
}
extern void traceclose(void)
{
	if (fp_trace && fp_trace != stderr) fclose(fp_trace);
	fp_trace = NULL;
	file_trace[0] = '\0';
}
#endif // WIN32

extern void tracelevel(int level)
{
	level_trace = level;
}
extern void trace(int level, const char* format, ...)
{
#if TRACEOPEN
#if defined(WIN32) || defined(LINUX)
	va_list ap;
	/* print error message to stderr */
	if (level <= 1) {
		va_start(ap, format); vfprintf(stderr, format, ap); va_end(ap);
	}

	if (!fp_trace || level > level_trace) return;
	// traceswap();
	fprintf(fp_trace, "%d ", level);
	va_start(ap, format); vfprintf(fp_trace, format, ap); va_end(ap);
	fflush(fp_trace);
#else
	if (level < 1 || level>level_trace) return;

	cb->printf_info("%d ", level);
	va_list ap;
	char buf[512];

	va_start(ap, format);
	vsnprintf(buf, 512, format, ap);
	va_end(ap);
	cb->printf_info("%s ", buf);
#endif
#endif
}

# if 0
extern void tracet(int level, const char* format, ...)
{
	va_list ap;

	if (!fp_trace || level > level_trace) return;
	traceswap();
	fprintf(fp_trace, "%d %9.3f: ", level, (tickget() - tick_trace) / 1000.0);
	va_start(ap, format); vfprintf(fp_trace, format, ap); va_end(ap);
	fflush(fp_trace);
}
#endif

extern void tracemat(int level, const double* A, int n, int m, int p, int q)
{
#if TRACEOPEN
#if defined(WIN32) || defined(LINUX)
	if (!fp_trace || level > level_trace) return;
	matfprint(A, n, m, p, q, fp_trace); fflush(fp_trace);
#else
	if (level > level_trace) return;
	matfprint(A, n, m, p, q, NULL);
#endif
#endif
}
extern void traceobs(rtk_t * rtk, int level, const obsd_t * obs, int n)
{
#if TRACEOPEN
	char str[64] = "", id[16] = "";
	int i = 0;
#if defined(WIN32) || defined(LINUX)
	if (!fp_trace || level > level_trace) return;
	for (i = 0; i < n; i++) {
		time2str(obs[i].time, str, 3);
		satno2id(obs[i].sat, id);
		if (rtk->opt.nf < 2)
		{
			fprintf(fp_trace, " (%2d) %s %4d %-3s rcv%d %13.3f %13.3f %13.3f %d %2d %3.1f\n",
				i + 1, str, obs[i].sat, id, obs[i].rcv, obs[i].L[0], obs[i].P[0], obs[i].D[0],
				obs[i].lli[0], obs[i].code[0], obs[i].snr[0] * 0.25);
		}
		else if (rtk->opt.nf > 1 && rtk->opt.nf < 3 && NFREQ>1)
		{
			fprintf(fp_trace, " (%2d) %s %4d %-3s rcv%d %13.3f %13.3f %13.3f %d %2d %3.1f %13.3f %13.3f %13.3f %d %2d %3.1f\n",
				i + 1, str, obs[i].sat, id, obs[i].rcv, obs[i].L[0], obs[i].P[0], obs[i].D[0],
				obs[i].lli[0], obs[i].code[0], obs[i].snr[0] * 0.25, obs[i].L[1], obs[i].P[1], obs[i].D[1],
				obs[i].lli[1], obs[i].code[1], obs[i].snr[1] * 0.25);
		}
	}
	fflush(fp_trace);
#else
	if (level <= 1 || level > level_trace) return;
	for (i = 0; i < n; i++) {
		time2str(obs[i].time, str, 3);
		satno2id(obs[i].sat, id);
		trace(5, " (%2d) %s %-3s rcv%d %13.3f %13.3f %13.3f %13.3f %d %d %d %d %3.1f %3.1f\r\n",
			i + 1, str, id, obs[i].rcv, obs[i].L[0], 0.0, obs[i].P[0],
			0.0, obs[i].lli[0], 0, obs[i].code[0],
			0, obs[i].snr[0] * 0.25, 0 * 0.25);
	}
#endif
#endif
}
//计算载波波长
extern double satwavelen(int sat, int frq, const nav_t * nav)
{
	const double freq_glo[] = { FREQ1_GLO,FREQ2_GLO };
	const double dfrq_glo[] = { DFRQ1_GLO,DFRQ2_GLO };
	int i = 0, sys = satsys(sat, NULL);

	if (sys == SYS_GLO) {
		if (0 <= frq && frq <= 1) {
			for (i = 0; i < nav->ng; i++) {
				if (nav->geph[i].sat != sat) continue;
				return CLIGHT / (freq_glo[frq] + dfrq_glo[frq] * nav->geph[i].frq);
			}
		}
		else if (frq == 2) { /* L3 */
			return CLIGHT / FREQ3_GLO;
		}
	}
	else if (sys == SYS_CMP) {
		if (frq == 0) return CLIGHT / FREQ1_CMP; /* B1 */
		if (FREQMODE_CMP == 0)//b1 b2模式
		{
			if (frq == 1) return CLIGHT / FREQ2_CMP; /* B2 */
			if (frq == 2) return CLIGHT / FREQ3_CMP; /* B3 */
		}
		else  if (FREQMODE_CMP == 1) {// b1 b2a模式
			if (frq == 1) return CLIGHT / FREQ2a_CMP; /* B2 */
			if (frq == 2) return CLIGHT / FREQ3_CMP; /* B3 */
		}
		else if (FREQMODE_CMP == 2) {// b1 b3 b2模式
			if (frq == 1) return CLIGHT / FREQ3_CMP; /* B3 */
			if (frq == 2) return CLIGHT / FREQ2_CMP; /* B2 */
		}
	}
	else {
		if (sys == SYS_GPS && (FREQMODE_GPS == 1)) {
			if (frq == 0) return CLIGHT / FREQ1; /* L1/E1 */
			else if (frq == 1) return CLIGHT / FREQ5; /* L5 */
			else if (frq == 2) return CLIGHT / FREQ2; /* L2 */
		}
		if (sys == SYS_GAL && (FREQMODE_GAL == 1)) {
			if (frq == 0) return CLIGHT / FREQ1; /* L1/E1 */
			else if (frq == 1) return CLIGHT / FREQ5_GAL; /* L5 */
			else if (frq == 2) return CLIGHT / FREQ2; /* L2 */
		}
		if (sys == SYS_GAL && (FREQMODE_GAL == 2)) {
			if (frq == 0) return CLIGHT / FREQ1; /* L1/E1 */
			else if (frq == 1) return CLIGHT / FREQ7; /* L5 */
			else if (frq == 2) return CLIGHT / FREQ2; /* L2 */
		}
		else {
			if (frq == 0) return CLIGHT / FREQ1; /* L1/E1 */
			else if (frq == 1) return CLIGHT / FREQ2; /* L2 */
			else if (frq == 2) return CLIGHT / FREQ5; /* L5/E5a */
			else if (frq == 3) return CLIGHT / FREQ6; /* L6/LEX */
			else if (frq == 4) return CLIGHT / FREQ7; /* E5b */
			else if (frq == 5) return CLIGHT / FREQ8; /* E5a+b */
			else if (frq == 6) return CLIGHT / FREQ9; /* S */
		}
	}
	return 0.0;
}
/* geometric distance ----------------------------------------------------------
* compute geometric distance and receiver-to-satellite unit vector
* args   : double *rs       I   satellilte position (ecef at transmission) (m)
*          double *rr       I   receiver position (ecef at reception) (m)
*          double *e        O   line-of-sight vector (ecef)
* return : geometric distance (m) (0>:error/no satellite position)
* notes  : distance includes sagnac effect correction
*-----------------------------------------------------------------------------*/
extern double geodist(const double* rs, const double* rr, double* e)
{
	double r = 0;
	int i = 0;

	if (norm(rs, 3) < RE_WGS84) return -1.0;
	for (i = 0; i < 3; i++) e[i] = rs[i] - rr[i];
	//trace(5,"satellite position: %19.9f %19.9f %19.9f,rover pos is: %19.9f %19.9f %19.9f \n", rs[0], rs[1], rs[2], rr[0], rr[1], rr[2]);
	r = norm(e, 3);
	for (i = 0; i < 3; i++) e[i] /= r;
	return r + OMGE * (rs[0] * rr[1] - rs[1] * rr[0]) / CLIGHT;
}
/* satellite azimuth/elevation angle -------------------------------------------
* compute satellite azimuth/elevation angle
* args   : double *pos      I   geodetic position {lat,lon,h} (rad,m)
*          double *e        I   receiver-to-satellilte unit vevtor (ecef)
*          double *azel     IO  azimuth/elevation {az,el} (rad) (NULL: no output)
*                               (0.0<=azel[0]<2*pi,-pi/2<=azel[1]<=pi/2)
* return : elevation angle (rad)
*-----------------------------------------------------------------------------*/
extern double satazel(const double* pos, const double* e, double* azel)
{
	double az = 0.0, el = PI / 2.0, enu[3] = { 0 };

	if (pos[2] > -RE_WGS84) {//根据经度值判断是否位置是否正确，是否可以计算方位角
		ecef2enu(pos, e, enu);
		az = dot(enu, enu, 2) < 1E-12 ? 0.0 : atan2(enu[0], enu[1]);
		if (az < 0.0) az += 2 * PI;
		el = asin(enu[2]);
	}
	if (azel) { azel[0] = az; azel[1] = el; }
	return el;
}
/* compute dops ----------------------------------------------------------------
* compute DOP (dilution of precision)
* args   : int    ns        I   number of satellites
*          double *azel     I   satellite azimuth/elevation angle (rad)
*          double elmin     I   elevation cutoff angle (rad)
*          double *dop      O   DOPs {GDOP,PDOP,HDOP,VDOP}
* return : none
* notes  : dop[0]-[3] return 0 in case of dop computation error
*-----------------------------------------------------------------------------*/

extern void dops(int ns, const double* azel, double elmin, double* dop, const int* vsat)
{
	double H[4 * MAXPNTOBS * DIF_L_IN_EVL] = { 0 }, Q[16] = { 0 }, cosel = 0, sinel = 0;
	int i = 0, n = 0;

	for (i = 0; i < 4; i++) dop[i] = 0.0;
	for (i = n = 0; i < ns && i < MAXPNTOBS * DIF_L_IN_EVL; i++) {
		if (vsat){
			if (vsat[i] < 1)
				continue;
		}
		trace(5,"dops:: %dth azel=%f %f elmin=%f \n",i,azel[0+i*2] * R2D,azel[1+i*2] * R2D,elmin * R2D);
		if (azel[1 + i * 2] < elmin || azel[1 + i * 2] <= 0.0) continue;
		cosel = cos(azel[1 + i * 2]);
		sinel = sin(azel[1 + i * 2]);
		H[4 * n] = cosel * sin(azel[i * 2]);
		H[1 + 4 * n] = cosel * cos(azel[i * 2]);
		H[2 + 4 * n] = sinel;
		H[3 + 4 * n++] = 1.0;
	}
	if (n < 4) return;

	matmul("NT", 4, 4, n, 1.0, H, H, 0.0, Q);
	if (!matinv(Q, 4)) {
		dop[0] = SQRT(Q[0] + Q[5] + Q[10] + Q[15]); /* GDOP */
		dop[1] = SQRT(Q[0] + Q[5] + Q[10]);       /* PDOP */
		dop[2] = SQRT(Q[0] + Q[5]);             /* HDOP */
		dop[3] = SQRT(Q[10]);                 /* VDOP */
	}
	trace(5,"dop is: G:%f P:%f H:%f V:%f\n",dop[0],dop[1],dop[2],dop[3]);
}
//针对差分解算的计算dop函数，for循环计算时使用MAXDIFOBS优化计算空间 
extern void dops_dif(int ns, const double* azel, double elmin, double* dop, const int* vsat)
{
	double H[4 * MAXDIFOBS * NFREQ] = { 0 }, Q[16] = { 0 }, cosel = 0, sinel = 0;
	int i = 0, n = 0;

	for (i = 0; i < 4; i++) dop[i] = 0.0;
	for (i = n = 0; i < ns && i < MAXDIFOBS * NFREQ; i++) {
		if (vsat){if (vsat[i] < 1) continue;}
		//trace(5,"dops %dth azel %f %f elm%f \n",i,azel[0+i*2],azel[1+i*2],elmin);
		if (azel[1 + i * 2] < elmin || azel[1 + i * 2] <= 0.0) continue;
		cosel = cos(azel[1 + i * 2]);
		sinel = sin(azel[1 + i * 2]);
		H[4 * n] = cosel * sin(azel[i * 2]);
		H[1 + 4 * n] = cosel * cos(azel[i * 2]);
		H[2 + 4 * n] = sinel;
		H[3 + 4 * n++] = 1.0;
	}
	if (n < 4) return;

	matmul("NT", 4, 4, n, 1.0, H, H, 0.0, Q);
	if (!matinv(Q, 4)) {
		dop[0] = SQRT(Q[0] + Q[5] + Q[10] + Q[15]); /* GDOP */
		dop[1] = SQRT(Q[0] + Q[5] + Q[10]);       /* PDOP */
		dop[2] = SQRT(Q[0] + Q[5]);             /* HDOP */
		dop[3] = SQRT(Q[10]);                 /* VDOP */
	}
	//trace(5,"dop is %f %f %f %f\n",dop[0],dop[1],dop[2],dop[3]);
}

/* ionosphere model ------------------------------------------------------------
* compute ionospheric delay by broadcast ionosphere model (klobuchar model)
* args   : gtime_t t        I   time (gpst)
*          double *ion      I   iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3}
*          double *pos      I   receiver position {lat,lon,h} (rad,m)
*          double *azel     I   azimuth/elevation angle {az,el} (rad)
* return : ionospheric delay (L1) (m)
*-----------------------------------------------------------------------------*/
extern double ionmodel(gtime_t t, const double* ion, const double* pos,
	const double* azel)
{
	const double ion_default[] = { /* 2004/1/1 */
		0.1118E-07,-0.7451E-08,-0.5961E-07, 0.1192E-06,
		0.1167E+06,-0.2294E+06,-0.1311E+06, 0.1049E+07
	};
	double tt = 0, f = 0, psi = 0, phi = 0, lam = 0, amp = 0, per = 0, x = 0;
	int week = 0;
	if (pos[2] < -1E3 || azel[1] <= 0) return 0.0;
	if (norm(ion, 8) <= 0.0) ion = ion_default;
	/* earth centered angle (semi-circle) */
	psi = 0.0137 / (azel[1] / PI + 0.11) - 0.022;
	/* subionospheric latitude/longitude (semi-circle) */
	phi = pos[0] / PI + psi * cos(azel[0]);
	if (phi > 0.416) phi = 0.416;
	else if (phi < -0.416) phi = -0.416;
	lam = pos[1] / PI + psi * sin(azel[0]) / cos(phi * PI);
	/* geomagnetic latitude (semi-circle) */
	phi += 0.064 * cos((lam - 1.617) * PI);
	/* local time (s) */
	tt = 43200.0 * lam + time2gpst(t, &week);
	tt -= floor(tt / 86400.0) * 86400.0; /* 0<=tt<86400 */
	/* slant factor */
	f = 1.0 + 16.0 * pow(0.53 - azel[1] / PI, 3.0);
	/* ionospheric delay */
	amp = ion[0] + phi * (ion[1] + phi * (ion[2] + phi * ion[3]));
	per = ion[4] + phi * (ion[5] + phi * (ion[6] + phi * ion[7]));
	amp = amp < 0.0 ? 0.0 : amp;
	per = per < 72000.0 ? 72000.0 : per;
	x = 2.0 * PI * (tt - 50400.0) / per;
	return CLIGHT * f * (fabs(x) < 1.57 ? 5E-9 + amp * (1.0 + x * x * (-0.5 + x * x / 24.0)) : 5E-9);
}

extern double tropmodel(gtime_t time, const double* pos, const double* azel,
	double humi)
{
	const double temp0 = 15.0; /* temparature at sea level */
	double hgt = 0, pres = 0, temp = 0, e = 0, z = 0, trph = 0, trpw = 0;
	if (pos[2] < -100.0 || 1E4 < pos[2] || azel[1] <= 0) return 0.0;
	/* standard atmosphere */
	hgt = pos[2] < 0.0 ? 0.0 : pos[2];
	pres = 1013.25 * pow(1.0 - 2.2557E-5 * hgt, 5.2568);
	temp = temp0 - 6.5E-3 * hgt + 273.16;
	e = 6.108 * humi * exp((17.15 * temp - 4684.0) / (temp - 38.45));
	/* saastamoninen model */
	z = PI / 2.0 - azel[1];
	trph = 0.0022768 * pres / (1.0 - 0.00266 * cos(2.0 * pos[0]) - 0.00028 * hgt / 1E3) / cos(z);
	trpw = 0.002277 * (1255.0 / temp + 0.05) * e / cos(z);
	return trph + trpw;
}

#ifndef IERS_MODEL

static double interpc(const double coef[], double lat)
{
	int i = (int)(lat / 15.0);
	if (i < 1) return coef[0]; else if (i > 4) return coef[4];
	return coef[i - 1] * (1.0 - lat / 15.0 + i) + coef[i] * (lat / 15.0 - i);
}
static double mapf(double el, double a, double b, double c)
{
	double sinel = sin(el);
	return (1.0 + a / (1.0 + b / (1.0 + c))) / (sinel + (a / (sinel + b / (sinel + c))));
}
static double nmf(gtime_t time, const double pos[], const double azel[],
	double* mapfw)
{
	/* ref [5] table 3 */
	/* hydro-ave-a,b,c, hydro-amp-a,b,c, wet-a,b,c at latitude 15,30,45,60,75 */
	const double coef[][5] = {
		{ 1.2769934E-3, 1.2683230E-3, 1.2465397E-3, 1.2196049E-3, 1.2045996E-3},
		{ 2.9153695E-3, 2.9152299E-3, 2.9288445E-3, 2.9022565E-3, 2.9024912E-3},
		{ 62.610505E-3, 62.837393E-3, 63.721774E-3, 63.824265E-3, 64.258455E-3},

		{ 0.0000000E-0, 1.2709626E-5, 2.6523662E-5, 3.4000452E-5, 4.1202191E-5},
		{ 0.0000000E-0, 2.1414979E-5, 3.0160779E-5, 7.2562722E-5, 11.723375E-5},
		{ 0.0000000E-0, 9.0128400E-5, 4.3497037E-5, 84.795348E-5, 170.37206E-5},

		{ 5.8021897E-4, 5.6794847E-4, 5.8118019E-4, 5.9727542E-4, 6.1641693E-4},
		{ 1.4275268E-3, 1.5138625E-3, 1.4572752E-3, 1.5007428E-3, 1.7599082E-3},
		{ 4.3472961E-2, 4.6729510E-2, 4.3908931E-2, 4.4626982E-2, 5.4736038E-2}
	};
	const double aht[] = { 2.53E-5, 5.49E-3, 1.14E-3 }; /* height correction */

	double y = 0, cosy = 0, ah[3] = { 0 }, aw[3] = { 0 }, dm, el = azel[1], lat = pos[0] * R2D, hgt = pos[2];
	int i = 0;

	if (el <= 0.0) {
		if (mapfw) *mapfw = 0.0;
		return 0.0;
	}
	/* year from doy 28, added half a year for southern latitudes */
	y = (time2doy(time) - 28.0) / 365.25 + (lat < 0.0 ? 0.5 : 0.0);

	cosy = cos(2.0 * PI * y);
	lat = fabs(lat);

	for (i = 0; i < 3; i++) {
		ah[i] = interpc(coef[i], lat) - interpc(coef[i + 3], lat) * cosy;
		aw[i] = interpc(coef[i + 6], lat);
	}
	/* ellipsoidal height is used instead of height above sea level */
	dm = (1.0 / sin(el) - mapf(el, aht[0], aht[1], aht[2])) * hgt / 1E3;

	if (mapfw) *mapfw = mapf(el, aw[0], aw[1], aw[2]);

	return mapf(el, ah[0], ah[1], ah[2]) + dm;
}
#endif /* !IERS_MODEL */

/* troposphere mapping function ------------------------------------------------
* compute tropospheric mapping function by NMF
* args   : gtime_t t        I   time
*          double *pos      I   receiver position {lat,lon,h} (rad,m)
*          double *azel     I   azimuth/elevation angle {az,el} (rad)
*          double *mapfw    IO  wet mapping function (NULL: not output)
* return : dry mapping function
* note   : see ref [5] (NMF) and [9] (GMF)
*          original JGR paper of [5] has bugs in eq.(4) and (5). the corrected
*          paper is obtained from:
*          ftp://web.haystack.edu/pub/aen/nmf/NMF_JGR.pdf
*-----------------------------------------------------------------------------*/
extern double tropmapf(gtime_t time, const double* pos, const double* azel,
	double* mapfw)
{
#ifdef IERS_MODEL
	const double ep[] = { 2000,1,1,12,0,0 };
	double mjd, lat, lon, hgt, zd, gmfh, gmfw;
#endif

	if (pos[2] < -1000.0 || pos[2]>20000.0) {
		if (mapfw) *mapfw = 0.0;
		return 0.0;
	}
#ifdef IERS_MODEL
	mjd = 51544.5 + (timediff(time, epoch2time(ep))) / 86400.0;
	lat = pos[0];
	lon = pos[1];
	hgt = pos[2] - geoidh(pos); /* height in m (mean sea level) */
	zd = PI / 2.0 - azel[1];

	/* call GMF */
	gmf_(&mjd, &lat, &lon, &hgt, &zd, &gmfh, &gmfw);

	if (mapfw) *mapfw = gmfw;
	return gmfh;
#else
	return nmf(time, pos, azel, mapfw); /* NMF */
#endif
}

//* show message --------------------------------------------------------------*/
#if defined(WIN32)||defined(LINUX)
extern int showmsg(char* format, ...)
{
	va_list arg = { 0 };
	va_start(arg, format); vfprintf(stderr, format, arg); va_end(arg);
	fprintf(stderr, "\r");
	return 0;
}
extern int checkbrk(const char* format, ...)
{
	va_list arg = { 0 };
	char buff[1024], * p = buff;
	if (!*format) return showmsg("");
	va_start(arg, format);
	p += vsprintf(p, format, arg);
	va_end(arg);
	return showmsg(buff);
}
#endif
/* add obs data --------------------------------------------------------------*/
static int addobsdata(obs_t * obs, const obsd_t * data, int rcv)
{
	int i = 0;
	double dt = 0.0;
	dt = timediff(data->time, obs->data[0].time);
	if (dt != 0.0)//检测是否有更新的vrs数据，如果则之前存储的清空重新存储
		obs->n = 0;    //重新存储
	for (i = 0; i < obs->n; i++)
	{
		if (obs->data[i].sat == data->sat)  //查找原有obs中是否存在有该颗星的观测值,如果有则判断是否是最新的，如是则替换
		{
			dt = timediff(data->time, obs->data[i].time);
			if (dt > 0)
			{
				obs->data[i] = *data;
			}
			break;
		}
	}
	if (obs->n >= (MAXRECOBS * rcv))
	{
		trace(2, "need more obs room\r\n");
		return 0;
	}
	if (i == obs->n)
		obs->data[obs->n++] = *data;
	return 1;
}

/* crc-24q parity --------------------------------------------------------------
* compute crc-24q parity for sbas, rtcm3
* args   : unsigned char *buff I data
*          int    len    I      data length (bytes)
* return : crc-24Q parity
* notes  : see reference [2] A.4.3.3 Parity
*-----------------------------------------------------------------------------*/
extern unsigned int rtk_crc24q(const unsigned char* buff, int len)
{
	unsigned int crc = 0;
	int i = 0;

	//trace(4, "rtk_crc24q: len=%d\r\n", len);

	for (i = 0; i < len; i++) crc = ((crc << 8) & 0xFFFFFF) ^ tbl_CRC24Q[(crc >> 16) ^ buff[i]];
	return crc;
}

static int add_eph(nav_t * nav, const eph_t * eph)
{
	int i = 0;
	for (i = 0; i < nav->n; i++)
	{
		if (eph->sat == nav->eph[i].sat)  //查找原有星历中是否存在有该星历,如果有则判断是否是最新的星历，如是则替换
		{
			if (eph->week > nav->eph[i].week || (eph->week == nav->eph[i].week && ((eph->toe.time) > (nav->eph[i].toe.time))))
			{
				nav->eph[i] = *eph;
			}
			else if (fabs(eph->cis - nav->eph[i].cis) > ZEROS_MIN)
			{
				nav->eph[i] = *eph;
				trace(5, "the same time but the eph is diff,need update nav:sat=%d\n", eph->sat);
			}
			else {
				trace(5, "don't need update nav\n");
			}
			break;
		}
	}
	if (i == nav->n && nav->n < (MAXRECOBS * 2))
		nav->eph[nav->n++] = *eph;

	return 1;
}

static int add_geph(nav_t * nav, const geph_t * geph)
{
	nav->geph[nav->ng++] = *geph;
	return 1;
}

void free_rtcm(rtcm_t * rtcm)
{
	if (rtcm->obs.data) {
		xy_free(rtcm->obs.data); rtcm->obs.data = NULL; rtcm->obs.n = 0;
	}
	if (rtcm->nav.eph) {
		xy_free(rtcm->nav.eph); rtcm->nav.eph = NULL; rtcm->nav.n = 0;
	}
	if (rtcm->nav.geph)
	{
		xy_free(rtcm->nav.geph); rtcm->nav.geph = NULL; rtcm->nav.ng = 0;
	}
}

int init_rtcm(rtcm_t * rtcm, prelock_t * prelock, int rcv, int staid)
{
	gtime_t time0 = { 0 };
	obsd_t data0 = { 0 };
	eph_t  eph0 = { 0,-1,-1 };
	geph_t geph0 = { 0,-1 };
	lg69t_pvt_t lgpvt0 = { 0 };

	int i = 0, j = 0;
	if (rcv == 0)
		rcv = 1;

	rtcm->staid = rtcm->stah = rtcm->seqno = rtcm->outtype = rtcm->type = 0;
	rtcm->time = rtcm->time_s = time0;
	rtcm->sta.name[0] = rtcm->sta.marker[0] = '\0';
	rtcm->sta.antdes[0] = rtcm->sta.antsno[0] = '\0';
	rtcm->sta.rectype[0] = rtcm->sta.recver[0] = rtcm->sta.recsno[0] = '\0';
	rtcm->sta.antsetup = rtcm->sta.itrf = rtcm->sta.deltype = 0;
	for (i = 0; i < 3; i++) {
		rtcm->sta.pos[i] = rtcm->sta.del[i] = 0.0;
	}
	rtcm->sta.hgt = 0.0;

	rtcm->msg[0] = rtcm->msgtype[0] = rtcm->opt[0] = '\0';
	for (i = 0; i < 6; i++) rtcm->msmtype[i][0] = '\0';
	rtcm->obsflag = rtcm->ephsat = 0;

	if (prelock)  //注意星历的时候未输入
	{
		for (i = 0; i < MAXRECOBS * rcv; i++) {
			for (j = 0; j < NFREQ; j++)
			{
				rtcm->prelock[i][j].sat = prelock[i + MAXRECOBS * rcv * j].sat;
				rtcm->prelock[i][j].lock = prelock[i + MAXRECOBS * rcv * j].lock;
			}
		}
	}
	else
	{
		for (i = 0; i < MAXRECOBS * rcv; i++) {
			for (j = 0; j < NFREQ; j++)
			{
				rtcm->prelock[i][j].sat = 0;
				rtcm->prelock[i][j].lock = 0;
			}
		}
	}
	rtcm->nbyte = rtcm->nbit = rtcm->len = 0;
	rtcm->word = 0;
	for (i = 0; i < 100; i++) rtcm->nmsg2[i] = 0;
	for (i = 0; i < 400; i++) rtcm->nmsg3[i] = 0;

	rtcm->obs.data = NULL;
	rtcm->nav.geph = NULL;
	rtcm->nav.eph = NULL;

	/* reallocate memory for observation and ephemris buffer */
	if (!(rtcm->obs.data = (obsd_t*)xy_malloc(sizeof(obsd_t) * MAXRECOBS * rcv)) ||
		!(rtcm->nav.eph = (eph_t*)xy_malloc(sizeof(eph_t) * MAXRECOBS * 2)) ||
		!(rtcm->nav.geph = (geph_t*)xy_malloc(sizeof(geph_t) * 1))) {
		free_rtcm(rtcm);
		return 0;
	}

	rtcm->nav.n = MAXRECOBS * 2;
	rtcm->nav.ng = 0;//MAXPRNGLO;
	rtcm->obs.n = 0;
	for (i = 0; i < MAXRECOBS * rcv; i++)
	{
		rtcm->obs.data[i] = data0;
		rtcm->obs.data[i].rcv = rcv;
	}

	rtcm->staid = staid;

	for (i = 0; i < MAXRECOBS * 2; i++) rtcm->nav.eph[i] = eph0;
	for (i = 0; i < 0; i++) rtcm->nav.geph[i] = geph0;//MAXPRNGLO

	rtcm->lgpvt = lgpvt0;
	return 1;
}

extern int input_rtcm3(rtcm_t * rtcm, unsigned char data)
{
	// trace(5, "input_rtcm3: data=%02x \r\n", data);

   /* synchronize frame */
	if (rtcm->nbyte == 0) {
		if (data != RTCM3PREAMB)
			return 0;
		rtcm->buff[rtcm->nbyte++] = data;
		return 0;
	}
	rtcm->buff[rtcm->nbyte++] = data;

	if (rtcm->nbyte == 3) {
		rtcm->len = getbitu(rtcm->buff, 14, 10) + 3; /* length without parity */
	}
	if (rtcm->nbyte < 3 || rtcm->nbyte < rtcm->len + 3)
		return 0;
	rtcm->nbyte = 0;

	/* check parity */
	if (rtk_crc24q(rtcm->buff, rtcm->len) != getbitu(rtcm->buff, rtcm->len * 8, 24)) {
		trace(2, "rtcm3 parity error: len=%d\r\n", rtcm->len);
		return 0;
	}
	/* decode rtcm3 message */
	return decode_rtcm3(rtcm);
}

//分割电文
extern int split_rtcm3(rtcm_t * rtcm, unsigned char data)
{
	//trace(5, "input_rtcm3: data=%02x\r\n", data);

	/* synchronize frame */
	if (rtcm->nbyte == 0) {
		if (data != RTCM3PREAMB)
			return 0;
		rtcm->buff[rtcm->nbyte++] = data;
		return 0;
	}
	rtcm->buff[rtcm->nbyte++] = data;

	if (rtcm->nbyte == 3) {
		rtcm->len = getbitu(rtcm->buff, 14, 10) + 3; /* length without parity */
	}
	if (rtcm->nbyte < 3 || rtcm->nbyte < rtcm->len + 3)
		return 0;
	rtcm->nbyte = 0;

	/* check parity */
	if (rtk_crc24q(rtcm->buff, rtcm->len) != getbitu(rtcm->buff, rtcm->len * 8, 24)) {
		trace(2, "split:rtcm3 parity error: type=%4d len=%d\r\n", rtcm->type, rtcm->len);
		return 0;
	}
	rtcm->type = getbitu(rtcm->buff, 24, 12);
	rtcm->len += 3;

	return 1;
}

static void byte2hex(char* bytestr, char* result, int len) {
	char c0 = "", c1 = "";
	for (int i = 0; i < len; i++) {
		c0 = (bytestr[i] >> 4) & 0x0f;
		c1 = bytestr[i] & 0x0f;
		sprintf(result + 2 * i, "%01x", c0);
		sprintf(result + 2 * i + 1, "%01x", c1);
	}
}

extern int rtcmdataread(unsigned char* data, int length, obs_t * obs, nav_t * nav,
	double* pos, prelock_t * prelock, int rcv, rtk_t * rtk, int staid)
{
	int i, j = 0, ret = 0;
	rtcm_t rtcm;
	int stat = 0;

	if (data == NULL) {
		trace(2, "data is null\r\n");
		return -1;
	}

	init_rtcm(&rtcm, prelock, rcv, staid);

	for (i = 0; i < length; i++)
	{
		ret = input_rtcm3(&rtcm, data[i]);

		if ((rtcm.type == 1074 || rtcm.type == 1075 || rtcm.type == 1077        //GPS
			|| rtcm.type == 1094 || rtcm.type == 1095 || rtcm.type == 1097     //GAL
			|| rtcm.type == 1124 || rtcm.type == 1125 || rtcm.type == 1127 || rtcm.type == 1300))   //BDS
		{
			rtk->staid = rtcm.staid;
			for (j = 0; j < rtcm.obs.n; j++) {
				rtcm.obs.data[j].rcv = (unsigned char)rcv;

				/* save obs data */
				stat = addobsdata(obs, rtcm.obs.data + j, rcv);
			}

			if ((rtcm.type == 1300) && (pos != NULL))
			{
				for (j = 0; j < 3; j++) {
					pos[j] = rtcm.sta.pos[j];
				}
				trace(3, "pos     : id=%d x=%16.3f y=%16.3f z=%16.3f\r\n", rtk->staid, pos[0], pos[1], pos[2]);
			}
			//存储数据后将rtcm.obs.data结构体初始化
			memset(rtcm.obs.data, 0, sizeof(obsd_t) * MAXRECOBS * rcv);
			rtcm.obs.data[0].rcv = rcv;//判断一条星历是否卫星数是否超出限制时，需要用到该项
			rtcm.obs.n = 0;
			rtcm.type = 0;
		}
		else if ((ret == 2) && (nav != NULL))//ret=2表示为星历
		{
			if (rtcm.outtype == 1020)
				add_geph(nav, rtcm.nav.geph + rtcm.ephsat - 1);  /* GLONASS ephemeris */
			else
				add_eph(nav, rtcm.nav.eph + rtcm.ephsat - 1);      /* GPS/QZS/GAL ephemeris */
		}
		else if ((ret == 5) && (pos != NULL))//1005 ret=5表示为位置信息
		{
			rtk->staid = rtcm.staid;
			for (j = 0; j < 3; j++)
			{
				pos[j] = rtcm.sta.pos[j];
			}
		}
		else if (ret == 9)
		{
			memcpy(&rtk->lgpvt, &rtcm.lgpvt, sizeof(rtcm.lgpvt));
		}
	}
	free_rtcm(&rtcm);
	return ret;
}

/* compare ephemeris ---------------------------------------------------------*/
static int cmpeph(const void* p1, const void* p2)
{
	eph_t* q1 = (eph_t*)p1, * q2 = (eph_t*)p2;
	return q1->ttr.time != q2->ttr.time ? (int)(q1->ttr.time - q2->ttr.time) :
		(q1->toe.time != q2->toe.time ? (int)(q1->toe.time - q2->toe.time) :
			q1->sat - q2->sat);
}

/* sort and unique ephemeris -------------------------------------------------*/
static void uniqeph(nav_t * nav)
{
	int  i = 0, j = 0;
	//trace(3, "uniqeph: n=%d\r\n", nav->n);

	if (nav->n <= 0) return;

	qsort(nav->eph, nav->n, sizeof(eph_t), cmpeph);

	for (i = 1, j = 0; i < nav->n; i++) {
		if (nav->eph[i].sat != nav->eph[j].sat ||
			nav->eph[i].iode != nav->eph[j].iode) {
			nav->eph[++j] = nav->eph[i];
		}
	}
	nav->n = j + 1;
}

extern void uniqnav(nav_t * nav)
{
	int  i = 0, j = 0;
	//trace(3, "uniqnav: neph=%d ngeph=%d nseph=%d\r\n", nav->n, nav->ng, nav->ns);

	/* unique ephemeris */
	uniqeph(nav);
	/* update carrier wave length */
	for (i = 0; i < MAXSAT; i++) for (j = 0; j < NFREQ; j++) {
		nav->lam[i][j] = satwavelen(i + 1, j, nav);
	}
}

static void uniqeph2(nav_t * nav, gtime_t t)
{
	int i = 0, j = 0;
	double dt = 0.0;
	double tmax = 0.0;

	tmax = MAXSAVEDTOE;
	//trace(3, "uniqeph: n=%d\r\n", nav->n);

	if (nav->n <= 0) return;

	qsort(nav->eph, nav->n, sizeof(eph_t), cmpeph);

	for (i = 1, j = 0; i < nav->n; i++) {
		dt = timediff(t, nav->eph[i].toe);//观测时间与导航电文toe差值
		if ((nav->eph[i].sat != nav->eph[j].sat ||
			nav->eph[i].iode != nav->eph[j].iode) && dt < tmax)
		{
			nav->eph[++j] = nav->eph[i];
		}
	}
	nav->n = j + 1;
}

extern void uniqnav2(nav_t * nav, gtime_t t)
{
	int i = 0, j = 0;

	//trace(3, "uniqnav: neph=%d ngeph=%d nseph=%d\r\n", nav->n, nav->ng, nav->ns);

	/* unique ephemeris */
	uniqeph2(nav, t);
	/* update carrier wave length */
	for (i = 0; i < MAXSAT; i++) for (j = 0; j < NFREQ; j++) {
		nav->lam[i][j] = satwavelen(i + 1, j, nav);
	}
}

/* compare for qsort 从小到大排序*/
extern int compare(const void* p1, const void* p2)
{
	if (p1 == NULL || p2 == NULL) {
		//printf("rtcmn.cpp,compare, operate null pointer.\n");
		return 0;
	}
	double* q1 = (double*)(p1), * q2 = (double*)(p2);
	double deta = *q1 - *q2;
	return deta < 0 ? -1 : 1;
}

/* compare for qsort */
extern int compare2(const void* p1, const void* p2)
{
	if (p1 == NULL || p2 == NULL) {
		//printf("rtcmn.cpp,compare, operate null pointer.\n");
		return 0;
	}
	double* q1 = (double*)(p1), * q2 = (double*)(p2);
	double deta = *q1 - *q2;
	return deta > 0 ? -1 : 1;
}

/* test navi system (m=0:gps/qzs/sbs,1:glo,2:gal,3:bds) ----------------------*/
extern int test_sys(int sys, int m) {
	switch (sys) {
	case SYS_GPS:
		return m == 0;
	case SYS_QZS:
		return m == 0;
	case SYS_SBS:
		return m == 0;
	case SYS_GLO:
		return m == 1;
	case SYS_GAL:
		return m == 2;
	case SYS_CMP:
		return m == 3;
	}
	return 0;
}

/* test navi system (m=0:gps/qzs/sbs,1:glo,2:gal,3:bds) ----------------------*/
extern int get_sys_num(int sys) {
	switch (sys) {
	case SYS_GPS:
		return  0;
	case SYS_GLO:
		return  1;
	case SYS_GAL:
		return  2;
	case SYS_CMP:
		return  3;
	}
	return -1;
}

/* zero Matrix -----------------------------------------------------------------
* generate new zero Matrix
* args   : int    n,m       I   number of rows and columns of Matrix
* return : Matrix pointer (if n<=0 or m<=0, return NULL)
*-----------------------------------------------------------------------------*/
extern int* izeros(int n, int m)
{
	int* p = NULL;
#if defined(WIN32)|| defined(LINUX)
#if NOCALLOC
	if ((p = mat(n, m))) for (n = n * m - 1; n >= 0; n--) p[n] = 0.0;
#else
	if (n <= 0 || m <= 0)
		return NULL;
	if (!(p = (int*)calloc(sizeof(int), n * m))) {
		trace(3, "matrix memory allocation error: n=%d,m=%d\n", n, m);
	}
#endif
#else
	if ((p = imat(n, m))) for (n = n * m - 1; n >= 0; n--) p[n] = 0.0;
#endif
	return p;
}

/*返回系统最大卫星PRN值
*sys int 系统编号值
* m  int 0:系统最大PRN值，1：最小卫星PRN值
* return 最大/最小PRN值，系统未知时返回-1
*/
extern int maxminsat(int sys, int m) {
	switch (sys) {
	case 0:
		return m == 0 ? MAXPRNGPS : MINPRNGPS;//SYS_GPS
	case 1:
		return m == 0 ? MAXPRNGLO : MINPRNGLO;//SYS_GLO
	case 3:
		return m == 0 ? MAXPRNCMP : MINPRNCMP;//SYS_CMP
	}
	return -1;
}


extern int select_sat(obs_t obsu, obs_t obsr, nav_t * nav, prcopt_t popt, obsd_t * obs, double* rb)
{
	int n = 0;  
	int n1 = 0; 
	int n2 = 0;  
	int n3 = 0;  

	int i = 0, j = 0;
	int iflag[MAXRECOBS] = { 0 };
	obsd_t tobs[MAXRECOBS] = { 0 };
	double elev[MAXRECOBS * 2] = { 0.0 }, t_elev[MAXRECOBS] = { 0.0 };//elev[方位角,高度角]
	int nobs = 0;
	int maxsatnum = MAXRECOBS;    


	if (norm(rb, 3) < 0.001){
		for (i = 0; i < obsu.n; i++)
			obs[i] = obsu.data[i];
		return obsu.n;  
	}
	
	if (popt.nf == 2){
		maxsatnum = 20;   
	}

	cal_azel(popt, obsu.data, obsu.n, nav, rb, elev);

	for (i = 0; i < obsu.n && i < MAXRECOBS; i++)
	{
		if ((satsys(obsu.data[i].sat, &obsu.data[i].trueid) & popt.navsys)
			&& (obsu.data[i].snr[0] * 0.25) > popt.snrthres             
			&& seleph(obsu.data[i].time, obsu.data[i].sat, -1, nav)
			&& (elev[2 * i + 1] >= popt.elmin))
		{
			elev[2 * n] = elev[2 * i];
			elev[2 * n + 1] = elev[2 * i + 1];
			obsu.data[n++] = obsu.data[i];
		}
	}

	trace(2, "select_sat：：Basic conditions eliminate：the agree sat num:=%3d maxsatnum=%3d\r\n", n, maxsatnum);
	if (n > maxsatnum)
	{
		trace(5, "select_sat：：Second satellite Selection...\r\n");
		for (i = 0; i < n && i < MAXRECOBS; i++)
		{
			if (obsr.n > 0)
			{
				for (j = 0; j < obsr.n && j < (MAXRECOBS * 2); j++)
				{
					if (obsu.data[i].sat == obsr.data[j].sat)  
					{
						if (obsu.data[i].P[0] != 0.0 && obsu.data[i].P[1] != 0.0)   
						{
							iflag[i] = 1;           
							n2++;
						}
						else{
							iflag[i] = 0;           //iflag=0表示有共视星但双频数据不全
							n1++;
						}
						break;                   
					}
				}
				if (j == obsr.n || j == (MAXRECOBS * 2)){
					iflag[i] = -1;                  //无共视卫星，优先级最低
					n3++;
				}
			}else {
				iflag[i] = -1;                  //无共视卫星，优先级最低
				n3++;
			}
		}
		trace(5, "maxsatnum=%d  commonviewingDual-frequency= %d  and  commonviewingnoDual-frequenc= %d  nocommonviewing=%d \n", maxsatnum, n2, n1, n3);

		j = 0;
		if (n2 > maxsatnum)  
		{
			for (i = 0; i < n && i < MAXRECOBS; i++)
			{
				if (iflag[i] == 1)
				{
					t_elev[j] = elev[2 * i + 1];
					tobs[j++] = obsu.data[i];
				}
			}

			qsort(t_elev, j, sizeof(double), compare2);

			for (i = 0; i < n2; i++)
			{
				if (elev[2 * i + 1] > t_elev[maxsatnum])
				{
					obs[nobs++] = tobs[i];
				}
			}
		}
		else {    
			if (n2 + n1 > maxsatnum)
			{
				for (i = 0; i < n && i < MAXRECOBS; i++)  
				{
					if (iflag[i] == 0)
					{
						t_elev[j++] = elev[2 * i + 1];
					}
				}

				qsort(t_elev, j, sizeof(double), compare2);

				for (i = 0; i < n && i < MAXRECOBS; i++) 
				{
					if (iflag[i] == 0 && elev[2 * i + 1] > t_elev[maxsatnum - n2])
					{
						obs[nobs++] = obsu.data[i];
					}
					if (iflag[i] == 1)
					{
						obs[nobs++] = obsu.data[i];
					}
				}
			}else {
				for (i = 0; i < n && i < MAXRECOBS; i++) 
				{
					if (iflag[i] < 0){
						t_elev[j++] = elev[2 * i + 1];
					}
				}

				qsort(t_elev, j, sizeof(double), compare2);
				for (i = 0; i < n && i < MAXRECOBS; i++) 
				{
					if (iflag[i]<0 && elev[2 * i + 1]>t_elev[maxsatnum - n2 - n1])
					{
						obs[nobs++] = obsu.data[i]; 
					}
					if (iflag[i] == 1 || iflag[i] == 0)
					{
						obs[nobs++] = obsu.data[i];
					}
				}
			}
		}
	}
	else {
		//选取成功，保存
		nobs = n;
		for (i = 0; i < n; i++)
			obs[i] = obsu.data[i];
	}
	return nobs;
}

//rtd、rtk同步spp的一些钟差、dop值、定速标识位等无需重新计算但在输出或者下个历元有用到的信息
extern void syncsolinfo(sol_t sol, sol_t * solnew)
{
	int i = 0;
	solnew->vtat = sol.vtat;
	solnew->age = sol.age;
	solnew->first_flag = sol.first_flag;
	for (i = 0; i < 6; i++) solnew->dtr[i] = sol.dtr[i];
	for (i = 0; i < 4; i++){
		solnew->rdop[i] = sol.rdop[i];
		solnew->nsat[i] = sol.nsat[i];
	}
}

int qvsort(int n, double* qv) {
	int i = 0, j = 0, k = 0;
	double temp = 0, t_ = 0;

	double l[MAXPNTOBS * 2 * NFREQ] = { 0.0 };
	for (i = 0; i < n; i++)
		l[i] = qv[i];
	for (i = 0; i < n - 1; i++) {
		k = i;
		for (j = i + 1; j < n; j++) {
			if (l[k] < l[j]) {
				temp = l[i];
				t_ = qv[i];
				l[i] = l[j];
				qv[i] = qv[j];
				l[j] = temp;
				qv[j] = t_;
			}
		}
	}
	return 1;
}
/*
* mid O 双差残差中位数
* std O 带最大最小值限制的双差产残差中位数std
*/
void calmidandstd(double* vq, int num, double* mid, double* std)
{
	int i = 0;
	double mintemp = 0.0, stdtemp = 0.0;
	qvsort(num, vq);
	mintemp = vq[num / 2];

	for (i = 0; i < num; i++) {
		stdtemp += (vq[i] - mintemp) * (vq[i] - mintemp);
	}
	stdtemp = sqrt(stdtemp / num);
	/*限制最大最小std*/
	stdtemp = MAX(stdtemp, 0.3);
	stdtemp = MIN(stdtemp, 3); 
	mintemp = MIN(mintemp, 3);
	*mid = mintemp;
	*std = stdtemp;
}

void rtd_igg3cal(rtk_t * rtk, const obsd_t * obs, const double* p, double* h, double* v, double* rvar, int* sat, int n, int m, int* vflg, midmsg_t * midmsg)
{
	int i = 0;
	double* vq = mat(m, 1);double* vpq = mat(m, 1);double* vdq = mat(m, 1);
	int dnum = 0, pnum = 0;
	double pmid = 0.0, pstd = 0.0, dmid = 0.0, dstd = 0.0, kv = 0.0;
	double k1 = 1.5, k2 = 3.0;
	int sati = 0, satj = 0, type = 0, freq = 0, j = 0;
	char* stype = NULL;
	int* flag = izeros(m, 1);
	/*提取相关数据*/
	for (i = 0; i < m; i++){
		satj = (vflg[i] >> 8) & 0xFF;
		type = (vflg[i] >> 4) & 0xF;
		freq = vflg[i] & 0xF;
		//r=MIN(sqrt(rvar[i+i*m]),3.0);
		vq[i] = v[i];//r;
		if (limitcmpgeo(obs[satj].sat) <= SYS_NONE) continue;
		if (type == 0){//多普勒
			if (midmsg[satj].ddop1) continue;
			flag[i] = 1;
			vdq[dnum++] = vq[i];
		}else {//载波伪距
			if (midmsg[satj].dpsr1) continue;
			flag[i] = 1;
			vpq[pnum++] = vq[i];
		}
	}
	trace(5, "rtd_igg3cal::filter_ double dif:\n");
	tracemat(5, vpq, 1, pnum, 9, 3);
	tracemat(5, vdq, 1, dnum, 9, 3);
	calmidandstd(vpq, pnum, &pmid, &pstd);
	calmidandstd(vdq, dnum, &dmid, &dstd);
	pstd = MAX(pstd, 1.0); 

	trace(5, "rtd_igg3cal:: pmid=%9.3f pstd=%9.3f dmid=%9.3f dstd=%9.3f\n", pmid, pstd, dmid, dstd);

	for (i = 0; i < m; i++)
	{
		type = (vflg[i] >> 4) & 0xF;
		kv = type ? fabs(vq[i]) / pstd : fabs(vq[i] - dmid) / dstd;
		k2 = flag[i] ? 3.0 : 2.9;
		if (kv > k1 && kv < k2){
			rvar[i + i * m] *= (k2 - k1) / (k2 - kv) * 10;
			trace(5, "IGG3 1:v=%9.3f vq=%9.3f kv=%9.3f %9.3f\n", v[i], vq[i], kv, (k2 - k1) / (k2 - kv));
		}
		else if (kv >= k2){/*有粗差处理*/
			if (vflg == NULL) continue;
			/*打粗差日志*/
			sati = (vflg[i] >> 16) & 0xFF;satj = (vflg[i] >> 8) & 0xFF;
			type = (vflg[i] >> 4) & 0xF;freq = vflg[i] & 0xF;
			stype = type == 0 ? "D" : (type == 1 ? "P" : "C");
			trace(5, "IGG3 refuse (sat=%3d-%3d %s%d v=%6.3f sig=%.3f vq=%9.3f kv=%9.3f)\n",
				obs[sati].sat, obs[satj].sat, stype, freq + 1, v[i], SQRT(rvar[i + i * m]), vq[i], kv);
			/*记录粗差卫星*/
			if (type == 1){
				midmsg[satj].dpsr2[freq] = 1;
			}else {
				midmsg[satj].ddop2[freq] = 1;
			}
			/*方差无限大*/
			rvar[i + i * m] *= 1e16;
		}
	}

	xy_free(vq); xy_free(vpq); xy_free(vdq); xy_free(flag);
}

extern void rtd_igg3test(rtk_t * rtk, const obsd_t * obs, double* x, const double* p, const double* h, double* v, double* rvar, int* sat, int n, int m, int* vflg, midmsg_t * midmsg)
{
	int i = 0, j = 0, k = 0;
	double* p_ = NULL, * h_ = NULL;
	int ix[MAXOBSNFA] = { 0 };
	/*数据提取*/
	for (i = k = 0; i < n; i++) {
		if (x[i] != 0.0 && p[i + i * n] > 0.0) ix[k++] = i;
	}
	p_ = zeros(k, k);  h_ = zeros(k, m);
	for (i = 0; i < k; i++) {
		for (j = 0; j < k; j++) p_[i + j * k] = p[ix[i] + ix[j] * n];
		for (j = 0; j < m; j++) h_[i + j * k] = h[ix[i] + j * n];
	}

	rtd_igg3cal(rtk, obs, p_, h_, v, rvar, sat, k, m, vflg, midmsg);
	xy_free(p_); xy_free(h_);
}

void rtk_igg3cal(rtk_t * rtk, int rflag, const obsd_t * obs, const double* p, double* h, double* v, double* rvar, const int* sat, int n, int m, int* vflg, midmsg_t * midmsg)
{
	int i = 0;
	double* q = mat(m, m);
	double* ff = mat(n, m);
	double* vq = mat(m, 1);
	double* vpq = mat(m, 1);
	double* vdq = mat(m, 1);
	int dnum = 0, pnum = 0;
	double pmid = 0.0, pstd = 0.0, dmid = 0.0, dstd = 0.0, kv = 0.0;
	double k1 = 1.5, k2 = 6.0;
	int sati = 0, satj = 0, type = 0, freq = 0, j = 0;
	char* stype = NULL;
	if (vflg == NULL || n * m < ZEROS_MIN){
		xy_free(vq); xy_free(vpq); xy_free(vdq);
		xy_free(ff); xy_free(q);
		return;
	}

	matcpy(q, rvar, m, m);
	matmul("NN", n, m, n, 1.0, p, h, 0.0, ff);       /* Q=H'*P*H+R */
	matmul("TN", m, m, n, 1.0, h, ff, 1.0, q);

	for (i = 0; i < m; i++){
		satj = (vflg[i] >> 8) & 0xFF;
		type = (vflg[i] >> 4) & 0xF;
		freq = vflg[i] & 0xF;

		if (sqrt(q[i + i * m]) < ZEROS_MIN) continue;
		vq[i] = v[i] / sqrt(q[i + i * m]);

		if (limitcmpgeo(obs[satj].sat) <= SYS_NONE) continue;

		if (type == 0){
			if (midmsg[satj].dpsr1) continue;
			vdq[dnum++] = vq[i];
		}else {
			if (midmsg[satj].dpsr1) continue;
			vpq[pnum++] = vq[i];
		}
	}

	trace(5, "filter_ vq:\n");
	tracemat(5, vpq, 1, pnum, 9, 3);
	tracemat(5, vdq, 1, dnum, 9, 3);

	calmidandstd(vpq, pnum, &pmid, &pstd);
	calmidandstd(vdq, dnum, &dmid, &dstd);

	trace(5, "rtk kv: pmid=%9.3f pstd=%9.3f dmid=%9.3f dstd=%9.3f\n", pmid, pstd, dmid, dstd);

	for (i = 0; i < m; i++)
	{
		sati = (vflg[i] >> 16) & 0xFF;
		satj = (vflg[i] >> 8) & 0xFF;
		type = (vflg[i] >> 4) & 0xF;
		freq = vflg[i] & 0xF;
		stype = type == 0 ? "L" : (type == 1 ? "P" : "C");
		if (type){
			kv = fabs(vq[i]) / pstd;  
		}else {
			kv = fabs(vq[i]) / dstd;
		}
		if (kv > k1 && kv < k2){
			rvar[i + i * m] *= (k2 - k1) / (k2 - kv) * 10;
			trace(5, "rtk IGG3 1:sat=%3d-%3d %s%d v=%9.3f vq=%9.3f kv=%9.3f %9.3f\n", obs[sati].sat, obs[satj].sat, stype, freq + 1, v[i], vq[i], kv, (k2 - k1) / (k2 - kv));
#if  1
			if (type == 0 && rflag){
				j = ix(obs[satj].sat, rtk->satid, rtk->presat);
				if (rtk->ssat[j].lock[freq] > 0) rtk->ssat[j].lock[freq] -= 1;
			}
#endif
		}else if (kv >= k2){
			trace(5, "IGG3 refuse (sat=%3d-%3d %s%d v=%6.3f sig=%.3f vq=%9.3f kv=%9.3f)\n",obs[sati].sat, obs[satj].sat, stype, freq + 1, v[i], SQRT(rvar[i + i * m]), vq[i], kv);

			j = ix(obs[satj].sat, rtk->satid, rtk->presat);
			rtk->ssat[j].vsat[freq] = 0;

			if (type){
				midmsg[satj].dpsr2[freq] = 1;
			}else {
				midmsg[satj].dlsr2[freq] = 1;
			}
			rvar[i + i * m] *= 1e16;
		}
	}

	xy_free(vq); xy_free(vpq); xy_free(vdq);
	xy_free(ff); xy_free(q);
}

extern void rtk_igg3test(rtk_t * rtk, int rflag, const obsd_t * obs, double* x, const double* p, const double* h, double* v, double* rvar, const int* sat, int n, int m, int* vflg, midmsg_t * midmsg)
{
	int i = 0, j = 0, k = 0;
	double* p_ = NULL, * h_ = NULL;
	int ix[MAXOBSNFA] = { 0 };

	//数据提取
	for (i = k = 0; i < n; i++) {
		if (x[i] != 0.0 && p[i + i * n] > 0.0)
			ix[k++] = i;
	}
	p_ = zeros(k, k);  h_ = zeros(k, m);

	for (i = 0; i < k; i++) {
		for (j = 0; j < k; j++) p_[i + j * k] = p[ix[i] + ix[j] * n];
		for (j = 0; j < m; j++) h_[i + j * k] = h[ix[i] + j * n];
	}

	rtk_igg3cal(rtk, rflag, obs, p_, h_, v, rvar, sat, k, m, vflg, midmsg);
	xy_free(p_); xy_free(h_);
}

//返回系统编号，同时限制北斗GEO卫星
extern int limitcmpgeo(int sat)
{
	int sys = SYS_NONE, prn = 0;
	sys = satsys(sat, &prn);
#if LIMITBDSGEO
	if (sys == SYS_CMP && BDSGEOFLAG(prn))
		return -1;
#endif
	return sys;
}