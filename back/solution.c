#include "rtklib.h"

int outflag = 1;//是否直接输出结果
int filternumber;//统计参与均值滤波解的个数
double dirp = 400.0;
double xfliter[3];//存储均值滤波解

/* constants and macros ------------------------------------------------------*
#define MAXFIELD   64           /* max number of fields in a record */
#define MAXNMEA    256          /* max length of nmea sentence */

#define KNOT2M     0.514444444  /* m/knot */

static const int solq_nmea[] = {  /* nmea quality flags to rtklib sol quality */
	/* nmea 0183 v.2.3 quality flags: */
	/*  0=invalid, 1=gps fix (sps), 2=dgps fix, 3=pps fix, 4=rtk, 5=float rtk */
	/*  6=estimated (dead reckoning), 7=manual input, 8=simulation */

	SOLQ_NONE ,SOLQ_SINGLE, SOLQ_DGPS, SOLQ_PPP , SOLQ_FIX,
	SOLQ_FLOAT,SOLQ_DR    , SOLQ_NONE, SOLQ_NONE, SOLQ_NONE
};

/* output solution in the form of nmea RMC sentence --------------------------*/
extern int outnmea_rmc(unsigned char* buff, const sol_t* sol)
{
	gtime_t time = { 0 };
	double ep[6] = { 0 }, pos[3] = { 0 }, enuv[3] = { 0 }, dms1[3] = { 0 }, dms2[3] = { 0 }, vel = 0, dir = 0, amag = 0.0;
	char* p = (char*)buff, * q = NULL, sum = ' ', * emag = "E";

	trace(3, "outnmea_rmc:\n");

	if (sol->stat <= SOLQ_NONE) {
		p += sprintf(p, "$GPRMC,,,,,,,,,,,,");
		for (q = (char*)buff + 1, sum = 0; *q; q++) sum ^= *q;
		trace(2, "$GPRMC,,,,,,,,,,,,*%02X\n", sum);
		p += sprintf(p, "*%02X", sum);
		return p - (char*)buff;
	}
	time = gpst2utc(sol->time);
	if (time.sec >= 0.995) { time.time++; time.sec = 0.0; }
	time2epoch(time, ep);
	ecef2pos(sol->rr, pos);
	ecef2enu(pos, sol->rr + 3, enuv);
	vel = norm(enuv, 3);
	if (vel >= 1.0) {
		dir = atan2(enuv[0], enuv[1]) * R2D;
		if (dir < 0.0) dir += 360.0;
		dirp = dir;
	}
	else {
		dir = dirp;
	}
	deg2dms(fabs(pos[0]) * R2D, dms1, 7);
	deg2dms(fabs(pos[1]) * R2D, dms2, 7);
	p += sprintf(p, "$GPRMC,%02.0f%02.0f%05.2f,A,%02.0f%010.7f,%s,%03.0f%010.7f,%s,%4.2f,%4.2f,%02.0f%02.0f%02d,%.1f,%s,%s",
		ep[3], ep[4], ep[5], dms1[0], dms1[1] + dms1[2] / 60.0, pos[0] >= 0 ? "N" : "S",
		dms2[0], dms2[1] + dms2[2] / 60.0, pos[1] >= 0 ? "E" : "W", vel / KNOT2M, dir,
		ep[2], ep[1], (int)ep[0] % 100, amag, emag,
		sol->stat == SOLQ_DGPS || sol->stat == SOLQ_FLOAT || sol->stat == SOLQ_FIX ? "D" : "A");
	for (q = (char*)buff + 1, sum = 0; *q; q++) sum ^= *q; /* check-sum */

	p += sprintf(p, "*%02X", sum);
	return p - (char*)buff;
}
//输出RMC和ENU速度
extern int outnmea_rmc2(unsigned char* buff, double* enuv1, const sol_t* sol)
{
    /* 静态变量保存上次方向值 */
    static double dirp = 0.0;
    
    /* 变量声明与初始化 */
    gtime_t time = {0};                 /* UTC时间 */
    double ep[6] = {0.0};               /* 历元时间 [年,月,日,时,分,秒] */
    double pos[3] = {0.0};              /* 位置坐标 [纬度,经度,高度] */
    double enuv[3] = {0.0};             /* ENU速度分量 */
    double dms1[3] = {0.0};             /* 纬度度分秒 */
    double dms2[3] = {0.0};             /* 经度度分秒 */
    double vel = 0.0;                   /* 速度大小 */
    double dir = 0.0;                   /* 方向角 */
    double amag = 0.0;                  /* 磁偏角 */
    char* p = (char*)buff;              /* 输出缓冲区指针 */
    char* q = NULL;                     /* 校验和计算用指针 */
    char sum = 0;                       /* 校验和 */
    char* emag = "E";                   /* 磁偏角方向 */
    
    /* 调试输出 */
    trace(3, "outnmea_rmc:\n");
    
    /* 初始化输出速度 */
    for (int i = 0; i < 3; i++) {
        enuv1[i] = 0;
    }
    
    /* 检查解算状态，无效解直接返回空RMC语句 */
    if (sol->stat <= SOLQ_NONE) {
        p += sprintf(p, RMC0);
        return (int)(p - (char*)buff);
    }
    
    /* GPS时间转UTC时间 */
    time = gpst2utc(sol->time);
    
    /* 处理时间进位 */
    if (time.sec >= 0.995) { 
        time.time++; 
        time.sec = 0.0; 
    }
    
    /* 时间转换为年月日时分秒格式 */
    time2epoch(time, ep);
    
    /* 坐标变换：ECEF->大地坐标系 */
    ecef2pos(sol->rr, pos);
    
    /* 速度变换：ECEF->ENU */
    ecef2enu(pos, sol->rr + 3, enuv);
    
    /* 计算速度大小 */
    vel = norm(enuv, 3);
    
    /* 保存速度分量供外部使用 */
    for (int i = 0; i < 3; i++) {
        enuv1[i] = enuv[i];
    }
    
    /* 计算运动方向 */
    if (vel >= 1.0) {
        /* 速度足够大时，计算当前方向 */
        dir = atan2(enuv[0], enuv[1]) * R2D;
        if (dir < 0.0) {
            dir += 360.0;
        }
        dirp = dir;  /* 保存当前方向供下次使用 */
    } else {
        /* 速度较小时，使用上次方向 */
        dir = dirp;
    }
    
    /* 坐标转换为度分秒格式 */
    deg2dms(fabs(pos[0]) * R2D, dms1, 7);  /* 纬度 */
    deg2dms(fabs(pos[1]) * R2D, dms2, 7);  /* 经度 */
    
    /* 生成NMEA-RMC语句 */
    p += sprintf(p, "$GPRMC,%02.0f%02.0f%05.2f,A,%02.0f%010.7f,%s,%03.0f%010.7f,%s,%4.2f,%4.2f,%02.0f%02.0f%02d,%.1f,%s,%s",
        ep[3], ep[4], ep[5],                                   /* 时分秒 */
        dms1[0], dms1[1] + dms1[2] / 60.0, pos[0] >= 0 ? "N" : "S",  /* 纬度 */
        dms2[0], dms2[1] + dms2[2] / 60.0, pos[1] >= 0 ? "E" : "W",  /* 经度 */
        vel / KNOT2M, dir,                                     /* 速度(节)和方向 */
        ep[2], ep[1], (int)ep[0] % 100,                        /* 日月年 */
        amag, emag,                                            /* 磁偏角 */
        sol->stat == SOLQ_DGPS || sol->stat == SOLQ_FLOAT || sol->stat == SOLQ_FIX ? "D" : "A");  /* 模式指示符 */
    
    /* 计算校验和 */
    for (q = (char*)buff + 1, sum = 0; *q; q++) {
        sum ^= *q;
    }
    
    /* 添加校验和 */
    p += sprintf(p, "*%02X", sum);
    
    /* 返回消息长度 */
    return p - (char*)buff;
}

int outnmea_gga(unsigned char* buff, const sol_t* sol) {
    /* NMEA质量指示符到RTKLIB解算状态的映射 */
    const int solq_nmea[] = {
        /* 0=invalid, 1=gps fix (sps), 2=dgps fix, 3=pps fix, 4=rtk, 5=float rtk */
        /* 6=estimated (dead reckoning), 7=manual input, 8=simulation */
        SOLQ_NONE, SOLQ_SINGLE, SOLQ_DGPS, SOLQ_PPP, SOLQ_FIX,
        SOLQ_FLOAT, SOLQ_DR, SOLQ_NONE, SOLQ_NONE, SOLQ_NONE
    };

    /* 变量声明与初始化 */
    gtime_t time = {0};               /* UTC时间 */
    double ep[6] = {0.0};             /* 历元时间 [年,月,日,时,分,秒] */
    double pos[3] = {0.0};            /* 位置坐标 [纬度,经度,高度] */
    double dms1[3] = {0.0};           /* 纬度度分秒 */
    double dms2[3] = {0.0};           /* 经度度分秒 */
    double h = 0.0;                   /* 大地水准面高 */
    double dop = 1.0;                 /* HDOP值 */
    double age = 0.0;                 /* 差分龄期 */
    double enuv[3] = {0.0};           /* ENU速度分量 */
    double vel = 0.0;                 /* 速度大小 */
    double dir = 0.0;                 /* 方向角 */
    int solq = 0;                     /* NMEA质量指示符 */
    int staid = 0;                    /* 基站ID */
    int ns = 0;                       /* 卫星数 */
    char* p = (char*)buff;            /* 输出缓冲区指针 */
    char* q = NULL;                   /* 校验和计算用指针 */
    char sum = 0;                     /* 校验和 */
    
    /* 检查解算状态，允许输出单点解 */
    int stat = sol->stat <= SOLQ_NONE;
    
    /* 设置基站ID */
    if (sol->stat == SOLQ_SINGLE) {
        staid = 0;
    } else {
        staid = sol->staid;
    }

    /* 无效解返回空GGA语句 */
    if (stat) {
        p += sprintf(p, GGA0);
        return (int)(p - (char*)buff);
    }

    /* 将RTKLIB解算状态转换为NMEA质量指示符 */
    for (solq = 0; solq < 8; solq++) {
        if (solq_nmea[solq] == sol->stat) {
            break;
        }
    }
    if (solq >= 8) {
        solq = 0;  /* 默认为无效解 */
    }

    /* 获取卫星数量 */
    ns = sol->ns;
    
    /* 坐标变换：ECEF->大地坐标系 */
    ecef2pos(sol->rr, pos);

    /* GPS时间转UTC时间并处理进位 */
    time = gpst2utc(sol->time);
    if (time.sec >= 0.995) {
        time.time++;
        time.sec = 0.0;
    }
    
    /* 时间转换为年月日时分秒格式 */
    time2epoch(time, ep);
    
    /* 处理差分龄期 */
    age = sol->age;
    if (sol->stat == 5 || sol->stat == 0) {
        age = 0;
        staid = 0;
    }
    
    /* 单点定位时基站ID为0 */
    if (solq == 1) {
        staid = 0;
    }
    
    /* 计算大地水准面高，注释掉的是原始代码 */
    /* h = geoidh(filtersol.pos); */
    
    /* 坐标转换为度分秒格式 */
    deg2dms(fabs(pos[0]) * R2D, dms1, 7);  /* 纬度 */
    deg2dms(fabs(pos[1]) * R2D, dms2, 7);  /* 经度 */
    
    /* 计算速度和方向 */
    ecef2enu(pos, sol->rr + 3, enuv);
    vel = norm(enuv, 3);
    if (vel >= 1.0) {
        dir = atan2(enuv[0], enuv[1]) * R2D;
        if (dir < 0.0) {
            dir += 360.0;
        }
        dirp = dir;  /* 全局变量保存方向 */
    } else {
        dir = dirp;
    }
    
    /* 获取HDOP值 */
    dop = sol->rdop[2];
    
    /* 调试输出GGA语句 */
    trace(4, "$GPGGA,%02.0f%02.0f%05.2f,%02.0f%010.7f,%s,%03.0f%010.7f,%s,%d,%02d,%.1f,%.3f,M,%.3f,M,%.1f,%d\r\n",
          ep[3], ep[4], ep[5], dms1[0], dms1[1] + dms1[2] / 60.0, pos[0] >= 0 ? "N" : "S",
          dms2[0], dms2[1] + dms2[2] / 60.0, pos[1] >= 0 ? "E" : "W", solq,
          sol->ns, dop, pos[2] - h, h, age, staid);
    
    /* 生成NMEA-GGA语句 */
    p += sprintf(p, "$GPGGA,%02.0f%02.0f%05.2f,%02.0f%010.7f,%s,%03.0f%010.7f,%s,%d,%02d,%.1f,%.3f,M,%.3f,M,%.1f,%d",
                ep[3], ep[4], ep[5],                                   /* 时分秒 */
                dms1[0], dms1[1] + dms1[2] / 60.0, pos[0] >= 0 ? "N" : "S",  /* 纬度 */
                dms2[0], dms2[1] + dms2[2] / 60.0, pos[1] >= 0 ? "E" : "W",  /* 经度 */
                solq,                                                  /* 定位质量指示符 */
                ns, dop,                                               /* 卫星数和HDOP */
                pos[2] - h, h,                                         /* 高度和大地水准面差 */
                age, staid);                                           /* 差分龄期和基站ID */

    /* 计算校验和 */
    for (q = (char*)buff + 1, sum = 0; *q; q++) {
        sum ^= *q;
    }
    
    /* 添加校验和 */
    p += sprintf(p, "*%02X", sum);
    
    /* 返回消息长度 */
    return p - (char*)buff;
}

/* output solution in the form of nmea ZDA sentence --------------------------*/
extern int outnmea_zda(unsigned char* buff, const sol_t* sol)
{
    /* 变量声明与初始化 */
    gtime_t time = {0};               /* UTC时间 */
    double ep[6] = {0.0};             /* 历元时间 [年,月,日,时,分,秒] */
    char* p = (char*)buff;            /* 输出缓冲区指针 */
    char* q = NULL;                   /* 校验和计算用指针 */
    char sum = 0;                     /* 校验和 */
    
    trace(3, "outnmea_zda:\n");
    
    /* 解算状态检查：无效解则输出空ZDA语句 */
    if (sol->stat <= SOLQ_NONE) {
        p += sprintf(p, "$GPZDA,,,,,,*");
        for (q = (char*)buff + 1, sum = 0; *q; q++) sum ^= *q;
        p += sprintf(p, "%02X", sum);
        return p - (char*)buff;
    }
    
    /* GPS时间转UTC时间 */
    time = gpst2utc(sol->time);
    
    /* 处理时间进位 */
    if (time.sec >= 0.995) { 
        time.time++; 
        time.sec = 0.0; 
    }
    
    /* 时间转换为年月日时分秒格式 */
    time2epoch(time, ep);
    
    /* 生成NMEA-ZDA语句 */
    p += sprintf(p, "$GPZDA,%02.0f%02.0f%05.2f,%02.0f,%02.0f,%04.0f,,",
                ep[3], ep[4], ep[5],  /* 时分秒 */
                ep[2], ep[1], ep[0]);  /* 日月年 */
    
    /* 计算校验和 */
    for (q = (char*)buff + 1, sum = 0; *q; q++) {
        sum ^= *q;
    }
    
    /* 添加校验和 */
    p += sprintf(p, "*%02X", sum);
    
    /* 返回消息长度 */
    return p - (char*)buff;
}

/* output solution in the form of nmea VTG sentence --------------------------*/
extern int outnmea_vtg(unsigned char* buff, const sol_t* sol)
{
    /* 变量声明与初始化 */
    double pos[3] = {0.0};            /* 位置坐标 [纬度,经度,高度] */
    double enuv[3] = {0.0};           /* ENU速度分量 */
    double vel = 0.0;                 /* 地面速度大小 */
    double dir = 0.0;                 /* 航向角 */
    char mode = 'A';                  /* 模式指示符 */
    char* p = (char*)buff;            /* 输出缓冲区指针 */
    char* q = NULL;                   /* 校验和计算用指针 */
    char sum = 0;                     /* 校验和 */
    
    trace(3, "outnmea_vtg:\n");
    
    /* 解算状态检查：无效解则输出空VTG语句 */
    if (sol->stat <= SOLQ_NONE) {
        p += sprintf(p, "$GPVTG,,T,,M,,N,,K,N*");
        for (q = (char*)buff + 1, sum = 0; *q; q++) sum ^= *q;
        p += sprintf(p, "%02X", sum);
        return p - (char*)buff;
    }
    
    /* 坐标变换：ECEF->大地坐标系 */
    ecef2pos(sol->rr, pos);
    
    /* 速度变换：ECEF->ENU */
    ecef2enu(pos, sol->rr + 3, enuv);
    
    /* 计算地面速度和航向 */
    vel = sqrt(enuv[0] * enuv[0] + enuv[1] * enuv[1]); /* 地面速度 */
    
    /* 计算航向角 (偏航角) */
    if (vel >= 0.1) {
        dir = atan2(enuv[0], enuv[1]) * R2D;
        if (dir < 0.0) {
            dir += 360.0;
        }
    } else {
        dir = dirp; /* 使用上次计算的方向，dirp是全局变量 */
    }
    
    /* 设置模式指示符 */
    if (sol->stat == SOLQ_DGPS || sol->stat == SOLQ_FLOAT || sol->stat == SOLQ_FIX) {
        mode = 'D'; /* 差分模式 */
    } else if (sol->stat == SOLQ_SINGLE) {
        mode = 'A'; /* 自主定位模式 */
    } else {
        mode = 'N'; /* 数据无效 */
    }
    
    /* 生成NMEA-VTG语句
       $GPVTG,真北航向,T,磁北航向,M,地速(节),N,地速(公里/小时),K,模式*校验和 */
    p += sprintf(p, "$GPVTG,%.3f,T,%.3f,M,%.3f,N,%.3f,K,%c",
                dir, dir,                            /* 真北航向和磁北航向(此处简化,相同) */
                vel / KNOT2M,                        /* 地面速度(节) */
                vel * 3.6,                           /* 地面速度(公里/小时) */
                mode);                               /* 模式指示符 */
    
    /* 计算校验和 */
    for (q = (char*)buff + 1, sum = 0; *q; q++) {
        sum ^= *q;
    }
    
    /* 添加校验和 */
    p += sprintf(p, "*%02X", sum);
    
    /* 返回消息长度 */
    return p - (char*)buff;
}

///* output solution in the form of novatel BESTVELA sentence ---------------------*/
//extern int outnovatel_bestvel(unsigned char* buff, const sol_t* sol, int port)
//{
//    /* 变量声明与初始化 */
//    static int week = 2363;           /* GPS周数，实际使用时需要获取 */
//    double pos[3] = {0.0};            /* 位置坐标 [纬度,经度,高度] */
//    double enuv[3] = {0.0};           /* ENU速度分量 */
//    double horvel = 0.0;              /* 水平速度大小 */
//    double dir = 0.0;                 /* 方向角 */
//    double vertvel = 0.0;             /* 垂直速度大小 */
//    double latency = 0.0;             /* 延迟量 */
//    double age = 0.0;                 /* 差分龄期 */
//    int sol_status = 0;               /* 解状态码 */
//    int vel_type = 8;                 /* 速度类型码，默认DOPPLER_VELOCITY(8) */
//    unsigned int crc = 0;             /* CRC校验 */
//    char* p = (char*)buff;            /* 输出缓冲区指针 */
//    
//    trace(3, "outnovatel_bestvel:\n");
//    
//    /* 解算状态检查：无效解则输出基本BESTVEL语句 */
//    if (sol->stat <= SOLQ_NONE) {
//        p += sprintf(p, "#BESTVELA,COM%d,0,0.0,UNKNOWN,0,0.000,00000000,0000,0;1,0,0.000,0.000,0.0000,0.000000,0.0000,0*", port);
//        crc = calculateCRC((const unsigned char*)buff, (int)(p - (char*)buff));
//        p += sprintf(p, "%08x", crc);
//        return p - (char*)buff;
//    }
//    
//    /* 坐标变换：ECEF->大地坐标系 */
//    ecef2pos(sol->rr, pos);
//    
//    /* 速度变换：ECEF->ENU */
//    ecef2enu(pos, sol->rr + 3, enuv);
//    
//    /* 计算水平速度和方向角 */
//    horvel = sqrt(enuv[0] * enuv[0] + enuv[1] * enuv[1]); /* 水平速度 */
//    
//    /* 计算垂直速度 */
//    vertvel = enuv[2]; /* 垂直速度：正值表示向上，负值表示向下 */
//    
//    /* 计算航向角 (对地运动的实际方向) */
//    if (horvel >= 0.1) {
//        dir = atan2(enuv[0], enuv[1]) * R2D;
//        if (dir < 0.0) {
//            dir += 360.0;
//        }
//    } else {
//        dir = dirp; /* 使用上次计算的方向，dirp是全局变量 */
//    }
//    
//    /* 设置延迟量和差分龄期 */
//    latency = 0.0;   /* 实际应用中可能需要计算 */
//    age = sol->age;  /* 差分龄期 */
//    
//    /* 设置解状态码（表27）：
//       0 - SOL_COMPUTED  - 解算成功
//       1 - INSUFFICIENT_OBS - 观测量不足
//       6 - COLD_START - 冷启动中
//       19 - INVALID_FIX - Fix position输入的坐标误差超限 */
//    if (sol->stat == SOLQ_NONE) {
//        sol_status = 1; /* INSUFFICIENT_OBS - 观测量不足 */
//    } else {
//        sol_status = 0; /* SOL_COMPUTED - 解算成功 */
//    }
//    
//    /* 设置速度类型码（表28）：
//       0 - NONE - 未解算
//       1 - FIXEDPOS - 通过指令FIX POSITION进行位置固定
//       8 - DOPPLER_VELOCITY - 利用实时多普勒计算速度
//       9 - SINGLE_SMOOTH - 单点平滑定位
//       16 - SINGLE - 单点定位
//       17 - PSRDIFF - 伪距差分定位
//       18 - SBAS - 加入SBAS改正的解
//       34 - NARROW_FLOAT - 浮点解
//       35 - FIX_DERIVATION - 推导解
//       49 - WIDE_INT - 宽巷解
//       50 - NARROW_INT - 固定解
//       etc. */
//    switch (sol->stat) {
//        case SOLQ_NONE:
//            vel_type = 0; /* NONE - 未解算 */
//            break;
//        case SOLQ_SINGLE:
//            vel_type = 16; /* SINGLE - 单点定位 */
//            break;
//        case SOLQ_DGPS:
//            vel_type = 17; /* PSRDIFF - 伪距差分定位 */
//            break;
//        case SOLQ_SBAS:
//            vel_type = 18; /* SBAS - 加入SBAS改正的解 */
//            break;
//        case SOLQ_FLOAT:
//            vel_type = 34; /* NARROW_FLOAT - 浮点解 */
//            break;
//        case SOLQ_FIX:
//            vel_type = 50; /* NARROW_INT - 固定解 */
//            break;
//        case SOLQ_PPP:
//            vel_type = 69; /* PPP - PPP浮点解 */
//            break;
//        default:
//            vel_type = 8; /* DOPPLER_VELOCITY - 默认使用多普勒计算速度 */
//            break;
//    }
//    
//    /* 获取GPS时间（秒） */
//    double gpst = time2gpst(sol->time, &week);
//    
//    /* 生成NOVATEL-BESTVELA语句，使用数字状态码而非字符串 */
//    p += sprintf(p, "#BESTVELA,COM%d,0,78.0,FINESTEERING,%d,%.3f,02404000,10a2,15340;%d,%d,%.3f,%.3f,%.4f,%.6f,%.4f,0*",
//                port,                         /* 端口号 */
//                week,                         /* GPS周 */
//                gpst,                         /* GPS周内秒 */
//                sol_status,                   /* 解状态码 */
//                vel_type,                     /* 速度类型码 */
//                latency,                      /* 延迟量 */
//                age,                          /* 差分龄期 */
//                horvel,                       /* 水平速度 */
//                dir,                          /* 对地运动方向 */
//                vertvel);                     /* 垂直速度 */
//    
//    /* 计算CRC校验和 */
//    crc = calculateCRC((const unsigned char*)buff, (int)(p - (char*)buff));
//    
//    /* 添加CRC校验和 */
//    p += sprintf(p, "%08x", crc);
//    
//    /* 返回消息长度 */
//    return p - (char*)buff;
//}
//
///* Calculate CRC for Novatel messages */
//static unsigned int calculateCRC(const unsigned char* buff, int len)
//{
//    unsigned int crc = 0;
//    int i, j;
//    
//    for (i = 0; i < len; i++) {
//        crc ^= buff[i] << 8;
//        
//        for (j = 0; j < 8; j++) {
//            if (crc & 0x8000) {
//                crc = (crc << 1) ^ 0x1021;
//            } else {
//                crc = crc << 1;
//            }
//        }
//    }
//    
//    return crc & 0xFFFFFFFF;
//}


extern void rtkoutnmea(rtk_t* rtk, nmea_t* nmea)
{
#define MAXNUMBER 600
    /* 变量声明与初始化 */
    double rb[3] = {0.0};            /* 基站位置 */
    double rr[3] = {0.0};            /* 当前解算位置 */
    double diff[3] = {0.0};          /* 基站和当前位置差值 */
    double lastdiff[3] = {0.0};      /* 当前位置与上次位置差值 */
    double dis = 0.0;                /* 基站和当前位置距离 */
    double lastdist = 0.0;           /* 位移距离 */
    int stat = 1;                    /* 解算状态标志 */
    
    /* 设置基站ID */
    rtk->sol.staid = rtk->vrs_staid;
    
    /* 计算时间差 */
    if (rtk->sol_last.time.time != 0 && rtk->sol.time.time != 0) {
        rtk->lasttt = fabs(timediff(rtk->sol.time, rtk->sol_last.time));
        trace(5, "lasttime =%s", time_str(rtk->sol_last.time, 3));
        trace(5, "currenttime=%s  \n", time_str(rtk->sol.time, 3));
    }
    
    /* 位移和速度检查 */
    if (norm(rtk->sol_last.rr, 3) > 0 && rtk->lasttt > 0) {
        /* 计算位移 */
        for (int i = 0; i < 3; i++) {
            lastdiff[i] = rtk->sol.rr[i] - rtk->sol_last.rr[i];
        }
        lastdist = norm(lastdiff, 3) / rtk->lasttt;  /* 速度 = 位移/时间 */
        
        /* 异常解算结果检测 */
        if (lastdist > 1 || norm(rtk->sol.rr + 3, 3) > 1) {
            /* 检测条件1: 速度异常且时间差小 */
            if (lastdist > 2 * norm(rtk->sol.rr + 3, 3) && rtk->lasttt < 10 && rtk->sol.stat == 5) {
                stat = 0;  /* 仅控制当前元的输出，不涉及复制 */
            }
            
            /* 检测条件2: spp失败，rtd解且速度异常 */
            if (lastdist > 2 * norm(rtk->sol.rr + 3, 3) && rtk->lasttt < 10 && 
                rtk->sol.stat == 4 && rtk->sol.spp_flag == 2) {
                stat = 0;  /* 仅控制当前元的输出，不涉及复制 */
            }
        }
        
        /* 极端情况检测 */
        if (lastdist > 45 && rtk->lasttt < 2) {
            rtk->sol.stat = 0;  /* 速度过大且时间差极小，认为解算失败 */
        }
        
        if (norm(rtk->sol.rr + 3, 3) > 45) {
            stat = 0;  /* 速度过大，认为解算失败 */
        }
    }
    
    /* 更新状态标志 */
    if (stat == 0) {
        rtk->sol.stat = 0;
    }
    
    /* 保存解算结果到上一次解算 */
    if (rtk->sol.stat > 0 && rtk->sol.spp_flag != 2) {
        memcpy(&rtk->sol_last, &rtk->sol, sizeof(sol_t));
    } else {
        trace(5, "do not copy to last_sol\n");
    }
    
    /* 计算基站与当前位置的距离 */
    for (int j = 0; j < 3; j++) {
        rb[j] = rtk->opt.rb[j];
        rr[j] = rtk->sol.rr[j];
        diff[j] = rb[j] - rr[j];
        lastdiff[j] = rtk->sol.rr[j] - rtk->sol_last.rr[j];
    }
    
    dis = sqrt(diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2]);
    
    /* 输出调试信息 */
    trace(3, "rtkoutnmea.the distance between rr and rb is %f,rb cor is %f,%f,%f,rr is %f,%f,%f\n ", 
          dis, rb[0], rb[1], rb[2], rr[0], rr[1], rr[2]);
    trace(3, "save_result: vel is %f outflag=%d,sol.stat=%d,filternumber=%d ,rtk.sol.spp_flag=%d,"
          "lastdist=%f lastt=%f laststate=%d %p \n", 
          norm(rtk->sol.rr + 3, 3), outflag, rtk->sol.stat, filternumber, rtk->sol.spp_flag, 
          lastdist, rtk->lasttt, rtk->sol_last.stat, &filternumber);
    
    /* 输出NMEA消息 */
    if (outflag) {
        /* 动态模式输出 */
        if (rtk->opt.dynamics) {
            outnmea_zda(nmea->zda, &rtk->sol);
            outnmea_gga(nmea->gga, &rtk->sol);
            outnmea_rmc2(nmea->rmc, nmea->enuv, &rtk->sol);
            outnmea_vtg(nmea->vtg, &rtk->sol);
        } else {
            /* 非动态模式且状态正常 */
            if (rtk->sol.stat != 4 && rtk->sol.stat != 0 && rtk->sol.stat != 5) {
                outnmea_zda(nmea->zda, &rtk->sol);
                outnmea_gga(nmea->gga, &rtk->sol);
                outnmea_rmc2(nmea->rmc, nmea->enuv, &rtk->sol);
                outnmea_vtg(nmea->vtg, &rtk->sol);
                memcpy(&rtk->sol_last, &rtk->sol, sizeof(sol_t));
            } else {
                /* 解算异常时使用上次正常解算结果 */
                if (rtk->sol_last.stat != 0) {
                    memcpy(&rtk->sol, &rtk->sol_last, sizeof(sol_t));
                }
            }
        }
    } else {
        /* 均值滤波平滑处理 */
        if (norm(rtk->sol.rr + 3, 3) < 0.6 && rtk->sol.stat > 0) {
            /* 当速度小且状态正常时执行滤波 */
            for (int index = 0; index < 3; index++) {
                xfliter[index] = xfliter[index] * filternumber + rtk->sol.rr[index];  /* 求和 */
                xfliter[index] = xfliter[index] / (1 + filternumber);                 /* 求平均 */
            }
            filternumber++;
            
            /* 当滤波次数足够时更新解算结果 */
            if (filternumber > 3) {
                for (int j = 0; j < 3; j++) {
                    rtk->sol.rr[j] = xfliter[j];
                }
            }
            
            /* 输出NMEA消息 */
            outnmea_rmc2(nmea->rmc, nmea->enuv, &rtk->sol);
            outnmea_gga(nmea->gga, &rtk->sol);
        } else {
            /* 速度较大时重置滤波参数 */
            for (int j = 0; j < 3; j++) {
                xfliter[j] = 0.0;
            }
            filternumber = 0;
            
            /* 输出NMEA消息 */
            outnmea_rmc2(nmea->rmc, nmea->enuv, &rtk->sol);
            outnmea_gga(nmea->gga, &rtk->sol);
        }
        outflag++;
    }
    
    /* 保存当前状态到上一次状态 */
    rtk->sol.last_stat = rtk->sol.stat;
    
#if 0
    for (int i = 0; i < 6; i++) {
        rtk->sol.l_rr[i] = rtk->sol.rr[i];
        if (i < 3) rtk->l_x[i] = rtk->sol.rr[i];
    }
#endif
}
int outlgpvt_gga(lg69t_pvt_t lgpvt, unsigned char* buff)
{
	double ep[6] = { 0.0 }, dms1[3] = { 0.0 }, dms2[3] = { 0.0 };
	double dop = 0.0;
	double h = 0.0;
	gtime_t time = { 0 };
	char* p = (char*)buff, * q = NULL, sum = 0;
	//h=lgpvt.h;   //由于目前rtk未计算高程异常，故为保持同步该处也不输出高程异常

	time = gpst2utc(lgpvt.time);
	time2epoch(time, ep);

	deg2dms(fabs(lgpvt.lat), dms1, 7);
	deg2dms(fabs(lgpvt.lon), dms2, 7);

	dop = lgpvt.dop[0];//HDOP

	/*gppgga,时分秒,纬度,N,经度,E,Q，ns,dop值，高差，M，高,M,差分龄期，*/
	trace(2, "lg69t pvt:$GPGGA,%02.0f%02.0f%05.2f,%02.0f%010.7f,%s,%03.0f%010.7f,%s,%d,%02d,%.1f,%.3f,M,%.3f,M,%.1f,%d\r\n",
		ep[3], ep[4], ep[5], dms1[0], dms1[1] + dms1[2] / 60.0, lgpvt.lat >= 0 ? "N" : "S",
		dms2[0], dms2[1] + dms2[2] / 60.0, lgpvt.lon >= 0 ? "E" : "W", lgpvt.qual,
		lgpvt.nsat_u, dop, lgpvt.ght - h, h, lgpvt.age, lgpvt.sta);
	p += sprintf(p, "$GPGGA,%02.0f%02.0f%05.2f,%02.0f%010.7f,%s,%03.0f%010.7f,%s,%d,%02d,%.1f,%.3f,M,%.3f,M,%.1f,%d",
		ep[3], ep[4], ep[5], dms1[0], dms1[1] + dms1[2] / 60.0, lgpvt.lat >= 0 ? "N" : "S",
		dms2[0], dms2[1] + dms2[2] / 60.0, lgpvt.lon >= 0 ? "E" : "W", lgpvt.qual,
		lgpvt.nsat_u, dop, lgpvt.ght - h, h, lgpvt.age, lgpvt.sta);
	for (q = (char*)buff + 1, sum = 0; *q; q++) sum ^= *q; /* check-sum */
	p += sprintf(p, "*%02X", sum);
	return p - (char*)buff;
}
int outlgpvt_rmc(lg69t_pvt_t lgpvt, unsigned char* buff)
{
	double ep[6] = { 0.0 }, dms1[3] = { 0.0 }, dms2[3] = { 0.0 };
	double amag = 0.0;
	char* emag = "E";
	char* p = (char*)buff, * q = NULL, sum = 0;
	gtime_t time = { 0 };

	time = gpst2utc(lgpvt.time);
	time2epoch(time, ep);

	deg2dms(fabs(lgpvt.lat), dms1, 7);
	deg2dms(fabs(lgpvt.lon), dms2, 7);

	trace(2, "lg69t pvt:$GPRMC,%02.0f%02.0f%05.2f,A,%02.0f%010.7f,%s,%03.0f%010.7f,%s,%4.2f,%4.2f,%02.0f%02.0f%02d,%.1f,%s,%s\r\n",
		ep[3], ep[4], ep[5], dms1[0], dms1[1] + dms1[2] / 60.0, lgpvt.lat >= 0 ? "N" : "S",
		dms2[0], dms2[1] + dms2[2] / 60.0, lgpvt.lon >= 0 ? "E" : "W", lgpvt.vh / KNOT2M, lgpvt.dir,
		ep[2], ep[1], (int)ep[0] % 100, amag, emag,
		lgpvt.qual == SOLQ_DGPS || lgpvt.qual == SOLQ_FLOAT || lgpvt.qual == SOLQ_FIX ? "D" : "A");

	p += sprintf(p, "$GPRMC,%02.0f%02.0f%05.2f,A,%02.0f%010.7f,%s,%03.0f%010.7f,%s,%4.2f,%4.2f,%02.0f%02.0f%02d,%.1f,%s,%s",
		ep[3], ep[4], ep[5], dms1[0], dms1[1] + dms1[2] / 60.0, lgpvt.lat >= 0 ? "N" : "S",
		dms2[0], dms2[1] + dms2[2] / 60.0, lgpvt.lon >= 0 ? "E" : "W", lgpvt.vh / KNOT2M, lgpvt.dir,
		ep[2], ep[1], (int)ep[0] % 100, amag, emag,
		lgpvt.qual == SOLQ_DGPS || lgpvt.qual == SOLQ_FLOAT || lgpvt.qual == SOLQ_FIX ? "D" : "A");
	for (q = (char*)buff + 1, sum = 0; *q; q++) sum ^= *q; /* check-sum */
	p += sprintf(p, "*%02X", sum);
	return p - (char*)buff;
}
extern void outlg69tpvt(lg69t_pvt_t lgpvt, nmea_t* nmea_t)
{
	outlgpvt_gga(lgpvt, (char*)nmea_t->gga);
	outlgpvt_rmc(lgpvt, (char*)nmea_t->rmc);

	nmea_t->enuv[0] = lgpvt.vh * sin(lgpvt.dir * D2R);
	nmea_t->enuv[1] = lgpvt.vh * cos(lgpvt.dir * D2R);
	nmea_t->enuv[2] = lgpvt.vv;

	trace(5, "lg69t pvt result:%s\n", nmea_t->gga);
	trace(5, "lg69t pvt result:%s\n", nmea_t->rmc);
}

extern int gga_judge_add(const unsigned char* ggain)
{
	char gga[BUFLENGTH] = "";
	nmea_gga_t sol_gga = { 0 };

	strcpy(gga, (char*)ggain);
	if (!decode_gga(gga, &sol_gga))
		return 0;

	return 1;
}

/* output solution in custom format to a .txt file --------------------------*/
extern void rtkoutcustom(rtk_t* rtk, FILE* fp)
{
    /* 变量声明与初始化 */
    double pos[3] = {0.0};            /* 位置坐标 [纬度,经度,高度] */
    double enuv[3] = {0.0};           /* ENU速度分量 */
    gtime_t time;                     /* UTC时间 */
    double ep[6] = {0.0};             /* 历元时间 [年,月,日,时,分,秒] */
    int week;                         /* GPS周 */
    double gpst;                      /* GPS时间（周内秒） */
    char tstr[32];                    /* 时间字符串 */
    
    /* 基本信息标题（仅第一次写入文件时输出） */
    static int header_written = 0;
    
    if (!header_written && fp != NULL) {
        fprintf(fp, "==================================================================================================================================================\n");
        fprintf(fp, "============================================ Leador Space Information Technology Co., Ltd. [LEADOR] ==============================================\n");
        fprintf(fp, "===================================================    Firmware Version: v2.05.000    ============================================================\n");
        fprintf(fp, "==================================================================================================================================================\n");
        fprintf(fp, "Datum:       WGS84 (2024.967)\n");
        fprintf(fp, "Remote:      Antenna height 0.000 m, to L1PC [Generic(NONE)]\n");
        fprintf(fp, "UTC Offset:  18 s\n");
        fprintf(fp, "Local time:  +8.0 h, (UTC+08:00) China Daylight Time\n\n");
        fprintf(fp, "Map projection Info:\n");
        fprintf(fp, "  Defined grid: Gauss Kruger (3 deg), Zone 36\n\n");
        fprintf(fp, " LocalDate   LocalTime       Latitude       Longitude        H-Ell     Northing      Easting     VEast    VNorth       VUp Q NS   GPSTime     Week\n");
        fprintf(fp, "     (YMD)       (HMS)          (deg)           (deg)          (m)          (m)          (m)     (m/s)     (m/s)     (m/s)          (sec)  (weeks)\n");
        header_written = 1;
    }
    
    /* 检查解算状态 */
    if (rtk->sol.stat <= SOLQ_NONE || fp == NULL) {
        return;
    }
    
    /* 将ECEF坐标转换为大地坐标 */
    ecef2pos(rtk->sol.rr, pos);
    
    /* 计算ENU速度分量 */
    ecef2enu(pos, rtk->sol.rr + 3, enuv);
    
    /* 获取当前时间 - 转换为UTC时间 */
    time = gpst2utc(rtk->sol.time);
    time2epoch(time, ep);
    
    /* 转换为北京时间（UTC+8小时） */
    ep[3] += 8.0; /* 加8小时 */
    
    /* 处理时间进位 */
    if (ep[3] >= 24.0) {
        ep[3] -= 24.0;
        time = epoch2time(ep);
        time = timeadd(time, 24*3600);
        time2epoch(time, ep);
    }
    
    /* 获取GPS周和周内秒 */
    gpst = time2gpst(rtk->sol.time, &week);
    
    /* 格式化输出行：
       日期 时间 纬度 经度 高度 北坐标 东坐标 东向速度 北向速度 垂直速度 定位质量 卫星数 GPS时间 GPS周 */
    fprintf(fp, "%04.0f/%02.0f/%02.0f %2.0f:%02.0f:%05.2f %14.10f %15.10f %12.3f %13.3f %12.3f %8.3f %9.3f %9.3f %d %2d %9.2f %8d\n",
            ep[0], ep[1], ep[2],                                      /* 年月日 */
            ep[3], ep[4], ep[5],                                      /* 时分秒 */
            pos[0] * R2D,                                             /* 纬度(度) */
            pos[1] * R2D,                                             /* 经度(度) */
            pos[2],                                                   /* 大地高(米) */
            0.0,                                                      /* 北坐标(米)近似值 */
            0.0,                                                      /* 东坐标(米)近似值 */
            enuv[0],                                                  /* 东向速度(米/秒) */
            enuv[1],                                                  /* 北向速度(米/秒) */
            enuv[2],                                                  /* 垂直速度(米/秒) */
            rtk->sol.stat,                                            /* 定位质量指示符 */
            rtk->sol.ns,                                              /* 卫星数量 */
            gpst,                                                     /* GPS时间(秒) */
            week                                                      /* GPS周 */
    );
}


