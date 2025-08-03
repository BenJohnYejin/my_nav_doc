#include "rtklib.h"

/* constants/macros ----------------------------------------------------------*/

#define LOOPMAX     500           /* maximum count of search loop */

#define SGN(x)      ((x)<=0.0?-1.0:1.0)
#define SWAP(x,y)   do {double tmp_; tmp_=x; x=y; y=tmp_;} while (0)

#define MINREFSATEL   20.0
#define MINFIX        20
#define MINCHECK      20

#define MINPARNUM   6
#define MAXTRYNUM   5
#define MINLOCK     0
const float table1[31][64] = { {
			0.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f,
			1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f,
			1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f,
			1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f,
			1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f,
			1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f,
			1.0000f, 1.0000f, 1.0000f, 1.0000f
		}, {
			0.0010f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f,
			1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f,
			1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f,
			1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f,
			1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f,
			1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f, 1.0000f,
			1.0000f, 1.0000f, 1.0000f, 1.0000f
		}, {
			0.0012f, 0.9400f, 0.9400f, 0.9400f, 0.9400f, 0.9400f, 0.9400f, 0.9400f, 0.9400f, 0.9600f,
			0.9600f, 0.9600f, 0.9600f, 0.9600f, 0.9600f, 0.9600f, 0.9600f, 0.9600f, 0.9600f, 0.9600f,
			0.9600f, 0.9600f, 0.9600f, 0.9600f, 0.9600f, 0.9700f, 0.9700f, 0.9700f, 0.9800f, 0.9800f,
			0.9800f, 0.9800f, 0.9800f, 0.9800f, 0.9800f, 0.9800f, 0.9800f, 0.9800f, 0.9800f, 0.9800f,
			0.9900f, 0.9900f, 0.9900f, 0.9900f, 0.9900f, 0.9900f, 0.9900f, 0.9900f, 0.9900f, 0.9900f,
			0.9900f, 0.9900f, 0.9900f, 0.9900f, 0.9900f, 0.9900f, 0.9900f, 0.9900f, 0.9900f, 0.9900f,
			0.9900f, 0.9900f, 0.9900f, 0.9900f
		}, {
			0.0015f, 0.8700f, 0.8700f, 0.8800f, 0.8800f, 0.8900f, 0.8900f, 0.8900f, 0.8900f, 0.9100f,
			0.9200f, 0.9200f, 0.9200f, 0.9200f, 0.9300f, 0.9300f, 0.9300f, 0.9300f, 0.9300f, 0.9300f,
			0.9300f, 0.9300f, 0.9400f, 0.9400f, 0.9400f, 0.9400f, 0.9400f, 0.9400f, 0.9500f, 0.9500f,
			0.9500f, 0.9500f, 0.9500f, 0.9500f, 0.9500f, 0.9500f, 0.9600f, 0.9600f, 0.9600f, 0.9600f,
			0.9600f, 0.9600f, 0.9600f, 0.9600f, 0.9600f, 0.9600f, 0.9600f, 0.9600f, 0.9600f, 0.9600f,
			0.9600f, 0.9700f, 0.9700f, 0.9700f, 0.9700f, 0.9700f, 0.9700f, 0.9700f, 0.9700f, 0.9700f,
			0.9700f, 0.9700f, 0.9700f, 0.9700f
		}, {
			0.0020f, 0.7800f, 0.7800f, 0.8000f, 0.8000f, 0.8100f, 0.8200f, 0.8300f, 0.8400f, 0.8400f,
			0.8600f, 0.8600f, 0.8600f, 0.8700f, 0.8800f, 0.8800f, 0.8800f, 0.8900f, 0.8900f, 0.9000f,
			0.9000f, 0.9000f, 0.9000f, 0.9000f, 0.9000f, 0.9100f, 0.9100f, 0.9100f, 0.9200f, 0.9200f,
			0.9200f, 0.9200f, 0.9300f, 0.9300f, 0.9300f, 0.9400f, 0.9400f, 0.9400f, 0.9400f, 0.9400f,
			0.9400f, 0.9400f, 0.9400f, 0.9400f, 0.9400f, 0.9400f, 0.9400f, 0.9400f, 0.9400f, 0.9400f,
			0.9400f, 0.9500f, 0.9500f, 0.9500f, 0.9500f, 0.9500f, 0.9500f, 0.9500f, 0.9500f, 0.9500f,
			0.9500f, 0.9500f, 0.9500f, 0.9600f
		}, {
			0.0050f, 0.5400f, 0.5400f, 0.5700f, 0.5700f, 0.5900f, 0.6400f, 0.6400f, 0.6800f, 0.6800f,
			0.6900f, 0.7100f, 0.7200f, 0.7300f, 0.7400f, 0.7500f, 0.7500f, 0.7700f, 0.7800f, 0.8000f,
			0.8000f, 0.8000f, 0.8000f, 0.8000f, 0.8100f, 0.8200f, 0.8200f, 0.8200f, 0.8400f, 0.8400f,
			0.8400f, 0.8400f, 0.8500f, 0.8500f, 0.8600f, 0.8600f, 0.8600f, 0.8600f, 0.8700f, 0.8800f,
			0.8800f, 0.8800f, 0.8800f, 0.8800f, 0.8800f, 0.8800f, 0.8800f, 0.8800f, 0.8900f, 0.8900f,
			0.8900f, 0.9100f, 0.9100f, 0.9100f, 0.9100f, 0.9100f, 0.9100f, 0.9100f, 0.9100f, 0.9100f,
			0.9100f, 0.9100f, 0.9100f, 0.9100f
		}, {
			0.0100f, 0.3600f, 0.3800f, 0.4100f, 0.4300f, 0.4500f, 0.5100f, 0.5200f, 0.5700f, 0.5700f,
			0.5900f, 0.6100f, 0.6200f, 0.6400f, 0.6500f, 0.6700f, 0.6700f, 0.6800f, 0.6900f, 0.7200f,
			0.7200f, 0.7200f, 0.7200f, 0.7200f, 0.7400f, 0.7600f, 0.7600f, 0.7700f, 0.7900f, 0.7900f,
			0.7900f, 0.7900f, 0.8100f, 0.8100f, 0.8200f, 0.8200f, 0.8200f, 0.8200f, 0.8300f, 0.8300f,
			0.8400f, 0.8400f, 0.8500f, 0.8500f, 0.8500f, 0.8500f, 0.8500f, 0.8500f, 0.8500f, 0.8500f,
			0.8500f, 0.8700f, 0.8800f, 0.8800f, 0.8800f, 0.8800f, 0.8800f, 0.8800f, 0.8800f, 0.8800f,
			0.8800f, 0.8800f, 0.8800f, 0.8800f
		}, {
			0.0150f, 0.2700f, 0.2900f, 0.3200f, 0.3600f, 0.3800f, 0.4300f, 0.4600f, 0.5100f, 0.5200f,
			0.5300f, 0.5500f, 0.5600f, 0.5800f, 0.6000f, 0.6200f, 0.6200f, 0.6400f, 0.6500f, 0.6700f,
			0.6700f, 0.6900f, 0.6900f, 0.6900f, 0.7100f, 0.7400f, 0.7400f, 0.7400f, 0.7600f, 0.7600f,
			0.7600f, 0.7700f, 0.7800f, 0.7800f, 0.7900f, 0.7900f, 0.8000f, 0.8000f, 0.8100f, 0.8100f,
			0.8200f, 0.8200f, 0.8300f, 0.8300f, 0.8300f, 0.8400f, 0.8400f, 0.8400f, 0.8400f, 0.8400f,
			0.8400f, 0.8500f, 0.8500f, 0.8600f, 0.8600f, 0.8600f, 0.8600f, 0.8600f, 0.8600f, 0.8600f,
			0.8700f, 0.8700f, 0.8700f, 0.8800f
		}, {
			0.0200f, 0.2100f, 0.2400f, 0.2700f, 0.3000f, 0.3300f, 0.3800f, 0.4200f, 0.4600f, 0.4800f,
			0.4900f, 0.5200f, 0.5200f, 0.5500f, 0.5700f, 0.5900f, 0.5900f, 0.6200f, 0.6200f, 0.6500f,
			0.6500f, 0.6700f, 0.6700f, 0.6700f, 0.7000f, 0.7200f, 0.7200f, 0.7200f, 0.7400f, 0.7500f,
			0.7500f, 0.7500f, 0.7700f, 0.7700f, 0.7800f, 0.7800f, 0.7800f, 0.7900f, 0.7900f, 0.8000f,
			0.8100f, 0.8100f, 0.8100f, 0.8200f, 0.8200f, 0.8200f, 0.8300f, 0.8300f, 0.8300f, 0.8300f,
			0.8300f, 0.8400f, 0.8400f, 0.8400f, 0.8500f, 0.8500f, 0.8500f, 0.8500f, 0.8500f, 0.8500f,
			0.8600f, 0.8600f, 0.8600f, 0.8700f
		}, {
			0.0250f, 0.1900f, 0.2200f, 0.2400f, 0.2700f, 0.2900f, 0.3500f, 0.3900f, 0.4200f, 0.4500f,
			0.4600f, 0.4900f, 0.4900f, 0.5300f, 0.5400f, 0.5600f, 0.5600f, 0.6000f, 0.6000f, 0.6300f,
			0.6400f, 0.6500f, 0.6600f, 0.6600f, 0.6800f, 0.7100f, 0.7100f, 0.7100f, 0.7200f, 0.7200f,
			0.7300f, 0.7400f, 0.7500f, 0.7500f, 0.7700f, 0.7700f, 0.7700f, 0.7800f, 0.7800f, 0.7900f,
			0.7900f, 0.8000f, 0.8000f, 0.8000f, 0.8100f, 0.8100f, 0.8100f, 0.8200f, 0.8200f, 0.8300f,
			0.8300f, 0.8300f, 0.8300f, 0.8300f, 0.8400f, 0.8400f, 0.8500f, 0.8500f, 0.8500f, 0.8500f,
			0.8600f, 0.8600f, 0.8600f, 0.8700f
		}, {
			0.0300f, 0.1700f, 0.2000f, 0.2200f, 0.2400f, 0.2600f, 0.3200f, 0.3500f, 0.4000f, 0.4200f,
			0.4400f, 0.4700f, 0.4700f, 0.5100f, 0.5300f, 0.5400f, 0.5500f, 0.5800f, 0.5900f, 0.6200f,
			0.6200f, 0.6400f, 0.6500f, 0.6500f, 0.6700f, 0.7000f, 0.7000f, 0.7000f, 0.7100f, 0.7100f,
			0.7200f, 0.7300f, 0.7400f, 0.7500f, 0.7600f, 0.7600f, 0.7600f, 0.7700f, 0.7700f, 0.7800f,
			0.7800f, 0.7900f, 0.7900f, 0.8000f, 0.8000f, 0.8100f, 0.8100f, 0.8100f, 0.8200f, 0.8200f,
			0.8200f, 0.8200f, 0.8300f, 0.8300f, 0.8400f, 0.8400f, 0.8400f, 0.8500f, 0.8500f, 0.8500f,
			0.8500f, 0.8600f, 0.8600f, 0.8600f
		}, {
			0.0350f, 0.1600f, 0.1800f, 0.2000f, 0.2200f, 0.2400f, 0.3000f, 0.3300f, 0.3800f, 0.4000f,
			0.4200f, 0.4500f, 0.4500f, 0.4900f, 0.5100f, 0.5300f, 0.5400f, 0.5700f, 0.5800f, 0.6100f,
			0.6100f, 0.6300f, 0.6400f, 0.6500f, 0.6600f, 0.6900f, 0.6900f, 0.6900f, 0.7100f, 0.7100f,
			0.7200f, 0.7200f, 0.7400f, 0.7400f, 0.7500f, 0.7500f, 0.7600f, 0.7600f, 0.7700f, 0.7700f,
			0.7800f, 0.7800f, 0.7900f, 0.7900f, 0.7900f, 0.8000f, 0.8100f, 0.8100f, 0.8100f, 0.8100f,
			0.8200f, 0.8200f, 0.8200f, 0.8300f, 0.8300f, 0.8400f, 0.8400f, 0.8400f, 0.8500f, 0.8500f,
			0.8500f, 0.8500f, 0.8600f, 0.8600f
		}, {
			0.0400f, 0.1500f, 0.1700f, 0.1800f, 0.2100f, 0.2200f, 0.2700f, 0.3100f, 0.3600f, 0.3900f,
			0.4000f, 0.4400f, 0.4400f, 0.4800f, 0.5000f, 0.5200f, 0.5300f, 0.5600f, 0.5600f, 0.6000f,
			0.6100f, 0.6200f, 0.6400f, 0.6500f, 0.6600f, 0.6800f, 0.6900f, 0.6900f, 0.7000f, 0.7100f,
			0.7100f, 0.7100f, 0.7300f, 0.7400f, 0.7400f, 0.7500f, 0.7500f, 0.7600f, 0.7600f, 0.7700f,
			0.7700f, 0.7800f, 0.7800f, 0.7900f, 0.7900f, 0.7900f, 0.8000f, 0.8100f, 0.8100f, 0.8100f,
			0.8100f, 0.8200f, 0.8200f, 0.8200f, 0.8300f, 0.8400f, 0.8400f, 0.8400f, 0.8500f, 0.8500f,
			0.8500f, 0.8500f, 0.8500f, 0.8600f
		}, {
			0.0450f, 0.1400f, 0.1600f, 0.1700f, 0.1900f, 0.2100f, 0.2600f, 0.2900f, 0.3400f, 0.3800f,
			0.3900f, 0.4300f, 0.4400f, 0.4800f, 0.4900f, 0.5100f, 0.5200f, 0.5600f, 0.5600f, 0.5900f,
			0.6000f, 0.6100f, 0.6300f, 0.6500f, 0.6500f, 0.6700f, 0.6800f, 0.6800f, 0.7000f, 0.7100f,
			0.7100f, 0.7100f, 0.7300f, 0.7300f, 0.7400f, 0.7400f, 0.7500f, 0.7500f, 0.7600f, 0.7700f,
			0.7700f, 0.7800f, 0.7800f, 0.7800f, 0.7900f, 0.7900f, 0.7900f, 0.8000f, 0.8000f, 0.8100f,
			0.8100f, 0.8100f, 0.8200f, 0.8200f, 0.8300f, 0.8300f, 0.8400f, 0.8400f, 0.8400f, 0.8400f,
			0.8400f, 0.8500f, 0.8500f, 0.8600f
		}, {
			0.0500f, 0.1300f, 0.1500f, 0.1600f, 0.1800f, 0.1900f, 0.2400f, 0.2800f, 0.3200f, 0.3700f,
			0.3900f, 0.4200f, 0.4300f, 0.4700f, 0.4800f, 0.5100f, 0.5200f, 0.5500f, 0.5500f, 0.5900f,
			0.6000f, 0.6100f, 0.6300f, 0.6400f, 0.6400f, 0.6700f, 0.6800f, 0.6800f, 0.6900f, 0.7100f,
			0.7100f, 0.7100f, 0.7200f, 0.7300f, 0.7400f, 0.7400f, 0.7500f, 0.7500f, 0.7600f, 0.7600f,
			0.7700f, 0.7700f, 0.7800f, 0.7800f, 0.7800f, 0.7900f, 0.7900f, 0.7900f, 0.8000f, 0.8100f,
			0.8100f, 0.8100f, 0.8200f, 0.8200f, 0.8300f, 0.8300f, 0.8400f, 0.8400f, 0.8400f, 0.8400f,
			0.8400f, 0.8400f, 0.8500f, 0.8600f
		}, {
			0.0550f, 0.1200f, 0.1400f, 0.1500f, 0.1700f, 0.1800f, 0.2300f, 0.2700f, 0.3100f, 0.3600f,
			0.3800f, 0.4100f, 0.4200f, 0.4600f, 0.4800f, 0.5000f, 0.5100f, 0.5400f, 0.5500f, 0.5800f,
			0.5900f, 0.6000f, 0.6200f, 0.6400f, 0.6400f, 0.6600f, 0.6700f, 0.6800f, 0.6900f, 0.7000f,
			0.7000f, 0.7000f, 0.7200f, 0.7300f, 0.7300f, 0.7400f, 0.7500f, 0.7500f, 0.7500f, 0.7600f,
			0.7700f, 0.7700f, 0.7700f, 0.7800f, 0.7800f, 0.7800f, 0.7900f, 0.7900f, 0.8000f, 0.8000f,
			0.8100f, 0.8100f, 0.8200f, 0.8200f, 0.8200f, 0.8300f, 0.8300f, 0.8400f, 0.8400f, 0.8400f,
			0.8400f, 0.8400f, 0.8500f, 0.8600f
		}, {
			0.0600f, 0.1200f, 0.1300f, 0.1500f, 0.1600f, 0.1700f, 0.2100f, 0.2600f, 0.3000f, 0.3600f,
			0.3700f, 0.4000f, 0.4200f, 0.4600f, 0.4700f, 0.5000f, 0.5100f, 0.5400f, 0.5400f, 0.5800f,
			0.5900f, 0.6000f, 0.6200f, 0.6300f, 0.6300f, 0.6600f, 0.6700f, 0.6700f, 0.6900f, 0.7000f,
			0.7000f, 0.7000f, 0.7100f, 0.7200f, 0.7300f, 0.7400f, 0.7400f, 0.7500f, 0.7500f, 0.7600f,
			0.7600f, 0.7700f, 0.7700f, 0.7800f, 0.7800f, 0.7800f, 0.7900f, 0.7900f, 0.8000f, 0.8000f,
			0.8100f, 0.8100f, 0.8100f, 0.8200f, 0.8200f, 0.8300f, 0.8300f, 0.8400f, 0.8400f, 0.8400f,
			0.8400f, 0.8400f, 0.8500f, 0.8600f
		}, {
			0.0650f, 0.1100f, 0.1200f, 0.1400f, 0.1500f, 0.1600f, 0.2000f, 0.2600f, 0.3000f, 0.3500f,
			0.3600f, 0.4000f, 0.4100f, 0.4500f, 0.4600f, 0.5000f, 0.5100f, 0.5300f, 0.5400f, 0.5800f,
			0.5900f, 0.5900f, 0.6100f, 0.6300f, 0.6300f, 0.6600f, 0.6700f, 0.6700f, 0.6800f, 0.7000f,
			0.7000f, 0.7000f, 0.7100f, 0.7200f, 0.7300f, 0.7300f, 0.7400f, 0.7500f, 0.7500f, 0.7600f,
			0.7600f, 0.7700f, 0.7700f, 0.7700f, 0.7800f, 0.7800f, 0.7900f, 0.7900f, 0.8000f, 0.8000f,
			0.8100f, 0.8100f, 0.8100f, 0.8200f, 0.8200f, 0.8300f, 0.8300f, 0.8400f, 0.8400f, 0.8400f,
			0.8400f, 0.8400f, 0.8500f, 0.8500f
		}, {
			0.0700f, 0.1100f, 0.1200f, 0.1300f, 0.1400f, 0.1500f, 0.2000f, 0.2500f, 0.3000f, 0.3500f,
			0.3600f, 0.3900f, 0.4100f, 0.4500f, 0.4600f, 0.4900f, 0.5000f, 0.5300f, 0.5400f, 0.5800f,
			0.5800f, 0.5900f, 0.6100f, 0.6300f, 0.6300f, 0.6600f, 0.6600f, 0.6700f, 0.6800f, 0.6900f,
			0.6900f, 0.7000f, 0.7100f, 0.7200f, 0.7300f, 0.7300f, 0.7400f, 0.7400f, 0.7500f, 0.7600f,
			0.7600f, 0.7700f, 0.7700f, 0.7700f, 0.7800f, 0.7800f, 0.7800f, 0.7900f, 0.8000f, 0.8000f,
			0.8100f, 0.8100f, 0.8100f, 0.8200f, 0.8200f, 0.8200f, 0.8300f, 0.8300f, 0.8400f, 0.8400f,
			0.8400f, 0.8400f, 0.8500f, 0.8500f
		}, {
			0.0750f, 0.1000f, 0.1100f, 0.1200f, 0.1300f, 0.1500f, 0.1900f, 0.2400f, 0.2900f, 0.3400f,
			0.3600f, 0.3900f, 0.4100f, 0.4400f, 0.4500f, 0.4900f, 0.5000f, 0.5200f, 0.5400f, 0.5700f,
			0.5800f, 0.5900f, 0.6100f, 0.6300f, 0.6300f, 0.6500f, 0.6600f, 0.6700f, 0.6800f, 0.6900f,
			0.6900f, 0.6900f, 0.7100f, 0.7200f, 0.7300f, 0.7300f, 0.7400f, 0.7400f, 0.7500f, 0.7600f,
			0.7600f, 0.7700f, 0.7700f, 0.7700f, 0.7800f, 0.7800f, 0.7800f, 0.7900f, 0.8000f, 0.8000f,
			0.8000f, 0.8100f, 0.8100f, 0.8200f, 0.8200f, 0.8200f, 0.8300f, 0.8300f, 0.8400f, 0.8400f,
			0.8400f, 0.8400f, 0.8400f, 0.8500f
		}, {
			0.0800f, 0.1000f, 0.1000f, 0.1200f, 0.1300f, 0.1400f, 0.1900f, 0.2300f, 0.2900f, 0.3400f,
			0.3500f, 0.3800f, 0.4000f, 0.4400f, 0.4500f, 0.4900f, 0.5000f, 0.5200f, 0.5300f, 0.5700f,
			0.5800f, 0.5900f, 0.6100f, 0.6200f, 0.6200f, 0.6500f, 0.6600f, 0.6700f, 0.6800f, 0.6900f,
			0.6900f, 0.6900f, 0.7100f, 0.7200f, 0.7300f, 0.7300f, 0.7400f, 0.7400f, 0.7500f, 0.7600f,
			0.7600f, 0.7600f, 0.7700f, 0.7700f, 0.7700f, 0.7800f, 0.7800f, 0.7900f, 0.7900f, 0.8000f,
			0.8000f, 0.8000f, 0.8100f, 0.8200f, 0.8200f, 0.8200f, 0.8300f, 0.8300f, 0.8400f, 0.8400f,
			0.8400f, 0.8400f, 0.8400f, 0.8500f
		}, {
			0.0850f, 0.1000f, 0.1000f, 0.1100f, 0.1200f, 0.1400f, 0.1800f, 0.2300f, 0.2800f, 0.3300f,
			0.3500f, 0.3800f, 0.4000f, 0.4400f, 0.4500f, 0.4900f, 0.5000f, 0.5100f, 0.5300f, 0.5700f,
			0.5800f, 0.5900f, 0.6100f, 0.6200f, 0.6200f, 0.6500f, 0.6600f, 0.6700f, 0.6800f, 0.6800f,
			0.6900f, 0.6900f, 0.7100f, 0.7200f, 0.7300f, 0.7300f, 0.7400f, 0.7400f, 0.7500f, 0.7600f,
			0.7600f, 0.7600f, 0.7600f, 0.7700f, 0.7700f, 0.7800f, 0.7800f, 0.7900f, 0.7900f, 0.8000f,
			0.8000f, 0.8000f, 0.8100f, 0.8200f, 0.8200f, 0.8200f, 0.8300f, 0.8300f, 0.8300f, 0.8300f,
			0.8300f, 0.8400f, 0.8400f, 0.8500f
		}, {
			0.0900f, 0.0900f, 0.0900f, 0.1100f, 0.1200f, 0.1300f, 0.1800f, 0.2200f, 0.2800f, 0.3300f,
			0.3500f, 0.3800f, 0.4000f, 0.4300f, 0.4500f, 0.4900f, 0.5000f, 0.5100f, 0.5300f, 0.5600f,
			0.5800f, 0.5900f, 0.6100f, 0.6200f, 0.6200f, 0.6500f, 0.6600f, 0.6600f, 0.6700f, 0.6700f,
			0.6900f, 0.6900f, 0.7100f, 0.7100f, 0.7300f, 0.7300f, 0.7300f, 0.7400f, 0.7500f, 0.7600f,
			0.7600f, 0.7600f, 0.7600f, 0.7700f, 0.7700f, 0.7800f, 0.7800f, 0.7900f, 0.7900f, 0.8000f,
			0.8000f, 0.8000f, 0.8100f, 0.8100f, 0.8200f, 0.8200f, 0.8200f, 0.8300f, 0.8300f, 0.8300f,
			0.8300f, 0.8400f, 0.8400f, 0.8500f
		}, {
			0.0950f, 0.0900f, 0.0900f, 0.1000f, 0.1100f, 0.1300f, 0.1800f, 0.2200f, 0.2800f, 0.3300f,
			0.3400f, 0.3800f, 0.4000f, 0.4300f, 0.4500f, 0.4800f, 0.4900f, 0.5100f, 0.5300f, 0.5600f,
			0.5700f, 0.5900f, 0.6100f, 0.6200f, 0.6200f, 0.6500f, 0.6500f, 0.6600f, 0.6700f, 0.6700f,
			0.6800f, 0.6900f, 0.7100f, 0.7100f, 0.7200f, 0.7300f, 0.7300f, 0.7400f, 0.7500f, 0.7500f,
			0.7600f, 0.7600f, 0.7600f, 0.7700f, 0.7700f, 0.7800f, 0.7800f, 0.7900f, 0.7900f, 0.8000f,
			0.8000f, 0.8000f, 0.8100f, 0.8100f, 0.8200f, 0.8200f, 0.8200f, 0.8200f, 0.8300f, 0.8300f,
			0.8300f, 0.8400f, 0.8400f, 0.8500f
		}, {
			0.1000f, 0.0900f, 0.0900f, 0.1000f, 0.1100f, 0.1200f, 0.1700f, 0.2200f, 0.2700f, 0.3200f,
			0.3400f, 0.3700f, 0.4000f, 0.4300f, 0.4400f, 0.4800f, 0.4900f, 0.5100f, 0.5300f, 0.5600f,
			0.5700f, 0.5800f, 0.6100f, 0.6200f, 0.6200f, 0.6400f, 0.6500f, 0.6600f, 0.6700f, 0.6700f,
			0.6800f, 0.6900f, 0.7100f, 0.7100f, 0.7200f, 0.7300f, 0.7300f, 0.7400f, 0.7500f, 0.7500f,
			0.7600f, 0.7600f, 0.7600f, 0.7700f, 0.7700f, 0.7800f, 0.7800f, 0.7900f, 0.7900f, 0.8000f,
			0.8000f, 0.8000f, 0.8100f, 0.8100f, 0.8200f, 0.8200f, 0.8200f, 0.8200f, 0.8300f, 0.8300f,
			0.8300f, 0.8300f, 0.8400f, 0.8500f
		}, {
			0.1200f, 0.0800f, 0.0800f, 0.0900f, 0.0900f, 0.1100f, 0.1600f, 0.2100f, 0.2600f, 0.3100f,
			0.3300f, 0.3700f, 0.3900f, 0.4200f, 0.4400f, 0.4800f, 0.4900f, 0.5100f, 0.5300f, 0.5500f,
			0.5700f, 0.5800f, 0.6000f, 0.6100f, 0.6100f, 0.6400f, 0.6400f, 0.6600f, 0.6700f, 0.6700f,
			0.6800f, 0.6800f, 0.7000f, 0.7100f, 0.7200f, 0.7200f, 0.7300f, 0.7400f, 0.7400f, 0.7500f,
			0.7600f, 0.7600f, 0.7600f, 0.7700f, 0.7700f, 0.7800f, 0.7800f, 0.7800f, 0.7900f, 0.7900f,
			0.8000f, 0.8000f, 0.8100f, 0.8100f, 0.8100f, 0.8200f, 0.8200f, 0.8200f, 0.8300f, 0.8300f,
			0.8300f, 0.8300f, 0.8400f, 0.8400f
		}, {
			0.1500f, 0.0700f, 0.0700f, 0.0800f, 0.0800f, 0.1000f, 0.1600f, 0.2000f, 0.2500f, 0.2900f,
			0.3200f, 0.3500f, 0.3900f, 0.4100f, 0.4400f, 0.4700f, 0.4900f, 0.5000f, 0.5200f, 0.5500f,
			0.5600f, 0.5700f, 0.6000f, 0.6100f, 0.6100f, 0.6400f, 0.6400f, 0.6600f, 0.6600f, 0.6600f,
			0.6800f, 0.6800f, 0.7000f, 0.7100f, 0.7100f, 0.7200f, 0.7300f, 0.7300f, 0.7400f, 0.7500f,
			0.7600f, 0.7600f, 0.7600f, 0.7600f, 0.7700f, 0.7700f, 0.7800f, 0.7800f, 0.7900f, 0.7900f,
			0.7900f, 0.8000f, 0.8100f, 0.8100f, 0.8100f, 0.8100f, 0.8200f, 0.8200f, 0.8300f, 0.8300f,
			0.8300f, 0.8300f, 0.8400f, 0.8400f
		}, {
			0.2000f, 0.0500f, 0.0500f, 0.0600f, 0.0600f, 0.1000f, 0.1500f, 0.1900f, 0.2400f, 0.2700f,
			0.3100f, 0.3400f, 0.3800f, 0.4100f, 0.4300f, 0.4600f, 0.4800f, 0.5000f, 0.5100f, 0.5400f,
			0.5500f, 0.5700f, 0.5900f, 0.6000f, 0.6100f, 0.6400f, 0.6400f, 0.6500f, 0.6600f, 0.6600f,
			0.6800f, 0.6800f, 0.7000f, 0.7100f, 0.7100f, 0.7100f, 0.7300f, 0.7300f, 0.7400f, 0.7500f,
			0.7600f, 0.7600f, 0.7600f, 0.7600f, 0.7600f, 0.7700f, 0.7700f, 0.7800f, 0.7800f, 0.7900f,
			0.7900f, 0.8000f, 0.8000f, 0.8100f, 0.8100f, 0.8100f, 0.8100f, 0.8200f, 0.8200f, 0.8200f,
			0.8300f, 0.8300f, 0.8300f, 0.8400f
		}, {
			0.2500f, 0.0400f, 0.0400f, 0.0500f, 0.0600f, 0.0900f, 0.1400f, 0.1800f, 0.2300f, 0.2500f,
			0.3000f, 0.3400f, 0.3700f, 0.4100f, 0.4300f, 0.4600f, 0.4700f, 0.5000f, 0.5100f, 0.5400f,
			0.5500f, 0.5700f, 0.5900f, 0.6000f, 0.6000f, 0.6300f, 0.6400f, 0.6500f, 0.6600f, 0.6600f,
			0.6800f, 0.6800f, 0.7000f, 0.7000f, 0.7100f, 0.7100f, 0.7300f, 0.7300f, 0.7400f, 0.7500f,
			0.7500f, 0.7600f, 0.7600f, 0.7600f, 0.7600f, 0.7700f, 0.7700f, 0.7700f, 0.7800f, 0.7800f,
			0.7900f, 0.8000f, 0.8000f, 0.8100f, 0.8100f, 0.8100f, 0.8100f, 0.8200f, 0.8200f, 0.8200f,
			0.8200f, 0.8300f, 0.8300f, 0.8400f
		}, {
			0.2600f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f,
			0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f,
			0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f,
			0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f,
			0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f,
			0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f,
			0.0000f, 0.0000f, 0.0000f, 0.0000f
		}, {
			1.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f,
			0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f,
			0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f,
			0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f,
			0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f,
			0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f,
			0.0000f, 0.0000f, 0.0000f, 0.0000f
		} };
/* LD factorization (Q=L'*diag(D)*L) -----------------------------------------*/
static int ld(int n, const double* Q, double* L, double* D)
{
	int i = 0, j = 0, k = 0, info = 0;
	double a = 1.0, * A = mat(n, n);

	memcpy(A, Q, sizeof(double) * n * n);
	for (i = n - 1; i >= 0; i--) {
		if ((D[i] = A[i + i * n]) <= 0.0) { info = -1; break; }
		a = sqrt(D[i]);
		for (j = 0; j <= i; j++) L[i + j * n] = A[i + j * n] / a;
		for (j = 0; j <= i - 1; j++) for (k = 0; k <= j; k++) A[j + k * n] -= L[i + k * n] * L[i + j * n];
		for (j = 0; j <= i; j++) L[i + j * n] /= L[i + i * n];
	}
	xy_free(A);
	if (info) trace(3, "%s : LD factorization error\r\n", __FILE__);
	return info;
}
/* integer gauss transformation ----------------------------------------------*/
static void gauss(int n, double* L, double* Z, int i, int j)
{
	int k = 0, mu = 0;

	if ((mu = (int)ROUND(L[i + j * n])) != 0) {
		for (k = i; k < n; k++) L[k + n * j] -= (double)mu * L[k + i * n];
		for (k = 0; k < n; k++) Z[k + n * j] -= (double)mu * Z[k + i * n];
	}
}
/* permutations --------------------------------------------------------------*/
static void perm(int n, double* L, double* D, int j, double del, double* Z)
{
	int k = 0;
	double eta = 0.0, lam = 0.0, a0 = 0.0, a1 = 0.0;

	eta = D[j] / del;
	lam = D[j + 1] * L[j + 1 + j * n] / del;
	D[j] = eta * D[j + 1]; D[j + 1] = del;
	for (k = 0; k <= j - 1; k++) {
		a0 = L[j + k * n]; a1 = L[j + 1 + k * n];
		L[j + k * n] = -L[j + 1 + j * n] * a0 + a1;
		L[j + 1 + k * n] = eta * a0 + lam * a1;
	}
	L[j + 1 + j * n] = lam;
	for (k = j + 2; k < n; k++) SWAP(L[k + j * n], L[k + (j + 1) * n]);
	for (k = 0; k < n; k++) SWAP(Z[k + j * n], Z[k + (j + 1) * n]);
}
/* lambda reduction (z=Z'*a, Qz=Z'*Q*Z=L'*diag(D)*L) (ref.[1]) ---------------*/
static void reduction(int n, double* L, double* D, double* Z)
{
	int i = 0, j = 0, k = 0;
	double del = 0.0;

	j = n - 2; k = n - 2;//n-2，n是模糊度数量，换到矩阵索引要-1，第j列的del要和j+1列比（j+1<=n-1）,所以n-2
	while (j >= 0) {
		if (j <= k) for (i = j + 1; i < n; i++) gauss(n, L, Z, i, j);//对L和Z整数变换降相关，从最后一列开始，对各列非对角线元素从上往下依次降相关
		del = D[j] + L[j + 1 + j * n] * L[j + 1 + j * n] * D[j + 1];
		if (del + 1E-6 < D[j + 1]) { /* compared considering numerical error */
			perm(n, L, D, j, del, Z);
			k = j; j = n - 2;
		}
		else j--;
	}
}

double interp1q(double* x, double* y, double input)
{
	if (x == NULL || y == NULL) return -1.0;
	int i = 0;
	double output = 0.0;

	for (i = 0; i < 31; i++) {
		if (x[i] <= input && x[i + 1] >= input) {
			output = y[i] + (y[i + 1] - y[i]) * (input - x[i]) / (x[i + 1] - x[i]);
			break;
		}
	}
	return output;
}

double normal(const double z)
{
	return exp((-1) * z * z / 2) / sqrt(2 * PI);
}
double mproduct(const double* x, const int n)
{
	if (x == NULL) return -1.0;
	int i = 0;
	double y = 1.0;
	for (i = 0; i < n; i++)
		y *= x[i];
	return y;
}

double normsdist(const double z)
{
	static const double gamma = 0.231641900,
		a1 = 0.319381530,
		a2 = -0.356563782,
		a3 = 1.781477973,
		a4 = -1.821255978,
		a5 = 1.330274429;
	double k = 1.0 / (1 + fabs(z) * gamma);
	double n = k * (a1 + k * (a2 + k * (a3 + k * (a4 + k * a5))));

	/* this guards against overflow */
	if (z > 6)
		return 1;
	if (z < -6)
		return 0;

	n = 1 - normal(z) * n;
	if (z < 0) return (1.0 - n);

	return n;
}
/*bootstrap 的参数估计和假设检验 https://blog.csdn.net/gaofeipaopaotang/article/details/130743005
 *D:样本
*/
double bootstrapd(const double* D, const int n)
{
	if (D == NULL) return -1.0;
	int i = 0;
	double* np = NULL, boot = 0.0;
	np = zeros(n, 1);

	for (i = 0; i < n; i++)
		np[i] = 2 * normsdist(1 / (2 * sqrt(D[i]))) - 1;
	boot = mproduct(np, n);
	xy_free(np);
	return boot;
}
/* modified lambda (mlambda) search (ref. [2]) -------------------------------*/
static int search(int n, int m, const double* L, const double* D,
	const double* zs, double* zn, double* s)
{
	int i = 0, j = 0, k = 0, c = 0, nn = 0, imax = 0;
	double newdist = 0.0, maxdist = 1E99, y = 0.0;
	double* S = zeros(n, n), * dist = mat(n, 1), * zb = mat(n, 1), * z = mat(n, 1), * step = mat(n, 1);
	k = n - 1; dist[k] = 0.0;
	zb[k] = zs[k];
	z[k] = ROUND(zb[k]); y = zb[k] - z[k]; step[k] = SGN(y);

	for (c = 0; c < LOOPMAX; c++) {
		newdist = dist[k] + y * y / D[k];
		if (newdist < maxdist) {
			if (k != 0) {
				dist[--k] = newdist;
				for (i = 0; i <= k; i++)
					S[k + i * n] = S[k + 1 + i * n] + (z[k + 1] - zb[k + 1]) * L[k + 1 + i * n];
				zb[k] = zs[k] + S[k + k * n];
				z[k] = ROUND(zb[k]); y = zb[k] - z[k]; step[k] = SGN(y);
			}
			else {
				if (nn < m) {
					if (nn == 0 || newdist > s[imax]) imax = nn;
					for (i = 0; i < n; i++) zn[i + nn * n] = z[i];
					s[nn++] = newdist;
				}
				else {
					if (newdist < s[imax]) {
						for (i = 0; i < n; i++) zn[i + imax * n] = z[i];
						s[imax] = newdist;
						for (i = imax = 0; i < m; i++) if (s[imax] < s[i]) imax = i;
					}
					maxdist = s[imax];
				}
				z[0] += step[0]; y = zb[0] - z[0]; step[0] = -step[0] - SGN(step[0]);
			}
		}
		else {
			if (k == n - 1) break;
			else {
				k++;
				z[k] += step[k]; y = zb[k] - z[k]; step[k] = -step[k] - SGN(step[k]);
			}
		}
	}
	for (i = 0; i < m - 1; i++) { /* sort by s */
		for (j = i + 1; j < m; j++) {
			if (s[i] < s[j]) continue;
			SWAP(s[i], s[j]);
			for (k = 0; k < n; k++) SWAP(zn[k + i * n], zn[k + j * n]);
		}
	}

	double ratio = s[0] > 0 ? (s[1] / s[0]) : 0.0;//
	trace(4, "lambda search time:%d ratio:%f\n", c, ratio);

	xy_free(S); xy_free(dist); xy_free(zb); xy_free(z); xy_free(step);

	if (c >= LOOPMAX) {
		trace(2, "%s : search loop count overflow\n", __FILE__);
		return 1;
	}
	return 0;
}
/* lambda/mlambda integer least-square estimation ------------------------------
* integer least-square estimation. reduction is performed by lambda (ref.[1]),
* and search by mlambda (ref.[2]).
* args   : int    n      I  number of float parameters
*          int    m      I  number of fixed solutions
*          double *a     I  float parameters (n x 1)
*          double *Q     I  covariance matrix of float parameters (n x n)
*          double *F     O  fixed solutions (n x m)
*          double *s     O  sum of squared residulas of fixed solutions (1 x m)
* return : status (0:ok,other:error)
* notes  : matrix stored by column-major order (fortran convension)
*-----------------------------------------------------------------------------*/
extern int lambda(int n, int m, const double* a, const double* Q, double* F,
	double* s)
{
	int info = 0;
	double* L = NULL, * D = NULL, * Z = NULL, * z = NULL, * E = NULL;

	if (n <= 0 || m <= 0) return -1;
	L = zeros(n, n); D = mat(n, 1); Z = eye(n); z = mat(n, 1); E = mat(n, m);

	/* LD factorization */

	if (!(info = ld(n, Q, L, D))) {
		/* lambda reduction */
		reduction(n, L, D, Z);
		matmul("TN", n, 1, n, 1.0, Z, a, 0.0, z); /* z=Z'*a */

		/* mlambda search */
		if (!(info = search(n, m, L, D, z, E, s))) {
			info = solve("T", Z, E, n, m, F); /* F=Z'\E */
		}
	}
	xy_free(L); xy_free(D); xy_free(Z); xy_free(z); xy_free(E);
	return info;
}

int ffrtlam(int n, int cand, const double* fa, const double* qa,double* aoptis, double* sqnorm, double ratio, double* mu, int* flag, prcopt_t opt)
{
	if (fa == NULL || qa == NULL || aoptis == NULL || sqnorm == NULL || mu == NULL || flag == NULL) return -1;
	if (n <= 0 || cand <= 0) return -1;
	int info = 0, i = 0;
	double* L = NULL, * D = NULL, * Z = NULL, * fz = NULL, * zoptis = NULL, ps = 0.0, pfils = 0.0;
	double t1[31] = { 0.0 }, tn[31] = { 0.0 };
	L = zeros(n, n);
	D = mat(n, 1);
	Z = eye(n);
	fz = mat(n, 1);
	zoptis = mat(n, cand);

	/* LD factorization */
	if (!(info = ld(n, qa, L, D))) {//将双差方差阵Qa进行三角分解，分解成L与D
		reduction(n, L, D, Z);//降相关得到Z
		matmul("TN", n, 1, n, 1.0, Z, fa, 0.0, fz); /* fz=Z'*fa  得到降相关后的*/

		/* \u8BA1\u7B97\u6240\u6709\u6A21\u7CCA\u5EA6\u7684Ps */
		ps = bootstrapd(D, n);

		/* \u8BA1\u7B97\u6700\u5C0F\u4E8C\u4E58\u5931\u8D25\u7387PfILS */
		pfils = 1 - ps;

		trace(5, "ffrtlam:ps=%6.3f\n", ps);
		//TODO12W
//        if (opt.dynamics && ps < 0.9) {
//            info = -1;
//            xy_free(L);xy_free(D);xy_free(Z);
//            xy_free(fz);xy_free(zoptis);

//            return info;
//        }
		/* SE-VB */
		if (!(info = search(n, cand, L, D, fz, zoptis, sqnorm))) {//zOptis中储存一个最优解一个次优解，sqnorm最优解与次优解的数量
			/* aOptis=inv(Z')*zOptis */
			if (!(info = solve("T", Z, zoptis, n, cand, aoptis))) {//将在新空间中固定的模糊度逆变换回双差模糊度空间中
				if (n > 63) n = 63; 
				for (i = 0; i < 31; i++) {//查表
					t1[i] = (double)table1[i][0];
					tn[i] = (double)table1[i][n];
				}
				*mu = interp1q(t1, tn, pfils);

				if (*mu == 0.0) *mu = 1.0e-10;

				*mu = 1 / (*mu);
				if (*mu > ratio) *mu = ratio;
				if (*mu < 1.3) {  /* 1.2  to  1.3 */
					if (opt.ionoopt == IONOOPT_IFLC) *mu = 1.5;
					else *mu = n >= 14 ? 1.15 : (n >= 8 ? 1.2 : 1.3);
				}
				/* ratio test */
				if (sqnorm[1] / sqnorm[0] >= *mu)
					*flag = 1;
				else
					*flag = 0;
			}
		}
	}
	xy_free(L);
	xy_free(D);
	xy_free(Z);
	xy_free(fz);
	xy_free(zoptis);
	return info;
}
//将参与双差星锁定时常从小到大排序
int orderlock(int n, const int* locknum, int* sortam) {
	if (locknum == NULL || sortam == NULL) {
		return -1;
	}
	int i = 0, j = 0, k = 0, t_ = 0;
	int temp = 0;

	int l[MAXDIFOBS * NFREQ] = { 0 };
	for (i = 0; i < n; i++)
		l[i] = locknum[i];
	for (i = 0; i < n - 1; i++) {
		k = i;
		for (j = i + 1; j < n; j++) {
			if (l[k] > l[j]) {
				temp = l[i];
				t_ = sortam[i];
				l[i] = l[j];
				sortam[i] = sortam[j];
				l[j] = temp;
				sortam[j] = t_;
			}
		}
	}

	return 1;
}

int orderlock2(int n, const int* locknum, const double* qa, int* sortam) {
	if (locknum == NULL || sortam == NULL) {
		return -1;
	}
	int i = 0, j = 0, k = 0, t_ = 0;
	int temp = 0;

	int l[MAXDIFOBS * NFREQ] = { 0.0 };
	for (i = 0; i < n; i++)
		l[i] = locknum[i];
	for (i = 0; i < n - 1; i++) {
		k = i;
		for (j = i + 1; j < n; j++) {
			if (l[k] > l[j]) {
				temp = l[i];
				t_ = sortam[i];
				l[i] = l[j];
				sortam[i] = sortam[j];
				l[j] = temp;
				sortam[j] = t_;
			}
			else if (l[k] == l[j])
			{
				if (qa[k + k * n] < qa[j + j * n]) {
					temp = l[i];
					t_ = sortam[i];
					l[i] = l[j];
					sortam[i] = sortam[j];
					l[j] = temp;
					sortam[j] = t_;
				}
			}
		}
	}

	return 1;
}

void rebdy(const double* qy, const int* index, double* pqy, int d, int n, int na)
{
	if (qy == NULL || index == NULL || pqy == NULL) return;
	if (!pqy || !qy) return;
	int i = 0, j = 0, k = 0, l = 0, m = 0;
	double* qd = NULL;
	n = n + na;
	qd = zeros(n, n);

	for (i = 0; i < d; i++)
	{
		k = index[i] + na;
		for (j = 0; j < n; j++){
			qd[k + j * n] = -1;
			qd[k * n + j] = -1;
		}
	}
	for (i = 0; i < n; i++)
	{
		for (j = 0; j < n; j++)
		{
			if (qd[j + i * n] < 0)
				continue;
			else
				pqy[m++] = qy[j + i * n];
		}
	}
	xy_free(qd);
}

void rebd(const double* aa, const double* qa, const int* index, double* fp, double* qfaa, int d, int n)
{
	if (aa == NULL || qa == NULL || index == NULL || fp == NULL || qfaa == NULL) {
		return;
	}
	int i = 0, j = 0, k = 0, l = 0, m = 0;
	double* fd = NULL, * qd = NULL;
	fd = zeros(n, 1);
	qd = zeros(n, n);
	for (i = 0; i < d; i++)
	{
		k = index[i];
		fd[k] = -1;
		for (j = 0; j < n; j++)
		{
			qd[k + j * n] = -1;
			qd[k * n + j] = -1;
		}
	}
	for (i = 0; i < n; i++)
	{
		if (fd[i] < 0)
			continue;
		else
			fp[l++] = aa[i];
	}

	for (i = 0; i < n; i++)
	{
		for (j = 0; j < n; j++)
		{
			if (qd[j + i * n] < 0)
				continue;
			else
				qfaa[m++] = qa[j + i * n];
		}
	}
	xy_free(fd);
	xy_free(qd);
}

int amblockpar_new(rtk_t* rtk, int* nb, int na, double* aa, double* qa, double* qy, double* py, double* pqy, double* pb, double* s, int* flag, int* locknum, int* index)
{
	int ofo = -1, i = 0, j = 0, k = 0, l = 0, num = 0, temp = 0, n = *nb, fixflag = 0;

	double* qfaa = NULL;
	double faa[MAXDIFOBS * NFREQ] = { 0.0 };
	int sortam[MAXDIFOBS * NFREQ] = { 0 };
	double mu = 0.0;
	int sat = 0, freq = 0;
	if (n <= 0)
		return -1;

	for (i = 0; i < n; i++)
		sortam[i] = i;
	orderlock2(n, locknum, qa, sortam);//将参与双差星锁定时常从小到大排序
	trace(5, "order amblocpar:");
	for (i = 0; i < n; i++)trace(5, "%d,", sortam[i]);
	trace(5, "\r\n");

	qfaa = mat(n, n);

	for (k = 1; k < MAXTRYNUM; k++) {
		if ((n - k) < MINPARNUM) {
			ofo = -1;  break;
		}

		//        if (locknum[sortam[k - 1]] >= locknum[sortam[n - 1]] || locknum[sortam[k - 1]] > 30) {
		//            trace(5, "no more smaller num.lock=%d max=%d\r\n", locknum[sortam[k - 1]], locknum[sortam[n - 1]]);
		//            ofo = -1; break;
		//        }
		num = n - k;
		memset(faa, 0, sizeof(double) * n);
		memset(qfaa, 0, sizeof(double) * n * n);

		memset(pb, 0, sizeof(double) * n * 2);
		memset(s, 0, sizeof(double) * 2);
		if (k > 0)
			*flag = 0;
		rebd(aa, qa, sortam, faa, qfaa, k, n);
		rebdy(qy, sortam, pqy, k, n, na);
		//add  ofo = 0,过程无误，flag为0,固定 否则失败
		ofo = ffrtlam(num, 2, faa, qfaa, pb, s, rtk->opt.thresar[0], &mu, &fixflag,//flag =1 FIX
			rtk->opt);
		if (!ofo) {
			/* Amb trans for parlambda */
			rtk->sol.ratio = s[0] > 0 ? (float)(s[1] / s[0]) : 0.0f;
			if (rtk->sol.ratio > 999.9) rtk->sol.ratio = 999.9f;
		}

		trace(5, "amblockpar info: ratio=%6.3f mu=%6.3f\n", rtk->sol.ratio, mu);

		if (!fixflag) {
			continue;
		}

		*flag = 2;
		for (i = 0; i < num; i++)
			py[i] = faa[i];

		break;
	}

	*nb = num;
	if (ofo < 0)
		*flag = 0;
	if (*flag) {
		sat = 0;
		freq = 0;
		for (i = 0; i < k; i++) {
			sat = (index[sortam[i]] >> 16) & 0xFF;
			freq = index[sortam[i]] & 0xF;
			//根据卫星号找索引
			j = ix(sat, rtk->satid, rtk->presat);
			rtk->ssat[j].fix[freq] = 0;
			trace(2, "amblockpar:exclude:sat=%2d  fix=%d \r\n", sat, rtk->ssat[j].fix[freq]);
		}
	}
	xy_free(qfaa);
	return ofo;
}

int orderamb(int n, const double* qa, int* sortam)
{
	if (qa == NULL || sortam == NULL) {
		return -1;
	}
	int i = 0, j = 0, k = 0, t_ = 0;
	double* Q = NULL, temp = 0;
	Q = mat(n, 1);
	for (i = 0; i < n; i++)
		Q[i] = qa[i + i * n];
	for (i = 0; i < n - 1; i++)
	{
		k = i;
		for (j = i + 1; j < n; j++)
		{
			if (Q[k] < Q[j])
			{
				temp = Q[i];
				t_ = sortam[i];
				Q[i] = Q[j];
				sortam[i] = sortam[j];
				Q[j] = temp;
				sortam[j] = t_;
			}
		}
	}
	xy_free(Q);
	return 1;
}

int ambvarpar_new(rtk_t* rtk, int* nb, int na, double* aa, double* qa, double* qy, double* py, double* pqy, double* pb, double* s, int* flag, int* index)
{
	int ofo = -1, i = 0, j = 0, k = 0, l = 0, num = 0, temp = 0, n = *nb;
	double mu = 0;
	int sortam[MAXDIFOBS * NFREQ] = { 0 };

	double* qfaa = NULL;
	double faa[MAXDIFOBS * NFREQ] = { 0.0 };

	int  fixflag = 0;

	if (n <= 0)
		return -1;

	for (i = 0; i < n; i++)
		sortam[i] = i;
	orderamb(n, qa, sortam);
	trace(5, "order ambvarpar22:");
	for (i = 0; i < n; i++)
		trace(5, "%d,", sortam[i]);
	trace(5, "\r\n");

	qfaa = mat(n, n);

	for (k = 1; k < MAXTRYNUM; k++) {
		if ((n - k) < MINPARNUM) {
			ofo = -1;
			break;
		}
		trace(5, "order ambvarpar: n=%4d k=%4d min=%4d\r\n", n, k, MINPARNUM);
		num = n - k;

		memset(faa, 0, sizeof(double) * n);
		memset(qfaa, 0, sizeof(double) * n * n);

		memset(pb, 0, sizeof(double) * n * 2);
		memset(s, 0, sizeof(double) * 2);
		if (k > 0)
			*flag = 0;
		rebd(aa, qa, sortam, faa, qfaa, k, n);
		rebdy(qy, sortam, pqy, k, n, na);
		ofo = ffrtlam(num, 2, faa, qfaa, pb, s, rtk->opt.thresar[0], &mu, &fixflag,//flag =1 FIX
			rtk->opt);
		if (!ofo) {
			/* Amb trans for parlambda */
			rtk->sol.ratio = s[0] > 0 ? (float)(s[1] / s[0]) : 0.0f;
			if (rtk->sol.ratio > 999.9) rtk->sol.ratio = 999.9f;
		}
		trace(5, "ambvarpar info: ratio=%6.3f mu=%6.3f\n", rtk->sol.ratio, mu);
		if (!fixflag) {
			continue;
		}

		*flag = 2;
		for (i = 0; i < num; i++)
			py[i] = faa[i];

		break;
	}

	*nb = num;
	if (ofo < 0)
		*flag = 0;
	if (*flag) {
		int sat, freq;
		for (i = 0; i < k; i++) {
			sat = (index[sortam[i]] >> 16) & 0xFF;
			freq = index[sortam[i]] & 0xF;
			j = ix(sat, rtk->satid, rtk->presat);
			rtk->ssat[j].fix[freq] = 0;
			trace(4, "ambvarpar exclude:sat=%2d\r\n", sat);
		}
	}
	xy_free(qfaa);
	return ofo;
}

int orderpar(int n, const double* refpar, int* sortam)
{
	if (refpar == NULL || sortam == NULL) {
		return -1;
	}
	int i = 0, j = 0, k = 0, t_ = 0;
	double temp = 0, * l = NULL;
	l = mat(n, 1);
	for (i = 0; i < n; i++)
		l[i] = refpar[i];
	for (i = 0; i < n - 1; i++) {
		k = i;
		for (j = i + 1; j < n; j++) {
			if (l[k] > l[j]) {
				temp = l[i];
				t_ = sortam[i];
				l[i] = l[j];
				sortam[i] = sortam[j];
				l[j] = temp;
				sortam[j] = t_;
			}
		}
	}
	xy_free(l);
	return 1;
}

int ambrefpar_new(rtk_t* rtk, int* nb, int na, double* aa, double* qa,
	double* qy, double* py, double* pqy, double* pb, double* s, int* flag, double* refpar, int* index)
{
	if (rtk == NULL || nb == NULL || aa == NULL || qa == NULL || qy == NULL || py == NULL ||
		pqy == NULL || pb == NULL || s == NULL || flag == NULL || refpar == NULL || index == NULL) {
		return -1;
	}
	int ofo = -1, i = 0, j = 0, k = 0, l = 0, num = 0, temp = 0, n = *nb;
	double mu = 0;
	double* qfaa = NULL;
	double faa[MAXDIFOBS * NFREQ] = { 0.0 };
	int sortam[MAXDIFOBS * NFREQ] = { 0 };
	int fixflag = 0;

	if (n <= 0)
		return -1;

	qfaa = mat(n, n);
	for (i = 0; i < n; i++)
		sortam[i] = i;
	orderpar(n, refpar, sortam);
	trace(5, "order ambrefpar:");
	for (i = 0; i < n; i++)
		trace(5, "%d,", sortam[i]);
	trace(5, "\r\n");

	for (k = 1; k < MAXTRYNUM; k++) {
		if ((n - k) < MINPARNUM) {
			ofo = -1;  break;
		}
		num = n - k;

		memset(faa, 0, sizeof(double) * n);
		memset(qfaa, 0, sizeof(double) * n * n);

		memset(pb, 0, sizeof(double) * n * 2);
		memset(s, 0, sizeof(double) * 2);
		if (k > 0)
			*flag = 0;
		rebd(aa, qa, sortam, faa, qfaa, k, n);
		rebdy(qy, sortam, pqy, k, n, na);

		ofo = ffrtlam(num, 2, faa, qfaa, pb, s, rtk->opt.thresar[0], &mu, &fixflag,//flag =1 FIX
			rtk->opt);

		if (!ofo) {
			/* Amb trans for parlambda */
			rtk->sol.ratio = s[0] > 0 ? (float)(s[1] / s[0]) : 0.0f;
			if (rtk->sol.ratio > 999.9) rtk->sol.ratio = 999.9f;
		}
		if (!fixflag) {
			continue;
		}

# if 0
		rtk->sol.ratio = (float)(s[1] / s[0]);
		if (rtk->sol.ratio < rtk->opt.thresar[0])
		{
			trace->trace(5, "ambrefpar failure: ratio=%f k=%d\n", rtk->sol.ratio, k);
			free(qfaa); free(faa);
			continue;
		}
#endif

		* flag = 2;
		for (i = 0; i < num; i++)
			py[i] = faa[i];
		break;
	}
	*nb = num;
	if (ofo < 0)
		*flag = 0;
	if (*flag) {
		int sat, freq;
		for (i = 0; i < k; i++) {
			sat = (index[sortam[i]] >> 16) & 0xFF;
			freq = index[sortam[i]] & 0xF;
			j = ix(sat, rtk->satid, rtk->presat);
			rtk->ssat[j].fix[freq] = 0;
			trace(4, "ambrefpar:exclude:sat=%2d\r\n", sat);
		}
	}
	xy_free(qfaa);
	return ofo;
}

#define MIN_INT       1.0e-4
#if 0

int ambtestbyhis(rtk_t* rtk, double* b, double* y, int* index, int n, int* nb, int* fixflag, int na, int* fb)
{
	if (rtk == NULL || b == NULL || y == NULL || index == NULL || nb == NULL || fixflag == NULL || fb == NULL) {
		return -1;
	}
	int i = 0, sat = 0, refsat = 0, freq = 0, nf = 0;
	double ddamb = 0.0;
	trace(3, "AMB_Test: fixflag=%d n=%4d\n", *fixflag, *nb);
	int j = 0;

	int ref_flag = 0;
	if (*fixflag == 0)
		return 0;
	for (i = 0; i < n; i++) {
		refsat = (index[i] >> 8) & 0xFF; sat = (index[i] >> 16) & 0xFF;
		freq = index[i] & 0xF;
		ref_flag = 0;
		trace(4, "sat L%1d: %3d-%3d fix=%2d lock=%2d\n", freq + 1, refsat, sat, rtk->ssat[sat - 1].fix[freq], rtk->ssat[sat - 1].lock[freq]);
		if (rtk->ssat[sat - 1].fix[freq] != 2)
			continue;

		trace(4, "sat L%1d: %3d-%3d History Amb: amb=%12.3f breamb=%12.3f refamb=%12.3f count=%6d ambrefsat=%4d slip=%2d\n",
			freq + 1, refsat, sat, b[i], y[i], rtk->ssat[sat - 1].amc[freq].refamb, rtk->ssat[sat - 1].amc[freq].ambcount, rtk->ssat[sat - 1].amc[freq].refsat, (int)rtk->ssat[sat - 1].slip[freq]);
		if (rtk->ssat[sat - 1].amc[freq].ambcount == 0)
		{
			rtk->ssat[sat - 1].fix[freq] = 0;
			rtk->ssat[sat - 1].amc[freq].ambcount = 1;
			rtk->ssat[sat - 1].amc[freq].refsat = refsat;
			rtk->ssat[sat - 1].amc[freq].refamb = b[i];
			continue;
		}
		//if (refsat == rtk->ssat[sat - 1].amc[freq].refsat && !(rtk->ssat[sat - 1].slip[freq] &2)&& !(rtk->ssat[refsat - 1].slip[freq]&2))//!(rtk->ssat[j - k].slip[freq] & 2))
		if (refsat == rtk->ssat[sat - 1].amc[freq].refsat && !rtk->ssat[sat - 1].slip[freq] && !rtk->ssat[refsat - 1].slip[freq])//!(rtk->ssat[j - k].slip[freq] & 2))
		{
			ddamb = rtk->ssat[sat - 1].amc[freq].refamb;
			if (fabs(b[i] - ddamb) < MIN_INT)                              //当模糊度与参考模糊度相比，未发生改变时参与固定位置解算
			{
				if (rtk->ssat[sat - 1].amc[freq].ambcount > 0)    //初步为5
				{
					rtk->ssat[sat - 1].fix[freq] = 2;
					nf = nf + 1;
					fb[i] = 1;
				}
				else {
					rtk->ssat[sat - 1].fix[freq] = 0;
				}
				//if (RaFailFlag)     //迭代时不参与连续历元统计
				rtk->ssat[sat - 1].amc[freq].failnum = 0;
				rtk->ssat[sat - 1].amc[freq].ambcount++;
			}
			else {
				//if(AmbTestFail)
				//rtk->ssat[sat - 1].slip[freq] = 1;
				rtk->ssat[sat - 1].amc[freq].failnum++;
				if (rtk->ssat[sat - 1].amc[freq].failnum > 3)   //连续5个不合格，视为参考模糊度选择错误
				{
					rtk->ssat[sat - 1].amc[freq].failnum = 0;
					rtk->ssat[sat - 1].amc[freq].ambcount = 1;
					rtk->ssat[sat - 1].amc[freq].refsat = refsat;
					rtk->ssat[sat - 1].amc[freq].refamb = b[i];
				}
				rtk->ssat[sat - 1].fix[freq] = 0;
			}
		}
		else {                                                                              //当参考星发生改变时或者发生周跳时重新存储模糊度
			rtk->ssat[sat - 1].amc[freq].failnum = 0;
			rtk->ssat[sat - 1].amc[freq].ambcount = 1;
			rtk->ssat[sat - 1].amc[freq].refsat = refsat;
			rtk->ssat[sat - 1].amc[freq].refamb = b[i];
			rtk->ssat[sat - 1].fix[freq] = 0;
		}

		/* Init refsat */
		rtk->ssat[refsat - 1].amc[freq].ambcount = 0;
		rtk->ssat[refsat - 1].amc[freq].amb = 0.0;
		rtk->ssat[refsat - 1].amc[freq].refsat = refsat;
	}
	if (nf < *nb) {
		if (nf < 5) {
			trace(4, "AMB_Test failed: Num=%2d\n", nf);
			*fixflag = 0;
			return 0;
		}
		else {
			*fixflag = 2;
			trace(4, "AMB_Test success: Num=%2d fixflag=%2d\n", nf, *fixflag);
			*nb = nf;
			return 2;
		}
	}

	trace(4, "AMB_Test success: Num=%2d fixflag=%2d\n", nf, *fixflag);
	return *fixflag;
}
#endif

int amblockpar(rtk_t* rtk, int* nb, int na, double* aa, double* qa,
	double* qy, double* py, double* pqy, double* pb, double* s, int* flag, int* locknum, int* index)
{
	int ofo = -1, i = 0, j = 0, k = 0, l = 0, num = 0, temp = 0, n = *nb;
	double* qfaa = NULL;
	double faa[MAXDIFOBS * NFREQ] = { 0.0 };

	int sortam[MAXDIFOBS * NFREQ] = { 0 };
	if (n <= 0)
		return -1;
	for (i = 0; i < n; i++)
		sortam[i] = i;
	orderlock(n, locknum, sortam);
	trace(5, "order amblocpar:");
	for (i = 0; i < n; i++)
		trace(5, "%d,", sortam[i]);
	trace(5, "\r\n");

	qfaa = mat(n, n);

	for (k = 1; k < MAXTRYNUM; k++) {
		if ((n - k) < MINPARNUM) {
			ofo = -1;  break;
		}
		num = n - k;

		memset(faa, 0, sizeof(double) * n);
		memset(qfaa, 0, sizeof(double) * n * n);

		memset(pb, 0, sizeof(double) * n * 2);
		memset(s, 0, sizeof(double) * 2);
		if (k > 0)
			*flag = 0;
		rebd(aa, qa, sortam, faa, qfaa, k, n);
		rebdy(qy, sortam, pqy, k, n, na);

		ofo = lambda(num, 2, faa, qfaa, pb, s);
		rtk->sol.ratio = (float)(s[1] / s[0]);
		if (rtk->sol.ratio < rtk->opt.thresar[0]) {
			trace(5, "amblockpar failure: ratio=%f k=%d\r\n", rtk->sol.ratio, k);
			continue;
		}

		*flag = 2;
		for (i = 0; i < num; i++)
			py[i] = faa[i];
		break;
	}

	*nb = num;
	if (ofo < 0)
		*flag = 0;
	if (*flag) {
		int sat, freq;
		for (i = 0; i < k; i++) {
			sat = (index[sortam[i]] >> 16) & 0xFF;
			freq = index[sortam[i]] & 0xF;
			j = ix(sat, rtk->satid, rtk->presat);
			rtk->ssat[j].fix[freq] = 0;
			trace(4, "amblockpar:exclude:sat=%2d\r\n", sat);
		}
	}
	xy_free(qfaa);
	return ofo;
}

int ambvarpar(rtk_t* rtk, int* nb, int na, double* aa, double* qa,
	double* qy, double* py, double* pqy, double* pb, double* s, int* flag, int* index)
{
	int ofo = -1, i = 0, j = 0, k = 0, l = 0, num = 0, temp = 0, n = *nb;

	double* qfaa = NULL;
	double faa[MAXDIFOBS * NFREQ] = { 0.0 };

	int sortam[MAXDIFOBS * NFREQ] = { 0 };

	if (n <= 0)
		return -1;
	for (i = 0; i < n; i++)
		sortam[i] = i;
	orderamb(n, qa, sortam);
	trace(5, "order ambvarpar:");
	for (i = 0; i < n; i++)
		trace(5, "%d,", sortam[i]);
	trace(5, "\r\n");
	qfaa = mat(n, n);

	for (k = 1; k < MAXTRYNUM; k++)
	{
		if ((n - k) < MINPARNUM)
		{
			ofo = -1;
			break;
		}
		num = n - k;
		memset(faa, 0, sizeof(double) * n);
		memset(qfaa, 0, sizeof(double) * n * n);

		memset(pb, 0, sizeof(double) * n * 2);
		memset(s, 0, sizeof(double) * 2);
		if (k > 0)
			*flag = 0;
		rebd(aa, qa, sortam, faa, qfaa, k, n);
		rebdy(qy, sortam, pqy, k, n, na);
		ofo = lambda(num, 2, faa, qfaa, pb, s);
		/* fix or not */
		rtk->sol.ratio = (float)(s[1] / s[0]);
		if (rtk->sol.ratio < rtk->opt.thresar[0])
		{
			trace(5, "ambvarpar failure: ratio=%f k=%d\r\n", rtk->sol.ratio, k);
			continue;
		}
		*flag = 2;
		for (i = 0; i < num; i++)
			py[i] = faa[i];
		break;
	}

	*nb = num;
	if (ofo < 0)
		*flag = 0;
	if (*flag)
	{
		int sat = 0, freq = 0;
		for (i = 0; i < k; i++)
		{
			sat = (index[sortam[i]] >> 16) & 0xFF;
			freq = index[sortam[i]] & 0xF;
			j = ix(sat, rtk->satid, rtk->presat);
			rtk->ssat[j].fix[freq] = 0;
			trace(4, "ambvarpar exclude:sat=%2d\r\n", sat);
		}
	}
	xy_free(qfaa);
	return ofo;
}

int ambrefpar(rtk_t* rtk, int* nb, int na, double* aa, double* qa,
	double* qy, double* py, double* pqy, double* b, double* pb, double* s, int* flag, double* refpar, int* index)
{
	if (rtk == NULL || nb == NULL || aa == NULL || qa == NULL || qy == NULL || py == NULL ||
		pqy == NULL || b == NULL || pb == NULL || s == NULL || flag == NULL || refpar == NULL || index == NULL) {
		return -1;
	}
	int ofo = -1, i = 0, j = 0, k = 0, l = 0, num = 0, temp = 0, n = *nb;

	double* qfaa = NULL;
	double faa[MAXDIFOBS * NFREQ] = { 0.0 };

	int sortam[MAXDIFOBS * NFREQ] = { 0 };

	if (n <= 0)
		return -1;
	for (i = 0; i < n; i++)
		sortam[i] = i;
	orderpar(n, refpar, sortam);
	trace(5, "order ambrefpar:");
	for (i = 0; i < n; i++)trace(5, "%d,", sortam[i]);
	trace(5, "\r\n");

	qfaa = mat(n, n);

	for (k = 1; k < MAXTRYNUM; k++) {
		if ((n - k) < MINPARNUM) {
			ofo = -1;  break;
		}
		num = n - k;

		memset(faa, 0, sizeof(double) * n);
		memset(qfaa, 0, sizeof(double) * n * n);

		memset(pb, 0, sizeof(double) * n * 2);
		memset(s, 0, sizeof(double) * 2);
		if (k > 0)
			*flag = 0;
		rebd(aa, qa, sortam, faa, qfaa, k, n);
		rebdy(qy, sortam, pqy, k, n, na);
		ofo = lambda(num, 2, faa, qfaa, pb, s);
		rtk->sol.ratio = (float)(s[1] / s[0]);
		if (rtk->sol.ratio < rtk->opt.thresar[0]) {
			trace(5, "ambrefpar failure: ratio=%f k=%d\r\n", rtk->sol.ratio, k);

			continue;
		}
		*flag = 2;
		for (i = 0; i < num; i++)
			py[i] = faa[i];
		for (i = 0, l = 0; i < n; i++) {
			if (l > num) {
				ofo = -1; break;
			}
			temp = 0;
			for (j = 0; j < k; j++) {
				if (i == sortam[j]) {
					temp = -1;
				}
			}
			if (temp < 0) {
				b[i] = aa[i];
				continue;
			}
			else {
				b[i] = pb[l++];
			}
		}

		break;
	}

	*nb = num;
	if (ofo < 0)
		*flag = 0;
	if (*flag) {
		int sat, freq;
		for (i = 0; i < k; i++) {
			sat = (index[sortam[i]] >> 16) & 0xFF;
			freq = index[sortam[i]] & 0xF;
			j = ix(sat, rtk->satid, rtk->presat);
			rtk->ssat[j].fix[freq] = 0;
			trace(4, "ambrefpar:exclude:sat=%2d\r\n", sat);
		}
	}
	xy_free(qfaa);
	return ofo;
}

/* let the maximum number of continuous tracking satellite as reference star  记录卫星的最长锁定时长--*/
int checkmaxsat(rtk_t* rtk, int sys, int f)
{
	if (rtk == NULL) return -1;
	int i = 0, na = rtk->na, maxlock = MINLOCK;
	
	for (i = 0; i < rtk->nsat; i++)
	{
		// TODO 这里指定了最小参考星高度角为25 MINREFSATEL
		if (rtk->x[IB2(i, f, &rtk->opt, rtk->nsat)] == 0.0 || !test_sys(rtk->ssat[i].sys, sys) || (rtk->ssat[i].lock[f] < MINLOCK) ||
			rtk->ssat[i].vsat[f] != 1 || rtk->ssat[i].azel[1] < MINREFSATEL * D2R || (rtk->ssat[i].slip[f] & 2))//TODO12W
		{
			continue;
		}

		if (sys == SYS_CMP && limitcmpgeo(rtk->ssat[i].sat) <= 0)
			continue;
		if (rtk->ssat[i].lock[f] > maxlock) {
			maxlock = rtk->ssat[i].lock[f];
		}
	}
	return maxlock;
}

/* select n satellites of highest elevation 选择最高高度角的三颗卫星*/
int checkmaxelsat(rtk_t* rtk, int sys, int k, int f, int* order) {
	if (rtk == NULL || order == NULL) return -1;
	int i = 0, j = 0, na = rtk->na, maxel = 0, n = 0, temp_i = 0;
	int sort[MAXDIFOBS] = { 0 };
	double el[MAXDIFOBS] = { 0.0 }, temp_d = 0.0;

	for (i = 0; i < rtk->nsat; i++) {
		//TODO 12
		if (fabs(rtk->x[IB2(i, f, &rtk->opt, rtk->nsat)]) < 1e-6 || !test_sys(rtk->ssat[i].sys, sys) || (rtk->ssat[i].lock[f] < MINLOCK) ||
			rtk->ssat[i].vsat[f] != 1 || rtk->ssat[i].azel[1] < MINREFSATEL * D2R || (rtk->ssat[i].slip[f] & 2) || (rtk->ssat[i].slip[f] & 1)) {
			continue;
		}

		if (sys == SYS_CMP && limitcmpgeo(rtk->ssat[i].sat) <= 0) continue;

		el[n] = rtk->ssat[i].azel[1];
		sort[n++] = i;
	}
	for (i = 0; i < n; i++) {//高度角由高到低排序
		for (j = i + 1; j < n; j++) {
			if (el[i] < el[j]) {
				temp_i = sort[i];
				temp_d = el[i];
				sort[i] = sort[j];
				el[i] = el[j];
				sort[j] = temp_i;
				el[j] = temp_d;
			}
		}
	}
	if (n >= 3){//记录k个高高度角卫星号    if (n >= 1)//TODO
		for (i = 0; i < k; i++) {
			order[i] = sort[i];
		}
	}else {
		return -1;
	}
	return 1;
}


int ddmat(rtk_t* rtk, double* D, int* locknum, int* index, double* snr, double* el)
{
	if (rtk == NULL || D == NULL || locknum == NULL || index == NULL || snr == NULL || el == NULL) return -1;
	int i = 0, j = 0, k = 0, m = 0, f = 0, nb = 0, nx = rtk->nx, na = rtk->na, nf = NF(&rtk->opt);
	int num = 0, kk = 0, maxlock = 0, order[3] = { 0 }, ordernum = 3, refstar = 0;
	double maxel = 0.0;
	for (i = 0; i < MAXDIFOBS; i++) {
		for (j = 0; j < NFREQ; j++) {
			rtk->ssat[i].fix[j] = 0;
			rtk->ssat[i].ref[j] = 0;
		}
	}
	for (i = 0; i < na; i++) D[i + i * nx] = 1.0;

	for (m = 0; m < 4; m++) {/* m=0:gps/qzs/sbs,1:glo,2:gal,3:bds */
		if ((m == 1 && rtk->opt.glomodear == 0) || (m == 3 && rtk->opt.bdsmodear == 0)) continue;
		for (f = 0, k = na; f < nf; f++, k += rtk->nsat) {
			refstar = -1;
			for (num = 0; num < ordernum; num++) order[num] = 0; //-1 TODO
			
			maxlock = checkmaxsat(rtk, m, f);//记录最长锁定时长:让锁定时间最长的星星作为参考星 怎么返回的不是那颗星星呢（可能是不太方便吧）
			if (checkmaxelsat(rtk, m, ordernum, f, order) > 0){//选择最高高度角的三颗卫星
				for (kk = 0; kk < ordernum; kk++) {//查看高度角最高的3颗星里是否有锁定时间最长的
					if (!test_sys(rtk->ssat[order[kk]].sys, m) || !rtk->ssat[order[kk]].vsat[f]) continue;

					if (rtk->ssat[order[kk]].lock[f] >= maxlock) {
						rtk->ssat[order[kk]].outc[f] = 0;
						refstar = order[kk];
						break;
					}
				}
			}else {
				trace(4, "first select refstar failed\r\n");
				//continue;
			}

			if (refstar + 1) {//高度角最高的3颗星里是有锁定时间最长的星就选取成功
				rtk->ssat[refstar].fix[f] = 2;
				rtk->ssat[refstar].ref[f] = 1;
				trace(4, "ddmat::ddmat successful select refstar:star[%2d]'s f=%2d m=%2d x=%12.6f P=%12.6f azel=%6.3f -reference -refstar \n", rtk->satid[refstar], f, m, rtk->x[refstar + k],rtk->P[refstar + k + (refstar + k) * nx], rtk->ssat[refstar].azel[1] * R2D);
				i = refstar + k;
			}else { //未找到符合条件的参考星则优先选取最大高度角的
				maxel = 0; i = -1;
				for (j = k; j < k + rtk->nsat; j++) {
					/*系统、vsat、lock、周跳等关键信息筛选*/
					if (rtk->x[j] == 0.0 || !test_sys(rtk->ssat[j - k].sys, m) || !rtk->ssat[j - k].vsat[f] || limitcmpgeo(rtk->ssat[j - k].sat) <= 0) continue;
					if (rtk->ssat[j - k].lock[f] >= MINLOCK && !(rtk->ssat[j - k].slip[f] & 2) && rtk->ssat[j - k].azel[1] >= MINREFSATEL * D2R){//lock>=maxlock<-lock>0 改变选参考星的策略 不能有周跳。应该不会选上吧？
						/*找最大高度角的星作为参考星，记为i*/
						if (rtk->ssat[j - k].azel[1] > maxel){
							maxel = rtk->ssat[j - k].azel[1];
							i = j;
							continue;
						}
					}
				}
				if ((i - k) >= 0) {
					rtk->ssat[i - k].outc[f] = 0;
					rtk->ssat[i - k].fix[f] = 2;
					rtk->ssat[i - k].ref[f] = 1;

					trace(4, "ddmat:star[%2d]'s x=%12.6f P=%12.6f  -reference \n", rtk->satid[i - k], rtk->x[i],rtk->P[i + i * nx]);
					trace(4, "by other  select refstar equals \r\n");
				}else {
					//rtk->ssat[i - k].fix[f] = 1; //(1:fix, 2 : float, 3 : hold)
					i = k + rtk->nsat;    
					trace(4, "no select refstar equals \r\n");
				}
			}

			if (i == k + rtk->nsat) continue;
			//记录模糊度差分星的锁定时长locknum、高度角el
			for (j = k; j < k + rtk->nsat; j++) {
				if (i == j || rtk->x[j] == 0.0 || !test_sys(rtk->ssat[j - k].sys, m) || !rtk->ssat[j - k].vsat[f] /*||rtk->P[j+j*nx]<=0.0 ||rtk->P[j+j*nx]>1*/) {
					continue;
				}
				if (rtk->ssat[j - k].lock[f] >= MINLOCK && !(rtk->ssat[j - k].slip[f] & 2) && rtk->ssat[j - k].vsat[f] && rtk->ssat[j - k].azel[1] >= MINREFSATEL * D2R /*rtk->opt.elmaskar*/) {
					D[i + (na + nb) * nx] = 1.0; //********重点*********
					D[j + (na + nb) * nx] = -1.0;
					locknum[nb] = rtk->ssat[j - k].lock[f];//记录每颗卫星的锁定时长
					snr[nb] = rtk->ssat[j - k].snr[f] * 0.25;
					el[nb] = rtk->ssat[j - k].azel[1] * R2D;

					index[nb] = ((rtk->satid[i - k]) << 8) | (rtk->satid[j - k] << 16) | (0 << 4) | (f % nf);//记录一下双差模糊度参考星和非参考星的下标
					nb++;
					rtk->ssat[j - k].fix[f] = 2;   //参与固定的卫星设置2
					rtk->ssat[j - k].ref[f] = 2;

					trace(4, "ddmat:star[%2d]'s f=%2d opt=%2d x=%12.6f P=%12.6f\n", rtk->satid[j - k], f, j, rtk->x[j],rtk->P[j + j * nx]);
					//trace->trace(3,"NR=%3d NP=%3d NI=%3d NT=%3d NL=%3d\n",NR(&rtk->opt),NP(&rtk->opt),NI(&rtk->opt),NT(&rtk->opt),NL(&rtk->opt));
				}else {
					rtk->ssat[j - k].fix[f] = 0;///还有啥是else的呢
					//trace(4, "no ddmat\n");
				}
			}
		}
	}
	return nb;
}

/* storage history ambiguity */
#if 0
int ambstore(rtk_t* rtk, double* b, int* index, int nb)
{
	if (rtk == NULL || b == NULL || index == NULL) {
		return -1;
	}
	int i = 0, j = 0, ref = 0, sat = 0, refsat = 0, freq = 0;
	double ddamb = 0.0;
	//trace(3, "AmbStore:\n");

	for (i = 0; i < nb; i++) {
		refsat = (index[i] >> 8) & 0xFF; sat = (index[i] >> 16) & 0xFF;
		freq = index[i] & 0xF;
		j = ix(sat, rtk->satid, rtk->nsat);
		ref = ix(refsat, rtk->satid, rtk->nsat);

		if (rtk->ssat[j].fix[freq] != 2)
			continue;
		if (refsat == rtk->ssat[j].amc[freq].refsat)
			ddamb = rtk->ssat[j].amc[freq].amb;
		else if ((refsat != rtk->ssat[j].amc[freq].refsat) && (rtk->ssat[j].fix[freq] == 2) && (rtk->ssat[ref].fix[freq] == 2))
			/* ddamb refsat convert */
			ddamb = rtk->ssat[j].amc[freq].amb - rtk->ssat[ref].amc[freq].amb;
		else
			continue;

		if (fabs(b[i] - ddamb) < MIN_INT) {
			rtk->ssat[j].amc[freq].count++;
			rtk->ssat[j].amc[freq].amb = b[i];
			rtk->ssat[j].amc[freq].refsat = refsat;
			//trace(4, "sat L%1d: %2d-%2d StoreAmb: %12.3lf %12.3lf refsat=%6d sat=%6d flag=%6d\n",
			//	freq + 1, refsat, sat, b[i], ddamb, rtk->ssat[refsat - 1].amc[freq].count,
			//	rtk->ssat[sat - 1].amc[freq].count, rtk->ssat[refsat - 1].fix[freq]);
		}
		else {
			rtk->ssat[j].amc[freq].count = 1;
			rtk->ssat[j].amc[freq].amb = b[i];
			rtk->ssat[j].amc[freq].refsat = refsat;
		}
		/* Init refsat */
		rtk->ssat[ref].amc[freq].count = 1;
		rtk->ssat[ref].amc[freq].amb = 0.0;
		rtk->ssat[ref].amc[freq].refsat = refsat;
	}

	return 0;
}
#endif

int sdambtrans(rtk_t* rtk, double* b, int* index, int nb, int* fb, int* nf, int* fixflag)
{
	if (rtk == NULL || b == NULL || index == NULL || fb == NULL || nf == NULL || fixflag == NULL) {
		return -1;
	}
	int i = 0, j = 0, ref = 0, sat = 0, refsat = 0, freq = 0;
	double ddamb = 0.0;
	*nf = 0;
	/* Store fix amb and count++ */
	if (*fixflag) {
		//AmbStore(rtk, b, index, nb);
		return 1;
	}
	return 0;
}

int ambtrans(rtk_t* rtk, double* b, int* index, int nb, int* fb, int* nf, int* fixflag, int mode)
{
	if (rtk == NULL || b == NULL || index == NULL || fb == NULL || nf == NULL || fixflag == NULL) {
		return -1;
	}
	int transflag = 0;
	switch (mode) {
	case 0:
		transflag = sdambtrans(rtk, b, index, nb, fb, nf, fixflag);
		break;
	default:
		transflag = 0;
		break;
	}
	return transflag;
}

int parinit(int n, int ny, double* py, double* pqy, double* pb)
{
	if (py == NULL || pqy == NULL || pb == NULL) {
		return -1;
	}
	int nb = 0;
	nb = n;
	memset(py, 0, sizeof(double) * ny);
	memset(pqy, 0, sizeof(double) * ny * ny);
	memset(pb, 0, sizeof(double) * nb * 2);
	return nb;
}

/* restore single-differenced ambiguity --------------------------------------*/
void restamb(rtk_t* rtk, const double* bias, double* xa)
{
	if (rtk == NULL || bias == NULL || xa == NULL) {
		return;
	}
	int i = 0, n = 0, m = 0, f = 0, index[MAXDIFOBS] = { 0 }, nv = 0, nf = NF(&rtk->opt);
	int ref = 0;
	int sat[MAXDIFOBS] = { 0 }, base = 0;
	char tstr[32] = { 0 };
	int nsat[4] = { 0 };

	time2str(rtk->time, tstr, 2);
	trace(3, "restamb :\r\n");

	for (i = 0; i < rtk->na; i++)
		xa[i] = rtk->xa[i];

	for (m = 0; m < 4; m++)
		for (f = 0; f < nf; f++) {
			ref = 0;
			for (n = 1, i = 0; i < rtk->nsat; i++) {
				if (!test_sys(rtk->ssat[i].sys, m) || rtk->ssat[i].fix[f] != 2) {
					continue;
				}

				if (rtk->ssat[i].ref[f] == 1) {
					index[0] = IB2(i, f, &rtk->opt, rtk->nsat);
					ref = 1;
					sat[0] = rtk->satid[i];
					base = sat[0];
				}
				else if (rtk->ssat[i].ref[f] == 2) {
					sat[n] = rtk->satid[i];

					index[n++] = IB2(i, f, &rtk->opt, rtk->nsat);
				}
				else
					continue;
			}
			if (n < 2 || !ref)
				continue;

			xa[index[0]] = rtk->x[index[0]];//参考星？

			for (i = 1; i < n; i++) {
				trace(3, "restamb:%s index=%4d nv=%2d sat %3d - %3d: f=%2d xa=%10.3f xa-bias=%10.3f bias=%10.3f\r\n", tstr, index[i], nv, base, sat[i], f, xa[index[0]], xa[index[0]] - bias[nv], bias[nv]);
				xa[index[i]] = xa[index[0]] - bias[nv++];
				//trace->trace(4, "restamb:sat[%d]=%10f bias[%d]=%6.3lf\n", index[i] - rtk->na + 1, xa[index[i]], nv - 1, bias[nv - 1]);
			}
			//if(f==0)  //以L1统计参与固定解算的卫星数 //改成以频点数统计最终输出的卫星数
			nsat[m] += n;
		}
	rtk->sol_rtk.ns = nsat[0] + nsat[1] + nsat[2] + nsat[3];
}

int fixsolution(int nb, rtk_t* rtk, double* bias, double* b, double* y, double* qb, double* qab, double* xa)
{
	if (rtk == NULL || bias == NULL || b == NULL || y == NULL || qb == NULL || qab == NULL || xa == NULL) return -1;
	int i = 0, j = 0, na = rtk->na, nx = rtk->nx;
	double* db = NULL, * qq = NULL;
	db = mat(nb, 1);
	qq = mat(na, nb);
	for (i = 0; i < na; i++) {
		rtk->xa[i] = rtk->x[i];
		for (j = 0; j < na; j++) rtk->pa[i + j * na] = rtk->P[i + j * nx];
	}
	for (i = 0; i < nb; i++) {
		bias[i] = b[i];
		y[na + i] -= b[i];
	}

	if (!matinv(qb, nb)) {//得到固定解
		matmul("NN", nb, 1, nb, 1.0, qb, y + na, 0.0, db);    // db=inv(Qb)*(y-b).
		matmul("NN", na, 1, nb, -1.0, qab, db, 1.0, rtk->xa); // xa-=Qab*Qb\(y-b).
		//covariance of fixed solution (Qa=Qa-Qab*Qb^-1*Qab')
		matmul("NN", na, nb, nb, 1.0, qab, qb, 0.0, qq);
		matmul("NT", na, na, nb, -1.0, qq, qab, 1.0, rtk->pa);
		//restore single-differenced ambiguity
		restamb(rtk, bias, xa);
	}
	else
		nb = 0;

	xy_free(db); xy_free(qq);
	return nb;
}

void submat(double* A, const double* B, int brow, int bcol, int srow,
	int scol, int arow, int acol)
{
	if (A == NULL || B == NULL) {
		return;
	}
	int i = 0, j = 0;
	if (srow >= brow || scol >= bcol)
		return;
	if (srow + arow > brow)
		arow = brow - srow;
	if (scol + acol > bcol)
		acol = bcol - scol;

	for (i = 0; i < acol; i++) {
		for (j = 0; j < arow; j++)
			A[i * arow + j] = B[(scol + i) * brow + srow + j];
	}
}

int ambdynamicratio(rtk_t* rtk, int* nb, int* locknum, int* index, double* el, double* y, double* qy, double* qb, double* b, double* py, double* pqy, double* pb, int* flag)
{
	int i = 0;
	int n = (*nb);
	int na = rtk->na;
	int ny = na + (*nb);
	int ofo = 0;
	double s[2] = { 0 };
	int ambdiff[MAXOBSNF] = { 0 };

	for (i = 0; i < (*nb); i++) {
		ambdiff[i] = (int)(-fabs(b[i] - b[(*nb) + i]));//
	}

	(*nb) = parinit(n, ny, py, pqy, pb); //初始化
	ofo = amblockpar_new(rtk, nb, na, y + na, qb, qy, py + na, pqy, pb, s, flag, locknum, index);

	if (ofo || (*flag) != 2) {
		(*nb) = parinit(n, ny, py, pqy, pb);
		ofo = ambvarpar_new(rtk, nb, na, y + na, qb, qy, py + na, pqy, pb, s, flag, index);
	}

#if 0
	if (ofo || (*flag) != 2) {
		(*nb) = parinit(n, ny, py, pqy, pb);
		ofo = ambrefpar_new(rtk, nb, na, y + na, qb, qy, py + na, pqy, pb, s, flag, el, index);
	}
#endif
#if 0
	if (ofo || (*flag) != 2) {
		(*nb) = parinit(n, ny, py, pqy, pb);
		ofo = amblockpar_new(rtk, nb, na, y + na, qb, qy, py + na, pqy, pb, s, flag, ambdiff, index);
	}
#endif

	if ((*flag) == 2) {
		if ((*flag) == 2) rtk->sol.ratio = s[0] > 0 ? (s[1] / s[0]) : 0.0;

		if (rtk->sol.ratio > 999.9f)
			rtk->sol.ratio = 999.9f;

		trace(4, "N(1)="); tracemat(4, pb, 1, (*nb), 10, 3);
		trace(4, "N(2)="); tracemat(4, pb + (*nb), 1, (*nb), 10, 3);
		trace(2, "amb par successful,ratio= %f!\n", rtk->sol.ratio);
		trace(2, "ratio=%6.3lf s[0]=%6.3lf s[1]=%6.3lf\r\n", rtk->sol.ratio, s[0], s[1]);
	}
	if ((*flag) != 2)
		trace(2, " par amb  fail \r\n ");

	return ofo;
}

int ambstaticratio(rtk_t* rtk, int* nb, int* locknum, int* index, double* el, double* y, double* qy, double* qb, double* b, double* py, double* pqy, double* pb, int* flag)
{
	int i = 0;
	int n = (*nb);
	int na = rtk->na;
	int ny = na + (*nb);
	int ofo = 0;
	double s[2] = { 0 };
	int ambdiff[MAXOBSNF] = { 0 };

	if (!(*flag)) {
		for (i = 0; i < (*nb); i++) {
			ambdiff[i] = (int)(-fabs(b[i] - b[(*nb) + i]));//
		}
	}

	(*nb) = parinit(n, ny, py, pqy, pb); //
	ofo = amblockpar(rtk, nb, na, y + na, qb, qy, py + na, pqy, pb, s, flag, locknum, index);

	if (ofo || (*flag) != 2) {
		(*nb) = parinit(n, ny, py, pqy, pb);
		ofo = ambvarpar(rtk, nb, na, y + na, qb, qy, py + na, pqy, pb, s, flag, index);
	}

#if 0
	if (ofo || (*flag) != 2) {
		(*nb) = parinit(n, ny, py, pqy, pb);
		ofo = ambrefpar(rtk, nb, na, y + na, qb, qy, py + na, pqy, pb, s, flag, el, index);
	}

	if (ofo || (*flag) != 2) {
		(*nb) = parinit(n, ny, py, pqy, pb);
		ofo = amblockpar(rtk, nb, na, y + na, qb, qy, py + na, pqy, pb, s, flag, ambdiff, index);
	}
#endif

	if ((*flag) == 2) {
		if ((*flag) == 2) rtk->sol.ratio = s[0] > 0 ? (s[1] / s[0]) : 0.0;
		trace(4, "N(1)="); tracemat(4, pb, 1, (*nb), 10, 3);
		trace(4, "N(2)="); tracemat(4, pb + (*nb), 1, (*nb), 10, 3);
		trace(2, "par amb successful!\n");
		trace(2, "ratio=%6.3lf s[0]=%6.3lf s[1]=%6.3lf\r\n", rtk->sol.ratio, s[0], s[1]);
	}
	if ((*flag) != 2)
		trace(2, " par amb  fail \r\n ");

	return ofo;
}

int ambfixdyratioall(rtk_t* rtk, int nb, int ny, double* y, double* qy, double* b, double* qb, double* bias, double* xa, int* flag)
{
	int i = 0, j = 0;
	int na = rtk->na;
	int ofo = 0;
	double mu = 0.0;
	double* qab = NULL;
	double s[2] = { 0.0 };

	/*ratio动态b为最优解与次优解*/
	ofo = ffrtlam(nb, 2, y + na, qb, b, s, rtk->opt.thresar[0], &mu, flag, rtk->opt);

	if ((*flag) == 1)//通过了
	{
		rtk->sol.ratio = s[0] > 0 ? (s[1] / s[0]) : 0.0;
		if (rtk->sol.ratio > 999.9f) rtk->sol.ratio = 999.9f;
		trace(2, " successful  ratio= %f!\r\n", rtk->sol.ratio);

		qab = mat(na, nb);
		for (i = 0; i < na; i++)
			for (j = 0; j < nb; j++)
				qab[i + j * na] = qy[i + (na + j) * ny];

		nb = fixsolution(nb, rtk, bias, b, y, qb, qab, xa);//重新计算单差相位偏移储存至xa中
		xy_free(qab);
		return ofo;
	}
	if ((*flag) != 1) trace(2, "full amb  fail \r\n");
	trace(4, "N(1)="); tracemat(4, b, 1, nb, 10, 3);
	trace(4, "N(2)="); tracemat(4, b + nb, 1, nb, 10, 3);

	return ofo;
}

int ambfixstratioall(rtk_t* rtk, int nb, int ny, double* y, double* qy, double* b, double* qb, double* bias, double* xa, int* flag)
{
	int i, j;
	int ofo = 0;
	int na = rtk->na;
	double s[2] = { 0.0 };
	double* qab = NULL;

	ofo = lambda(nb, 2, y + na, qb, b, s); 
	if (!ofo)//
	{
		rtk->sol.ratio = s[0] > 0 ? (s[1] / s[0]) : 0.0;
		if (rtk->sol.ratio > 999.9) rtk->sol.ratio = 999.9;
		if (rtk->sol.ratio > rtk->opt.thresar[0])
			(*flag) = 1;//

		if ((*flag) == 1)
		{
			trace(3, " successful  ratio= %f!\r\n", rtk->sol.ratio);

			qab = zeros(na, nb);
			for (i = 0; i < na; i++)
				for (j = 0; j < nb; j++)
					qab[i + j * na] = qy[i + (na + j) * ny];

			nb = fixsolution(nb, rtk, bias, b, y, qb, qab, xa);
			xy_free(qab);
			return ofo;
		}
		if ((*flag) != 1)
			trace(2, "full amb  fail \r\n");
		trace(4, "N(1)="); tracemat(4, b, 1, nb, 10, 3);
		trace(4, "N(2)="); tracemat(4, b + nb, 1, nb, 10, 3);
	}
	return ofo;
}

int ambfixdyratiopar(rtk_t* rtk, int* nb, int* locknum, int* index, double* el, double* y, double* qy, double* qb, double* b, double* bias, double* xa, int* flag)
{
	int ofo = 0;
	int na = rtk->na;
	int ny = (*nb) + na;

	double pb[MAXOBSNF * 2] = { 0 }, py[MAXOBSNFA] = { 0 };
	double* pqy = NULL;
	double* pqb = NULL, * pqab = NULL;

	pqy = zeros(ny, ny);
	//部分模糊度搜索
	ofo = ambdynamicratio(rtk, nb, locknum, index, el, y, qy, qb, b, py, pqy, pb, flag);

	//部分模糊度解算
	if ((*flag) == 2)
	{
		pqb = zeros((*nb), (*nb));
		pqab = zeros(na, (*nb));

		submat(pqb, pqy, na + (*nb), na + (*nb), na, na, *nb, (*nb));
		submat(pqab, pqy, na + (*nb), na + (*nb), 0, na, na, (*nb));

		*nb = fixsolution((*nb), rtk, bias, pb, py, pqb, pqab, xa);
		xy_free(pqb); xy_free(pqab);
	}
	else {
		*nb = 0;
	}

	xy_free(pqy);
	return (*nb);
}

int ambfixstratiopar(rtk_t* rtk, int* nb, int* locknum, int* index, double* el, double* y, double* qy, double* qb, double* b, double* bias, double* xa, int* flag)
{
	int ofo = 0;
	int na = rtk->na;
	int ny = (*nb) + na;

	double pb[MAXOBSNFA * 2] = { 0 }, py[MAXOBSNFA] = { 0 };
	double* pqy = NULL;
	double* pqb = NULL, * pqab = NULL;

	pqy = zeros(ny, ny);
	//部分模糊度搜索
	ofo = ambstaticratio(rtk, nb, locknum, index, el, y, qy, qb, b, py, pqy, pb, flag);

	//部分模糊度解算
	if ((*flag) == 2)
	{
		pqb = zeros((*nb), (*nb));
		pqab = zeros(na, (*nb));

		submat(pqb, pqy, na + (*nb), na + (*nb), na, na, *nb, (*nb));
		submat(pqab, pqy, na + (*nb), na + (*nb), 0, na, na, (*nb));

		*nb = fixsolution((*nb), rtk, bias, pb, py, pqb, pqab, xa);
		xy_free(pqb); xy_free(pqab);
	}
	else {
		*nb = 0;
	}

	xy_free(pqy);
	return (*nb);
}

extern int resamb(rtk_t* rtk, double* bias, double* xa)
{
	if (rtk == NULL || bias == NULL || xa == NULL) return -1;
	int i = 0, j = 0, ny = 0, nb = 0, n = 0, nx = rtk->nx, na = rtk->na;           /* number of float states/fixed states */
	int flag = 0;
	double qr = 0.0;
	double* d = NULL, * dp = NULL, * qy = NULL, * qb = NULL;
	double b[MAXOBSNF * 2] = { 0 };//b为最优解与次优解
	double y[MAXOBSNFA] = { 0 };
	int locknum[MAXOBSNF] = { 0 }, index[MAXOBSNF] = { 0 };
	double snr[MAXOBSNF] = { 0.0 }, el[MAXOBSNF] = { 0.0 };
	rtk->sol.ratio = 0.0;
	trace(3, "RTK::resamb : nx=%d  na=%d\r\n", nx, na);

	/*调用ddmat函数，创建将卡尔曼状态量从单差转到双差的转换矩阵D’,主要是将单差相位偏移状态量转换为双差相位偏移,这里的D’阵实际就是RTKlib manual 165页中的G阵。*/
	d = zeros(nx, nx);
	if ((nb = ddmat(rtk, d, locknum, index, snr, el)) < MINPARNUM) {
		trace(4, "no valid double-difference\r\n");
		xy_free(d);
		return 0;
	}
	trace(2, "ddmat success :nb=%4d\r\n", nb);
	/* transform single to double-differenced phase-bias (y=D'*x, Qy=D'*P*D) */
	n = nb;
	ny = na + nb;//y模糊度单差转双差后的浮点解ny*1   Qy相应的方差阵
	dp = mat(ny, nx); qy = mat(ny, ny);//将模糊度单差转双差，生成双差后的浮点解y与相应的方差阵Qy
	matmul("TN", ny, 1, nx, 1.0, d, rtk->x, 0.0, y);  //是双差模糊度浮点解
	matmul_new(6, "TN", ny, nx, nx, 1.0, d, rtk->P, 0.0, dp);
	matmul_new(5, "NN", ny, ny, nx, 1.0, dp, d, 0.0, qy);  //Qy是双差协方差矩阵
#if 1
	/*速度约束:qy双差协方差矩阵*/
	trace(5, "qy=\r\n");
	tracemat(5, qy, ny, ny, 16, 9);
	for (i = 0; i < ny; i++){
		if (i < na) continue;
		if (qy[i + i * ny] <= 0.0) trace(4, "ddmat erro sat:i=%2d\r\n", i);
		qr += qy[i + i * ny] * qy[i + i * ny];
	}
	qr = sqrt(qr) / (ny - na);
	trace(2, "ddmat:Qr=%9.3f\n", qr);
	xy_free(d); xy_free(dp);
	
	if (rtk->vel < 0.5 && qr>0.25) { //动态固定错误不大明显，静态固定错误较为明显，故此处做动静态区分
		xy_free(qy);
		return 0;
	}
#else
	xy_free(d); xy_free(dp);
#endif
	qb = zeros(nb, nb);
	/* phase-bias covariance (Qb) and real-parameters to bias covariance (Qab) */
	for (i = 0; i < nb; i++)
		for (j = 0; j < nb; j++)
			qb[i + j * nb] = qy[na + i + (na + j) * ny];
	/*模糊度固定*/
	trace(2, "ratio maode %d\r\n", rtk->ratiomodel);
	if (rtk->ratiomodel == 0) {
		//动态ratio整周模糊度固定及解算
		ambfixdyratioall(rtk, nb, ny, y, qy, b, qb, bias, xa, &flag);
		if (flag == 1)  {
			xy_free(qy); xy_free(qb);
			return nb;
		}else {
			//动态ratio部分模糊度固定及解算
			if (nb >= MINPARNUM) ambfixdyratiopar(rtk, &nb, locknum, index, el, y, qy, qb, b, bias, xa, &flag);
			else nb = 0;
		}
	}
	else {
		//固定ratio全模糊度固定及解算
		ambfixstratioall(rtk, nb, ny, y, qy, b, qb, bias, xa, &flag);
		if (flag == 1)  {
			xy_free(qy); xy_free(qb);
			return nb;
		}else {
			//固定ratio部分模糊度固定及解算
			if (nb >= MINPARNUM) ambfixstratiopar(rtk, &nb, locknum, index, el, y, qy, qb, b, bias, xa, &flag);
			else nb = 0;
		}
	}

	xy_free(qy); xy_free(qb);
	return nb;
}

extern int manage_resamb(rtk_t* rtk, double* bias, double* xa)
{
	int nb = 0;
	int i = 0, f = 0;
	int nf = rtk->opt.nf;
	int removen = 0;
	double ratio = rtk->sol.ratio;

	nb = resamb(rtk, bias, xa);

	if (rtk->sol_last.stat != SOLQ_FIX || rtk->sol.pre_ratio < 2.0) return nb;
	if (ratio > 10|| (ratio >= (rtk->sol.pre_ratio / 2.0) && ratio >= 2)) return nb;

	for (i = 0; i < rtk->nsat; i++){
		for (f = 0; f < nf; f++){
			if (rtk->ssat[i].fix[f] != 2) continue;
			/* check for new sats */
			if (rtk->ssat[i].lock[f] == MINLOCK) {
				removen++;
			}
		}
	}

	if (removen > 0 && removen < 3)
	{
		for (i = 0; i < rtk->nsat; i++){
			for (f = 0; f < nf; f++){
				if (rtk->ssat[i].fix[f] != 2) continue;
				/* check for new sats */
				if (rtk->ssat[i].lock[f] == MINLOCK) {
					trace(3, "remove sat %d:%d lock=%d\n", rtk->ssat[i].sat, f, rtk->ssat[i].lock[f]);
					rtk->ssat[i].lock[f] = -rtk->opt.minlock;  /* delay use of this sat with stagger */
					removen++;
				}
			}
		}
		nb = resamb(rtk, bias, xa);
	}

	return nb;
}

extern void holdamb_t(rtk_t* rtk, const double* xa, int* vflag, int ns)
{
	if (rtk == NULL || xa == NULL || vflag == NULL) {
		return;
	}
	double* H = NULL, * R = NULL;
	int i = 0, n = 0, m = 0, f = 0, info = 0, nb = rtk->nx - rtk->na, nv = 0, nf = NF(&rtk->opt);
	int index[MAXDIFOBS * NFREQ] = { 0 };
	double v[MAXDIFOBS * NFREQ] = { 0.0 };
	trace(3, "holdamb :\r\n");
	H = zeros(nb, rtk->nx);

	for (m = 0; m < 4; m++)
		for (f = 0; f < nf; f++) {
			for (n = 1, i = 0; i < ns; i++) {
				if (!test_sys(rtk->ssat[i].sys, m) || rtk->ssat[i].fix[f] != 2 ||
					rtk->ssat[i].azel[1] < rtk->opt.elmaskhold) {
					continue;
				}
				trace(5, "second sat=%d ref=%d\r\n", i + 1, rtk->ssat[i].ref[f]);
				if (rtk->ssat[i].ref[f] == 1) {
					index[0] = IB2(i, f, &rtk->opt, rtk->nsat);
					//trace(5,"index0=%d\r\n",index[0]);
					rtk->ssat[i].fix[f] = 3; /* hold */
				}
				else if (rtk->ssat[i].ref[f] == 2) {
					index[n++] = IB2(i, f, &rtk->opt, rtk->nsat);
					rtk->ssat[i].fix[f] = 3; /* hold */
					//trace(5,"indexhold=%d\r\n",index[n-1]);
				}
				else {
					trace(5, "index continue=%d\r\n", index[n - 1]);
					continue;
				}
			}

			/* constraint to fixed ambiguity */
			for (i = 1; i < n; i++) {
				v[nv] = (xa[index[0]] - xa[index[i]]) - (rtk->x[index[0]] - rtk->x[index[i]]);
				H[index[0] + nv * rtk->nx] = 1;//0.001;//.0;
				H[index[i] + nv * rtk->nx] = -1;//-0.001;//-1.0;
				trace(4, "ref-index=%2d  (not sat num) index2=%2d v=%f\r\n", index[0], index[i], v[nv]);
				nv++;
			}
		}

	if (nv > 0) {
		R = zeros(nv, nv);
		for (i = 0; i < nv; i++) R[i + i * nv] = VAR_HOLDAMB;

		/* update states with constraints */
		if ((info = filter_leador(rtk->x, rtk->P, H, v, R, rtk->nx, nv, 2, 5))) {
			trace(5, "filter error (info=%d)\r\n", info);
		}

		xy_free(R);
	}
	xy_free(H);
}