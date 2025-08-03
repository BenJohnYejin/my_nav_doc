#include "rtklib.h"
static int model_geoid = GEOID_EMBEDDED; /* geoid model */

//* geoid height ----------------------------------------------------------------
//* get geoid height from geoid model
//* args   : double *pos      I   geodetic position {lat,lon} (rad)
//* return : geoid height (m) (0.0:error)
//* notes  : to use external geoid model, call function opengeoid() to open
//*          geoid model before calling the function. If the external geoid model
//*          is not open, the function uses embedded geoid model.
//*-----------------------------------------------------------------------------*/
extern double geoidh(const double* pos)
{
	double posd[2], h;

	posd[1] = pos[1] * R2D; posd[0] = pos[0] * R2D; if (posd[1] < 0.0) posd[1] += 360.0;

	if (posd[1] < 0.0 || 360.0 - 1E-12 < posd[1] || posd[0] < -90.0 || 90.0 < posd[0]) {
		trace(2, "out of range for geoid model: lat=%.3f lon=%.3f\n", posd[0], posd[1]);
		return 0.0;
	}
	switch (model_geoid) {
# if 0
	case GEOID_EMBEDDED: h = geoidh_emb(posd); break;
	case GEOID_EGM96_M150: h = geoidh_egm96(posd); break;
	case GEOID_EGM2008_M25: h = geoidh_egm08(posd, model_geoid); break;
	case GEOID_EGM2008_M10: h = geoidh_egm08(posd, model_geoid); break;
	case GEOID_GSI2000_M15: h = geoidh_gsi(posd); break;
#endif
	default: return 0.0;
	}
}