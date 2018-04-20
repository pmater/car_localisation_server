#include "gps_definition.h"

double convert_FloatRadian(double position_data)
{
	double degree = 0;
	double minute = 0;
	degree = floor((position_data/100));
	minute = (position_data - degree*100)/60;
	degree = ((degree + minute)*PI)/180;
	return degree;
}

double * convert_LLAtoECEF(double _lat, double _long, float _height)
{
	double e = WGS84_E;
	double a = WGS84_A;
	static double xyz[3];
	double NE = 0;

	memset(xyz, 0, sizeof(xyz));
	NE = a/(sqrt(1 - (e*e*sin(_lat)*sin(_lat)) ));
 
	xyz[0] = (NE + _height)*cos(_lat)*cos(_long);
	xyz[1] = (NE + _height)*cos(_lat)*sin(_long);
	xyz[2] = ((1 - (e*e))*NE + _height)*sin(_lat);
	return xyz;
}
