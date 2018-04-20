#include "gps_definition.h"

struct gps_data ParseConvert_Operation(struct data_passing _data)
{	
	struct gps_data gps;

	// Convert hecadecimal data to either character or float
	double latitude = 0, longitude = 0; 
	float hdop = 0, height = 0, height_sealevel = 0;
	float geoid = 0, utc = 0;

	utc = parseUTC(_data._utc);
	gps.latitude = parseLatitude(_data._latitude);
	gps.longitude = parseLongitude(_data._longitude);
	gps.hdop = parseHDOP(_data._hdop);
	height_sealevel = parseHeight(_data._height);
	gps.geoid = parseGeoid(_data._geoid);
	gps.height = height_sealevel + geoid;

	// Convert LLA data to ECEF x y z data
	double lat_radian = 0, long_radian = 0;
	double * xyz;

	lat_radian = convert_FloatRadian(gps.latitude);
	long_radian = convert_FloatRadian(gps.longitude);
	xyz = convert_LLAtoECEF(lat_radian, long_radian, gps.height);

	gps.x = *xyz;
	gps.y = *(xyz+1);
	gps.z = *(xyz+2);
	gps.time = utc;

	return gps;		
}

