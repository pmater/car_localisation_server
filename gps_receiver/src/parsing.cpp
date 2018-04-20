#include "gps_definition.h"

float parseUTC(uint8_t * utc_)
{
	char temp_buffer[SIZE_OF_UTC];
	memset(temp_buffer, 0, sizeof(temp_buffer));
	for(int i = 0; i < SIZE_OF_UTC; i++)
	{
		temp_buffer[i] = (char)*(utc_ + i);
	}
	float utcvalue = 0;
	utcvalue = atof(temp_buffer);
	return utcvalue;
}

double parseLatitude(uint8_t * latitude_)
{
	char temp_buffer[SIZE_OF_LATITUDE];
	memset(temp_buffer, 0, sizeof(temp_buffer));
	for(int i = 0; i < SIZE_OF_LATITUDE; i++)
	{
		temp_buffer[i] = (char)*(latitude_ + i);
		//printf("%c", temp_buffer[i]);
	}
	//printf("\n");
	double latitudevalue = 0;
	latitudevalue = atof(temp_buffer);
	return latitudevalue;
}

double parseLongitude(uint8_t * longitude_)
{
	char temp_buffer[SIZE_OF_LONGITUDE];
	memset(temp_buffer, 0, sizeof(temp_buffer));
	for(int i = 0; i < SIZE_OF_LONGITUDE; i++)
	{
		temp_buffer[i] = (char)*(longitude_ + i);
	}
	double longitudevalue = 0;
	longitudevalue = atof(temp_buffer);
	return longitudevalue;
}

float parseHeight(uint8_t * height_)
{
	char temp_buffer[SIZE_OF_HEIGHT];
	memset(temp_buffer, 0, sizeof(temp_buffer));
	for(int i = 0; i < SIZE_OF_HEIGHT; i++)
	{
		temp_buffer[i] = (char)*(height_ + i);
	}
	float heightvalue = 0;
	heightvalue = atof(temp_buffer);
	return heightvalue;
}

float parseHDOP(uint8_t * hdop_)
{
	char temp_buffer[SIZE_OF_HDOP];
	memset(temp_buffer, 0, sizeof(temp_buffer));
	for(int i = 0; i < SIZE_OF_HDOP; i++)
	{
		temp_buffer[i] = (char)*(hdop_ + i);
	}
	float hdopvalue = 0;
	hdopvalue = atof(temp_buffer);
	return hdopvalue;
}

float parseGeoid(uint8_t * geoid_)
{
	char temp_buffer[SIZE_OF_GEOID];
	memset(temp_buffer, 0, sizeof(temp_buffer));
	for(int i = 0; i < SIZE_OF_GEOID; i++)
	{
		temp_buffer[i] = (char)*(geoid_ + i);
	}
	float geoidvalue = 0;
	geoidvalue = atof(temp_buffer);
	return geoidvalue;
}

struct timetag parseTimeTag(uint8_t * recv_time_)
{
	struct timetag time;
	uint8_t temp_buffer[SIZE_OF_TIMETAG];
	memset(temp_buffer, 0, sizeof(temp_buffer));
	for(int i =0; i < SIZE_OF_TIMETAG; i++)
	{
		temp_buffer[i] = *(recv_time_ + i);
	}
	time.sec = (int)((temp_buffer[0] << 24) | (temp_buffer[1] << 16) | (temp_buffer[2] << 8) | (temp_buffer[3] << 0)); 
	time.nsec = (int)((temp_buffer[4] << 24) | (temp_buffer[5] << 16) | (temp_buffer[6] << 8) | (temp_buffer[7] << 0));
	return time; 	
}

