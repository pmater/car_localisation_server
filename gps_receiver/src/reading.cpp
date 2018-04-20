#include "gps_definition.h"


uint8_t * readGPStype(int sockfd)
{
	uint8_t start_byte[2] = {0x00, 0x00};
	uint8_t searchcoma = 0x00;
	int n = 0;
	int counter = 0;
	uint8_t *dataPointer;
	static uint8_t gps_type[SIZE_OF_GPSTYPE];

	memset(gps_type, 0, sizeof(gps_type));		
	dataPointer = &gps_type[0];

	printf("Reading GPS data type...\n");
	while((start_byte[0] != 0x47) || (start_byte[1] != 0x50))
	{
		start_byte[0] = start_byte[1];
		read(sockfd, &start_byte[1], 1);
	}	
	// GPS data type
	while(searchcoma != 0x2c)
	{
		n = read(sockfd, &searchcoma, 1);
		if(n < 0)
			printf("Failed to read gps type\n");
		if(searchcoma == 0x2c)
		{
			break;
		}
		else
		{
			memcpy((dataPointer+counter), &searchcoma, 1);
			counter++;
		}
	}
	return gps_type;
}

uint8_t * readLatitude(int sockfd)
{
	uint8_t searchcoma = 0x00;
	int n = 0;
	int counter = 0;
	uint8_t *latPointer;
	static uint8_t latitude[SIZE_OF_LATITUDE];

	memset(latitude, 0, sizeof(latitude));
	latPointer = &latitude[0];		

	while(searchcoma != 0x2c)
	{
		n = read(sockfd, &searchcoma, 1);
		if(n < 0)
			printf("Failed to read latitude data\n");

		if(searchcoma == 0x2c)
		{
			break;
		}
		else
		{
			memcpy((latPointer+counter), &searchcoma, 1);
			counter++;
		}
	}
	//printf("\nNumber %d\n", counter);
	return latitude;
}

uint8_t * readUTC(int sockfd)
{
	uint8_t searchcoma = 0x00;
	int n = 0;
	int counter = 0;
	uint8_t *dataPointer;
	static uint8_t utc[SIZE_OF_UTC];

	memset(utc, 0, sizeof(utc));		
	dataPointer = &utc[0];

	//printf("Reading UTC...\n");
	while(searchcoma != 0x2c)
	{
		n = read(sockfd, &searchcoma, 1);
		if(n < 0)
			printf("Failed to read utc data\n");

		if(searchcoma == 0x2c)
		{
			break;
		}
		else
		{
			memcpy((dataPointer+counter), &searchcoma, 1);
			counter++;
		}
	}
	return utc;
}

uint8_t * readLongitude(int sockfd)
{
	uint8_t searchcoma = 0x00;
	int n = 0;
	int counter = 0;
	uint8_t *longPointer;
	static uint8_t longitude[SIZE_OF_LONGITUDE];
	memset(longitude, 0, sizeof(longitude));
	longPointer = &longitude[0];	
	
	while(searchcoma != 0x2c)
	{
		n = read(sockfd, &searchcoma, 1);
		if(n < 0)
			printf("Failed to read longitude data\n");

		if(searchcoma == 0x2c)
		{
			break;
		}
		else
		{
			memcpy((longPointer+counter), &searchcoma, 1);
			counter++;
		}
	}
	return longitude;
}

uint8_t readDirection(int sockfd)
{
	uint8_t direction = 0x00;
	uint8_t previous = 0x00;
	int n = 0;
	while(direction != 0x2c)
	{
		n = read(sockfd, &direction, 1);
		
		if(direction == 0x2c)
		{
			direction = previous;
			break;
		}
		else previous = direction;
		if(n < 0)
			printf("Failed to read direction\n");
	}
	if((direction != 0x4e)&&(direction != 0x45)&&(direction != 0x53)&&(direction != 0x57))
	{
		printf("Read wrong direction\n");
		return 0;
	}
	else
		return direction;
}

uint8_t readGPSquality(int sockfd)
{
	uint8_t quality = 0x00;
	uint8_t previous = 0x00;
	int n = 0;
	while(quality != 0x2c)
	{
		n = read(sockfd, &quality, 1);
		if(n < 0)
			printf("Failed to read direction\n");

		if(quality == 0x2c)
		{
			quality = previous;
			break;
		}
		else previous = quality;		
	}
	return quality;
}

uint8_t * readHDOP(int sockfd)
{
	static uint8_t hdop[SIZE_OF_HDOP];
	uint8_t * hdopPointer;
	uint8_t searchcoma = 0x00;
	memset(hdop, 0, sizeof(hdop));
	int n = 0, counter = 0;
	hdopPointer = &hdop[0];
	while(searchcoma != 0x2c)
	{
		n = read(sockfd, &searchcoma, 1);
		if(n < 0)
			printf("Failed to read direction\n");

		if(searchcoma == 0x2c)
		{
			break;
		}
		else 
		{
			memcpy((hdopPointer+counter), &searchcoma, 1);
			counter++;
		}		
	}
	return hdop;
}

int readnoSV(int sockfd)
{
	uint8_t no_sv[2];
	uint8_t * pointer;
	uint8_t searchcoma = 0x00;
	int counter = 0;
	memset(no_sv, 0, sizeof(no_sv));
	pointer = &no_sv[0];
	while(searchcoma != 0x2c)
	{	
		read(sockfd, &searchcoma, 1);
		if(searchcoma == 0x2c)
		{
			break;
		}
		else
		{
			memcpy((pointer+counter), &searchcoma, 1);
			counter++;
		}
	}
	int noSV = 0;
	char passing[2] = {0x00,0x00};
	passing[0] = (char)no_sv[0];
	passing[1] = (char)no_sv[1];
	noSV = atoi(passing);
	return noSV;
}

uint8_t * readHeight(int sockfd)
{
	static uint8_t height[SIZE_OF_HEIGHT];
	uint8_t searchcoma = 0x00;
	uint8_t unit[2] = {0x00,0x00};
	memset(height, 0, sizeof(height));
	int n = 0, counter = 0;
	while(searchcoma != 0x2c)
	{
		n = read(sockfd, &searchcoma, 1);
		if(n < 0)
			printf("Failed to read height\n");

		if(searchcoma == 0x2c)
		{
			break;
		}
		else 
		{
			memcpy(&height[counter], &searchcoma, 1);
			counter++;
		}		
	}
	read(sockfd, unit, sizeof(unit));
	return height;
}

uint8_t * readGeoid(int sockfd)
{
	static uint8_t geoid[SIZE_OF_GEOID];
	uint8_t searchcoma = 0x00;
	uint8_t end[4];
	memset(geoid, 0, sizeof(geoid));
	memset(end, 0, sizeof(end));
	int n = 0, counter = 0;
	while(searchcoma != 0x2c)
	{
		n = read(sockfd, &searchcoma, 1);
		if(n < 0)
			printf("Failed to read direction\n");

		if(searchcoma == 0x2c)
		{
			break;
		}
		else 
		{
			memcpy(&geoid[counter], &searchcoma, 1);
			counter++;
		}		
	}
	read(sockfd, end, sizeof(end));
	return geoid;
}

uint8_t * readTimetag(int sockfd)
{
	static uint8_t _timetag[SIZE_OF_TIMETAG];
	int n = 0;
	memset(_timetag, 0, sizeof(_timetag));
	n = read(sockfd, _timetag, SIZE_OF_TIMETAG);

	if(n < 0) printf("Failed to read time tag\n");

	return _timetag;			
}

