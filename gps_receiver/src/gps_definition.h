#ifndef GPS
#define GPS

#include "ros/ros.h"
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>

#define HOSTNUM 8889
#define HOSTIP "192.168.1.2"
#define PI 3.14159265
#define SIZE_OF_GPSTYPE 5
#define SIZE_OF_UTC 9
#define SIZE_OF_LATITUDE 13
#define SIZE_OF_LONGITUDE 14
#define SIZE_OF_HDOP 2
#define SIZE_OF_HEIGHT 8
#define SIZE_OF_GEOID 6
#define SIZE_OF_XYZ 3
#define SIZE_OF_TIMETAG 8

// Semi-major axis of the Earth, A
#define WGS84_A 6378137.0
// Inverse flattening of the Earth, IF
#define WGS84_IF 298.257223563
// Flattening of the Earth, F
#define WGS84_F (1/WGS84_IF)
// Eccentricity of the Earth, E
#define WGS84_E (sqrt(2*WGS84_F - WGS84_F*WGS84_F))

struct gps_data{
	double latitude;
	double longitude;
	float height;
	float hdop;
	float geoid;
	double x;
	double y;
	double z;
	float time;
};

struct data_passing{
	uint8_t *gpsType;
	uint8_t *_utc;
	uint8_t *_latitude;
	uint8_t *_longitude;
	uint8_t *_height;
	uint8_t *_hdop;
	uint8_t *_geoid;
	char n_s;
	char e_w;
	uint8_t gps_quality;
	int no_of_sv;
	uint8_t *recv_time;
};

struct timetag{
	unsigned long sec;
	unsigned long nsec;
};

// Open socket functions
int openSocket(void);
int listenSocket(int sockfd);
int acceptNconnectClient(int sockfd);

// Reading data functions
uint8_t * readUTC(int sockfd);
uint8_t * readGPStype(int sockfd);
uint8_t * readLatitude(int sockfd);
uint8_t * readLongitude(int sockfd);
uint8_t readDirection(int sockfd);
uint8_t readGPSquality(int sockfd);
uint8_t * readHDOP(int sockfd);
int readnoSV(int sockfd);
uint8_t * readHeight(int sockfd);
uint8_t * readGeoid(int sockfd);
uint8_t * readTimetag(int sockfd);

// Parsing message functions
double parseLatitude(uint8_t * latitude_);
double parseLongitude(uint8_t * longitude_);
float parseHeight(uint8_t * height_);
float parseHDOP(uint8_t * hdop_);
float parseGeoid(uint8_t * geoid_);
float parseUTC(uint8_t * utc_);
struct timetag parseTimetag(uint8_t * recv_time_);

// Converting data functions
double convert_FloatRadian(double position_data);
double * convert_LLAtoECEF(double _lat, double _long, float _height);

// Operation functions
struct gps_data ParseConvert_Operation(struct data_passing _data);

// Closing socket functions
uint8_t readMode(int sockfd);
void closeSocket(int sockfd);

// Utility function
void setTimeout(unsigned int second, unsigned int microsecond);

#endif
