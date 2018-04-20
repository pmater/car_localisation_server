#ifndef LASER
#define LASER

#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <sensor_msgs/LaserScan.h>
//#include <geometry_msgs/PoseStamped.h>
#include "seek_lidar_receiver/lms_message.h"
#include <math.h>

using namespace std;

#define HOSTNUM 8888
#define HOSTIP "192.168.1.2"
#define PI 3.14159265359
#define LASER_DATA_LENGTH 726
#define NUMBER_READING 361
#define FREQUENCY 37.5
#define ANGLE_INCREMENT PI/360
#define TIME_INCREMENT ((1/FREQUENCY)/NUMBER_READING) 
#define ANGLE_MAX PI
#define ANGLE_MIN 0
#define RANGE_MAX 8
#define RANGE_MIN 0
#define SIZE_OF_TIMETAG 8


struct lms_data{
	unsigned long second;
	unsigned long nanosecond;
	float laser_range[NUMBER_READING];
};

int openSocket(void);
int listenSocket(int sockfd);
int acceptNconnectClient(int sockfd);
uint8_t * readSocket(int sockfd);
uint8_t * readTimetag(int sockfd);
void closeSocket(int sockfd);

void setTimeout(unsigned int second, unsigned int microsecond);

#endif
