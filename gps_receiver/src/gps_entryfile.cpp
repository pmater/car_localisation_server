#include "gps_definition.h"
#include "sensor_msgs/NavSatFix.h"
#include <iostream>

using namespace std;

int main(int argc, char * argv[])
{
	ros::init(argc, argv, "gps_pub");
	ros::NodeHandle n;
	ros::Publisher gpsPosePub = n.advertise<geometry_msgs::PoseStamped>("/gps_pose", 1000);
	ros::Publisher gpsPub = n.advertise<sensor_msgs::NavSatFix>("/gps", 1000);
	ros::Rate loop_rate(1000);

	float frequency = 10;
	int sock_fd, connect_fd;
	uint8_t *timeout;
	double start_x = 0, start_y = 0, start_z = 0;
	int set_flag = 0;

	socklen_t client_len;
	fd_set toread_sock;

	sock_fd = openSocket();		
	listenSocket(sock_fd);

	cout << "Server is ready to receive data" << endl;

	// Accept and Connect to the client
	connect_fd = acceptNconnectClient(sock_fd);		
	cout << "Connection accepted" << endl;

	// Start receiving data
	while(n.ok())
	{	
		struct data_passing data;
		struct timetag dataTime;
		memset(&data, 0, sizeof(struct data_passing));
		memset(&dataTime, 0, sizeof(struct timetag));

		double xGH = 0, yGH = 0, zGH = 0;
		unsigned long timetag_second = 0, timetag_nsecond = 0;
		uint8_t *timePointer;

		// Read GPS data in hexcadecimal
		data.gpsType = readGPStype(connect_fd);
		data._utc = readUTC(connect_fd);
		data._latitude = readLatitude(connect_fd);
		data.n_s = (char)readDirection(connect_fd);	
		data._longitude = readLongitude(connect_fd);
		data.e_w = (char)readDirection(connect_fd);
		data.gps_quality = readGPSquality(connect_fd);
		data.no_of_sv = readnoSV(connect_fd);
		data._hdop = readHDOP(connect_fd);
		data._height = readHeight(connect_fd);
		data._geoid = readGeoid(connect_fd);
		timePointer = readTimetag(connect_fd);
		
		gps_data gpsData = ParseConvert_Operation(data);
		xGH = gpsData.x;
		yGH = gpsData.y;
		zGH = gpsData.z;

		timetag_second = (unsigned long)((*(timePointer) << 24) | (*(timePointer+1) << 16) | (*(timePointer+2) << 8) | (*(timePointer+3) << 0));
		timetag_nsecond = (unsigned long)((*(timePointer+4) << 24) | (*(timePointer+5) << 16) | (*(timePointer+6) << 8) | (*(timePointer+7) << 0));

		if (set_flag < 100)
		{
			cout << "Calibrating Starting point" << endl;
			start_x = xGH;
			start_y = yGH;
			start_z = zGH;
			set_flag++;
		}
		else
		{	
			xGH = xGH - start_x;
			yGH = yGH - start_y;
			zGH = zGH - start_z;
		//	printf("X:%.8f Y:%.8f Z:%.8f\n", start_x, start_y, start_z);	
			//printf("X:%.8f Y:%.8f Z:%.8f\n t:%lu s %lu ns\n", xGH, yGH, zGH, timetag_second, timetag_nsecond);
		}

		// Define published message
		geometry_msgs::PoseStamped poseMessage;
		poseMessage.header.stamp.sec = timetag_second;
		poseMessage.header.stamp.nsec = timetag_nsecond;
		poseMessage.header.frame_id = "gps_tf";		
		poseMessage.pose.position.x = xGH;
		poseMessage.pose.position.y = yGH;
		poseMessage.pose.position.z = zGH;
		poseMessage.pose.orientation.x = 0;
		poseMessage.pose.orientation.y = 0;
		poseMessage.pose.orientation.z = 0;
		poseMessage.pose.orientation.w = 0;
		gpsPosePub.publish(poseMessage);

		sensor_msgs::NavSatFix fixMessage;
		fixMessage.header.stamp.sec = timetag_second;
		fixMessage.header.stamp.nsec = timetag_nsecond;
		fixMessage.header.frame_id = "gps_tf";	
		fixMessage.latitude = gpsData.latitude;
		fixMessage.longitude = gpsData.longitude;
		fixMessage.altitude = gpsData.height;
		gpsPub.publish(fixMessage);

		ros::spinOnce();
		loop_rate.sleep();
	}
	closeSocket(sock_fd);			
	return 0;
}
