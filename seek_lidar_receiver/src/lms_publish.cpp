#include "lms_definition.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "lms_pub");
	ros::NodeHandle n;
	ros::Publisher lms = n.advertise<sensor_msgs::LaserScan>("/lms_topic", 1000);
	ros::Rate loop_rate(100);
	
	int counter = 0;
	int sock_fd, connection_fd;
	uint8_t *timeout;

	socklen_t client_len;
	fd_set toread_sock;

	sock_fd= openSocket();		
	listenSocket(sock_fd);

	printf("Server is ready to receive data\n");

	// Accept and Connect to the client
	connection_fd = acceptNconnectClient(sock_fd);	
	printf("Connection accepted\n");

	while(n.ok())
	{
		uint8_t *data_pointer;
		uint8_t * time_pointer;

		struct lms_data data;
		memset(&data, 0, sizeof(struct lms_data));

		data_pointer = readSocket(connection_fd);
		time_pointer = readTimetag(connection_fd);

		data.second/*timetag_second*/ = (unsigned long)((*(time_pointer) << 24) | (*(time_pointer+1) << 16) | (*(time_pointer+2) << 8) | (*(time_pointer+3) << 0));
		data.nanosecond/*timetag_nsecond*/ = (unsigned long)((*(time_pointer+4) << 24) | (*(time_pointer+5) << 16) | (*(time_pointer+6) << 8) | (*(time_pointer+7) << 0));
		printf("Time tag %lu s %lu ns\n", data.second, data.nanosecond);

		for(int i = 0; i < NUMBER_READING; i++)
		{
			data.laser_range[i] = (float)(*(data_pointer+7+i*2) | (*(data_pointer+8+i*2) << 8))/1000;
//			printf("%f ", data->laser_range[i]);
		}

		sensor_msgs::LaserScan message;
		message.header.stamp.sec= data.second;
		message.header.stamp.nsec = data.nanosecond;
		message.header.frame_id = "lms_tf";
		message.angle_min = ANGLE_MIN;
		message.angle_max = ANGLE_MAX;
		message.angle_increment = ANGLE_INCREMENT;
		message.time_increment = TIME_INCREMENT;
		message.range_min = RANGE_MIN;
		message.range_max = RANGE_MAX;
		message.ranges.resize(NUMBER_READING);				
		for(int i = 0; i < NUMBER_READING; i++)
		{
			message.ranges[i] = data.laser_range[i];
		}
		printf("Time %lu %lu\n", data.second, data.nanosecond);
		lms.publish(message);
		ros::spinOnce();
		loop_rate.sleep();	
	               
		printf("Counter number %d\n", counter);
		counter++;
	}

	closeSocket(sock_fd);			
	return 0;
}	

