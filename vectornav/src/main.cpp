#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "vn/sensors.h"

using namespace std;
using namespace vn::math;
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;

void AsyncMessageReceived(void *userData, Packet &p, size_t index);

std::string frame_id;
ros::Publisher pubIMU, pubMag;

int main(int argc, char *argv[])
{

	// ROS node init
	ros::init(argc, argv, "vectornav");
	ros::NodeHandle n;
	pubIMU = n.advertise<sensor_msgs::Imu>("imu", 1000);
	pubMag = n.advertise<sensor_msgs::MagneticField>("mag", 1000);

	n.param<std::string>("frame_id", frame_id, "vectornav");

	// Serial Port Settings
	string SensorPort;
	int SensorBaudrate;

	n.param<std::string>("serial_port", SensorPort, "/dev/ttyUSB0");
	n.param<int>("serial_baud", SensorBaudrate, 115200);

	ROS_INFO("Connecting to : %s @ %d Baud", SensorPort.c_str(), SensorBaudrate);

	// Create a VnSensor object and connect to sensor
	VnSensor vs;
	vs.connect(SensorPort, SensorBaudrate);

	// Query the sensor's model number.
	string mn = vs.readModelNumber();
	ROS_INFO("Model Number: %s", mn.c_str());

	// Set Data output Freq [Hz]
	int async_output_rate;
	n.param<int>("async_output_rate", async_output_rate, 40);
	ROS_INFO_STREAM("Writing freq");
	vs.writeAsyncDataOutputFrequency(async_output_rate);

	vs.writeAsyncDataOutputType(VNQMR); //Need to specify the data type I want to receive in the packet. See types.h in the SDK.
	AsciiAsync asyncType = vs.readAsyncDataOutputType();
	ROS_INFO_STREAM("ASCII Async Type: " << asyncType);

	vs.registerAsyncPacketReceivedHandler(NULL, AsyncMessageReceived);

	ROS_INFO_STREAM("Spinning");
	ros::spin();

	// Node has been terminated
	vs.unregisterAsyncPacketReceivedHandler();
	vs.disconnect();
	return 0;
}

void AsyncMessageReceived(void *userData, Packet &p, size_t index)
{
	// Make sure we have an ASCII packet and not a binary packet.
	if (p.type() != Packet::TYPE_ASCII)
	{
		ROS_WARN_STREAM("Unexpected packet format: " << p.type());
		return;
	}

	if (p.determineAsciiAsyncType() != VNQMR)
	{
		ROS_WARN_STREAM("Unexpected packet contents: " << p.determineAsciiAsyncType());
		return;
	}

	vec4f quaternion;
	vec3f magnetic;
	vec3f acceleration;
	vec3f angularVelocity;

	p.parseVNQMR(&quaternion, &magnetic, &acceleration, &angularVelocity);

	// IMU
	sensor_msgs::Imu msgIMU;

	msgIMU.header.stamp = ros::Time::now();
	msgIMU.header.frame_id = frame_id;

	msgIMU.orientation.x = quaternion[0];
	msgIMU.orientation.y = quaternion[1];
	msgIMU.orientation.z = quaternion[2];
	msgIMU.orientation.w = quaternion[3];

	msgIMU.angular_velocity.x = angularVelocity[0];
	msgIMU.angular_velocity.y = angularVelocity[1];
	msgIMU.angular_velocity.z = angularVelocity[2];

	msgIMU.linear_acceleration.x = acceleration[0];
	msgIMU.linear_acceleration.y = acceleration[1];
	msgIMU.linear_acceleration.z = acceleration[2];

	pubIMU.publish(msgIMU);

	// Magnetic Field
	sensor_msgs::MagneticField msgMag;

	msgMag.header.stamp = msgIMU.header.stamp;
	msgMag.header.frame_id = msgIMU.header.frame_id;

	msgMag.magnetic_field.x = magnetic[0];
	msgMag.magnetic_field.y = magnetic[1];
	msgMag.magnetic_field.z = magnetic[2];

	pubMag.publish(msgMag);
}
