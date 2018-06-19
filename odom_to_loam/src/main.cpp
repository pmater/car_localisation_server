#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <iostream>

using namespace std;

geometry_msgs::TransformStamped map_to_loam_init;
geometry_msgs::TransformStamped quatRot;
ros::Publisher loamPub;

void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg);
void PoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg); //Used to test transformation when EKF is disabled

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "odom_to_loam");
	ros::NodeHandle n;

    auto odomSub = n.subscribe<nav_msgs::Odometry>("/ekf/odom_gps_corrected", 2, OdomCallback);
    //auto poseSubTest = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/laser_odom_map", 2, PoseCallback);
    loamPub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/ekf/loam_converted_odom_gps_corrected", 5);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tf2_listener(tfBuffer);
    ROS_INFO_STREAM("OTL: Waiting for TF");
    map_to_loam_init = tfBuffer.lookupTransform("loam_init", "map", ros::Time(0), ros::Duration(10.0));
    ROS_INFO_STREAM("OTL: TF received");

    tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, 3.141592654 / 2);
    quatRot.transform.translation.x = 0;
    quatRot.transform.translation.y = 0;
    quatRot.transform.translation.z = 0;
    quatRot.transform.rotation.x = q.x();
    quatRot.transform.rotation.y = q.y();
    quatRot.transform.rotation.z = q.z();
    quatRot.transform.rotation.w = q.w();

    ros::spin();
    return 0;
}

//Converts from ROS frame back to LOAM frame
void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    geometry_msgs::PoseStamped msgTransformTemp, msgQuaternionTemp;
    msgQuaternionTemp.pose.orientation = msg->pose.pose.orientation;
    tf2::doTransform(msgQuaternionTemp, msgQuaternionTemp, quatRot);

    msgTransformTemp.pose.orientation = msgQuaternionTemp.pose.orientation;
    msgTransformTemp.pose.position = msg->pose.pose.position;

    tf2::doTransform(msgTransformTemp, msgTransformTemp, map_to_loam_init); //Internally, LOAM has data in loam_init frame. Therefore, we need to transform it to map frame.

    geometry_msgs::PoseWithCovarianceStamped loamMsg;
    loamMsg.header = msg->header;
    loamMsg.header.frame_id = "loam_init";
    loamMsg.pose.pose.position = msgTransformTemp.pose.position;
    loamMsg.pose.pose.orientation = msgTransformTemp.pose.orientation;
    loamPub.publish(loamMsg);
}

/*void PoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    ROS_INFO_STREAM("No EKF mode");
    geometry_msgs::PoseStamped msgTransformTemp, msgQuaternionTemp;
    msgQuaternionTemp.pose.orientation = msg->pose.pose.orientation;
    tf2::doTransform(msgQuaternionTemp, msgQuaternionTemp, quatRot);

    msgTransformTemp.pose.orientation = msgQuaternionTemp.pose.orientation;
    msgTransformTemp.pose.position = msg->pose.pose.position;

    tf2::doTransform(msgTransformTemp, msgTransformTemp, map_to_loam_init); //Internally, LOAM has data in loam_init frame. Therefore, we need to transform it to map frame.

    geometry_msgs::PoseWithCovarianceStamped loamMsg;
    loamMsg.header = msg->header;
    loamMsg.header.frame_id = "loam_init";
    loamMsg.pose.pose.position = msgTransformTemp.pose.position;
    loamMsg.pose.pose.orientation = msgTransformTemp.pose.orientation;
    loamPub.publish(loamMsg);
}*/