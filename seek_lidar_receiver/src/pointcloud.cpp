#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>

class My_Filter {
     public:
        My_Filter();
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
     private:
        ros::NodeHandle nH;
        laser_geometry::LaserProjection projector_;
        tf::TransformListener tfListener_;

        ros::Publisher point_cloud_publisher_;
        ros::Subscriber scan_sub_;
};

My_Filter::My_Filter()
{
        scan_sub_ = nH.subscribe<sensor_msgs::LaserScan> ("/lms_sync", 1000, &My_Filter::scanCallback, this);
        point_cloud_publisher_ = nH.advertise<sensor_msgs::PointCloud2> ("/cloud", 1000, false);
        tfListener_.setExtrapolationLimit(ros::Duration(0.1));
}

void My_Filter::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    sensor_msgs::PointCloud2 cloud;
    projector_.transformLaserScanToPointCloud("lms_tf", *scan, cloud, tfListener_);
    point_cloud_publisher_.publish(cloud);
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_filter");

    My_Filter filter;

    ros::spin();

    return 0;
}
