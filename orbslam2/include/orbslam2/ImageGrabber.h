#include <memory>

#include "System.h"
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class ImageGrabber
{
    private:
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;

    public:
        ImageGrabber(ros::NodeHandle& nh, std::shared_ptr<ORB_SLAM2::System> pSLAM);

        void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);

        std::shared_ptr<ORB_SLAM2::System> mpSLAM;
        bool do_rectify;
        cv::Mat M1l,M2l,M1r,M2r;

        std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>> leftCamSub;
        std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>> rightCamSub;
        std::unique_ptr<message_filters::Synchronizer<sync_pol>> sync;
        ros::Publisher posePub;
};