#include "ImageGrabber.h"
#include <geometry_msgs/PoseStamped.h>

using namespace std;

ImageGrabber::ImageGrabber(ros::NodeHandle& nh, shared_ptr<ORB_SLAM2::System> pSLAM)
{
    mpSLAM = pSLAM;
    posePub = nh.advertise<geometry_msgs::PoseStamped>("pose", 100);

    leftCamSub = make_unique<message_filters::Subscriber<sensor_msgs::Image>>(nh, "/camera/left/image_raw", 10);
    rightCamSub = make_unique<message_filters::Subscriber<sensor_msgs::Image>>(nh, "/camera/right/image_raw", 10);
    sync = make_unique<message_filters::Synchronizer<sync_pol>>(sync_pol(10), *leftCamSub, *rightCamSub);
    sync->registerCallback(boost::bind(&ImageGrabber::GrabStereo, this, _1, _2));
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cout << "Processing: " << cv_ptrLeft->header.seq << endl;

    cv::Mat transformation;
    if (do_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        transformation = mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
    }
    else
    {
        transformation = mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
    }

    if (!transformation.empty())    //Occasionally an empty matrix can be returned
    {
        Eigen::Matrix3d rotMatrix;
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                rotMatrix(i,j) = transformation.at<double>(i,j);
            }
        }

        Eigen::Quaterniond eigenQuat(rotMatrix);    //Converts rotation matrix to quaternion

        geometry_msgs::PoseStamped poseMsg; //seq value is automatic
        poseMsg.header.stamp = cv_ptrLeft->header.stamp;    //todo: is overflowing?
        poseMsg.header.frame_id = "map";    //todo: need to think this through
        poseMsg.pose.position.x = transformation.at<double>(0, 3);  //todo: translation not working as I expected
        poseMsg.pose.position.y = transformation.at<double>(1, 3);
        poseMsg.pose.position.z = transformation.at<double>(2, 3);
        poseMsg.pose.orientation.x = eigenQuat.x();
        poseMsg.pose.orientation.y = eigenQuat.y();
        poseMsg.pose.orientation.z = eigenQuat.z();
        poseMsg.pose.orientation.w = eigenQuat.w();

        posePub.publish(poseMsg);   //May also want to try http://docs.ros.org/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html
    }
}