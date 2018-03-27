#include <iostream>
#include <memory>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <ros/ros.h>
#include "std_msgs/Bool.h"

#include <opencv2/core/core.hpp>

#include "System.h"
#include "ImageGrabber.h"

using namespace std;

shared_ptr<ORB_SLAM2::System> SLAM;

void ShutdownSLAMCallback(const std_msgs::Bool& msg);

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "orbslam2");
    ros::NodeHandle nh;
    ros::start();

    if(argc != 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Stereo path_to_vocabulary path_to_settings do_rectify" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    SLAM = make_shared<ORB_SLAM2::System>(argv[1], argv[2], ORB_SLAM2::System::STEREO);

    ImageGrabber igb(nh, SLAM);

    stringstream ss(argv[3]);
	ss >> boolalpha >> igb.do_rectify;

    if (igb.do_rectify)
    {      
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() || rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
    }

    auto shutdownSLAMSub = nh.subscribe("shutdown", 10, ShutdownSLAMCallback);
    ros::spin();
    ros::shutdown();
    SLAM->Shutdown(false);

    return 0;
}

void ShutdownSLAMCallback(const std_msgs::Bool& msg)
{
    cout << "ORB-SLAM2 shutting down" << endl;
    SLAM->Shutdown(msg.data);

    // Save camera trajectory
    SLAM->SaveKeyFrameTrajectoryTUM("/home/roby/slam/maps/logs/KeyFrameTrajectory_TUM_Format.txt");
    SLAM->SaveTrajectoryTUM("/home/roby/slam/maps/logs/FrameTrajectory_TUM_Format.txt");
    SLAM->SaveTrajectoryKITTI("/home/roby/slam/maps/logs/FrameTrajectory_KITTI_Format.txt");

    ros::shutdown();
}
