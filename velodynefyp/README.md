Install ROS

Clone this repo into the workspace

Install the Velodyne driver: http://wiki.ros.org/velodyne

Clone this repo into the workspace: https://github.com/laboshinl/loam_velodyne

Launch with "roslaunch velodynefyp velodynefyp.launch", which starts the Velodyne driver and LOAM SLAM.

In my configuration, the Velodyne and my laptop are both plugged with ethernet into a router. No config was necessary. You can check if the Velodyne is connected by going to 192.168.1.201 in a browser.

The XML file in the config folder is a device-specific calibration file that came with the Velodyne. The yaml file was generated off this by the Velodyne driver (see the wiki).
