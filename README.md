# General prereqs
Must install ROS. Also, I recommend you use "catkin build" rather than "catkin_make": http://catkin-tools.readthedocs.io/en/latest/installing.html

# Velodyne prereqs
sudo apt install ros-kinetic-velodyne

# How to install Zed camera driver
Driver is from here: https://github.com/stereolabs/zed-ros-wrapper

Ensure that you pick a release (or pull latest version) that supports the Zed SDK you have installed. Each Zed SDK version requires a specific CUDA version. Changing the version of CUDA requires flashing the relevant JetPack (Full OS reinstall!!!)


# How to install ORB-SLAM2
Install this: https://github.com/stevenlovegrove/Pangolin

Change the CPU architecture at the bottom of car_localisation/orbslam2/CMakeLists.txt, otherwise you'll get "error adding symbols: File in wrong format" while compiling. If you want to add a new architecture, you'll need to download ORB-SLAM2 (https://github.com/raulmur/ORB_SLAM2), compile g2o and DBoW2 in the ThirdParty folder, and copy their .so files across.

Compiling them is as easy as going into their folders, entering "cmake ." then "make". Their .so files are found in their lib folders.



