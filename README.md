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

# Getting read/write permissions to IMU
It's very likely you'll get permission errors when attempting to start the vectornav launch file. To resolve this, enter the command: "sudo usermod -a -G dialout ENTER-YOUR-USERNAME-HERE". Also make sure the launch file contains the correct USB port, which should be ttyUSB0 if you have no other devices connected.

# Network setup
On every machine in the network run the command "export ROS_MASTER_URI=http://192.168.1.1:11311", which is the static IP address of the main server. Can also add this line to the bottom of .bashrc to make it automatic each boot.

Do the same for "export ROS_IP=[WHATEVER THE IP ADDRESS OF THAT COMPUTER IS]"

# Network explanation
Unfortunately, every device has been assigned a static IP address due to the limitations in place. The switch can't act as a DHCP server, so every IP address is static. Also, the switch can't act as a DNS server so we can't use convenient host names. I've chosen to use 192.168.1.x IP addresses because the default IP address of the LIDAR is 192.168.1.201, and I want to leave it as default so that in the future it'll be easy for people to connect to. It may be possible to run a DNS and DHCP server on the main computer, but I don't want to dive into that.
