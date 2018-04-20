cmake_minimum_required(VERSION 2.8.3)
project(seek_lidar_receiver)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs geometry_msgs tf message_generation laser_geometry)

add_message_files(FILES lms_message.msg)
generate_messages(DEPENDENCIES std_msgs sensor_msgs)
catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs tf geometry_msgs message_runtime laser_geometry)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(pointcloud src/pointcloud.cpp )
target_link_libraries(pointcloud ${catkin_LIBRARIES})
add_dependencies(pointcloud seek_lidar_receiver_generate_messages_cpp)

add_executable(lms_publish src/lms_publish.cpp src/lms_sourcecode.cpp)
target_link_libraries(lms_publish ${catkin_LIBRARIES})
add_dependencies(lms_publish seek_lidar_receiver_generate_messages_cpp)

#LMS is the sole tf broadcaster
#add_executable(lms_position_tf src/lms_tf_position.cpp)
#target_link_libraries(lms_position_tf ${catkin_LIBRARIES})
#add_dependencies(lms_position_tf lms_package_generate_messages_cpp)
