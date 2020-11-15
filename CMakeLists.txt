cmake_minimum_required(VERSION 3.0.2)
project(kalman_filter)

find_package(catkin REQUIRED
	roscpp
	std_msgs
	keypoint_3d_matching_msgs
	cv_bridge
    pcl_ros
)

find_package(OpenCV REQUIRED)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME} 
	CATKIN_DEPENDS roscpp std_msgs keypoint_3d_matching_msgs cv_bridge pcl_ros
)


include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

add_executable(kalman_filter src/KalmanFilterObj.cpp src/main.cpp src/utilities.cpp)

target_link_libraries(kalman_filter ${catkin_LIBRARIES})