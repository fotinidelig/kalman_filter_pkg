#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <keypoint_3d_matching_msgs/Keypoint3d_list.h>

#include <vector>
#include <string>
#include <cmath>

#include <kalman_filter/KalmanFilterObj.h>

std::vector<int> keypoint_codes;
int freq;

int main(int argc, char** argv){

	ros::init(argc, argv, "kalmanFilter");

	ros::NodeHandle nh;
	ros::Subscriber sub;

	std::string keypoint_topic;
	bool online;

	nh.param("kalmanFilter/keypoint_topic", keypoint_topic, std::string("/raw_points_online"));
	nh.param("kalmanFilter/frequency", freq, 30);
	nh.param("kalmanFilter/online", online, false);

	// keypoint_3d_matching_msgs::Keypoint3d_list temp_points;

	// temp_points = keypointsStructure(keypoint_codes);

	// ROS_INFO("keypoints: %d",int(keypoint_codes.size()));
	ROS_INFO("keypoint topic: %s", keypoint_topic.c_str());

	// KalmanFilterObj kf_obj(keypoint_codes.size(), is3D, temp_points);
	KalmanFilterObj kf_obj(freq, online);

	// sub = nh.subscribe(keypoint_topic, 1000, &KalmanFilterObj::KalmanFilterCallback, &kf_obj);
	sub = nh.subscribe(keypoint_topic, 1, &KalmanFilterObj::KalmanFilterCallback, &kf_obj);

	ros::spin();

	return 0;
}
