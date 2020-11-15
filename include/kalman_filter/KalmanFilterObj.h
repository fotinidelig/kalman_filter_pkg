#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <keypoint_3d_matching_msgs/Keypoint3d_list.h>

#include <opencv2/video/tracking.hpp>
#include "opencv2/highgui.hpp"
#include <opencv2/opencv.hpp>

class KalmanFilterObj
{
public:
	KalmanFilterObj( bool is3D, float freq, bool online);
	void KalmanFilterCallback(const keypoint_3d_matching_msgs::Keypoint3d_list msg);
	void Init(int size);
	bool is3D;
	bool online;
private: 
	int measLen, stateLen, keypnt_num;
	
	long double prevTime;
	// long double curdT;
	float dT;

	cv::Mat state, measurement, velocity;
	cv::Mat x_t1, x_t2;
	cv::Mat timer;
	cv::Mat window; /* will be used to detect outliers, window size == 5 */
	cv::KalmanFilter kf;
	bool first_call = true, second_call = false;
	ros::Publisher pub, debug_pub;
	keypoint_3d_matching_msgs::Keypoint3d_list corrected_msg, debug_msg;
};
