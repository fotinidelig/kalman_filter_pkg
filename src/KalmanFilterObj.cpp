#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <keypoint_3d_matching_msgs/Keypoint3d_list.h>

#include <vector>
#include <string>
#include <cmath>
#include <cstdint>

#include "cxcore.h"
#include "cv.h"
#include "highgui.h"
#include <opencv2/video/tracking.hpp>
#include <opencv2/core.hpp>

#include <kalman_filter/KalmanFilterObj.h>

/****
Kalman Filter Variables

state: estimated state variables, contains all xyz coordinates for each keypoint
measurement: xyz coordinates of keyopints
H: measurement matrix
F: transition matrix
Q: process noise covariance
R: measurement noise covariance
P: estimation error covariance

****/


keypoint_3d_matching_msgs::Keypoint3d_list keypointsStructure(keypoint_3d_matching_msgs::Keypoint3d_list );
unsigned int type = CV_64FC1;

KalmanFilterObj::KalmanFilterObj(bool _is3D, float _freq, bool _online){
	int i;
	ros::NodeHandle nh;
	this->is3D = _is3D;
	this->dT = 1/_freq;
	this->online = _online;

	printf("Message intervals: %lf\n",this->dT);

	pub = nh.advertise<keypoint_3d_matching_msgs::Keypoint3d_list>("/kalman_points", 1000);
	debug_pub = nh.advertise<keypoint_3d_matching_msgs::Keypoint3d_list>("/debug_kalman_points", 1000);
}

/* Initialize parameters for Kalman Filter */
void KalmanFilterObj::Init(int size){
	int i;

	keypnt_num = size;

	if(this->is3D)
		measLen = 3*keypnt_num; /* xyz */
	else
		measLen = 2*keypnt_num; /* xy */

	stateLen = measLen;

	kf = cv::KalmanFilter(stateLen, measLen, measLen, type);

	state = cv::Mat(stateLen, 1, type);
	measurement = cv::Mat(measLen, 1, type);
	velocity = cv::Mat(measLen, 1, type);

	timer = cv::Mat(measLen, 1, type);
	if(!online)
		timer.setTo(this->dT);
	else
		timer.setTo(0);
	x_t1 = cv::Mat(measLen, 1, type);
	x_t2 = cv::Mat(measLen, 1, type);

	cv::setIdentity(kf.transitionMatrix);
	cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(15e-3)); /* R = 0.015, OpenPose calculated error */

	cv::setIdentity(kf.processNoiseCov, cv::Scalar(8e-3)); /* Q = 0.008 */
	kf.measurementMatrix = cv::Mat::zeros(measLen, stateLen, type);
	cv::setIdentity(kf.measurementMatrix);
}

void KalmanFilterObj::KalmanFilterCallback(const keypoint_3d_matching_msgs::Keypoint3d_list msg){
	
	int i;
	static double currentTime, curdT = 0;

	static cv::Mat temp_vel;
	cv::Mat predicted, corrected;

	ROS_INFO("Received the points");

	currentTime = msg.keypoints[0].points.header.stamp.toSec(); /* time in seconds */

	curdT = currentTime - this->prevTime;
	this->prevTime = currentTime;

	if(online)
		this->dT = curdT;

	if(this->first_call){
		this->first_call = false;
		this->second_call = true;

		corrected_msg = keypointsStructure(msg);
		debug_msg = keypointsStructure(msg);

		Init(msg.keypoints.size());

		ROS_INFO("First Callback!");

		/*initialize state*/
		for(i = 0; i<keypnt_num; i++){
			x_t2.at<double>(3*i) = msg.keypoints[i].points.point.x;
			x_t2.at<double>(3*i+1) = msg.keypoints[i].points.point.y;
			x_t2.at<double>(3*i+2) = msg.keypoints[i].points.point.z;
		}

		/* publish the initial measurements unchanged*/
		pub.publish(msg);

		return;
	}

	if(this->second_call){
		this->second_call = false;
	
		corrected_msg = keypointsStructure(msg);
		debug_msg = keypointsStructure(msg);	

		std::cout.precision(20);
		Init(msg.keypoints.size());

		/* initialize state and velocity */
		for(i = 0; i<keypnt_num; i++){
			x_t1.at<double>(3*i) = msg.keypoints[i].points.point.x;
			x_t1.at<double>(3*i+1) = msg.keypoints[i].points.point.y;
			x_t1.at<double>(3*i+2) = msg.keypoints[i].points.point.z;

			state.at<double>(3*i) = msg.keypoints[i].points.point.x;
			state.at<double>(3*i+1) = msg.keypoints[i].points.point.y;
			state.at<double>(3*i+2) = msg.keypoints[i].points.point.z;
		}

		state.copyTo(x_t1);

		cv::subtract(x_t1, x_t2, velocity);
		velocity = velocity * (1/this->dT); /* velocity = (x_t1 - x_t2)/dT */
		velocity.copyTo(temp_vel);

		pub.publish(msg);

		kf.statePost = state;
		return;
	}

	std::cout << "dT: " << dT << std::endl;

	/* TODO is3D */
	for(i = 0; i<keypnt_num; i++){
		measurement.at<double>(3*i) = msg.keypoints[i].points.point.x;
		measurement.at<double>(3*i+1) = msg.keypoints[i].points.point.y;
		if(is3D)
			measurement.at<double>(3*i+2) = msg.keypoints[i].points.point.z;
	}

	/* fix controlMatrix with new current dT */
	/**
		B = 
			[dT 0 .....]
			[0 dT 0....]
				 . 
			   	   .
			  		 .
			[0 ......dT]
	**/

	// ROS_INFO("Debug 1");

	/* predict state */
	kf.predict(velocity);
	predicted = kf.statePre;
	
	// /* update */
	kf.correct(measurement);
	corrected = kf.statePost;

	// ROS_INFO("Debug 2");

	/* create new point msg with corrected state */
	for(i = 0; i < keypnt_num; i++){
		corrected_msg.keypoints[i].points.header = msg.keypoints[i].points.header;
		corrected_msg.keypoints[i].points.header.stamp = ros::Time::now(); /* only change timsetamp jtb precise */

		/* check if keypoint wasn't found */
		/* keep current prediction and ignore measurement value NaN */
		if(msg.keypoints[i].points.point.x == 0 && msg.keypoints[i].points.point.y == 0 && msg.keypoints[i].points.point.z == 0){
			
			ROS_INFO("Found NaN");
			corrected_msg.keypoints[i].points.point.x = 0;
			corrected_msg.keypoints[i].points.point.y = 0;
			corrected_msg.keypoints[i].points.point.z = 0;

			kf.statePost.at<double>(3*i)   = x_t1.at<double>(3*i); 
			kf.statePost.at<double>(3*i+1) = x_t1.at<double>(3*i+1);
			kf.statePost.at<double>(3*i+2) = x_t1.at<double>(3*i+2);
		}
		else{
			corrected_msg.keypoints[i].points.point.x = corrected.at<double>(3*i);
			corrected_msg.keypoints[i].points.point.y = corrected.at<double>(3*i+1);
			corrected_msg.keypoints[i].points.point.z = corrected.at<double>(3*i+2);

		}

		debug_msg.keypoints[i].points.header  = msg.keypoints[i].points.header;
		debug_msg.keypoints[i].points.point.x = corrected.at<double>(3*i) - msg.keypoints[i].points.point.x;
		debug_msg.keypoints[i].points.point.y = corrected.at<double>(3*i+1) - msg.keypoints[i].points.point.y;
		debug_msg.keypoints[i].points.point.z = corrected.at<double>(3*i+2) - msg.keypoints[i].points.point.z;
	}

	pub.publish(corrected_msg);
	debug_pub.publish(debug_msg);

	/* calculate new velocity */

	timer = timer + this->dT;

	x_t1.copyTo(x_t2); //x_t2 = x_t1
	measurement.copyTo(x_t1); //x_t1 = measurement

	cv::subtract(x_t1,x_t2,velocity);
	cv::divide(velocity, timer, velocity);

	/* velocity for NaN points */
	for(i = 0; i < keypnt_num; i++){
		if(msg.keypoints[i].points.point.x == 0 && msg.keypoints[i].points.point.y == 0 && msg.keypoints[i].points.point.z == 0){
			velocity.at<double>(3*i)   = temp_vel.at<double>(3*i);
			velocity.at<double>(3*i+1) = temp_vel.at<double>(3*i+1);
			velocity.at<double>(3*i+2) = temp_vel.at<double>(3*i+2);

			// ROS_INFO("NaN velocity change");
			x_t1.at<double>(3*i)   = x_t2.at<double>(3*i); /* keep previous non-NaN :) measurement of point */
			x_t1.at<double>(3*i+1) = x_t2.at<double>(3*i+1);
			x_t1.at<double>(3*i+2) = x_t2.at<double>(3*i+2);			
		}
		else{
			timer.at<double>(3*i)   = 0;
			timer.at<double>(3*i+1) = 0;
			timer.at<double>(3*i+2) = 0;
		}
	}

	velocity.copyTo(temp_vel);
}
