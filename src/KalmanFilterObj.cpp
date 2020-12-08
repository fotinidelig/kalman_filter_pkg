#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <keypoint_3d_matching_msgs/Keypoint3d_list.h>
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

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
measurement: xyz coordinates of keypοints
H: measurement matrix
F: transition matrix
Q: process noise covariance
R: measurement noise covariance
P: estimation error covariance

****/

#define DIST_1 0.10
#define DIST_2 0.18

bool is_outlier(geometry_msgs::Point newP, geometry_msgs::Point oldP, int was_outlier){

	double distance;
	distance = pow(newP.x-oldP.x,2) + pow(newP.y-oldP.y,2);
	distance = pow(distance, 0.5);

	if(newP.x == 0 && newP.y == 0 && newP.z == 0)
		return true;

	if(!was_outlier && distance > DIST_1)
		return true;
	
	if(was_outlier && distance > DIST_2)
		return true;

	return false;
}

keypoint_3d_matching_msgs::Keypoint3d_list keypointsStructure(keypoint_3d_matching_msgs::Keypoint3d_list );
unsigned int type = CV_64FC1;
ros::Publisher vel;

KalmanFilterObj::KalmanFilterObj(float _freq, bool _online){

	int i;
	ros::NodeHandle nh;
	if(!_online){
		this->dT = 1/_freq;
		printf("Message intervals: %lf\n",this->dT);
	}
	this->online = _online;
	pub = nh.advertise<keypoint_3d_matching_msgs::Keypoint3d_list>("/kalman_points", 10000);
	debug_pub = nh.advertise<keypoint_3d_matching_msgs::Keypoint3d_list>("/debug_kalman_points", 10000);
	vel = nh.advertise<std_msgs::Float64MultiArray>("/velocity", 10000);
}

/* Initialize parameters for Kalman Filter */
void KalmanFilterObj::Init(int size){
	int i;

	this->keypnt_num = size;

	this->measLen = 3*keypnt_num; /* xyz */

	this->stateLen = measLen;

	kf = cv::KalmanFilter(stateLen, measLen, measLen, type);

	state = cv::Mat(stateLen, 1, type);
	measurement = cv::Mat(measLen, 1, type);
	velocity = cv::Mat(measLen, 1, type);

	timer = cv::Mat(measLen, 1, type);
	timer.setTo(0);

	x_t1 = cv::Mat(measLen, 1, type);
	x_t2 = cv::Mat(measLen, 1, type);

	cv::setIdentity(kf.transitionMatrix);
	cv::setIdentity(kf.measurementNoiseCov, 0.015); /* R = 0.015, OpenPose calculated error */ /*TODO*/

	for (i=0;i<=keypnt_num;i++){
		kf.measurementNoiseCov.at<double>(3*i+1,3*i+1) = 0.018; /*y-axis is noisier*/ /*TODO*/
	}

	cv::setIdentity(kf.processNoiseCov, cv::Scalar(8e-3)); /* Q = 0.008 */ /*TODO*/
	kf.measurementMatrix = cv::Mat::zeros(measLen, stateLen, type);
	cv::setIdentity(kf.measurementMatrix);
	// cv::setIdentity(kf.errorCovPost, cv::Scalar::all(1)); /*TODO*/
	cv::setIdentity(kf.controlMatrix);
}

static double currentTime = 0.0, curdT = 0.0;
static cv::Mat temp_vel; /* here we store previous velocity */
static cv::Mat predicted, corrected; /* kalman predicted/corrected keypoints */
static keypoint_3d_matching_msgs::Keypoint3d_list prev_keypoints; /* used to spot outliers */
static cv::Mat was_outlier; /* true if previous keypoint measurement was an outlier */

void KalmanFilterObj::KalmanFilterCallback(const keypoint_3d_matching_msgs::Keypoint3d_list msg){
	
	int i;
	if(msg.keypoints.size() == 0){
		ROS_INFO("Message has NO keypoints, continuing...");
		pub.publish(msg);
		return;
	}

	currentTime = msg.keypoints[0].points.header.stamp.toSec(); /* time in seconds */

	curdT = currentTime - this->prevTime;

	if(curdT == 0.0 && this->online){
		ROS_INFO("***Zero Time Difference***");
		pub.publish(msg);
		return;
	}

	std::cout << "[Current Difference: "<<std::setprecision(25)<<curdT<<"]"<<std::endl;
	this->prevTime = currentTime;

	if(this->online)
		this->dT = curdT;

	if(this->first_call){
		this->first_call = false;
		this->second_call = true;

		corrected_msg = keypointsStructure(msg);
		debug_msg = keypointsStructure(msg);
		prev_keypoints = keypointsStructure(msg);

		ROS_INFO("First Callback!");
		
		/*initialization*/

		Init(msg.keypoints.size());
		
		was_outlier = cv::Mat(this->keypnt_num, 1, CV_32SC1);
		was_outlier.setTo(0); /*at the beginning no outlier*/

		for(i = 0; i<msg.keypoints.size(); i++){
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
		ROS_INFO("Second Callback!");
		/* initialize state and velocity */
		for(i = 0; i<keypnt_num; i++){
			state.at<double>(3*i) = msg.keypoints[i].points.point.x;
			state.at<double>(3*i+1) = msg.keypoints[i].points.point.y;
			state.at<double>(3*i+2) = msg.keypoints[i].points.point.z;

			prev_keypoints.keypoints[i].points.point.x = msg.keypoints[i].points.point.x;
			prev_keypoints.keypoints[i].points.point.y = msg.keypoints[i].points.point.y;
			prev_keypoints.keypoints[i].points.point.z = msg.keypoints[i].points.point.z;
		}

		state.copyTo(x_t1);

		cv::subtract(x_t1, x_t2, velocity);
		velocity = velocity * (1/this->dT); /* velocity = (x_t1 - x_t2)/dT */
		velocity.copyTo(temp_vel);

		pub.publish(msg);

		kf.statePost = state;
		return;
	}

	for(i = 0; i<keypnt_num; i++){
		measurement.at<double>(3*i) = msg.keypoints[i].points.point.x;
		measurement.at<double>(3*i+1) = msg.keypoints[i].points.point.y;
		measurement.at<double>(3*i+2) = msg.keypoints[i].points.point.z;
	}

	/* fix controlMatrix with current dT */

	timer = timer + this->dT;
	for(i = 0; i<keypnt_num; i++){
		kf.controlMatrix.at<double>(3*i, 3*i) = timer.at<double>(3*i);
		kf.controlMatrix.at<double>(3*i+1, 3*i+1) = timer.at<double>(3*i+1);
		kf.controlMatrix.at<double>(3*i+2, 3*i+2) = timer.at<double>(3*i+2);
	}

	/* predict state */
	predicted = kf.predict();

	/* update state */
	corrected = kf.correct(measurement);

	/* create new point msg with corrected state */
	for(i = 0; i < msg.keypoints.size(); i++){
		corrected_msg.keypoints[i].points.header = msg.keypoints[i].points.header;
		corrected_msg.keypoints[i].points.header.stamp = ros::Time::now(); /* only change timestamp jtb precise */

		/* check if keypoint wasn't found */
		/* keep current prediction and ignore measurement value NaN */
		if(is_outlier(msg.keypoints[i].points.point, prev_keypoints.keypoints[i].points.point, was_outlier.at<int>(i))){

			corrected_msg.keypoints[i].points.point.x = predicted.at<double>(3*i);
			corrected_msg.keypoints[i].points.point.y = predicted.at<double>(3*i+1);
			corrected_msg.keypoints[i].points.point.z = predicted.at<double>(3*i+2);

			// corrected_msg.keypoints[i].points.point.x = 0;
			// corrected_msg.keypoints[i].points.point.y = 0;
			// corrected_msg.keypoints[i].points.point.z = 0;

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
		debug_msg.keypoints[i].points.point.x = predicted.at<double>(3*i);
		debug_msg.keypoints[i].points.point.y = predicted.at<double>(3*i+1);
		debug_msg.keypoints[i].points.point.z = predicted.at<double>(3*i+2);
	}

	pub.publish(corrected_msg);
	debug_pub.publish(debug_msg);

	/* calculate new velocity */

	x_t1.copyTo(x_t2); // x_t2 <- x_t1
	measurement.copyTo(x_t1); // x_t1 <- measurement

	cv::subtract(x_t1,x_t2,velocity);
	cv::divide(velocity, timer, velocity);

	// /* velocity for NaN points */
	for(i = 0; i < keypnt_num; i++){
		if(is_outlier(msg.keypoints[i].points.point, prev_keypoints.keypoints[i].points.point, was_outlier.at<int>(i))){
			was_outlier.at<int>(i)	   = 1;

			velocity.at<double>(3*i)   = temp_vel.at<double>(3*i);
			velocity.at<double>(3*i+1) = temp_vel.at<double>(3*i+1);
			velocity.at<double>(3*i+2) = temp_vel.at<double>(3*i+2);

			x_t1.at<double>(3*i)   = x_t2.at<double>(3*i); /* keep previous non-NaN :) measurement of point */
			x_t1.at<double>(3*i+1) = x_t2.at<double>(3*i+1);
			x_t1.at<double>(3*i+2) = x_t2.at<double>(3*i+2);			
		}
		else{
			was_outlier.at<int>(i)  = 0;

			timer.at<double>(3*i)   = 0;
			timer.at<double>(3*i+1) = 0;
			timer.at<double>(3*i+2) = 0;
		}
	}
	velocity.copyTo(temp_vel);

	/* publish velocity */
	/*
	std_msgs::Float64MultiArray velmsg;


    // push data into velocity msg
    for(i = 0; i < measLen; i++) {
        velmsg.data.push_back(velocity.at<double>(i)); 
    }

	vel.publish(velmsg);
	*/

	/* update previous keypoint values */
	for (i = 0; i < keypnt_num; i++){
		prev_keypoints.keypoints[i].points.point.x = x_t1.at<double>(3*i);
		prev_keypoints.keypoints[i].points.point.y = x_t1.at<double>(3*i+1);
		prev_keypoints.keypoints[i].points.point.z = x_t1.at<double>(3*i+2);
	}
}