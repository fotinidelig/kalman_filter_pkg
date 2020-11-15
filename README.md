## kalman_filter_pkg

A ROS package with an implemented kalman filter, using the OpenCV KalmanFilter class.\
Implemented and used for smoothing OpenPose detected keypoints.\
The filter can accept and correct multiple keypoints, from a Keypoint3d_list message.
***
### Usage

The package listens to the topic where the Keypoint3d_list messages are published, and it publishes the corrected points to the /kalman_points topic.

- change the variable keypoint_topic in config/params.yaml, this is the subscribed topic,
- run roslaunch kalman_filter kalman_filter.launch,
- begin publishing (via rosbag or other packages) the 3d keypoints to the subscribed topic and receive the /kalman_points.
