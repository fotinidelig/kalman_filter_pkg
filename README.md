## kalman_filter_pkg

![Build Status](https://upload.wikimedia.org/wikipedia/commons/thumb/3/32/OpenCV_Logo_with_text_svg_version.svg/180px-OpenCV_Logo_with_text_svg_version.svg.png)
<img src="http://wiki.ros.org/melodic?action=AttachFile&do=get&target=melodic.jpg" width="220">
<img src="https://encrypted-tbn0.gstatic.com/images?q=tbn%3AANd9GcQhykQfMn6vBr8q24DRapZ_PqBerW491szxvg&usqp=CAU" width="220">
***
A ROS package with an implemented kalman filter, using the OpenCV KalmanFilter class.\
Implemented and used for smoothing OpenPose detected keypoints.\
The filter can accept and correct multiple keypoints, from a Keypoint3d_list message.
***
### Usage

The package listens to the topic where the Keypoint3d_list messages are published, and it publishes the corrected points to the /kalman_points topic.

- change the variable keypoint_topic in config/params.yaml, this is the subscribed topic,
- run ```roslaunch kalman_filter kalman_filter.launch```,
- begin publishing (via rosbag or other packages) the 3d keypoints to the subscribed topic and receive the /kalman_points.
#### Configuration parameters
- <em>online</em>: set to true if time is calculated from the message's timestamps
- <em>keypoint_topic</em>: topic to subscribe to
- <em>frequency</em>: frame frequency, used to calculate time between messages in off-line mode
