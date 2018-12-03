# honors-project-camera-stream

This repository is part of a Fall 2018 Honors Project by Christopher Gibbs and mentored by Dr. Holly Yanco. This contains code for streaming video from a webcam into a format that can be accepted by an Android application using ROS. Video is published to a sensor_msgs/CompressedImage topic called /webcam/image_raw/compressed.

# How To Run

1) Clone this repository into a catkin workspace and run 'catkin_make' in a terminal.
2) Run 'roscore' to launch the master node.
3) In a separate terminal, run 'roslaunch honors-project-camera-stream webcam.launch'.
4) To test that everything is working, open a third terminal and run the compressed_image_subscriber with 'rosrun honors_project_camera_stream compressed_image_subscriber'. This should open a window displaying the video feed.
