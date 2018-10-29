/*
 * FILE: compressed_image_subscriber.cpp
 * AUTHOR: Christopher Gibbs
 * DATE: 10/29/2018
 * DESCRIPTION: A ROS node that subscribes to a compressed image stream and
 *	plays it.
 * NOTE: This file is modified from one shown in a ROS tutorial for
 *	image_transport, which can be found here:
 *	http://wiki.ros.org/image_transport/Tutorials/SubscribingToImages
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace cv;

/*
 * The callback method for the subscriber node. Attempts to display the
 * compressed image stream.
 *
 * msg: The compressed image sensor message.
 */
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
        try
        {
                cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
                cv::waitKey(30);
        }
        catch (cv_bridge::Exception& exception)
        {
                ROS_ERROR("Could not convert from '%s' to 'bgr8'",
                                msg->encoding.c_str());
        }
}

/*
 * The main entry point of the program. Creates a subscriber node for a
 * compressed image stream and displays the stream.
 */
int main(int argc, char **argv)
{
        // Set up the node
        ros::init(argc, argv, "compressed_image_subscriber");
        ros::NodeHandle nh;

        // Create a window to display the stream
        cv::namedWindow("view");
        cv::startWindowThread();

        // Subscribe to the image topic
        // Note - Must subscribe to base raw image topic instead of the
        // compressed one
        image_transport::ImageTransport it(nh);
        image_transport::Subscriber sub =
                it.subscribe("webcam/image_raw", 1, imageCallback);

        ros::spin();

        // If we ever get here, properly destroy things
        cv::destroyWindow("view");

        return 0;
}
