

#include <cv_bridge/cv_bridge.h>
// #include <dvs_msgs/EventArray.h>
#include <image_transport/image_transport.h>

#include <opencv2/highgui/highgui.hpp>

#include "ros/ros.h"
#include "std_msgs/String.h"


void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
}

void eventCallback(const EventsVector& events) {}


int main(int argc, char** argv) {
  ros::init(argc, argv, "listener");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);

  ros::Subscriber img_sub = nh.subscribe("topic", 10, imageCallback);
  ros::Subscriber event_sub = nh.subscribe("topic", 10, eventCallback);
  ros::spin();
}