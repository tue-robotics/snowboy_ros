#include "snowboy_ros/hotword_detector.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "snowboy_node");

  ros_snowboy::HotwordDetector ros_hotword_detector;

  ros::spin();
}

