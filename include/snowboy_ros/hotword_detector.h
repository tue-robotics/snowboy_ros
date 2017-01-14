#ifndef SNOWBOY_ROS_HOTWORD_DETECTOR_H_
#define SNOWBOY_ROS_HOTWORD_DETECTOR_H_

#include <ros/node_handle.h>
#include <audio_common_msgs/AudioData.h>
#include <boost/circular_buffer.hpp>
#include <snowboy/include/snowboy-detect.h>

namespace ros_snowboy
{

class HotwordDetector
{

public:

  HotwordDetector();

  ~HotwordDetector();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_p_;
  ros::Subscriber audio_sub_;
  ros::Publisher trigger_pub_;
  snowboy::SnowboyDetect* detector_;

  boost::circular_buffer<audio_common_msgs::AudioDataConstPtr> audio_buffer_;

  void audioCallback(const audio_common_msgs::AudioDataConstPtr &msg);
};

}  // namespace ros_snowboy

#endif  // SNOWBOY_ROS_HOTWORD_DETECTOR_H_
