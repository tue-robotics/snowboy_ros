#include "snowboy_ros/hotword_detector.h"

#include <ros/node_handle.h>
#include <audio_common_msgs/AudioData.h>
#include <std_msgs/Int16.h>
#include <boost/circular_buffer.hpp>

namespace ros_snowboy
{

class HotwordDetectorNode
{
public:
  HotwordDetectorNode():
    nh_(""),
    nh_p_("~")
  {
    audio_sub_ = nh_.subscribe("audio", 1000, &HotwordDetectorNode::audioCallback, this);
    trigger_pub_ = nh_.advertise<std_msgs::Int16>("detection", 10);

    std::string resource_filename("");
    nh_p_.getParam("resource_filename", resource_filename);

    std::string model_filename("");
    nh_p_.getParam("model_filename", model_filename);

    std::string sensitivity_str("");
    nh_p_.getParam("sensitivity_str", sensitivity_str);

    double audio_gain = 1.0;
    nh_p_.getParam("audio_gain", audio_gain);

    detector_ = new ros_snowboy::HotwordDetector(resource_filename.c_str(), model_filename.c_str(), sensitivity_str.c_str(), audio_gain);
  }

  // ----------------------------------------------------------------------------------------------------

  ~HotwordDetectorNode()
  {
    delete detector_;
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_p_;
  ros::Subscriber audio_sub_;
  ros::Publisher trigger_pub_;
  ros_snowboy::HotwordDetector* detector_;

  boost::circular_buffer<audio_common_msgs::AudioDataConstPtr> audio_buffer_;

  // ----------------------------------------------------------------------------------------------------

  void audioCallback(const audio_common_msgs::AudioDataConstPtr& msg)
  {
    audio_buffer_.push_back(msg);

    if (msg->data.size() != 0)
    {
      // msg is encoded in uint8 and RunDetection wants it in int16_t

      int16_t decoded_data[msg->data.size()];
      for ( size_t i = 0; i < msg->data.size(); i++)
      {
        decoded_data[i] = (int16_t) (msg->data[i] - 0x80) << 8;
      }
      int result = detector_->RunDetection( &decoded_data[0], msg->data.size());
      if (result > 0)
      {
        std::cout << "Hotword detected!" << std::endl;
        std_msgs::Int16 trigger_msg;
        trigger_msg.data = (int16_t) result;
        trigger_pub_.publish(trigger_msg);
      }
    }
  }
};

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "snowboy_node");

  ros_snowboy::HotwordDetectorNode ros_hotword_detector_node;

  ros::spin();
}

