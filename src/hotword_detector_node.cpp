#include "snowboy_ros/hotword_detector.h"

#include <ros/node_handle.h>
#include <ros/debug.h>
#include <audio_common_msgs/AudioData.h>
#include <std_msgs/Int16.h>

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

    publish_wave_ = false;
    nh_p_.getParam("publish_wave", publish_wave_);

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
  ros::Publisher wave_pub_;
  ros_snowboy::HotwordDetector* detector_;

  bool publish_wave_;

  // ----------------------------------------------------------------------------------------------------

  void audioCallback(const audio_common_msgs::AudioDataConstPtr& msg)
  {
    if (msg->data.size() != 0)
    {
      /* Sound signal is encoded with bit depth 16 and cut up in a byte array. RunDetection wants it as an array of
       * int16_t. Therefore, we bit shift the second (MSB) byte of a sample by 8 and cast it to an int16_t and add the
       * first (LSB) byte of the sample to the result (also after typecast).
       */

      if ( msg->data.size() % 2 )
        ROS_ERROR("Not an even number of bytes in this message!");

      int16_t sample_array[msg->data.size()/2];
      for ( size_t i = 0; i < msg->data.size(); i+=2 )
      {
        sample_array[i/2] = ((int16_t) (msg->data[i+1]) << 8) + (int16_t) (msg->data[i]);

        if ( publish_wave_ )
        {
          std_msgs::Int16 sample;
          sample.data = sample_array[i/2];
          wave_pub_.publish(sample);
        }
      }

      int result = detector_->RunDetection( &sample_array[0], msg->data.size()/2);
      if (result > 0)
      {
        ROS_DEBUG("Hotword detected!");
        std_msgs::Int16 trigger_msg;
        trigger_msg.data = result;
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

