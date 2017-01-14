#include "snowboy_ros/hotword_detector.h"
#include <std_msgs/String.h>
#include <memory>


namespace ros_snowboy
{

// ----------------------------------------------------------------------------------------------------

HotwordDetector::HotwordDetector():
  nh_(""),
  nh_p_("~")
{
  audio_sub_ = nh_.subscribe("audio", 1000, &HotwordDetector::audioCallback, this);
  trigger_pub_ = nh_.advertise<std_msgs::String>("detection", 10);

  std::string resource_filename("");
  nh_p_.getParam("resource_filename", resource_filename);

  std::string model_filename("");
  nh_p_.getParam("model_filename", model_filename);

  std::string sensitivity_str("");
  nh_p_.getParam("sensitivity_str", sensitivity_str);

  double audio_gain = 0;
  nh_p_.getParam("audio_gain", audio_gain);

  detector_ = new snowboy::SnowboyDetect(resource_filename, model_filename);
  detector_->SetSensitivity(sensitivity_str);
  detector_->SetAudioGain(audio_gain);
}

// ----------------------------------------------------------------------------------------------------

HotwordDetector::~HotwordDetector()
{
  delete detector_;
}

// ----------------------------------------------------------------------------------------------------

void HotwordDetector::audioCallback(const audio_common_msgs::AudioDataConstPtr &msg)
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
      std::cout << "Hotword " << result << " detected!" << std::endl;
      std_msgs::String trigger_msg;
      trigger_msg.data = "Hey Amigo";
      trigger_pub_.publish(trigger_msg);
    }
  }
}

}

