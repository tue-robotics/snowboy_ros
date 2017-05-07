#include "snowboy_ros/hotword_detector.h"

#include <ros/node_handle.h>
#include <ros/debug.h>
#include <audio_common_msgs/AudioData.h>
#include <std_msgs/String.h>

#include "snowboy_ros/resampler.h"

#include <dynamic_reconfigure/server.h>
#include <snowboy_ros/SnowboyReconfigureConfig.h>

#include <boost/filesystem.hpp>


namespace snowboy_ros
{

//!
//! \brief The HotwordDetectorNode class Wraps the C++ 11 Snowboy detector in a ROS node
//!
class HotwordDetectorNode
{
public:
  HotwordDetectorNode():
    nh_(""),
    nh_p_("~")
  {
  }

  bool initialize()
  {
    audio_sub_ = nh_.subscribe("audio", 1000, &HotwordDetectorNode::audioCallback, this);
    trigger_pub_ = nh_.advertise<std_msgs::String>("trigger", 10);

    std::string resource_filename;
    if (!nh_p_.getParam("resource_filename", resource_filename))
    {
      ROS_ERROR("Mandatory parameter 'resource_filename' not present on the parameter server");
      return false;
    }

    if ( !boost::filesystem::exists( resource_filename ) )
    {
      ROS_ERROR("Resource '%s' does not exist", resource_filename.c_str());
      return false;
    }

    std::string resource_extension = boost::filesystem::extension(resource_filename);
    if ( resource_extension != ".res" )
    {
      ROS_ERROR("'%s' not a valid Snowboy resource extension ('.res').", resource_filename.c_str());
      return false;
    }

    std::string model_filename;
    if (!nh_p_.getParam("model_filename", model_filename))
    {
      ROS_ERROR("Mandatory parameter 'model_filename' not present on the parameter server");
      return false;
    }

    if ( !boost::filesystem::exists( model_filename ) )
    {
      ROS_ERROR("Model '%s' does not exist", model_filename.c_str());
      return false;
    }

    std::string model_extension = boost::filesystem::extension(model_filename);
    if ( model_extension  != ".pmdl" && model_extension != ".umdl" )
    {
      ROS_ERROR("Model '%s', not a valid Snowboy model extension ('.pmdl', '.umdl').", resource_filename.c_str());
      return false;
    }

    trigger_string_ = nh_p_.param("trigger_string", std::string("hotword_detection"));

    detector_.initialize(resource_filename.c_str(), model_filename.c_str());

    dynamic_reconfigure_server_.setCallback(boost::bind(&HotwordDetectorNode::reconfigureCallback, this, _1, _2));

    return true;
  }

private:

  //!
  //! \brief nh_ Global nodehandle for topics
  //!
  ros::NodeHandle nh_;

  //!
  //! \brief nh_p_ Local nodehandle for parameters
  //!
  ros::NodeHandle nh_p_;

  //!
  //! \brief audio_sub_ Subscriber to incoming audio feed
  //!
  ros::Subscriber audio_sub_;

  //!
  //! \brief trigger_pub_ Trigger publisher
  //!
  ros::Publisher trigger_pub_;

  //!
  //! \brief dynamic_reconfigure_server_ In order to online tune the sensitivity and audio gain
  //!
  dynamic_reconfigure::Server<SnowboyReconfigureConfig> dynamic_reconfigure_server_;

  //!
  //! \brief trigger_string_ String to be published when hotword is detected
  //!
  std::string trigger_string_;

  //!
  //! \brief detector_ C++ 11 Wrapped Snowboy detect
  //!
  HotwordDetector detector_;

  //!
  //! \brief no_of_channels_ Number of audio channels in incoming signal
  //!
  int no_of_channels_;

  //!
  //! \brief bit_depth_ The bit depth of the incoming audio samples
  //!
  int bit_depth_;

  //!
  //! \brief sample_rate_ The sample rate of the incoming audio signal
  //!
  int sample_rate_;

  //!
  //! \brief resamper_ Resampler using libsamplerate
  //!
  Resampler resampler_;

  //!
  //! \brief reconfigureCallback Reconfigure update for sensitiviy and audio level
  //! \param cfg The updated config
  //!
  void reconfigureCallback(SnowboyReconfigureConfig cfg, uint32_t /*level*/)
  {
    detector_.configure(cfg.sensitivity, cfg.audio_gain);

    int no_of_channels = cfg.channels;
    int bit_depth = cfg.bit_depth;
    int sample_rate = cfg.sample_rate;

    resampler_.configure(no_of_channels, bit_depth, sample_rate);

    ROS_INFO("SnowboyROS (Re)Configured");
  }

  //!
  //! \brief audioCallback Audio stream callback
  //! \param msg The audo data
  //!
  void audioCallback(const audio_common_msgs::AudioDataConstPtr& msg)
  {
    if (msg->data.size() != 0)
    {

      /* Sound signal is encoded with bit depth 16 and cut up in a byte array. RunDetection wants it as an array of
       * int16_t. Therefore, we bit shift the second (MSB) byte of a sample by 8 and cast it to an int16_t and add the
       * first (LSB) byte of the sample to the result (also after typecast).
       */

      if ( msg->data.size() % 2 )
      {
        ROS_ERROR("Not an even number of bytes in this message!");
      }

      int16_t sample_array[msg->data.size()/2];
      for ( size_t i = 0; i < msg->data.size(); i+=2 )
      {
        sample_array[i/2] = ((int16_t) (msg->data[i+1]) << 8) + (int16_t) (msg->data[i]);
      }

      int result = detector_.runDetection( &sample_array[0], msg->data.size()/2);

      std::vector<int16_t> data;
      if (result > 0)
      {
        ROS_DEBUG("Hotword detected!");
        std_msgs::String trigger_msg;
        trigger_msg.data = trigger_string_;
        trigger_pub_.publish(trigger_msg);
      }
      else if (result == -3)
      {
        ROS_ERROR("Hotword detector not initialized");
      }
      else if (result == -1)
      {
        ROS_ERROR("Snowboy error");
      }
    }
  }
};

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "snowboy_node");

  snowboy_ros::HotwordDetectorNode ros_hotword_detector_node;

  if (ros_hotword_detector_node.initialize())
  {
    ros::spin();
  }
  else
  {
    ROS_ERROR("Failed to initialize snowboy_node");
    return 1;
  }

  return 0;
}

