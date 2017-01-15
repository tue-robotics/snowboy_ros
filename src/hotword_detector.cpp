#define _GLIBCXX_USE_CXX11_ABI 0

#include "snowboy_ros/hotword_detector.h"

namespace ros_snowboy
{

// ----------------------------------------------------------------------------------------------------

HotwordDetector::HotwordDetector(const char* resource_filename_cpp11,
                                 const char* model_filename_cpp11,
                                 const char* sensitivity_cpp11,
                                 const double audio_gain)
{
  std::string resource_filename(resource_filename_cpp11);
  std::string model_filename(model_filename_cpp11);
  std::string sensitivity(sensitivity_cpp11);

  detector_ = new snowboy::SnowboyDetect(resource_filename, model_filename);
  detector_->SetAudioGain(audio_gain);
  detector_->SetSensitivity("0.6");
}

// ----------------------------------------------------------------------------------------------------

HotwordDetector::~HotwordDetector()
{
  delete detector_;
}

// ----------------------------------------------------------------------------------------------------

int HotwordDetector::RunDetection(const int16_t* const data, const int array_length)
{
  return detector_->RunDetection(data, array_length);
}

}

