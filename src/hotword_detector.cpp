#define _GLIBCXX_USE_CXX11_ABI 0

#include "snowboy_ros/hotword_detector.h"

#include <sstream>

namespace snowboy_ros
{

HotwordDetector::HotwordDetector() : detector_(0)
{
}

void HotwordDetector::initialize(const char* resource_filename, const char* model_filename)
{
  // Delete detector if we already had one
  if (detector_)
  {
    delete detector_;
  }

  // We need to use cpp98 therefore we cannot pass std::strings
  std::string resource_filename_cpp98(resource_filename);
  std::string model_filename_cpp98(model_filename);

  detector_ = new snowboy::SnowboyDetect(resource_filename_cpp98, model_filename_cpp98);
}

bool HotwordDetector::configure(double sensitivity, double audio_gain)
{
  // Return false if detector not initialized
  if (!detector_)
  {
    return false;
  }

  std::stringstream sensitivity_ss; sensitivity_ss << sensitivity;

  detector_->SetAudioGain(audio_gain);
  detector_->SetSensitivity(sensitivity_ss.str());

  return true;
}

HotwordDetector::~HotwordDetector()
{
  if (detector_)
  {
    delete detector_;
  }
}

int HotwordDetector::runDetection(const int16_t* const data, const int array_length)
{
  if (!detector_)
  {
    return -3;
  }
  return detector_->RunDetection(data, array_length);
}

}

