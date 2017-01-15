#ifndef SNOWBOY_ROS_HOTWORD_DETECTOR_H_
#define SNOWBOY_ROS_HOTWORD_DETECTOR_H_

#include <snowboy/include/snowboy-detect.h>

namespace ros_snowboy
{

class HotwordDetector
{

public:

  HotwordDetector(const char* resource_filename_cpp11,
                  const char* model_filename_cpp11,
                  const char* sensitivity_cpp11,
                  const double audio_gain);

  ~HotwordDetector();

  int RunDetection(const int16_t* const data, const int array_length);

private:
  snowboy::SnowboyDetect* detector_;
};

}  // namespace ros_snowboy

#endif  // SNOWBOY_ROS_HOTWORD_DETECTOR_H_
