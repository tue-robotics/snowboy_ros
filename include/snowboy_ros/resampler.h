#ifndef SNOWBOY_ROS_RESAMPLER_H_
#define SNOWBOY_ROS_RESAMPLER_H_

#include <samplerate.h>

namespace snowboy_ros
{

class Resampler
{
public:
  Resampler(){}

  ~Resampler(){}

  void configure(int number_of_channels, int bit_depth, int sample_rate){}

//  template<typename T_in, typename T_out>
//  std::vector<T_out> resample<T_out>(T_in* data);

};

}  // namespace snowboy_ros

#endif  // SNOWBOY_ROS_RESAMPLER_H_
