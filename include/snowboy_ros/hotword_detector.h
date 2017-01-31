#ifndef SNOWBOY_ROS_HOTWORD_DETECTOR_H_
#define SNOWBOY_ROS_HOTWORD_DETECTOR_H_

#include <snowboy/include/snowboy-detect.h>

namespace snowboy_ros
{

//!
//! \brief The HotwordDetector class wraps Snowboy detect so we can use C++ 11
//!
class HotwordDetector
{

public:

  HotwordDetector();
  ~HotwordDetector();

  //!
  //! \brief initialize Initializes the Snowbody
  //! \param [in]  resource_filename   Filename of resource file.
  //! \param [in]  model_str           A string of multiple hotword models,
  //!
  void initialize(const char* resource_filename, const char* model_filename);

  //!
  //! \brief configure Configure the detector on runtime
  //! \param sensitivity Hotword sensitivity
  //! \param audio_gain Fixed gain to the input audio.
  //! \return True if success, False otherwise
  //!
  bool configure(double sensitivity, double audio_gain);

  //!
  //! \brief runDetection Runs hotword detection of Snowboy, see Snowboy API for more docs
  //! \param data Small chunk of data to be detected
  //! \param array_length Length of the data array.
  //! \return -3 not initialized, -2 Silence, -1 Error, 0 No event, 1 Hotword triggered
  //!
  int runDetection(const int16_t* const data, const int array_length);

private:

  //!
  //! \brief detector_ Instance of Snowboy detect
  //!
  snowboy::SnowboyDetect* detector_;
};

}  // namespace snowboy_ros

#endif  // SNOWBOY_ROS_HOTWORD_DETECTOR_H_
