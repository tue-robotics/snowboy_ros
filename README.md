# snowboy_ros
ROS wrapper for [Kitt.ai's Snowboy](https://snowboy.kitt.ai/) hotword detection.

The `hotword_detector_node` listens to the topic `/audio  [audio_common_msgs/AudioData]` from an `audo_capture` node. When a hotword is detected, it publishes the number of the hotword on `/trigger  [std_msgs/String]`.
