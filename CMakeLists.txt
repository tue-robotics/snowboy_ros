cmake_minimum_required(VERSION 2.8.3)
project(snowboy_ros)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    audio_common_msgs
)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

# find_package(Boost REQUIRED COMPONENTS system program_options)
# find_package(PCL REQUIRED)
# find_package(OpenCV REQUIRED)

find_path(BLAS_LIBRARIES cblas.a
  /usr/lib
  /usr/local/lib
  $ENV{BLAS_HOME}/lib)
#find_package(BLAS REQUIRED)

# ------------------------------------------------------------------------------------------------
#                                     ROS MESSAGES AND SERVICES
# ------------------------------------------------------------------------------------------------

# Generate messages
# add_message_files(
#    FILES
#    message1.msg
#    ...
# )

# Generate services
# add_service_files(
#    FILES
#    service1.srv
#    ...
# )

# Generate added messages and services with any dependencies listed here
# generate_messages(
#    DEPENDENCIES
#    geometry_msgs
#    ...
# )

# ------------------------------------------------------------------------------------------------
#                                          CATKIN EXPORT
# ------------------------------------------------------------------------------------------------

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hotword_detector
#  CATKIN_DEPENDS roscpp audio_common_msgs
#  DEPENDS system_lib
)

# ------------------------------------------------------------------------------------------------
#                                              BUILD
# ------------------------------------------------------------------------------------------------

include_directories(
    include
    3rdparty
    ${catkin_INCLUDE_DIRS}
)

file(GLOB_RECURSE HEADER_FILES include/*.h)

add_library(hotword_detector
    src/hotword_detector.cpp
    ${HEADER_FILES}
)

target_link_libraries(hotword_detector
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/snowboy/lib/ubuntu64/libsnowboy-detect.a
    ${catkin_LIBRARIES}
    ${BLAS_LIBRARIES}
)

add_executable(hotword_detector_node
    src/hotword_detector_node.cpp
)
target_link_libraries(hotword_detector_node hotword_detector ${catkin_LIBRARIES})


