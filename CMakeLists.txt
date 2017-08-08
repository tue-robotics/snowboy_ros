cmake_minimum_required(VERSION 2.8.3)
project(snowboy_ros)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    audio_common_msgs
    dynamic_reconfigure
)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)

find_package(BLAS)

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

#add dynamic reconfigure api
#find_package(catkin REQUIRED dynamic_reconfigure)
generate_dynamic_reconfigure_options(
    cfg/SnowboyReconfigure.cfg
)

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
file(GLOB_RECURSE 3RD_PARTY_FILES 3rdparty/*.h)

add_library(hotword_detector
    src/hotword_detector.cpp
    ${HEADER_FILES}
    ${3RD_PARTY_FILES}
)

target_link_libraries(hotword_detector
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/snowboy/lib/ubuntu64/libsnowboy-detect.a
    ${catkin_LIBRARIES}
    ${BLAS_LIBRARIES}
)

add_executable(hotword_detector_node
    src/hotword_detector_node.cpp
)
target_link_libraries(hotword_detector_node
    hotword_detector
    ${catkin_LIBRARIES}
)
add_dependencies(hotword_detector_node ${PROJECT_NAME}_gencfg)

install(
  TARGETS
  hotword_detector
  hotword_detector_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(
  DIRECTORY resources/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/resources
)
