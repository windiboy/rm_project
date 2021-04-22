cmake_minimum_required(VERSION 3.0.2)
project(openvino)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  sensor_msgs
  std_msgs
  image_transport
  geometry_msgs
  message_filters
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)
find_package(InferenceEngine 2.0 REQUIRED)
find_package(ngraph REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES monitors
#  CATKIN_DEPENDS cv_bridge roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)


include_directories(
  include/monitor
  include/human_pose
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${InferenceEngine_INCLUDE_DIRS}
)

add_library(monitors STATIC
   include/monitors/presenter.cpp
   include/monitors/cpu_monitor.cpp
   include/monitors/memory_monitor.cpp
   include/monitors/presenter.h
   include/monitors/cpu_monitor.h
   include/monitors/memory_monitor.h
 )
#target_include_directories(monitors PUBLIC include/monitor/)
target_link_libraries(monitors PRIVATE opencv_core opencv_imgproc)

add_executable(human_pose_node 
   src/human_pose.cpp
   src/human_pose_estimator.cpp
   src/peak.cpp
   src/render_human_pose.cpp
   src/human_pose_node.cpp
)
#target_include_directories(${PROJECT_NAME}_node PRIVATE include )
target_link_libraries(human_pose_node ${OpenCV_LIBRARIES} ${InferenceEngine_LIBRARIES} monitors ${catkin_LIBRARIES} pthread)


add_executable(face_detect_node 
   src/face_detect.cpp
)
target_link_libraries(face_detect_node ${OpenCV_LIBRARIES} ${InferenceEngine_LIBRARIES} monitors ${catkin_LIBRARIES} pthread ngraph::ngraph)

add_executable(obj_node 
   src/obj.cpp
)
target_link_libraries(obj_node ${OpenCV_LIBRARIES} ${InferenceEngine_LIBRARIES} monitors ${catkin_LIBRARIES} pthread ngraph::ngraph)

add_executable(color_node 
   src/color.cpp
)
target_link_libraries(color_node ${OpenCV_LIBRARIES} ${InferenceEngine_LIBRARIES} monitors ${catkin_LIBRARIES} pthread ngraph::ngraph)
add_executable(pin_node 
   src/pin.cpp
)
target_link_libraries(pin_node ${OpenCV_LIBRARIES} ${InferenceEngine_LIBRARIES} monitors ${catkin_LIBRARIES} pthread ngraph::ngraph)

add_executable(safe_node 
   src/safe.cpp
)
target_link_libraries(safe_node ${OpenCV_LIBRARIES} ${InferenceEngine_LIBRARIES} monitors ${catkin_LIBRARIES} pthread ngraph::ngraph)

