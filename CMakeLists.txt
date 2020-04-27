
cmake_minimum_required(VERSION 2.8.3)
project(bkth_lidar_tracking)
add_compile_options(-std=c++11)
#SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O1 -Wall -g -ggdb")
#SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O1 -Wall")
## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  rospy 
  cv_bridge 
  image_transport 
  std_msgs 
  genmsg 
  message_generation
)

find_package(PCL REQUIRED)
find_package(VTK REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

add_message_files(FILES 
  lidar_process_data.msg 
  boatpose.msg
  xyz_msg.msg
  stereo_vision_msg.msg
  obstacal_msg.msg
)

generate_messages(DEPENDENCIES 
  std_msgs
)

catkin_package(
  
)


include_directories(include 
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIR}
)


add_executable(bkth_lidar_tracking src/bkth_lidar_tracking.cpp)

target_link_libraries(bkth_lidar_tracking 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${VTK_LIBRARIES}
)
# SET(CMAKE_CXX_FLAGS "-std=c++11 -O3")
