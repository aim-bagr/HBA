cmake_minimum_required(VERSION 3.16)
project(hba LANGUAGES CXX)

set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-O3 -pthread -fexceptions)

option(BUILD_STANDALONE "Build standalone HBA runner (pure C++, no ROS)" ON)
option(BUILD_WITH_ROS "Build legacy ROS nodes (requires catkin & ROS1)" OFF)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(GTSAM REQUIRED QUIET)
find_package(OpenMP REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

if(BUILD_STANDALONE)
  add_executable(hba_standalone source/hba_standalone.cpp)
  target_link_libraries(hba_standalone
    PRIVATE
    ${PCL_LIBRARIES}
    gtsam
    OpenMP::OpenMP_CXX
    pthread
  )
  target_compile_definitions(hba_standalone PRIVATE FULL_HESS)
  set_target_properties(hba_standalone PROPERTIES
    INSTALL_RPATH "$ORIGIN;$ORIGIN/../lib;/usr/local/lib"
    BUILD_WITH_INSTALL_RPATH TRUE
  )
  install(TARGETS hba_standalone DESTINATION bin)
endif()

if(BUILD_WITH_ROS)
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    rosbag
    std_msgs
    geometry_msgs
    visualization_msgs
    tf
  )

  catkin_package(
    CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
    DEPENDS EIGEN3 PCL
  )

  include_directories(
    ${catkin_INCLUDE_DIRS}
  )

  add_executable(hba source/hba.cpp)
  target_link_libraries(hba ${catkin_LIBRARIES} ${PCL_LIBRARIES} gtsam)

  add_executable(visualize_map source/visualize.cpp)
  target_link_libraries(visualize_map ${catkin_LIBRARIES} ${PCL_LIBRARIES})

  add_executable(calculate_MME source/calculate_MME.cpp)
  target_link_libraries(calculate_MME ${catkin_LIBRARIES} ${PCL_LIBRARIES} gtsam)
endif()