cmake_minimum_required(VERSION 3.8)
project(depth2pcl)
set(OpenCV_DIR "/home/jugaad/Documents/VICI/OpenCV")
set(PCL_DIR "/usr/local/include/pcl-1.13/pcl")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(PCL REQUIRED COMPONENTS common visualization io)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
find_package(OpenMP REQUIRED)

include_directories($(OpenMP INCLUDE_DIRS))
link_directories($(OpenMP LIBRARY_DIRS))


include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

add_executable(depth_visualizer_node src/depth2pcl.cpp)
ament_target_dependencies(depth_visualizer_node rclcpp std_msgs rosbag2_cpp sensor_msgs ament_index_cpp rosbag2_storage OpenCV cv_bridge PCL Eigen3 OpenMP)
target_link_libraries(depth_visualizer_node ${OpenCV_LIBRARIES} ${PCL_LIBRARIES}  OpenMP::OpenMP_CXX)

install(TARGETS
  depth_visualizer_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
