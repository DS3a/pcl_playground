cmake_minimum_required(VERSION 3.5)
project(traversability_layer)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)




# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(ament_cmake_python REQUIRED)

install(
  DIRECTORY scripts launch
  DESTINATION share/${PROJECT_NAME}
)


ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/find_obstacles.py scripts/pcd_open3d.py
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(pcl_test src/pcl_test.cpp)
ament_target_dependencies(pcl_test rclcpp std_msgs PCL pcl_ros pcl_conversions Eigen Boost)

add_executable(pcl_rescale src/pcl_rescale.cpp)
ament_target_dependencies(pcl_rescale rclcpp std_msgs PCL pcl_ros pcl_conversions Eigen Boost)


add_executable(pcl_filter_right src/pcl_filter_right.cpp)
ament_target_dependencies(pcl_filter_right rclcpp std_msgs PCL pcl_ros pcl_conversions Eigen Boost)

add_executable(pcl_filter src/pcl_filter.cpp)
ament_target_dependencies(pcl_filter rclcpp std_msgs PCL pcl_ros pcl_conversions Eigen Boost)

install(TARGETS
  pcl_test
  pcl_filter
  pcl_filter_right
  pcl_rescale
  DESTINATION lib/${PROJECT_NAME})

target_link_libraries(pcl_test
  ${PCL_LIBRARIES}
)

target_link_libraries(pcl_rescale
  ${PCL_LIBRARIES}
)

target_link_libraries(pcl_filter_right
  ${PCL_LIBRARIES}
)

target_link_libraries(pcl_filter
  ${PCL_LIBRARIES}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND  TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
