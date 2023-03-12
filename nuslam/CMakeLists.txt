cmake_minimum_required(VERSION 3.8)
project(nuslam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nuturtlebot_msgs REQUIRED)
find_package(turtlelib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Armadillo REQUIRED)

# add slam.cpp executable
add_executable(slam src/slam.cpp)
ament_target_dependencies(slam
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  tf2
  tf2_ros
  nav_msgs
  sensor_msgs
  nuturtlebot_msgs
  visualization_msgs
)

# add landmarks.cpp executable
add_executable(landmarks src/landmarks.cpp)
ament_target_dependencies(landmarks
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  tf2
  tf2_ros
  nav_msgs
  sensor_msgs
  nuturtlebot_msgs
  visualization_msgs
)

# include armadillo library headers
include_directories(${ARMADILLO_INCLUDE_DIRS})

# link armadillo and turtlelib
target_link_libraries(slam
  turtlelib::turtlelib
  ${${ARMADILLO_LIBRARIES}}
)
target_link_libraries(landmarks
  turtlelib::turtlelib
  ${${ARMADILLO_LIBRARIES}}
)

# install custom service definitions
rosidl_generate_interfaces(
  ${PROJECT_NAME}_srv
  "srv/Control.srv"
  "srv/InitialPose.srv"
  LIBRARY_NAME ${PROJECT_NAME}
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}_srv "rosidl_typesupport_cpp")
target_link_libraries(slam "${cpp_typesupport_target}")


# install nodes
install(TARGETS
  slam
  landmarks
  DESTINATION lib/${PROJECT_NAME}
)

# install launchfiles
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# install configuration files
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)


if(NOT CMAKE_CROSSCOMPILING)

find_package(Doxygen)
set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md) # Use the readme in your doxygen docs
doxygen_add_docs(doxygen src/ README.md ALL)

endif()



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)

  find_package(Catch2 3 REQUIRED)
  enable_testing()
  add_executable(nuslam_test tests/tests.cpp)
  target_link_libraries(nuslam_test Catch2::Catch2WithMain)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
