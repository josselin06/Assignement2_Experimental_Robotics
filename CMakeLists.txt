cmake_minimum_required(VERSION 3.8)
project(my_opencv)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)


find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Assignment2 / PlanSys2 / Nav2
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)


find_package(lifecycle_msgs REQUIRED)

find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# ---- Existing executables ----
add_executable(image_subscriber src/subscriber.cpp)
ament_target_dependencies(image_subscriber rclcpp sensor_msgs cv_bridge OpenCV image_transport)

add_executable(it_subscriber src/subscriber_it.cpp)
ament_target_dependencies(it_subscriber rclcpp sensor_msgs cv_bridge OpenCV image_transport)

add_executable(it_modifier src/modifier_it.cpp)
ament_target_dependencies(it_modifier rclcpp sensor_msgs cv_bridge OpenCV image_transport)

add_executable(aruco_explorer src/aruco_explorer.cpp)
target_include_directories(aruco_explorer PRIVATE ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(aruco_explorer
  rclcpp sensor_msgs geometry_msgs cv_bridge OpenCV image_transport
  tf2 tf2_ros tf2_geometry_msgs
)


add_executable(explore_action src/explore_action.cpp)
target_include_directories(explore_action PRIVATE ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(explore_action
  rclcpp rclcpp_action
  plansys2_executor
  nav2_msgs
  sensor_msgs cv_bridge OpenCV image_transport nav_msgs lifecycle_msgs tf2_ros tf2_geometry_msgs geometry_msgs
)

add_executable(take_picture_next_action src/take_picture_next_action.cpp)
target_include_directories(take_picture_next_action PRIVATE ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(take_picture_next_action
  rclcpp
  rclcpp_action         
  nav2_msgs              
  lifecycle_msgs         
  plansys2_executor
  geometry_msgs sensor_msgs std_msgs cv_bridge OpenCV image_transport
)

install(TARGETS
  image_subscriber
  it_subscriber
  it_modifier
  aruco_explorer
  explore_action
  take_picture_next_action
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and pddl folders (so get_package_share_directory works)
install(DIRECTORY launch pddl
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
