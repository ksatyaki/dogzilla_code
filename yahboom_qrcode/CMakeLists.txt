cmake_minimum_required(VERSION 3.5)
project(yahboom_qrcode)
set(OpenCV_DIR "/home/pi/opencv_install/lib/cmake/opencv4")

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
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_color_lab REQUIRED)
find_package(image_transport REQUIRED)


include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(yahboom_qrcode_node src/yahboom_qrcode.cpp)
target_link_libraries(yahboom_qrcode_node ${OpenCV_LIBRARIES})


ament_target_dependencies(yahboom_qrcode_node rclcpp std_msgs cv_bridge OpenCV sensor_msgs image_color_lab)



#ament_export_dependencies(rosidl_default_generators)
#ament_export_dependencies(rosidl_default_runtime)


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

#rosidl_generate_interfaces(${PROJECT_NAME} "msg/ObjString.msg")
install(TARGETS
        yahboom_qrcode_node
        DESTINATION lib/${PROJECT_NAME})


ament_package()

