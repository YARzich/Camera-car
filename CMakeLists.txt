cmake_minimum_required(VERSION 3.8)
project(camera_car_simple)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)


#controller_converter
add_executable(controller_converter src/controller_converter.cpp)
ament_target_dependencies(
        controller_converter
        rclcpp
        geometry_msgs
        sensor_msgs
        std_msgs
)

install(TARGETS controller_converter
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})


ament_package()
foreach(dir . description/meshes description/meshes/stl description)
  install(DIRECTORY ${dir}/
          DESTINATION share/${PROJECT_NAME}/${dir})
endforeach(dir)

