cmake_minimum_required(VERSION 3.8)
project(vive_ros2)

find_package(ament_cmake REQUIRED)
find_package(ament_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
