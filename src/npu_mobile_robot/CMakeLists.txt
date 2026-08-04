cmake_minimum_required(VERSION 3.8)
project(npu_mobile_robot)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch maps
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(slam_launch test/test_slam_launch.py TIMEOUT 600)
  ament_add_pytest_test(amcl_nav2_launch test/test_amcl_nav2_launch.py TIMEOUT 900)
  set_tests_properties(slam_launch amcl_nav2_launch PROPERTIES RUN_SERIAL TRUE)
endif()

ament_package()
