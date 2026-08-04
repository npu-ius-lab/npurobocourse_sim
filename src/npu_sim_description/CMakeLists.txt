cmake_minimum_required(VERSION 3.8)
project(npu_sim_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config models worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(sdf_assets test/test_sdf_assets.py)
  ament_add_pytest_test(camera_presets test/test_camera_presets.py)
  ament_add_pytest_test(gz_spawn test/test_gz_spawn.py TIMEOUT 300)
endif()

ament_package()
