cmake_minimum_required(VERSION 3.8)
project(simulation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch worlds config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/world_markers.py scripts/esp32_sim_bridge.py DESTINATION lib/${PROJECT_NAME})

ament_package()
