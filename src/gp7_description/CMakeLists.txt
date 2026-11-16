cmake_minimum_required(VERSION 3.8)

project(gp7_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch meshes rviz urdf worlds scripts
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/pick_and_place.py
  DESTINATION lib/${PROJECT_NAME})

install(FILES model.config
  DESTINATION share/${PROJECT_NAME})

ament_package()
