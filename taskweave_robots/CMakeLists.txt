cmake_minimum_required(VERSION 3.8)
project(taskweave_robots NONE)
find_package(ament_cmake REQUIRED)

# every folder with a pack.yaml is a robot pack: installed to share/taskweave_robots/<robot>/
file(GLOB pack_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/*/pack.yaml)
foreach(pack_file ${pack_files})
  get_filename_component(robot ${pack_file} DIRECTORY)
  install(DIRECTORY ${robot} DESTINATION share/${PROJECT_NAME})
endforeach()

ament_package()
