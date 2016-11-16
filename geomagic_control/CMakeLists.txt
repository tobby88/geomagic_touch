cmake_minimum_required(VERSION 2.8.3)
project(tapi_geomagic_control)

# Qt
# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  std_msgs
  tapi_lib
  tf
  urdf
)

add_message_files(DIRECTORY msg
  FILES
  OmniFeedback.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs message_runtime tf urdf tapi_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(omni
  src/omni.cpp
)

add_dependencies(omni ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(omni
  HD
  HDU
  ${catkin_LIBRARIES}
)
