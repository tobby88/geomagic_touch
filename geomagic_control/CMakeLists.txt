cmake_minimum_required(VERSION 2.8.3)
project(tapi_geomagic_control)

# Qt
# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  rosconsole
  tf
  urdf
)

add_message_files(DIRECTORY msg
  FILES
  PhantomButtonEvent.msg
  OmniFeedback.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs message_runtime rosconsole tf urdf
)

include_directories(
#  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_executable(omni
  src/omni.cpp
)

add_dependencies(omni ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(omni
  HD
  HDU
  rt
  ncurses
  ${catkin_LIBRARIES}
)
