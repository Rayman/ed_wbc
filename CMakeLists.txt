cmake_minimum_required(VERSION 2.8.3)
project(ed_wbc)

find_package(catkin REQUIRED COMPONENTS
  ed
  fcl
  amigo_whole_body_controller
)

# find_package(Boost REQUIRED COMPONENTS system program_options)

catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES
  CATKIN_DEPENDS ed fcl
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# show the headers in qtcreator
file(GLOB HEADERS
  RELATIVE ${PROJECT_SOURCE_DIR} "include/*/*.h"
)

# ed plugin
add_library(wbc_probe
  src/wbc_probe.cpp
  src/serialization.cpp
)
target_link_libraries(wbc_probe ${catkin_LIBRARIES})

# whole body controller, configured to use the ed world model
add_executable(ed_wbc
  ${HEADERS}
  src/main.cpp
  src/edworldclient.cpp
  src/edworld.cpp
  src/ed_client.cpp
  src/serialization.cpp
)
target_link_libraries(ed_wbc ${catkin_LIBRARIES})

# test client
add_executable(wbc_probe_node
  src/wbc_probe_node.cpp
  src/serialization.cpp
)
target_link_libraries(wbc_probe_node ${catkin_LIBRARIES})

