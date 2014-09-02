cmake_minimum_required(VERSION 2.8.3)
project(ed_wbc)

find_package(catkin REQUIRED COMPONENTS
  ed
  fcl
)

# find_package(Boost REQUIRED COMPONENTS system program_options)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bla
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(wbc_probe
    src/wbc_probe.cpp
)
target_link_libraries(wbc_probe ${catkin_LIBRARIES})

add_executable(wbc_probe_node
    src/wbc_probe_node.cpp
)
target_link_libraries(wbc_probe_node ${catkin_LIBRARIES})
