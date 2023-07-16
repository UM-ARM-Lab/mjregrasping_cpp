cmake_minimum_required(VERSION 3.0.2)
project(mjregrasping_cpp)

set(CMAKE_CXX_STANDARD 20)
set(CATKIN_DEPENDS
  arc_utilities
  moveit_core
  moveit_visual_tools
  pybind11_catkin
  sdf_tools
  roscpp
)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPENDS})

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${CATKIN_DEPENDS}
)

set(FLAGS -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Werror=return-type -Werror=missing-field-initializers -Wno-unknown-pragmas)

include_directories(include)

add_library(mjregrasping_cpp src/first_order_homotopy.cpp)
target_link_libraries(mjregrasping_cpp PUBLIC ${catkin_LIBRARIES})
target_include_directories(mjregrasping_cpp PUBLIC ${catkin_INCLUDE_DIRS})

add_executable(planning src/planning.cpp)
target_link_libraries(planning PUBLIC ${catkin_LIBRARIES})
target_include_directories(planning PUBLIC ${catkin_INCLUDE_DIRS})

pybind_add_module(pymjregrasping_cpp src/bindings.cpp)
target_link_libraries(pymjregrasping_cpp PUBLIC mjregrasping_cpp)
