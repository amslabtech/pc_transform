cmake_minimum_required(VERSION 2.8.3)
project(pc_transform)

add_compile_options(-std=c++14 -O2 -g -Wall)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
)
find_package(PCL)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# add_executable(tf_transform src/tf_transform.cpp)
# target_link_libraries(tf_transform
#   ${catkin_LIBRARIES}
# )

add_executable(eular_transform src/eular_transform.cpp)
target_link_libraries(eular_transform
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(rpy_transform src/rpy_transform.cpp)
target_link_libraries(rpy_transform
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
