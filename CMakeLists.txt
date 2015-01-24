cmake_minimum_required(VERSION 2.8.3)
project(moveit_benchmarks)

set(MOVEIT_LIB_NAME moveit_benchmarks)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost COMPONENTS program_options REQUIRED)

find_package(catkin COMPONENTS
  moveit_ros_planning
  moveit_ros_warehouse
  roscpp
  REQUIRED)

# catkin_package defines the env vars used in install later
catkin_package(
   LIBRARIES ${MOVEIT_LIB_NAME}
   CATKIN_DEPENDS moveit_ros_planning moveit_ros_warehouse roscpp
   DEPENDS
   INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/include
)

include_directories(${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_LIST_DIR}/include)
link_directories(${catkin_LIBRARY_DIRS})

add_library(${MOVEIT_LIB_NAME} src/BenchmarkOptions.cpp
                               src/BenchmarkServer.cpp)
target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(
  TARGETS
    ${MOVEIT_LIB_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

