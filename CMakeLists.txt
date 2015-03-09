cmake_minimum_required(VERSION 2.8.3)
project(simple_heap)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  cmake_modules
  tf
  message_generation
  std_msgs
  moveit_ros_planning_interface
  )

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include/)


SET(SOURCES ${PROJECT_SOURCE_DIR}/src/simple_heap.cpp
            ${PROJECT_SOURCE_DIR}/src/simple_heap_lib.cpp
    )

SET(HEADERS ${PROJECT_SOURCE_DIR}/include/simple_heap_lib.h
    )

catkin_package(
  DEPENDS
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp 
  LIBRARIES
  roscpp
  cmake_modules
  
)

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${ROS_LIBRARIES} ${catkin_LIBRARIES}
)
