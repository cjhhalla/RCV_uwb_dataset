cmake_minimum_required(VERSION 3.0.2)
project(config_sensor)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

find_package(catkin REQUIRED COMPONENTS 
geometry_msgs 
roscpp 
std_msgs 
visualization_msgs)

catkin_package(CATKIN_DEPENDS geometry_msgs roscpp std_msgs visualization_msgs)

include_directories(
  ${catkin_INCLUDE_DIRS}
  )

add_executable(visual
	src/visual.cpp
	)

target_link_libraries(visual
	${catkin_LIBRARIES}
	)
  
