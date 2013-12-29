cmake_minimum_required(VERSION 2.8.3)
project(dm_lizi)

find_package(catkin REQUIRED COMPONENTS
  decision_making
  decision_making_parser
  geometry_msgs
  lizi_description
  sensor_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(lizi_wandering 
	src/Wandering.cpp 
	src/WanderingNode.cpp
)

target_link_libraries(lizi_wandering ${catkin_LIBRARIES})

decision_making_parsing(
	src/Wandering.cpp
)

#############
## Install ##
#############


install(TARGETS lizi_wandering
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)