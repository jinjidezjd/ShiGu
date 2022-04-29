cmake_minimum_required(VERSION 2.8.3)
project(shigu_gazebo)

set(DEPS
  roscpp
  sensor_msgs
)


find_package(catkin REQUIRED COMPONENTS 
  roslaunch
  sensor_msgs
  roscpp
  pcl_ros
)

generate_messages(DEPENDENCIES 
  sensor_msgs
)
include_directories(include ${catkin_INCLUDE_DIRS} )
catkin_package()

add_executable(dataset_tampstamp_transform src/dataset_tampstamp_transform.cpp)
target_link_libraries(dataset_tampstamp_transform ${catkin_LIBRARIES})

add_executable(pcl_conversion src/pcl_conversion.cpp)
target_link_libraries(pcl_conversion ${catkin_LIBRARIES})

add_executable(map_conversion src/map_conversion.cpp)
target_link_libraries(map_conversion ${catkin_LIBRARIES})

add_executable(initial_localization src/initial_localization.cpp)
target_link_libraries(initial_localization ${catkin_LIBRARIES})

roslaunch_add_file_check(launch)

install(
  DIRECTORY launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
