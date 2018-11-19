cmake_minimum_required(VERSION 2.8.3)
project(betty)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  genmsg
  actionlib_msgs
  actionlib
  tf2_ros
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(teleop src/teleop.cpp)
target_link_libraries(teleop
  ${catkin_LIBRARIES}
)

add_executable(vacuum src/vacuum.cpp)
target_link_libraries(vacuum
  ${catkin_LIBRARIES}
)

add_executable(ps3_teleop src/ps3_teleop.cpp)
target_link_libraries(ps3_teleop
  ${catkin_LIBRARIES}
)

add_executable(dj src/dj.cpp)
target_link_libraries(dj
  ${catkin_LIBRARIES}
)

add_executable(busroute src/busroute.cpp)
target_link_libraries(busroute
  ${catkin_LIBRARIES}
)

add_executable(salesman src/salesman.cpp)
target_link_libraries(salesman
  ${catkin_LIBRARIES}
)

add_executable(pure_pursuit src/pure_pursuit.cpp)
target_link_libraries(pure_pursuit
  ${catkin_LIBRARIES}
)
