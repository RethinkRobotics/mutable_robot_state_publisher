cmake_minimum_required(VERSION 2.8)
project(robot_state_publisher)

find_package(orocos_kdl REQUIRED) 
find_package(catkin REQUIRED
  COMPONENTS roscpp rosconsole rostime tf tf_conversions kdl_parser cmake_modules std_msgs
)
find_package(Eigen REQUIRED)

add_message_files(DIRECTORY msg FILES
  URDFConfiguration.msg
)

generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}_solver
  INCLUDE_DIRS include
  DEPENDS roscpp rosconsole rostime tf tf_conversions kdl_parser orocos_kdl
)

include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})

add_library(${PROJECT_NAME}_solver
  src/robot_state_publisher.cpp src/robot_kdl_tree.cpp src/robot_urdf.cpp
)
target_link_libraries(${PROJECT_NAME}_solver ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_executable(${PROJECT_NAME} src/joint_state_listener.cpp)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_solver ${orocos_kdl_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_solver ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
