cmake_minimum_required(VERSION 2.8.3)
project(tenacity_gazebo)

find_package(catkin REQUIRED)

catkin_package(
    DEPENDS
        controller_manager
        tenacity_description
        curio_control
        gazebo_ros
        gazebo_ros_control
        joint_state_controller
        robot_state_publisher
        rqt_robot_steering
)

install(DIRECTORY launch models worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
