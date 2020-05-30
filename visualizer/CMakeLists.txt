cmake_minimum_required(VERSION 2.8.3)
project(rviz_visualizer)

find_package(catkin REQUIRED COMPONENTS message_generation rospy shot_executer nav_msgs)
catkin_package()