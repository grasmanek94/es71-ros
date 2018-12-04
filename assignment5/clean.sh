#!/bin/bash
rm -rf ~/catkin_ws/install/share/assignment5
rm -rf ~/catkin_ws/install/share/roseus/ros/assignment5
rm -rf ~/catkin_ws/install/share/common-lisp/ros/assignment5
rm -rf ~/catkin_ws/install/include/assignment5
rm -rf ~/catkin_ws/install/lib/python2.7/dist-packages/assignment5
rm -rf ~/catkin_ws/devel/share/assignment5
rm -rf ~/catkin_ws/devel/share/roseus/ros/assignment5
rm -rf ~/catkin_ws/devel/share/gennodejs/ros/assignment5
rm -rf ~/catkin_ws/devel/include/assignment5
rm -rf ~/catkin_ws/devel/lib/python2.7/dist-packages/assignment5
rm -rf ~/catkin_ws/install/share/gennodejs/ros/assignment5
rm -rf ~/catkin_ws/install/lib/pkgconfig/assignment5.pc
rm -rf ~/catkin_ws/devel/share/common-lisp/ros/assignment5
rm -rf ~/catkin_ws/devel/lib/pkgconfig/assignment5.pc
rm -rf ~/catkin_ws/devel/lib/assignment5
rm -rf ~/catkin_ws/build
catkin_make clean
catkin_make
