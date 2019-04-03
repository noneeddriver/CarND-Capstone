#!/bin/bash
catkin_make
. devel/setup.sh
roslaunch launch/styx.launch
