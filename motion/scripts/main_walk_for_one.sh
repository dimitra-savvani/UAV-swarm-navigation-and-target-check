#!/bin/sh
cd
source ~/catkin_ws/devel/setup.bash
gnome-terminal --tab -- /bin/bash -c "source ~/catkin_ws/devel/setup.bash;rosrun motion main_walk_for_one.py;bash"

exit 0