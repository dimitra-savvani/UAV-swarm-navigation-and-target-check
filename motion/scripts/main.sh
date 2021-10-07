#!/bin/sh
cd
source ~/catkin_ws/devel/setup.bash
gnome-terminal --tab -- /bin/bash -c "source ~/catkin_ws/devel/setup.bash;rosrun motion main.py 4;bash"

exit 0