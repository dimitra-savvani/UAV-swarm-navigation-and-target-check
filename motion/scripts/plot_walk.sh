#!/bin/sh
cd
gnome-terminal --tab -- /bin/bash -c "source ~/catkin_ws/devel/setup.bash;rosrun motion plot_walk.py;bash"

exit 0