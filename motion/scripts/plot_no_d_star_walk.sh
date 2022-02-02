#!/bin/sh
cd
gnome-terminal --tab -- /bin/bash -c "source ~/catkin_ws/devel/setup.bash;rosrun motion plot_no_d_star_walk.py;bash"

exit 0