#!/bin/sh
cd
gnome-terminal --tab -- /bin/bash -c "source ~/catkin_ws/devel/setup.bash;rosrun motion plot_walks_onePlot.py;bash"

exit 0