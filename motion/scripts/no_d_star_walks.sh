#!/bin/sh
cd
source ~/catkin_ws/devel/setup.bash
gnome-terminal --tab -- /bin/bash -c "source ~/catkin_ws/devel/setup.bash;rosrun motion no_d_star_walks 0;bash"
gnome-terminal --tab -- /bin/bash -c "source ~/catkin_ws/devel/setup.bash;rosrun motion no_d_star_walks 1;bash"
gnome-terminal --tab -- /bin/bash -c "source ~/catkin_ws/devel/setup.bash;rosrun motion no_d_star_walks 2;bash"
gnome-terminal --tab -- /bin/bash -c "source ~/catkin_ws/devel/setup.bash;rosrun motion no_d_star_walks 3;bash"
# gnome-terminal --tab -- /bin/bash -c "source ~/catkin_ws/devel/setup.bash;rosrun motion no_d_star_walks 4;bash"
# gnome-terminal --tab -- /bin/bash -c "source ~/catkin_ws/devel/setup.bash;rosrun motion no_d_star_walks 5;bash"
# gnome-terminal --tab -- /bin/bash -c "source ~/catkin_ws/devel/setup.bash;rosrun motion no_d_star_walks 6;bash"
# gnome-terminal --tab -- /bin/bash -c "source ~/catkin_ws/devel/setup.bash;rosrun motion no_d_star_walks 7;bash"

exit 0