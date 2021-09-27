#!/bin/sh
cd
source ~/catkin_ws/devel/setup.bash
gnome-terminal --tab -- /bin/bash -c "source ~/catkin_ws/devel/setup.bash;rosrun motion initiator 0;bash"
gnome-terminal --tab -- /bin/bash -c "source ~/catkin_ws/devel/setup.bash;rosrun motion initiator 1;bash"
gnome-terminal --tab -- /bin/bash -c "source ~/catkin_ws/devel/setup.bash;rosrun motion initiator 2;bash"
gnome-terminal --tab -- /bin/bash -c "source ~/catkin_ws/devel/setup.bash;rosrun motion initiator 3;bash"
# gnome-terminal --tab -- /bin/bash -c "source ~/catkin_ws/devel/setup.bash;rosrun motion initiator 4;bash"
# gnome-terminal --tab -- /bin/bash -c "source ~/catkin_ws/devel/setup.bash;rosrun motion initiator 5;bash"
# gnome-terminal --tab -- /bin/bash -c "source ~/catkin_ws/devel/setup.bash;rosrun motion initiator 6;bash"
# gnome-terminal --tab -- /bin/bash -c "source ~/catkin_ws/devel/setup.bash;rosrun motion initiator 7;bash"

exit 0