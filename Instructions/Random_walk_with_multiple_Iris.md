# Launch multiple Iris, let them do  a random walk

## Prerequisites
Follow instructions on [Inastall_and_test_multiple_Iris.md](https://github.com/dimitra-savvani/ROS_multiple_iris/blob/main/Instructions/Inastall_and_test_multiple_Iris.md)

## Create a package named motion
```
cd ~/catkin_ws/src
catkin_create_pkg motion geometry_msgs mavros_msgs roscpp rospy sensor_msgs std_msgs
```
At the end of the `CMakeList.txt` (which is under the src/motion folder of your catkin workspace), add the following lines:

```
add_executable(rand_walks src/rand_walks.cpp)
target_link_libraries(rand_walks ${catkin_LIBRARIES})
#add_dependencies(rand_walks  motion_generate_messages_cpp)
```

## Add and compile rand_walks.cpp file to your motion package:

* Download [rand_walks.cpp](https://github.com/dimitra-savvani/ROS_multiple_iris/blob/main/motion/rand_walks.cpp) and place it under src/motion/src of your catkin workspace

* On  a terminal run
```
cd catkin_ws
catkin build
```

## Execute random walks with Iris drones on Gazebo

Asuming that you downloaded the [bashScript](https://github.com/dimitra-savvani/ROS_multiple_iris/tree/main/bashScripts) folder as described in [Inastall_and_test_multiple_Iris.md](https://github.com/dimitra-savvani/ROS_multiple_iris/blob/main/Instructions/Inastall_and_test_multiple_Iris.md).


On a terminal:

```
cd bashScripts
bash mul_launch1.sh
```

On a second terminal:

```
cd bashScripts
bash mul_launch2.sh
```

On a third terminal:

* To run rand_walks node for a specific drone:

```
source ~/catkin_ws/devel/setup.bash
rosrun motion rand_walks <number>
```
number could be any integer from 0 to 3.

* or to run rand_walks node for all drones simultaneously: 
```
cd bashScripts
bash rand_walks.sh
```
*If you want to fly less drones you have to open rand_walks.sh script and comment out unwanted drones.
If y

## plot random walks for drone 0 to 3

* Plot node plots dynamically a drone 's path and its goal locations, so it is a prerequisite to have one or more drones executing rand_walks node. 

* Download [plot.py](https://github.com/dimitra-savvani/ROS_multiple_iris/blob/main/motion/plot.py) and place it under src/motion/src of your catkin workspace
(don 't forget to change permissions in properties in order to make it executable)

* To run plot node for a specific drone:

On a terminal run: 
```
source ~/catkin_ws/devel/setup.bash
rosrun motion plot.py <number>
```
number could be any integer from 0 to 3.

* or to run plot node for all drones simultaneously:

Asuming that you downloaded the [bashScript](https://github.com/dimitra-savvani/ROS_multiple_iris/tree/main/bashScripts) folder as described in [Inastall_and_test_multiple_Iris.md](https://github.com/dimitra-savvani/ROS_multiple_iris/blob/main/Instructions/Inastall_and_test_multiple_Iris.md).

On a terminal run: 
```
cd bashScripts
bash plots.sh
```
*If you want to plot less drones you have to open rand_walks.sh script and comment out unwanted drones
