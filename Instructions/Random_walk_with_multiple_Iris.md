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
add_executable(random_walks src/random_walks.cpp)
target_link_libraries(random_walks ${catkin_LIBRARIES})
#add_dependencies(random_walks  motion_generate_messages_cpp)
```

## Add and compile rand_walks.cpp file to your motion package:

* Take [random_walks.cpp](https://github.com/dimitra-savvani/ROS_multiple_iris/blob/main/motion/src/random_walks.cpp) and place it under src/motion/src of your catkin workspace

* On  a terminal run
```
cd ~/catkin_ws
catkin build
```

## Execute random walks with Iris drones on Gazebo

Place the [scripts](https://github.com/dimitra-savvani/ROS_multiple_iris/tree/main/motion/scripts) folder in the motion folder. 

Don' t forget to use the chmod command to make the scripts executable, e.g. $ chmod +x run_script.sh


On a terminal:

```
cd ~/catkin_ws/src/motion/scripts
bash mul_launch1.sh
```

On a second terminal:

```
cd ~/catkin_ws/src/motion/scripts
bash mul_launch2.sh
```

On a third terminal:

* To run random_walks node for a specific drone:

```
source ~/catkin_ws/devel/setup.bash
rosrun motion random_walks <number>
```
number could be any integer from 0 to 3.

* Alternatively you can run random_walks node for all drones simultaneously, with one command: 
```
source ~/catkin_ws/devel/setup.bash
roslaunch motion random_walks.launch 
```
For that mace sure to place the [launch](https://github.com/dimitra-savvani/ROS_multiple_iris/tree/main/motion/launch) folder in the motion folder first. 

*If you want to fly less drones you have to open rand_walks.sh script and comment out unwanted drones.

## plot random walks for drone 0 to 3

* Plot node plots dynamically a drone 's path and its goal locations, so it is a prerequisite to have one or more drones executing random_walks node. 
