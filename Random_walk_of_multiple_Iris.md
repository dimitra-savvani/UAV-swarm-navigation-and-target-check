# Launch multiple Iris, let them do  a random walk

## Prerequisites
Follow instructions on [Inastall_and_test_multiple_Iris_on Gazebo.md](https://github.com/dimitra-savvani/ROS_multiple_iris/blob/main/Inastall_and_test_multiple_Iris_on%20Gazebo.md)

## Create a package named motion
```
cd ~/catkin_ws/src
catkin_create_pkg motion geometry_msgs mavros_msgs roscpp rospy sensor_msgs std_msgs
```
At the end of the `CMakeList.txt` (which is under the src/motion folder of your catkin workspace), add the following lines:

```
add_executable(offb_node src/offb_node.cpp)
target_link_libraries(offb_node ${catkin_LIBRARIES})
```

## Add and compile rand_walks.cpp file to your motion package:

* Download [rand_walks.cpp]() and place it under src/motion/src of your catkin workspace

* On  a terminal run
```
cd catkin_ws
catkin build
```

## Execute random walks with Iris drones on Gazebo
