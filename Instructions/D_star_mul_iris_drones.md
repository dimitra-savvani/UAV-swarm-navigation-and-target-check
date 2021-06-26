# Multiple Iris, navigate with D* algorithm, avoid each other and check if detected smoke corresponds to fire.

## Prerequisites
Follow instructions on [Inastall_and_test_multiple_Iris.md](https://github.com/dimitra-savvani/ROS_multiple_iris/blob/main/Instructions/Inastall_and_test_multiple_Iris.md)

Then follow instructions on [Random_walk_with_multiple_Iris.md](https://github.com/dimitra-savvani/ROS_multiple_iris/blob/main/Instructions/Random_walk_with_multiple_Iris.md)

## Add and compile operator.cpp file to your motion package:

At the end of the `CMakeList.txt` (which is under the src/motion folder of your catkin workspace), add the following lines:

```
add_executable(initiator src/operator.cpp)
target_link_libraries(initiator ${catkin_LIBRARIES})
#add_dependencies(initiator motion_generate_messages_cpp)
```

* Download [initiator.cpp](https://github.com/dimitra-savvani/ROS_multiple_iris/blob/main/motion/initiator.cpp) and place it under src/motion/src of your catkin workspace

* On  a terminal run
```
cd catkin_ws
catkin build
```
