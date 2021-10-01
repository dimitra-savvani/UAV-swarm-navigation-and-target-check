# Multiple Iris, navigate with D* algorithm, avoid each other and check if detected smoke corresponds to fire.

## Prerequisites
Follow instructions on [Inastall_and_test_multiple_Iris.md](https://github.com/dimitra-savvani/ROS_multiple_iris/blob/main/Instructions/Inastall_and_test_multiple_Iris.md)

Then follow instructions on [Random_walk_with_multiple_Iris.md](https://github.com/dimitra-savvani/ROS_multiple_iris/blob/main/Instructions/Random_walk_with_multiple_Iris.md)

## Add and compile initiator.cpp file to your motion package:

At the end of the `CMakeList.txt` (which is under the src/motion folder of your catkin workspace), add the following lines:

```
add_executable(initiator src/operator.cpp)
target_link_libraries(initiator ${catkin_LIBRARIES})
#add_dependencies(initiator motion_generate_messages_cpp)
```
* Download [initiator.cpp](https://github.com/dimitra-savvani/ROS_multiple_iris/blob/main/motion//src/initiator.cpp) and place it under src/motion/src of your catkin workspace

* On  a terminal run
```
cd catkin_ws
catkin build
```

## Do the following to create the custom msg and srv.

* In the `CMakeList.txt` (which is under the src/motion folder of your catkin workspace), add the following lines:

1. At the end of `find_package(catkin REQUIRED COMPONENTS)`:
```
message_generation
```
2. Remove # to uncomment the following lines:
```
## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )
```
and then replace the stand in Message*.msg files with your .msg file
```
# Generate messages in the 'msg' folder
add_message_files(
  FILES
  take_off.msg
)
```
Also remove # to uncomment the following lines:
```
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )
```
And replace the placeholder Service*.srv files for our service file:
```
add_service_files(
  FILES
  next_step.srv
)
```
3. Also replace
```
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES motion
#  CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)
```
with
```
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES motion
 CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp rospy sensor_msgs std_msgs message_runtime
 DEPENDS system_lib
)
```
4. Finally replace
```
## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   mavros_msgs#   sensor_msgs#   std_msgs
# )
```

```
# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs geometry_msgs#   mavros_msgs#   sensor_msgs#   
)
```


* In the `package.xml` (which is under the src/motion folder of your catkin workspace), add the following lines:

1. At the end of <build_depend> tags:
```
<build_depend>message_generation</build_depend>
```

3. And at the end of <exec_depend> tags:
```
<exec_depend>message_runtime</exec_depend>
```
* To generate the custom messages on  a terminal run
```
cd catkin_ws
catkin build
```

## Install pygame 

Open a terminal and type:
```
sudo apt-get install python-pygame
```
## Run simulation

Download the [launch](https://github.com/dimitra-savvani/ROS_multiple_iris/tree/main/motion/launch) folder and place it in your motion folder.

On a terminal run:
```
roslaunch motion simulation.launch
```
