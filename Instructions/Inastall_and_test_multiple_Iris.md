# Install packages

## Install mavros

* Prerequisites
```
sudo apt-get install python-catkin-tools python-rosinstall-generator -y
```

* Create a catkin workspace for source installation
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
wstool init sr
````

* If this is your first time using wstool you will need to initialize your source space with:
```
wstool init ~/catkin_ws/src
```

* Install MAVLink:
```
# We use the Kinetic reference for all ROS distros as it's not distro-specific and up to date
rosinstall_generator --rosdistro kinetic mavlink | tee /tmp/mavros.rosinstall
```

* Install MAVROS from source using released/stable version
```
rosinstall_generator --upstream mavros | tee -a /tmp/mavros.rosinstall
```

* Create workspace & deps
```
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src -j4
rosdep install --from-paths src --ignore-src -y
```

* Install [GeographicLib](https://geographiclib.sourceforge.io) datasets:
```
# It might need sudo
./src/mavros/mavros/scripts/install_geographiclib_datasets.sh
```

* Build source
```
cd ~/catkin_ws
catkin build
```

* Make sure that you use setup.bash or setup.zsh from workspace
```
# Needed or rosrun can't find nodes from this workspace.
source devel/setup.bash
```
In the case of error, there are addition installation and troubleshooting notes in the [mavros repo](https://github.com/mavlink/mavros/tree/master/mavros#installation).

## Install PX4 firmware

```
mkdir -p ~/src
cd ~/src
git clone https://github.com/PX4/Firmware.git
cd Firmware
git submodule update --init --recursive
make px4_sitl_default # it may ask for some installations using pip
make px4_sitl_default gazebo
make px4_sitl_default gazebo_rover
```
* it might need gstrem installation:
```
sudo apt-get install libgstreamer1.0-dev
```

* to list the possible simulations:
```
make px4_sitl list_vmd_make_targets
```
## Run simulation for one Iris drone

Simulate iris drone with mavros and px4

Terminal 1:
```
cd ~/src/Firmware
source ~/catkin_ws/devel/setup.bash
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

Terminal 2:
```
cd ~/src/Firmware
DONT_RUN=1 make px4_sitl_default gazebo
source ~/catkin_ws/devel/setup.bash
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo

roslaunch px4 posix_sitl.launch
```
(Alternatively, for commands on terminal 1 and 2)

Download or move (if you have already downloaded) the [script](https://github.com/dimitra-savvani/ROS_multiple_iris/tree/main/motion/script) folder, and place it in the motion folder.

Use the chmod command to make the scripts executable, e.g. $ chmod a+x run_script.sh

Now on terminal 1:
```
cd ~/catkin_ws/src/motion/script
bash one_launch1.sh
```
(the first time you run `one_launch2.sh`, uncomment `#DONT_RUN=1 make px4_sitl_default gazebo`, then comment it out again for the future runs)

And on terminal 2:
```
cd ~/catkin_ws/src/motion/script
bash one_launch2.sh
```
Continue normally on Terminal 3

Terminal 3:
```
cd
source ~/catkin_ws/devel/setup.bash
rosrun mavros mavcmd takeoffcur 0.1 0.1 0.2
rosrun mavros mavsafety arm
```

Kill all terminals

## Modifications to run multiple Iris drones

* Download [files_for_mul_iris](https://github.com/dimitra-savvani/ROS_multiple_iris/tree/main/files_for_mul_Iris)

* Paste .launch files under launch folder (which is under src/Firmware folder)

*To control how many drones you want to be launched, when you launch multiple drones, comment or uncomment coresponding code lines on add_sdf.launch (if you need more than 10 drones, you have to add some sdf files as well.)

* On a terminal:
```
cd ~/.ignition/fuel/
sudo nano config.yaml
```
replace url line with →  url: https://api.ignitionrobotics.org

* Paste sdf files (from sfds folder) under src/Firmware/Tools/sitl_gazebo/models/iris/

* Run the following lines, in seperate terminals, to launch multiple drones

Terminal 1:
```
cd ~/src/Firmware;source ~/catkin_ws/devel/setup.bash;source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default;export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd);export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo;roslaunch px4 simulation.launch
```

Terminal 2:
```
cd ~/src/Firmware;source ~/catkin_ws/devel/setup.bash;source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default;export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd);export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo;roslaunch px4 add_sdf.launch

```
(Alternatively, for commands on terminal 1 and 2)

Asuming that you have downloaded and made executable the scripts from [script](https://github.com/dimitra-savvani/ROS_multiple_iris/tree/main/motion/script) folder as described above.

On Terminal 1:
```
cd ~/catkin_ws/src/motion/script
bash mul_launch1.sh
```

On Terminal 2:
```
cd ~/catkin_ws/src/motion/script
bash mul_launch2.sh
```
