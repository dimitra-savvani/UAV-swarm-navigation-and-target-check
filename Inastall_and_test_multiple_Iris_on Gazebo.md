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

Terminal 3:
```
cd
source ~/catkin_ws/devel/setup.bash
rosrun mavros mavcmd takeoffcur 0.1 0.1 0.2
rosrun mavros mavsafety arm
```
Kill all terminals

Modifications to runn multiple Iris drones

* Download [files_for_mul_iris](https://github.com/dimitra-savvani/ROS_multiple_iris/tree/main/files_for_mul_Iris)

* Paste .launch files under launch folder (which is under Firmware folder)

* On a terminal:
```
cd ~/.ignition/fuel/
sudo nano config.yaml
```
replace url line with →  url: https://api.ignitionrobotics.org

* Paste sdf files (from sfds folder) under src/Firmware/Tools/sitl_gazebo/models/iris/

* Run the following lines, in to seperate terminals, to launch multiple drones
```
cd ~/src/Firmware;source ~/catkin_ws/devel/setup.bash;source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default;export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd);export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo;roslaunch px4 simulation.launch

cd ~/src/Firmware;source ~/catkin_ws/devel/setup.bash;source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default;export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd);export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo;roslaunch px4 add_sdf.launch

```
