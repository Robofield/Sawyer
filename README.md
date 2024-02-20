# Sawyer Robot Workstation setup and Tutorial Walkthrough
To get started with using Sawyer robot, you could follow the instruction provided from this Readme page.  

## Workstation recommendation:
1. Software requirement:
* Ubuntu 20.04 LTS and ROS Noetic (Recommendded)
* Ubuntu 18.04 LTS and ROS Kinetic (Depricated)

For more information to install ROS and configure workspace, please refer to [ROS Tutorial](http://wiki.ros.org/ROS/Tutorials)

2. Hardware requirement:
* Intel i5 or above
* 4 GB memory or above
* Ethernet port

If any visualization (RViz) or simulation (Gazebo) is required for your application, a dedicated NVidia graphics card with proprietary NVidia drivers is recommended.

## Install Intera SDK Dependencies:
* For ROS Noetic:
```Shell
sudo apt-get update 
sudo apt-get install git-core python3-wstool python3-vcstools python3-rosdep ros-noetic-control-msgs ros-noetic-joystick-drivers ros-noetic-xacro ros-noetic-tf2-ros ros-noetic-rviz ros-noetic-cv-bridge ros-noetic-actionlib ros-noetic-actionlib-msgs ros-noetic-dynamic-reconfigure ros-noetic-trajectory-msgs ros-noetic-rospy-message-converter 
pip install argparse 
```
* For ROS Kinetic:
```Shell
sudo apt-get update 
sudo apt-get install git-core python-argparse python-wstool python-vcstools python-rosdep ros-melodic-control-msgs ros-melodic-joystick-drivers ros-melodic-xacro ros-melodic-tf2-ros ros-melodic-rviz ros-melodic-cv-bridge ros-melodic-actionlib ros-melodic-actionlib-msgs ros-melodic-dynamic-reconfigure ros-melodic-trajectory-msgs ros-melodic-rospy-message-converter 
```

## Install Intera Robot SDK
### Download the SDk for workstation
Download or clone the packages directly from GitHub
```Shell
cd ~/ros_ws/src
wstool init .
git clone https://github.com/RethinkRobotics/sawyer_robot.git
wstool merge sawyer_robot/sawyer_robot.rosinstall
wstool update
```

### Source ROS setup
```shell
source /opt/ros/XXX/setup.bash  # XXX is respective ROS version
```

### Build
```shell
cd ~/ros_ws/src
catkin_make
```

## Configure Robot Communication / ROS workspace
### Copy the intera.sh sscript
``` shell
cp ~/ros_ws/src/intera_sdk/intera.sh ~/ros_ws
```
### Customize the intera.sh script
```shell
cd ~/ros_ws
nano intera.sh
```

### Modify the Intera.sh scripts
#### Change these three fields in the initial script to match wit your configuration.

Robot hostname use the Controller Serial's Number with a '.local' suffix as default naming convension. This number is located on the back of the controller box. 
```bash
robot_hostname="XXXXXXXXXXXXX.local"
```

Then fill the field your_ip with that IP address ( most straightforward method ) or with your computer hostname

```bash
your_ip="192.168.X.XXX"
```

Finally, modify the ROS distribution your are using within the ROS workspace you setup the SDK
``` bash
# Specify ROS distribution (e.g. kinetic, indigo, hydro, etc.)
ros_version="XXX"
```

** Note:
Eventually, when installing dependencies, the errors log related to joystick driver dependencies will pop up. Just ignore that. The error won't be affected when you develop with Sawyer. 




