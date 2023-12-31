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
```shell
sudo apt-get update 
sudo apt-get install git-core python-argparse python-wstool python-vcstools python-rosdep ros-melodic-control-msgs ros-melodic-joystick-drivers ros-melodic-xacro ros-melodic-tf2-ros ros-melodic-rviz ros-melodic-cv-bridge ros-melodic-actionlib ros-melodic-actionlib-msgs ros-melodic-dynamic-reconfigure ros-melodic-trajectory-msgs ros-melodic-rospy-message-converter 
```
**Note:
Eventually, when installing dependencies, the errors log related to joystick driver dependencies will pop up. Just ignore that. The error won't be affected when you develop with Sawyer. 




