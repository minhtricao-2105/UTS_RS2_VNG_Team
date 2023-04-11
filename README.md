# University of Technology Sydney -Robotics Studio 2 - Robothon Team 

## Description of the project
The problem statement for the Robothon Competition is centred around the issue of e-waste, which poses a significant threat to the environment and human health due to hazardous materials like lead, mercury, and cadmium. With the increased usage of electronic devices worldwide, it has become crucial to find an efficient and safe way to disassemble e-waste. Therefore, the competition aims to develop autonomous robotic solutions to handle e-waste more sustainably and efficiently, promoting ecologically responsible e-waste management practices

The Technical Statement of the Robothon Competition is to develop an autonomous robotic solution for handling electronic waste and make the disassembly and sorting of electronic waste more efficient and sustainable through automated recycling. The increase in electronic waste is a significant environmental concern due to hazardous materials, which can endanger human health and the planet's well-being. Robot platforms are necessary to ensure workers' safety and develop innovative and sustainable solutions for dealing with increased electronic waste.

## Contents of the project

This project will be divided into three main programs:

1. Controlling the UR3e Robot by using the Robotic Toolbox for Python. (Please find in RTB -P Test Files)
2. Communicate between the Robotic Toolbox and ROS using action-client command. (Please find in ActionClientFeedback)
3. Computer Vision using the OpenCV Library and the Intel D435 RealSense Camera. (Please find in Realsense and ComputerVision)

## Installation

To use this project, first of all, you need to install the Universal Driver. Here is step by step to install the Universal Driver:

## Universal_Robots_ROS_Driver
### Requirement
This driver requires a system setup with ROS. It is recommended to use Ubuntu 18.04 with ROS melodic, however using Ubuntu 20.04 with ROS noetic should also work.

To make sure that robot control isn't affected by system latencies, it is highly recommended to use a real-time kernel with the system. See the real-time setup guide on information how to set this up.

### How to install:
```
# source global ros
$ source /opt/ros/<your_ros_version>/setup.bash

# create a catkin workspace (if you already have skip this step and go strathforward to your workplace)
$ mkdir -p catkin_ws/src && cd catkin_ws

# clone the driver
$ git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver

# clone the description. Currently, it is necessary to use the melodic-devel branch.
$ git clone -b melodic-devel https://github.com/ros-industrial/universal_robot.git src/universal_robot

# install dependencies
$ sudo apt update -qq
$ rosdep update
$ rosdep install --from-paths src --ignore-src -y

# build the workspace
$ catkin_make

# activate the workspace (ie: source it)
$ source devel/setup.bash 
```

### Setting up a UR robot for ur_robot_driver

Extract calibration information

Each UR robot is calibrated inside the factory giving exact forward and inverse kinematics. To also make use of this in ROS, you first have to extract the calibration information from the robot.

Though this step is not necessary to control the robot using this driver, it is highly recommended to do so, as otherwise endeffector positions might be off in the magnitude of centimeters.

For this, there exists a helper script:

```
$ roslaunch ur_calibration calibration_correction.launch \
  robot_ip:=<robot_ip> target_filename:="${HOME}/my_robot_calibration.yaml" 
```
For the parameter robot_ip insert the IP address on which the ROS pc can reach the robot. As target_filename provide an absolute path where the result will be saved to.

#### Quick Start

1. Open the terminal and launch the driver:
``` 
$ roslaunch ur_robot_driver <robot_type>_bringup.launch robot_ip:=192.168.56.101
``` 
where <robot_type> is one of ur3, ur5, ur10, ur3e, ur5e, ur10e, ur16e. Note that in this example we load the calibration parameters for the robot "ur10_example".

2. On the robot controller, select Program -> external control -> create a program and launch the program on the robot

Note: please make sure that the external control IP should be the HOST IP (YOUR COMPUTER IP)

For more information and additional package, please find it in here: https://github.com/UniversalRobots/Universal_Robots_ROS_Driver#readme

## Universal_Robot_Driver Moveit

### Description:
There are two different ways to install the packages in this repository. The following sections detail installing the packages using the binary distribution and building them from source in a Catkin workspace.

### First Way: Using apt (Ubuntu, Debian)
On supported Linux distributions (Ubuntu, 18.04 (Bionic) and 20.04 (Focal), i386 and amd64) and ROS versions:
```
sudo apt-get install ros-$ROS_DISTRO-universal-robots
```
replace $ROS_DISTRO with melodic or noetic, depending on which ROS version you have installed.

### Second Way: Building from Source

The following instructions assume that a Catkin workspace has been created at $HOME/catkin_ws and that the source space is at $HOME/catkin_ws/src. Update paths appropriately if they are different on the build machine.

In all other cases the packages will have to be build from sources in a Catkin workspace:
```
$ cd $HOME/catkin_ws/src

# retrieve the sources (replace '$ROS_DISTRO' with the ROS version you are using)
$ git clone -b $ROS_DISTRO-devel https://github.com/ros-industrial/universal_robot.git
$ cd $HOME/catkin_ws

# checking dependencies (again: replace '$ROS_DISTRO' with the ROS version you are using)
$ rosdep update
$ rosdep install --rosdistro $ROS_DISTRO --ignore-src --from-paths src

# building
catkin_make

# activate this workspace
source $HOME/catkin_ws/devel/setup.bash
```
### Usage with REAL HARDWARD:

For setting up the MoveIt! nodes to allow motion planning run e.g.:
```
roslaunch ur5_moveit_config moveit_planning_execution.launch
```

For starting up RViz with a configuration including the MoveIt! Motion Planning plugin run:
```
roslaunch ur5_moveit_config moveit_rviz.launch
```

Note: replace ur5 to your appropriate robot platform(ur3, ur3e, ur10,...)

### Usage with Gazebo Simulation

To bring up the simulated robot in Gazebo, run:

```
roslaunch ur_gazebo ur5_bringup.launch
```

MoveIt! with a simulated robot Again, you can use MoveIt! to control the simulated robot.

For setting up the MoveIt! nodes to allow motion planning run:

```
roslaunch ur5_moveit_config moveit_planning_execution.launch sim:=true
```

For starting up RViz with a configuration including the MoveIt! Motion Planning plugin run:

```
roslaunch ur5_moveit_config moveit_rviz.launch
```

## OnRobot_RG2_Gripper Driver

This package was written by Harada Laboratory from Osaka University, if you have some question, please ask them and here is the link of their github: https://github.com/Osaka-University-Harada-Laboratory/onrobot

### Dependency
+ pymodbus==2.5.3
+ roboticsgroup/roboticsgroup_upatras_gazebo_plugins

Note: the pymodbus version is really important. please make sure you have a correct version.

### Installation
```
cd catkin_ws/src
git clone https://github.com/takuya-ki/onrobot.git --depth 1
git clone https://github.com/roboticsgroup/roboticsgroup_upatras_gazebo_plugins.git --depth 1
cd ../
sudo rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro noetic -y --os=ubuntu:focal -y
sudo apt install ros-noetic-ros-control ros-noetic-ros-controllers
catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3
```

### Usage: send motion command:
Interactive mode
```
roslaunch onrobot_rg_control bringup.launch gripper:=[rg2/rg6] ip:=XXX.XXX.XXX.XXX
rosrun onrobot_rg_control OnRobotRGSimpleController.py
```

ROS service call:
```
roslaunch onrobot_rg_control bringup.launch gripper:=[rg2/rg6] ip:=XXX.XXX.XXX.XXX
rosrun onrobot_rg_control OnRobotRGSimpleControllerServer.py
rosservice call /onrobot_rg/set_command c
rosservice call /onrobot_rg/set_command o
rosservice call /onrobot_rg/set_command '!!str 300'
rosservice call /onrobot_rg/restart_power
```
### Simulation

#### Display models
```
roslaunch onrobot_rg_description disp_rg6_model.launch
roslaunch onrobot_rg_description disp_rg2_model.launch
```
#### Gazebo simulation
```
roslaunch onrobot_rg_gazebo bringup_rg6_gazebo.launch
rostopic pub -1 /onrobot_rg6/joint_position_controller/command std_msgs/Float64 "data: 0.5"
roslaunch onrobot_rg_gazebo bringup_rg2_gazebo.launch
rostopic pub -1 /onrobot_rg2/joint_position_controller/command std_msgs/Float64 "data: 0.5"
```
## Peter Corke Robotics ToolBox for Python
### Installation
```
pip3 install roboticstoolbox-python
```
Available options are:
collision install collision checking with pybullet
Put the options in a comma separated list like
```
pip3 install roboticstoolbox-python[optionlist]
```

Install Swift for simulate the cobot with the toolbox:
```
git clone https://github.com/petercorke/robotics-toolbox-python.git
cd robotics-toolbox-python
pip3 install -e .
```

### Example

Example and tutorial how to use the toolbox can be found in here: https://github.com/petercorke/robotics-toolbox-python
