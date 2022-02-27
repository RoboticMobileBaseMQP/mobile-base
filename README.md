# Mobile Robotic Base

Repository for the description and gazebo simulation of a Mobile Robotic Base, a project developed for the completion of the WPI Major Qualifying Project requirement.

## Installation

This simulation was built using ROS-Noetic. 

## Usage

This simulation is still in development. To run a simulation with both the base and arm, launch the following command in a terminal, specifying the arm argument as either "panda" or "kortex":

`roslaunch mobile_base_simulation abstract_arm_and_base.launch arm:=<arm>`


To connect to the base and run teleop, run this command: 

`roslaunch base_package base_teleop.launch`

Note that ROS will try to connect to a host called "raspberrypi" to launch some low level control nodes. You will need to add the line `raspberrypi <raspberry pi's ip address>` to `/etc/hosts` on the host computer, and setup both computers according to the [ROS network config](http://wiki.ros.org/ROS/NetworkSetup), namely ROS_MASTER_URI and ROS_HOSTNAME.