# Mobile Robotic Base

Repository for the description and gazebo simulation of a Mobile Robotic Base, a project developed for the completion of the WPI Major Qualifying Project requirement.

## Installation

This simulation was built using ROS-Noetic. 

The mobile base also requires ros-kortex for simulating the robotic arm. Follow the installation instructions [here](https://github.com/Kinovarobotics/ros_kortex/tree/d53b135d3741bb265bb6908f59600fa03dbc2dc9).

## Usage

This simulation is still in early development. To run the simulation with both the base and arm, launch the following command in a terminal:

`roslaunch mobile_base_simulation arm_and_base.launch`

### NOTE

To get the kortex arm to move in the simulation, `kortex_arm/ros_kortex/kortex_description/robots/kortex_robot.xacro` has a line that links the arm to the world. Comment out lines 25-32 in that file (`<xacro:if value="${sim}">...`).

Also, to get the insert's position controllers to work, comment out lines 64-68 in the above file (`<gazebo> <plugin name=...`).