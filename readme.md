# MPC path follower based on ros navigation stack 

## Abstract
MPC path follower, Using ros navigation stack, Working with move_base, Not stable version    

## About us 
Developer:   
* Will, zhang  
Date: 2019.07.18
 

## Features
* Nonlinear Bicycle Model Based MPC (through ipopt solver) 
* Based on ros navigation

## References  
* https://github.com/udacity/CarND-MPC-Project
* https://github.com/Hypha-ROS/hypharos_minicar    
  

### Dependency 
1. Install ROS Kinetic (Desktop-Full) (http://wiki.ros.org/kinetic/Installation/Ubuntu)  
2. Install dependencies:  
$ sudo apt-get install remmina synaptic gimp git ros-kinetic-navigation* ros-kinetic-gmapping ros-kinetic-ackermann-msgs ros-kinetic-turtlebot*-y  
3. Install Ipopt: Please refer (https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md)  
4. create your own catkin_ws   
(http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment#Create_a_ROS_Workspace)  
5. cd catkin_ws/src  
6. git clone https://github.com/yinzixuan126/mpc_path_follower_ros   
7. cd ..  
8. catkin_make  

## Operation
### Simulation
* roslaunch turtlebot_gazebo turtlebot_world.launch
* roslaunch mpc_path_follower mpc_test.launch
* roslaunch mpc_path_follower view_navigation.launch
   
  


