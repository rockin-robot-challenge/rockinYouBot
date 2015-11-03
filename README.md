#RoCKIn youBot Repository
This repository is based on the more extensive repository from
        
     https://github.com/mas-group/robocup-at-work

Most of the components used have been removed to make it easier for an unexperienced team to find its way around the repository.
The functionalities provided will help you to setup your youBot in less than a day and be able to navigate autonomously and grasp objects at pre-defined positions.

## Install Ubuntu
The repository and its related components have been tested under the following Ubuntu distributions:

- ROS Hydro: Ubuntu 12.04

If you do not have a Ubuntu distribution on your computer you can download it here

     http://releases.ubuntu.com/12.04.5/

## Git - Version Control
### Install Git Software
Install the Git core components and some additional GUI's for the version control:

     sudo apt-get install git-core gitg gitk

### GIT Tutorial
If you have never worked with git before, we recommend to go through the following basic git tutorial:

     http://excess.org/article/2008/07/ogre-git-tutorial/


## ROS - Robot Operating System
### Install ROS
The repository has been tested successfully with the following ROS distributions. Use the link behind a ROS distribution to get to the particular ROS installation instructions.

- ROS Hydro - http://wiki.ros.org/hydro/Installation/Ubuntu

NOTE: Do not forget to update your .bashrc!
 

### ROS Tutorials
If you have never worked with ROS before, we recommend to go through the beginner tutorials provided by ROS:

     http://wiki.ros.org/ROS/Tutorials

In order to understand at least the different core components of ROS, you have to start from tutorial 1 ("Installing and Configuring Your ROS Environment") till tutorial 7 ("Understanding ROS Services and Parameters"). 


## Set up a catkin workspace

    source /opt/ros/hydro/setup.bash
    mkdir -p ~/catkin_ws/src;
    cd ~/catkin_ws/src
    catkin_init_workspace
    cd ..
    catkin_make
    
## Clone and compile the rockinYouBot source code
First of all you have to clone the repository.

    cd ~/catkin_ws/src;
    git clone git@github.com:rockin-robot-challenge/rockinYouBot.git

Then go on with installing further external dependencies:
       
    cd ~/catkin_ws/src/rockinYouBot/
    ./repository.debs
    
    source ~/catkin_ws/devel/setup.bash

The last command should be added to the ~/.bashrc file so that it does not need to be executed everytime you open a new terminal.

And finally compile the repository:

    cd ~/catkin_ws
    catkin_make


If no errors appear everything is ready to use.

## Setting up the hardware sensors
### Hokuyo node
To be able to use the Hokuyo with the repository as easy as possible, you have to create a udev-rule to create a sym-link that always points to the correct device.
Navigate to the directory:

     cd /etc/udev/rules.d

and create a file called "47-hokuyo.rules". Open it and copy+paste the following:

     SUBSYSTEMS=="usb", KERNEL=="ttyACM0", ATTRS{manufacturer}=="Hokuyo Data Flex for USB", ATTRS{product}=="URG-Series USB Driver", MODE="0666", SYMLINK+="sensors/hokuyo"

After saving make sure everything works (plug in your Hokuyo again) and enter:

     ls -lha /dev/sensors/hokuyo

You should see something similar to:

     lrwxrwxrwx  1 root root   10 Dec  7 13:57 hokuyo_front -> ../ttyACM0

### Setting the Environment Variables
#### ROBOT variable
With the ROBOT variable you can choose which hardware configuration should be loaded when starting the robot. The following line will add the variable to your .bashrc:

     echo "export ROBOT=rockin_youBot" >> ~/.bashrc
     source ~/.bashrc

If you want to create your own robot configuration you can find the "rockin_youBot" in the directory "rockinYouBot/robots/rockin_hardware_config/". Simply copy and paste the "rockin_youBot", rename it and start changing it. The "urdf" folder is where the complete youbot is made up from single parts, eg. a platform, an arm, a laserscanner etc.... The parts are defined in the youbot_description package. If you want to add new parts, you can create a new package and simple add the parts to the robot.urdf.xacro.

#### ROBOT_ENV Variable
The ROBOT_ENV variable can be used to switch between different environments. The following line will add the variable to your .bashrc:

     echo "export ROBOT_ENV=rockin-arena" >> ~/.bashrc
     source ~/.bashrc

This environment is used for RViz as well as for Gazebo. For RViz you can find the "rockin-arena" in the directory "rockinYouBot/environments/rockin_default_env_config/", for Gazebo in "rockinYouBot/environments/rockin_gazebo_worlds/common/worlds".

## Bring up the robot and its basic components
### In Simulation
To launch the robot in a simulated environment you can use the command:

     roslaunch rockin_bringup_sim robot.launch

or specifically for the RoCKIn-Environment used in Lisboa:
	
	roslaunch rockin_gazebo rockin-2015.launch

You can find the robot.launch in "rockinYouBot/simulation/rockin_bringup_sim" and the rockin-2015.launch in "rockinYouBot/simulation/rockin_gazebo/ros/launch"
     
In a new terminal you can open the Gazebo GUI to see the environment and the robot

     rosrun gazebo_ros gzclient
     

### At the Real Robot
If you want to use the real robot run the command:

     roslaunch rockin_bringup robot.launch
     
To modify the launch file navigate to "rockinYouBot/robots/rockin_bringup".

## Visualize the robot state and sensor data
RViz works for simulation as well as for the real robot. Simply run

     rosrun rviz rviz

and you should see a robot and some lines from a laserscanner. If the window in the middle is black, try restarting RViz until you see  at least a grid in the middle window.

If you don't see any/everything, just add the "RobotModel" and "LaserScan" from the menu. Set the topics to "/scan_front" for the LaserScan and to "/base_footprint" for the GlobalOptions/FixedFrame. This should also fix an error, if you can only see a complety white youBot-RobotModel. You can save these settings, so you don't have to add them everytime you restart RViz.

## Build a map for base navigation
If we want to navigate through an environment we should first create a map of it. You can create a map in simulation and in real. If you've run the rockin_bringup_sim, the joypad-application hasn't started yet. Make sure the joypad is plugged in and configured correctly. Then run:
     
     roslaunch rockin_teleop rockin_teleop_joypad.launch

To see whats going on, add a "Map" in RViz and set its topic to "/map". If you now run:

     roslaunch rockin_2dslam 2dslam.launch
     
you should be able to see some grey areas in the RViz-map. Drive around with the joypad and see your map grow.

To save your map you have to run the:

     rosrun map_server map_saver

Depending from where you have run the last command, navigate to the same directory and copy+paste the two files map.pgm and map.yaml to the directory "rockinYouBot/environments/rockin_default_env_config/rockin-arena". From now on you should be able to use your own map of the rockin-arena. (The map should now look like the Gazebo-Simulation)

## Use autonomous navigation
### Omni-directional navigation
If you want to navigate through your own map, restart the different modules and run:

     roslaunch rockin_2dnav 2dnav.launch

With RViz you are now able to set a "2d Pose Estimate". This is used to tell the robot its initial position in the map. The "2d Nav Goal" is used to tell the youBot where to move next. If you keep the mouse button pressed, you are also able to set the orientation for the youBot.


## Manipulation
### Using MoveIt!
To create your own robot using MoveIt! have a look here:

     http://moveit.ros.org/wiki/PR2/Setup_Assistant/Quick_Start

Moving the robot to a pre-defined grasp position is simple. You can use the joypad-application to move the robot to a position where you want it to be. If you now press the L2-Button on the joypad the current joint positions of the arm will be printed to console. Keep these values in mind and navigate to "rockinYouBot/manipulation/rockin_moveit_rockinYoubot/config". Open the youbot.srdf file in an editor of your choice and go to the "group_state". Copy+paste one of these sections, change the name-Attribute of group_state and change the values of the arm_joints_X to the values you kept from the console output.

You can now use the two commands:

     roslaunch rockin_moveit_rockin_youbot move_group.launch

and in a new terminal

     rosrun moveit_commander moveit_commander_cmdline.py

In the second terminal enter following commands:
     
     use arm_1
     go candle

The arm should now move to the candle position. If you want to move it to your own pre-defined position (for example "graspObject") enter:
     
     go graspObject

To control the gripper you have to change from the group_state "arm_1" to the group_state "arm_1_gripper":
     
     use arm_1_gripper
     go open
     go close

After entering one of the "go" commands MoveIt! plans a path to this position and executes it, if it finds a solution.

## Command overview

roslaunch rockin_bringup_sim robot.launch
roslaunch rockin_bringup robot.launch
roslaunch rockin_teleop rockin_teleop_joypad.launch
roslaunch rockin_2dslam 2dslam.launch
roslaunch rockin_2dnav 2dnav.launch
roslaunch rockin_moveit_rockin_youbot move_group.launch
rosrun moveit_commander moveit_commander_cmdline.py
rosrun gazebo_ros gzclient
rosrun rviz rviz
rosrun map_server map_saver
    
