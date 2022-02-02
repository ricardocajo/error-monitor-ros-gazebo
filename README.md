# Simulation Monitor Compiler
 This repository is a ROS (Robot Operating System) package that provides a convenient DSL (Domain Specific Language) for specifying robots expected behaviour, as well as the respective compiler to monitor these behaviours while in a simulation.


## Table of content
* [Installs](#installs)
* [Structure and Files](#structure-and-files)
* [Language](#language)


## Installs
This project was made using Ubuntu 20.04 in a virtual environment. For this reason specific versions of the following software were installed. You might want to adjust it to your operating system, versions or preferences.

In this project Gazebo was chosen as the simulation software, while it is possible to use some other simulation software, it might still need some type of implementation to be used within this project.

### ROS
To install ROS follow the link [ros_install](http://wiki.ros.org/ROS/Installation) and have in mind your own specifications. While making this project ROS noetic was used.


To create a ROS workspace in your computer to be able to run ROS projects follow the link [ros_workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

### Gazebo
To install Gazebo in Ubuntu through the command line follow the link [gazebo_install](http://gazebosim.org/tutorials?tut=install_ubuntu). For any other method of installation follow the documentation on the official site [Gazebo](http://gazebosim.org/)


## Structure and Files
Being this repository a ROS package, the files **_CMakeLists.txt_** and **_package.xml_** are automatically created.

For this package to be imported and used by other ROS packages, the file **_setup.py_** exists as well as the directory path **_src/package_name_** where are the files which can be imported.
 
In this path **_src/sim_monitor_compiler_** there is the file **_utils.py_** which has functions that will be helpful for the compiler of the DSL, the other files in this directory are auxiliary functions as well as configuration files for **_utils.py_** to work properly.

Inside this path there is also the directory **_dsl/_** where all the files for the DSL are gathered.


## Language

### Operators
eventually X (X eventually has to hold, somewhere on the subsequent path)

always X (X has to hold on the entire subsequent path)

X until Y (Y holds at the current or future position, and X has to hold until that position. At that position X does not have to hold any more)

X implies Y

X == Y | X != Y

X > Y | X >= Y | X < Y | X <= Y

X + Y | X - Y | X * Y | X / Y

### Usefull Predicates
distance X Y (The absolute distance between two objects in the simulation) 

localization_error X (The difference between to robot perception of its position and the actual position in the simulation)
