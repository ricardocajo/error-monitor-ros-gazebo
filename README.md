# Simulation Monitor Compiler
 This repository provides a convenient DSL (Domain Specific Language) for specifying robots expected behaviour, in the **_/dsl/_** directory are the respective implementation and compiler to monitor these behaviours while in a simulation.

 The **_/ros_func_lib/_** directory is an implementation of a ROS (Robot Operating System) package, that functions as a library required for the output code of the compiler to run properly in ROS.

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

- The directory **_/dsl/_** is where all the files for the DSL operators, compiler, etc.. are gathered:
**_tokens.py_** has all the available tokens that will be used to build the lexer of the language in ***language_lex.py***. ***language_yacc.py*** has the grammar specification of the language. ***verify_language.py*** has the implementation of a function to verify the syntax of the code. ***compile_language.py*** has the implementation of the compiler. **_utils.py_** are usefull classes for the function of the whole DSL.
- The directory **_/ros_func_lib/_** is where all the files for implementing ROS package library for the monitoring are gathered:
**_CMakeLists.txt_** and **_package.xml_** are used to define the ROS package. **_setup.py_** allows this package to be used by other ROS packages. ***/src/ros_func_lib/*** directory is where the library functions are implemented (it needs to be in this directory so the functions can be properly imported by other ROS packages).
***predicates_func.py*** acts like an API of available functions that will be used by the compiler. ***utils_func.py*** are utility functions that are used by ***predicates_func.py***. **_config.py_** is a configuration file where the used simulator, among others, are specified.

## Language

### Operators
always X (X has to hold on the entire subsequent path)

eventually X (X eventually has to hold, somewhere on the subsequent path)

X until Y (Y holds at the current or future position, and X has to hold until that position. At that position X does not have to hold any more)

X implies Y

not X

X and Y | X or Y

X == Y | X != Y

X > Y | X >= Y | X < Y | X <= Y

X + Y | X - Y | X * Y | X / Y

### Usefull Predicates
position_x X (The position in the x axis of the robot in the simulation)

position_y X (The position in the y axis of the robot in the simulation)

distance X Y (The absolute distance between two objects in the simulation)

orientation X (The orientation of a object in the simulation)

velocity X (The velocity of a object in the simulation)

localization_error X (The difference between to robot perception of its position and the actual position in the simulation)

### Examples

#### A robot always stops at the stop sign:
always (eventually ((distance robot1 stop_sign1) < 2) and ((orientation robot1) - (orientation stop_sign1) < 90) implies eventually ((velocity robot1) <= 0))

#### The localization error of the robot is always below a certain value:
always ((localization_error robot1) < 0.2)

#### Robot1 is never at more than a certain distance from robot2
always ((distance robot1 robot2) < X)

#### A robot never makes a rotation of more than X degrees in a period of time (maybe usefull for an airplane/drone simulation)
How would i express the period of time? And how would i even monitor this

#### A robot's wheels never turn more than 90 degrees:
The problem i sent an email, if i write something like left_wheel_orientation how will my compiler internally know which topic to search for? (it depends on the robot and can change dynamically)

In the case that the topic name doesn't change dynamically, i can have a robot_config_file that the user fills before running so the compiler has this type of information 

Even if i could have this information how would i be able to use it? different robots will have different Message structures
