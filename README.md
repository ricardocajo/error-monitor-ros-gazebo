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
***language_lex.py*** gathers the regexes and tokens of the language. ***language_yacc.py*** has the grammar specification of the language. ***verify_language.py*** has the implementation of a function to verify the syntax of the code. ***compile_language.py*** has the implementation of the compiler. **_/templates/_** directory has the jinja files with code templates that are used by the compiler. **_utils.py_** are usefull classes/structures that are used by the previous files.
- The directory **_/ros_func_lib/_** is where all the files for implementing a ROS package library for the monitoring are gathered:
**_CMakeLists.txt_** and **_package.xml_** are used to define the ROS package. **_setup.py_** allows this package to be used by other ROS packages. ***/src/ros_func_lib/*** directory is where the library functions are implemented (it needs to be in this directory so the functions can be properly imported by other ROS packages).
***predicates_func.py*** acts like an API of available functions that will be used by the compiler. ***utils_func.py*** are utility functions that are used by ***predicates_func.py***. **_config.py_** is a configuration file where the used simulator, among others, are specified.

## Language

### Operators
always(X) (X has to hold on the entire subsequent path)

never(X) (X never holds on the entire subsequent path)

eventually(X) (X eventually has to hold, somewhere on the subsequent path)

after(X,Y) (after the event X is observed, Y has to hold on the entire subsequent path)

(X)until(Y) (Y holds at the current or future position, and X has to hold until that position. At that position X does not have to hold any more)

after(X,Y)until(Z) (after the event X is observed, Y has to hold on the entire subsequent path up until Z happens, at that position X does not have to hold any more)

(X)implies(Y)

not(X)

(X)and(Y) | (X)or(Y)

@{X, -y} (the value of the variable X in the point in time -y)

X = y

X + Y | X - Y | X * Y | X / Y

X == Y | X != Y | X > Y | X >= Y | X < Y | X <= Y

For any comparison operator X: X{y} (the values being compared will have an error margin of y)

### Usefull Predicates
X.position (The position of the robot in the simulation)

X.position.y (The position in the y axis of the robot in the simulation)

X.distance.Y (The absolute distance between two objects in the simulation)

X.orientation (The orientation of a object in the simulation)

X.velocity (The velocity of a object in the simulation)

X.localization_error (The difference between to robot perception of its position and the actual position in the simulation)

### Examples

#### A robot always stops at the stop sign:
```
always( after( (robot1.distance.stop_sign1 < 2) and (robot1.orientation - stop_sign1.orientation < 90), eventually(robot1.velocity <= 0)) until ( (robot1.distance.stop_sign1 > 2) or (robot1.orientation - stop_sign1.orientation > 90))
```

#### The localization error (difference between the robot perception of its location and the simulation actual location) of the robot is never above a certain value:
```
# There are a set of topics that can be modeled by robot like "position", "velocity", etc..
# These will be used by the compiler to call specific functions that need this information
model robot1:
    position /odom Odometry.pose.pose.position

never (robot1.localization_error > 0.2)
```

#### After a drone is at a certain altitude both rotors always have the same velocity up until the drone decreases to a certain altitude
```
# The language can't inherently have a way to interact with specific components of a robot 
# like the rotors, because it doesn't know which topic to get information from. The user
# needs to declare these specific topics to be able to interact with them.
decl rotor1_vel /drone_mov/rotor1 Vector3.linear.x
decl rotor2_vel /drone_mov/rotor2 Vector3.linear.x

after (drone.position.z > 5, rotor1_vel =={0.2} rotor2_vel) until (drone.position.z < 5)
```

#### A robot never makes a rotation of more than X degrees in a period of time
```
robot_ori = robot.orientation
robot_ori_prev1 = @{robot_ori, -1}
robot_ori_prev2 = @{robot_ori, -2}
robot_ori_prev3 = @{robot_ori, -3}

never ((robot_ori - robot_ori_prev1 > 12) or (robot_ori - robot_ori_prev2 > 12) or (robot_ori - robot_ori_prev3 > 12))
```