# Simulation Monitor Compiler
  This repository provides a convenient DSL (Domain Specific Language) for specifying robots' expected behavior, as well as the respective compiler to monitor these behaviors while in a simulation.

## Table of contents
* [Introduction](#introduction)
* [Installs](#installs)
* [Language](#language)
    * [Operators](#operators)
    * [ProtectedVariables](#protectedvariables)
    * [UsefullPredicates](#usefullpredicates)
    * [Examples](#examples)
    * [Grammar](#grammar)
* [Assessment](#assessment)

## Introduction

The development came from the idea of aliviating the burden of human interaction when testing a robotic system.
*EXEMPLE OF THE STOP SIGN*

## Installs
This project was made using Ubuntu 20.04 in a virtual environment. For this reason, specific versions of the following software were installed. You might want to adjust it to your operating system, versions, or preferences.

In this project Gazebo was chosen as the simulation software, if you want to use some other simulator be aware that you will need to declare all the topics to use, the default functions like 'position' or 'velocity' are based on the Gazebo simulator.

### ROS
To install ROS follow the link [ros_install](http://wiki.ros.org/ROS/Installation) and have in mind your own specifications. While making this project ROS noetic was used.


To create a ROS workspace on your computer to be able to run ROS projects follow the link [ros_workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

### Gazebo
To install Gazebo in Ubuntu through the command line follow the link [gazebo_install](http://gazebosim.org/tutorials?tut=install_ubuntu). For any other method of installation follow the documentation on the official site [Gazebo](http://gazebosim.org/)


## Language

### Operators
always X - X has to hold on the entire subsequent path

never X - X never holds on the entire subsequent path

eventually X - X eventually has to hold, somewhere on the subsequent path

after X, Y - after the event X is observed, Y has to hold on the entire subsequent path

until X, Y - X holds at the current or future position, and Y has to hold until that position. At that position Y does not have to hold any more

after_until X, Y, Z - after the event X is observed, Z has to hold on the entire subsequent path up until Y happens, at that position Z does not have to hold anymore

@{X, -Y} - the value of the variable X in the point in time -Y

X = Y

X implies Y    X and Y    X or Y

X + Y | X - Y | X * Y | X / Y

X == Y | X != Y | X > Y | X >= Y | X < Y | X <= Y

For any comparison operator X: X{y} - the values being compared will have an error margin of y (Example: X =={0.05} Y)

### ProtectedVariables
\_rate_ - Set the frame rate which properties are checked (By default the rate is 30hz)

\_timeout_ - Set the timeout for how long the verification will last (By default the timeout is 100 seconds)

\_margin_ - Set the error margin for comparisons

### UsefullPredicates
X.position - The position of the robot in the simulation

X.position.y - The position in the y axis of the robot in the simulation (also works for x and z)

X.distance.Y - The absolute distance between two objects in the simulation (x and y axis)

X.distanceZ.Y - The absolute distance between two objects in the simulation (x, y and z axis)

X.velocity - The velocity of an object in the simulation (this refers to linear velocity)

X.velocity.x - The velocity in the x axis of an object in the simulation (this refers to linear velocity)

X.localization_error - The difference between the robot perception of its position and the actual position in the simulation

*Yet to implement:*

X.orientation - The orientation of an object in the simulation

X.orientation_between.Y - The orientation difference between two objects in the simulation

X.velocity_error - The difference between the robot perception of its velocity and the actual velocity in the simulation (this refers to linear velocity)

X.velocity_angular - The angular velocity of an object in the simulation

### Examples

#### The robot velocity will be above 2 sometime in the duration of the simulation:
```
eventually robot1.velocity > 2.0
```

#### After a drone is at a certain altitude both rotors always have the same velocity up until the drone decreases to a certain altitude
```
# The language can't inherently have a way to interact with specific components of a robot
# like the rotors, because it doesn't know which topic to get information from. The user
# needs to declare these specific topics to be able to interact with them.
decl rotor1_vel /drone_mov/rotor1 Vector3.linear.x
decl rotor2_vel /drone_mov/rotor2 Vector3.linear.x

after drone.position.z > 5, rotor1_vel =={0.2} rotor2_vel until drone.position.z < 5
```

#### The localization error (difference between the robot perception of its location and the simulation actual location) of the robot's is never above a certain value:
```
# There are a set of specific topics that can be modeled by robot-like "position", "velocity", etc...
# These will be used by the compiler to call specific functions that need this information
model robot1:
    position /odom Odometry.pose.pose.position
    ;

never robot1.localization error > 0.002
```

#### A robot never makes a rotation of more than X degrees in a period of time
```
robot_ori = robot.orientation
robot_ori_prev1 = @{robot_ori, -1}
robot_ori_prev2 = @{robot_ori, -2}
robot_ori_prev3 = @{robot_ori, -3}

never robot_ori - robot_ori_prev1 > 12 or robot_ori - robot_ori_prev2 > 12 or robot_ori - robot_ori_prev3 > 12
```

#### A robot always stops at the stop sign:
```
always after robot1.distance.stop_sign1 < 2 and robot1.orientation - stop_sign1.orientation < 90, eventually robot1.velocity <= 0 until robot1.distance.stop_sign1 > 2 or robot1.orientation - stop_sign1.orientation > 90
```

### Grammar
```
       <program> → <command>
                 | <command> <program>

       <command> → <association>
                 | <declaration>
                 | <model>
                 | <pattern>

   <association> → name = <pattern>
                 | _rate_ = integer
                 | _timeout_ = number
                 | _default_margin_ = number

   <declaration> → decl name topic_name <msgtype>
                 | decl name name <msgtype>

         <model> → model name : <modelargs> ;

     <modelargs> → <name> topic_name <msgtype>
                 | <name> <name> <msgtype>
                 | <name> topic_name <msgtype> <modelargs>
                 | <name> <name> <msgtype> <modelargs>

          <name> → name
                 | <func_main>

     <func_main> → position
                 | velocity
                 | distance
                 | localization_error
                 | orientation

       <msgtype> → <name>
                 | <name> . <msgtype>

       <pattern> → always <pattern>
                 | never <pattern>
                 | eventually <pattern>
                 | after <pattern> , <pattern>
                 | until <pattern> , <pattern>
                 | after_until <pattern> , <pattern> , <pattern>
                 | <conjunction>

   <conjunction> → <conjunction> and <comparison>
                 | <conjunction> or <comparison>
                 | <conjunction> implies <comparison>
                 | <comparison>

    <comparison> → <multiplication> <opbin> <multiplication>
                 | <multiplication> <opbin> { <number> } <multiplication>
                 | <multiplication>

         <opbin> → <
                 | >
                 | <=
                 | >=
                 | ==
                 | !=

<multiplication> → <multiplication> * <addition>
                 | <multiplication> / <addition>
                 | <addition>

      <addition> → <addition> + <operand>
                 | <addition> - <operand>
                 | <operand>

       <operand> → name
                 | <number>
                 | true
                 | false
                 | <func>
                 | <temporalvalue>
                 | ( <pattern> )

        <number> → float
                 | integer

          <func> → name . <func_main>
                 | name . <func_main> <funcargs>

      <funcargs> → . <name>
                 | . <name> <funcargs>

 <temporalvalue> → @ { name , integer }
```

## Assessment

#### Q1. What kind of errors do you often found when developing a robot?

#### Q2. Do you think these errors could be somewhat described by a property of the robot in relation to its environment? Is there any exception? (Give a property example if needed)

*Give a brief introduction of the work (follow script from introduction(#introduction))*

#### Q3. Do you think the DSL covers all before mentioned possible properties? Is there any exception?

#### Q4. What's your opinion on the intuitiveness of the DSL?

#### Q5. Would this DSL or an improvement be useful in your current work? If yes, do you think the learning/maintenance would be worth the hassle?

#### Q6. What's your opinion on the helpfulness of some type of integration between the DSL and Gzscenic? (Only for Afsoon)
