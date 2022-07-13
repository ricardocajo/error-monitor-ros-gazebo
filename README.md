# Error Monitor using ROS + Gazebo
  The project contemplates a Domain Specific Language (DSL) for specifying robots' expected behavior and the respective compiler that generates code to monitor these behaviors while in a Gazebo simulation.

## Table of contents
* [Abstract](#abstract)
* [Language](#language)
    * [Operators](#operators)
    * [Usefull-Predicates](#usefull-predicates)
    * [Protected-Variables](#protected-variables)
    * [Grammar](#grammar)
* [Examples](#examples)
    * [Stop-Sign](#stop-sign)
    * [Miscellaneous](#miscellaneous)
* [Testing](#testing)
    * [Docker](#docker)
    * [Local](#local)
* [Assessment](#assessment)

## Abstract

Robotics has a significant influence in today's society, so much that a potential failure in a robot may have extraordinary costs, not only financial but can also cost lives.

Current practices in robot testing are vast and involve such methods as simulations, log checking, or field tests. The frequent common denominator between these practices is the need for human visualization to determine the correctness of a given behavior.

Automating this analysis can not only relieve the burden from a high-skilled engineer but also allow for massive parallel executions of tests that can detect behavioral faults in the robots. These faults could otherwise not be found due to human error or a lack of time.

We have developed a domain-specific language to specify the properties of robotic systems in ROS. Developer written specifications in this language compile to a monitor ROS module that detects violations of those properties in runtime. We have used this language to express the temporal and positional properties of robots, and we have automated the monitoring of some behavioral violations of robots in relation to their state or events during a simulation.


## Language

### Operators

The DSL relies on specific operators to express temporal relations between simulation objects.

`always X` - X has to hold on the entire subsequent path.

`never X` - X never holds on the entire subsequent path.

`eventually X` - X eventually has to hold somewhere on the subsequent path.

`after X, Y` - after the event X is observed, Y has to hold on the entire subsequent path.

`until X, Y` - X holds at the current or future position, and Y has to hold until that position. At that position, Y does not have to hold anymore.

`after_until X, Y, Z` - after the event X is observed, Z has to hold on the entire subsequent path up until Y happens. At that position, Z does not have to hold anymore.

`@{X, -Y}` - the value of the variable X in the point in time -Y

`X = Y`

`X implies Y`    `X and Y`    `X or Y`

`X + Y`    `X - Y`    `X * Y`    `X / Y`

`X == Y`    `X != Y`    `X > Y`    `X >= Y`    `X < Y`    `X <= Y`

For any comparison operator X: `X{y}` - the values being compared will have an error margin of y (Example: X =={0.05} Y)

### Useful-Predicates

The DSL also has shortcuts to express the absolute values of certain useful concepts of objects in a simulation.

`X.position` - The position of the robot in the simulation.

`X.position.y` - The position in the y axis of the robot in the simulation (also works for x and z).

`X.distance.Y` - The absolute distance between two objects in the simulation (x and y-axis).

`X.distanceZ.Y` - The absolute distance between two objects in the simulation (x, y, and z-axis).

`X.velocity` - The velocity of an object in the simulation (this refers to linear velocity).

`X.velocity.x` - The velocity in the x-axis of an object in the simulation (this refers to linear velocity).

`X.localization_error` - The difference between the robot's perception of its position and the actual position in the simulation.

*Yet to implement:*

`X.orientation` - The orientation of an object in the simulation.

`X.orientation_between.Y` - The orientation difference between two objects in the simulation.

`X.velocity_error` - The difference between the robot's perception of its velocity and the actual velocity in the simulation (this refers to linear velocity).

`X.velocity_angular` - The angular velocity of an object in the simulation.

### Protected-Variables

Variable names restricted to set determined monitoring parameters

`_rate_` - Set the frame rate which properties are checked (By default, the rate is 30hz).

`_timeout_` - Set the timeout for how long the verification will last (By default, the timeout is 100 seconds).

`_margin_` - Set the error margin for comparisons.

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

## Examples

### Stop-sign

For instance, if we wanted to express that a robot always needs to stop when coming near a stop sign, we could write something like:
```
_margin_ = 0.01
after_until turtlebot3_burger.distance.stop_sign2 < 1, turtlebot3_burger.distance.stop_sign2 > 1, eventually turtlebot3_burger.velocity == 0
```
Translating into natural language, the property states in the first section that after the turtlebot3 distance to the stop-sign2 is below the value of 1 in the simulator, and in the second section that up until the distance is again above 1, then in the third section, the turtlebot3 velocity will eventually be equal to 0.

After compiling the above property, a python file capable of monitoring said property will be generated.

#### The robot doesn't stop at the stop sign:

![no_stop](https://user-images.githubusercontent.com/82663158/167421559-9acebddb-9370-40ff-8912-058f7edb3b79.gif)

![err](https://user-images.githubusercontent.com/82663158/167425264-b1395455-b6f9-4f04-aba0-1aebefcd3ec7.png)

#### The robot stops at the stop sign:

![stop](https://user-images.githubusercontent.com/82663158/167421668-622e7ca9-ee6d-434e-baf9-44b7d33d0fe1.gif)

![tout](https://user-images.githubusercontent.com/82663158/167425226-f86592de-c532-4c79-bf0d-342150872dff.png)

### Miscalleneous

#### The robot velocity will be above 2 sometime in the duration of the simulation:
```
eventually robot1.velocity > 2.0
```

#### After a drone is at a certain altitude, both rotors always have the same velocity up until the drone decreases to a certain altitude
```
# The language cannot inherently have a way to interact with specific components of a robot
# like the rotors because it does not know which topic to get information from. The user
# needs to declare these specific topics to be able to interact with them.
decl rotor1_vel /drone_mov/rotor1 Vector3.linear.x
decl rotor2_vel /drone_mov/rotor2 Vector3.linear.x

after_until drone.position.z > 5, drone.position.z < 5, rotor1_vel =={0.2} rotor2_vel
```

#### The localization error (difference between the robot's perception of its location and the actual simulation location) of the robot is never above a specific value:
```
# There are a set of specific topics that can be modeled by robot-like "position", "velocity", etc...
# The compiler will use these to call specific functions that need this information
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

never (robot_ori - robot_ori_prev1 > 12 or robot_ori - robot_ori_prev2 > 12 or robot_ori - robot_ori_prev3 > 12)
```

#### The car always stops at the stop sign:
```
after_until car1.distance.stop_sign2 < 1, car1.distance.stop_sign2 > 1, eventually car1.velocity =={0.01} 0
```

#### A car is never at less than X distance from another car
```
always car1.distance.car > 0.35
```

#### Car1 being above 1 velocity implies that car2 is at least at 0.8 distance from car1 up until they reach a specific location.
```
until (car1.position.x > 45 and car1.position.y > 45), always (car1.velocity > 1 implies car2.distance.car1 > 0.8)
```

## Testing

### Docker
It is possible to test the monitor running alongside Gazebo and ROS without installing anything more than docker and a VNC viewer.
In this methodology, we take advantage of the [TheRobotCooperative](https://github.com/TheRobotCooperative/TheRobotCooperative) repository to build the docker images. Every robot present in the repository can be tested.
Install TigerVNC in Ubuntu with the command:
```
$ apt-get install tigervnc-viewer
```

Start by cloning the [TheRobotCooperative](https://github.com/TheRobotCooperative/TheRobotCooperative) repository, change to the ros1 directory, and create the docker image for a specific robot:
```
$ git clone https://github.com/TheRobotCooperative/TheRobotCooperative.git
TheRobotCooperative/ros1$ make turtlebot3
```

Build the image of the previous robot with all the project dependencies
```
error-monitor-ros-gazebo$ docker build --build-arg robot=turtlebot3 -t turtlebot3_image_name .
```

Run the docker image and [start the VNC viewer](https://github.com/TheRobotCooperative/TheRobotCooperative#using-vnc-to-provide-visualisation):
```
$ docker run --rm -it turtlebot3_image_name
```

Inside the VNC viewer, create a file with the properties to monitor. Then open a terminal and compile it to generate the monitor file: (make sure to use the python3.8 version for it is necessary for the compilation)
```
error-monitor-ros-gazebo$ python3.8 language.py test.txt /ros_ws/src/test_pkg/src
```

Run the Gazebo simulator (the first time might take a while), the monitor node, and the teleop node to command the robot: (in separate terminals)
```
$ export TURTLEBOT3_MODEL="burger"
$ roslaunch turtlebot3_gazebo turtlebot3_house.launch
```
```
$ rosrun test_pkg test.py
```
```
$ export TURTLEBOT3_MODEL="burger"
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

### Local
There is a need to adjust versions of the below software depending on your operating system.
Some Robots only have support up until some version of the below software.

#### ROS
To install ROS, follow the link [ros_install](http://wiki.ros.org/ROS/Installation) and have in mind your operating system

To create a ROS workspace on your computer to be able to run ROS projects, follow the link [ros_workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

#### Gazebo
To install Gazebo in Ubuntu through the command line, follow the link [gazebo_install](http://gazebosim.org/tutorials?tut=install_ubuntu). For any other method of installation, follow the documentation on the official site [Gazebo](http://gazebosim.org/)

## Assessment

[Google Form](https://docs.google.com/forms/d/e/1FAIpQLSe9FAW0o-U1JwjH5vFV_AedoVMDs6if2MxSOvHW1SzA15ZmWg/viewform?usp=sf_link)
