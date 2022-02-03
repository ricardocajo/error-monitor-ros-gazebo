## Operators
always X (X has to hold on the entire subsequent path)

eventually X (X eventually has to hold, somewhere on the subsequent path)

X until Y (Y holds at the current or future position, and X has to hold until that position. At that position X does not have to hold any more)

X implies Y

not X

X and Y | X or Y

X == Y | X != Y

X > Y | X >= Y | X < Y | X <= Y

X + Y | X - Y | X * Y | X / Y


## Usefull Predicates
distance X Y (The absolute distance between two objects in the simulation) 

orientation X (The orientation of a object in the simulation)

velocity X (The velocity of a object in the simulation)

localization_error X (The difference between to robot perception of its position and the actual position in the simulation)


## Examples

### A robot always stops at the stop sign:
always (eventually ((distance robot1 stop_sign1) < 2) and ((orientation robot1) - (orientation stop_sign1) < 90) implies eventually ((velocity robot1) <= 0))

### The localization error of the robot is always below a certain value:
always ((localization_error robot1) < 0.2)

### Robot1 is never at more than a certain distance from robot2
always ((distance robot1 robot2) < X)

### A robot never makes a rotation of more than X degrees in a period of time (maybe usefull for an airplane/drone simulation)
How would i express the period of time? And how would i even monitor this

### A robot's wheels never turn more than 90 degrees:
The problem i sent an email, if i write something like left_wheel_orientation how will my compiler internally know which topic to search for? (it depends on the robot and can change dynamically)

In the case that the topic name doesn't change dynamically, i can have a robot_config_file that the user fills before running so the compiler has this type of information 

Even if i could have this information how would i be able to use it? different robots will have different Message structures
