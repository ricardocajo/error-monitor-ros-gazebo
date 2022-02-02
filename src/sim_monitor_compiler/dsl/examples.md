## Operators
always X (X has to hold on the entire subsequent path)

eventually X (X eventually has to hold, somewhere on the subsequent path)

X until Y (Y holds at the current or future position, and X has to hold until that position. At that position X does not have to hold any more)

X implies Y

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

### A robot wheels never turn more than 90 degrees:
