# WAET Robotics Project
> The goal of the program is for the turtle to write the string "0Ty". The turtle can only move when the "setVel" method is used, which sets its speed (linear or angular). And the color of the drawn lines depends on the speed of the moving turtle.

## A Brief Explanation of the Program
> The "Turtle" class was created to store methods for handling "turtle_api" thanks to which it is possible to move and draw in a way specified by the turtle. The 3 movement methods are key:
> - "move_forward" - allows the turtle to move forward a specified distance;
> - "move_arc" - allows the turtle to move along a specified arc (taken as fragments of a given circle)
> - "rotate_in_place" - allows the turtle to rotate in place by a given angle (despite the use of correcting algorithms, the rotation angle may still be off by 0.1 degrees in a few cases)

## Mathematical Explanation
> ### Time From Speed Formula
> From the formula for speed (speed = distance / time), it is easy to calculate the time it takes the turtle to cover a distance (time = distance / speed). Thanks to this, turtle can be told to accelerate to half of the given situation (maximum speed in half the time to cover the distance) and decelerate from that moment to the end of this distance.

> ### Acceleration
> The acceleration of the turtle is defined by the formula (the maximum speed was given by the programmer, it is 10 m/s): speed = maximum speed * (2 * current time / time calculated to cover the entire distance)
by multiplying the above quotient by 2, we are in able to catch the moment of half the time to travel, then the speed will be equal to the maximum speed. We multiply the above quotient by 2 because the quotient of the current time and the time given in half the distance will equal 0.5, 0.5 times 2 is 1 then the speed will equal the maximum speed

> ### Braking
> Braking the turtle is defined by the formula:
speed = maximum speed * (2 - 2 * current time / time calculated to travel the entire distance)
When we reach half the time given to travel, we can start braking. By subtracting the previously described quotient multiplied by 2 from 2, it is possible to brake the turtle. we subtract from 2 because we have divided the road into two halves: the half in which the turtle accelerates and the one in which it brakes. when the current time is equal to the set time then the quotient will be equal to 1, multiplied by 2 equals 2, so when we subtract 2 from 2 then we will get 0 so the speed will be equal to 0
