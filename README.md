# Tuning-PID-Controller-for-the-double-pendulum
This is a tuning PID controller with Ziegler Nichols and CTM(computed torque method) for the double pendulum in Matlab.

1. PID tuning with Zieglar Nichols for the double pendulum
I wrote a simulator of motion for a double pendulum with Coulomb friction using Lagrange's equation.
In this code, its PID controller is designed with two methods of Z-N tuning. 
I gave a simple trajectory to see whether this controller follows it properly. 

2. PID tuning with CTM
In this code, 5*dth in tau equation is for disturbance to see how this simulator actually works with it.
