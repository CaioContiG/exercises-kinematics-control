# exercises-kinematics-control
MatLab Simulation of simple exercises in kinematics and control.

## RRP Robotic Manipulator
A robotic leg simplified to a RRP manipulator. The file rrp_FK_IK.m contains the forward and inverse kinematics calculations for a RRP manipulator, as shown in the figure below. The code plots the result given joint values [q1, q2, q3] (FK) or [xc,yc,zc] end-point position (IK). It also plots the workspace.

![RRP-illustration](https://github.com/CaioContiG/exercises-robotics-dynamics-control/assets/41450841/b58cbd83-c3e9-4b93-b1dd-855d98fae958)

## Slider Crank
Forward and inverse kinematics of a closed-chain system, slider crank linkage (1-RRPR) as shown below. The file slider_crank_IF_FK.m generates a random angle for q1 and using IK computes q3 to match the desired angle. It plots the final configuration.

![slider-crank](https://github.com/CaioContiG/exercises-robotics-dynamics-control/assets/41450841/ba019780-dd46-4a4f-b84c-3550a4823d32)


## Pendulum Free-Fall
Simlating a simple free fall of a pendulum, without damping and with string length equals to 1. The file pendulum_free_fall.m computes the free fall using Euler's method and ODE45 to integrate for comparison. It shows the animation for Euler's method and plot both Euler and ODE45 theta results over time. The file pendulum_FF_RK does the same, but it uses fourth order Runge-Kutta to compare with Euler's method.

![pend-free-fall](https://github.com/CaioContiG/exercises-robotics-dynamics-control/assets/41450841/2c1dfa00-0a79-47e9-b0cb-0c45189c9750)

## Pendulum Control
Perform swing-up of the pendulum, with damping and l = 0.5, using Euler's method to integrate. The file pendulum_PD.m control the pendulum with a proportional-derivative controller (PD) with no restrictions on the maximum torque. The file pendulum_LQR.m performs the swing-up with torque maximum of 1. With maximum torque limited to 1, the swing-up can be performed by changing the previous PD so it swings from one side to the other and when it cross a theta threshold, LQR controller (inear–quadratic regulator) activates and PD deactivates. Both files shows an animation, plot theta and torque over time.

![pend-control](https://github.com/CaioContiG/exercises-robotics-dynamics-control/assets/41450841/db94eaf7-00da-4cc9-bf05-4c186929fddf)


