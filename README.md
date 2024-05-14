# exercises-robotics-dynamics-control
Simulation and Solution of simple exercises in Robotics Dynamics and Control

## RRP - Robotic Leg
A robotic leg simplified to a RRP manipulator. The file rrp_FK_IK.m contains the forward and inverse kinematics calculations for a RRP manipulator, as shown in the figure below. The code plots the result given joint values [q1, q2, q3] (FK) or [xc,yc,zc] end-point position. It also plots the workspace.

![RRP-illustration](https://github.com/CaioContiG/exercises-robotics-dynamics-control/assets/41450841/b58cbd83-c3e9-4b93-b1dd-855d98fae958)

## Slider Crank
Forward and inverse kinematics of a closed-chain system, slider crank linkage (1-RRPR) as shown below. The file slider_crank_IF_FK.m generates a random angle for q1 and using IK computes q3 to match the desired angle. It plots the final configuration.

![slider-crank](https://github.com/CaioContiG/exercises-robotics-dynamics-control/assets/41450841/ba019780-dd46-4a4f-b84c-3550a4823d32)


## Pendulum Free-Fall
Simlating a simple free fall of a pendulum, without damping and with string length equals to one. The file pendulum_free_fall.m computes the free fall using Euler's method and ODE45 to integrate for comparison. It shows the animation as the figure below for Euler's method and plot both Euler and ODE45 theta results.

![pend-free-fall](https://github.com/CaioContiG/exercises-robotics-dynamics-control/assets/41450841/2c1dfa00-0a79-47e9-b0cb-0c45189c9750)

