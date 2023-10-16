# SCARA-manipulator-Trajectory-Planning
The aim of this project is to compute the trajectory in the operational space. We consider the given points at the time stamps given with sample time Ts. The trajectory is generated using  the trapezoidal velocity profile. The total trajectory is completed in 3.4 seconds while considering the anticipation time.

## Generating the Trajectory

We are given 5 waypoints through which the end-effector must pass. We first compute the constant acceleration and time for each of the consecutive segments.'

<img src = "https://github.com/PranayG/SCARA-manipulator-Trajectory-Planning/assets/9202531/cf34ccdd-961c-4c96-a4e1-d9323beecfe1" width = "675" height ="400">

After finding the position of the end-effector at every instance of 0.001 seconds we take into the 
anticipation time consideration.
The arc length ‘s’ is calculated for each point and is then added to its previous arc length from the 
previous point to point p0. This is given by :

<img src ="https://github.com/PranayG/SCARA-manipulator-Trajectory-Planning/assets/9202531/3832b674-2d65-4930-ad4b-407dac08f0c1"  width = "800" height ="85" >
<img src = "https://github.com/PranayG/SCARA-manipulator-Trajectory-Planning/assets/9202531/531cc99f-6960-4dda-8c42-2e0f7439fcda" width = "800" height ="75" >

## End- Effector trajectory : Position , Velocity and Acceleration

<div align = "center">
<img src = "https://github.com/PranayG/SCARA-manipulator-Trajectory-Planning/assets/9202531/d73b288b-8803-45d8-9221-79abca06e5b6"  width ="300">   

<img src ="https://github.com/PranayG/SCARA-manipulator-Trajectory-Planning/assets/9202531/e50fc4d1-fc89-4d74-949f-58d14fd7e6b7" width ="300">  

<img src = "https://github.com/PranayG/SCARA-manipulator-Trajectory-Planning/assets/9202531/5f380308-acd0-4485-b493-9a724de445ba"  width ="300"> </div>

### 3-D space operational space trajectory

<div align = "center">
  <img src= "https://github.com/PranayG/SCARA-manipulator-Trajectory-Planning/assets/9202531/bc22a8fe-6970-413c-94e3-84c311574e4f"></div>

## Inverse Dynamic Control

We computed the joint torques when a 5kg load is placed at the End-effector


