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

## End- Effector trajectory : 
### Position , Velocity and Acceleration

<div align = "center">
<img src = "https://github.com/PranayG/SCARA-manipulator-Trajectory-Planning/assets/9202531/d73b288b-8803-45d8-9221-79abca06e5b6"  width ="300">   

<img src ="https://github.com/PranayG/SCARA-manipulator-Trajectory-Planning/assets/9202531/e50fc4d1-fc89-4d74-949f-58d14fd7e6b7" width ="300">  

<img src = "https://github.com/PranayG/SCARA-manipulator-Trajectory-Planning/assets/9202531/5f380308-acd0-4485-b493-9a724de445ba"  width ="300"> </div>

### 3-D space operational space trajectory

<div align = "center">
  <img src= "https://github.com/PranayG/SCARA-manipulator-Trajectory-Planning/assets/9202531/bc22a8fe-6970-413c-94e3-84c311574e4f"></div>

## Inverse Dynamic Control

We computed the joint torques when a 5kg load is placed at the End-effector using the Lagrangian equation of motion

<img src = "https://github.com/PranayG/SCARA-manipulator-Trajectory-Planning/assets/9202531/fab4a26c-8093-40e5-abf7-ea5a257c0700">

For computing the Kinetic and Potential energy of all the joints we use an inverse dynamic approach
Kinetic energy is given by : 
![image](https://github.com/PranayG/SCARA-manipulator-Trajectory-Planning/assets/9202531/ce12130a-6ec6-412d-88db-24ba4be36d7a)

where : **B(q) is an inertia matrix** , **C(q,q_dot) is a matrix considering Centrifugal forces , Fv matrix is a diagonal matrix considering viscous force and G(q) is the 4x1 matrix considering moment due to gravity for each link**

The idea behind the Inverse Dynamics approach in order to control / track the joint space 
trajectory is to implement a function in way which is capable of realizing an input/ output 
relationship of linear type. This approach performs exact linearization of the dynamics of the 
system obtained by the help of a non-linear state feedback
<div align = "center">
  
  ![image](https://github.com/PranayG/SCARA-manipulator-Trajectory-Planning/assets/9202531/03b987f9-9185-42af-b601-9d44de3a49f8)

  **Figure : Block scheme of joint space inverse dynamics control** </div>

  ## Joint trajectory

  ![image](https://github.com/PranayG/SCARA-manipulator-Trajectory-Planning/assets/9202531/bc94d4c4-4a99-4791-8bec-dd762d6a746e)

The above are the joint trajectories computed using the inverse dynamic control approach for 
every joint with respect to time



