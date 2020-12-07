# Milestone
Milestone Project

# Overview
This repository is part of the Robotic Manipulation course from Northwestern.
Milestone 3 final project [link](http://hades.mech.northwestern.edu/index.php/Mobile_Manipulation_Capstone).


Write software that plans and executes a trajectory for the youBot mobile manipulator.
The goal is to grab a cube and place it in a different position.

# Architecture 
For the similation part we used Coppelia Simulator, you can download the scened from the link.
1. The input of the program is:
the initial resting configuration of the cube object (which has a known geometry), represented by a frame attached to the center of the object
2. the desired final resting configuration of the cube object
3. the actual initial configuration of the youBot
4. the reference initial configuration of the youBot (which will generally be different from the actual initial configuration, to allow us to test feedback control)

Input arguments are given from the `input.csv` file in the format of :
1. x,y,z,theta
2. x,y,z,theta
3. chassis phi, chassis x, chassis y, J1, J2, J3, J4, J5, W1, W2, W3, W4, gripper state
4. chassis phi, chassis x, chassis y, J1, J2, J3, J4, J5, W1, W2, W3, W4, gripper state

3 main functions are being used GenerateTrajectory, nextState, Feedforward_Control each on representing Mileston 1,2,3.

1. GenerateTrajectory : Given the cube initial,desired position and eef initial position we create a trajectory.
2. nextState : calculates the next configuration state given the current configuration state, joints velocitys and time step dt.
3. Feedforward_Control calculates the joint velocitys(with no limits) given the current state where we wanted to be and where we want to go after dt.

# Execute program
Navigate to code directory and execute : `<python main.py id>`

id is and integer number selecting different executions (for the final implementation set id=5)

1. id-->1 Executing test nextState (Mileston 2)
2. id-->2 Executing test trajectory Generator (Mileston 1)
3. id-->3 Executing test Feedforward Control (Mileston 3)
4. id-->4 Executing test matrix convert!
5. id-->5 Executing pick and place! (Mileston Project)

# Output
Executing pick and place! (id=5) will generate 4 files under the data folder

1. trajectory.csv --> used on coppelisSim Scene8_gripper_csv
2. pick_place.csv --> used on coppelisSim Scene6_youBot_cube
3. error.csv --> contains all the error from controler for each step
4. image of the error plot 
5. log file with all outputs

# Examples

![](https://github.com/jimas95/Milestone/blob/main/gifs/task_default.gif)
![](https://github.com/jimas95/Milestone/blob/main/gifs/task_1.gif)
