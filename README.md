# Milestone
Milestone Project

# Overview
This repository is part of the Robotic Manipulation course from Northwestern.
Milestone 3 final project [link](http://hades.mech.northwestern.edu/index.php/Mobile_Manipulation_Capstone).


Write software that plans and executes a trajectory for the youBot mobile manipulator.
The goal is to grab a cube and place it in a different position.

# Architecture 
For the simulation part we used Coppelia Simulator, you can download the scene from the link. For the kinematics and robot manipulation ModernRobotics library is being used.
The input of the program is:
1.the initial resting configuration of the cube object (which has a known geometry), represented by a frame attached to the center of the object
2. the desired final resting configuration of the cube object
3. the actual initial configuration of the youBot
4. the reference initial configuration of the youBot (which will generally be different from the actual initial configuration, to allow us to test feedback control)

Input arguments are given from the `input.csv` file in the format of :
1. x,y,z,theta
2. x,y,z,theta
3. chassis phi, chassis x, chassis y, J1, J2, J3, J4, J5, W1, W2, W3, W4, gripper state
4. chassis phi, chassis x, chassis y, J1, J2, J3, J4, J5, W1, W2, W3, W4, gripper state

3 main functions are being used GenerateTrajectory, nextState, Feedforward_Control each on representing Milestone 1,2,3.

1. GenerateTrajectory : Given the cube initial,desired position and eef initial position we create a trajectory.
2. nextState : calculates the next configuration state given the current configuration state, joints velocities and time step dt.
3. Feedforward_Control calculates the joint velocities(with no limits) given the current state where we wanted to be and where we want to go after dt.

In more detail about the Architecture of the code. `python main.py` if the file that starts the program and take an input argument for which case to execute. `robotControl.py` is a class that handles the important stuff such as generating trajectory,nextState , feedforward Control. `functions.py` is a file containing helpful functions such as writing the csv file. Finally the last file is `myMatrix` is a class containing all matrices that we are using and help functions such as going from lists to matrix etc.

# Execute program
Navigate to code directory and execute : `<python main.py id>`

id is and integer number selecting different executions (for the final implementation set id=5)

1. id-->1 Executing test nextState (Milestone 2)
2. id-->2 Executing test trajectory Generator (Milestone 1)
3. id-->3 Executing test Feedforward Control (Milestone 3)
4. id-->4 Executing test matrix convert!
5. id-->5 Executing pick and place! (Milestone Project)

# Output
Executing pick and place! (id=5) will generate 4 files under the data folder

1. trajectory.csv --> used on coppelisSim Scene8_gripper_csv
2. pick_place.csv --> used on coppelisSim Scene6_youBot_cube
3. error.csv --> contains all the error from controller for each step
4. image of the error plot 
5. log file with all outputs

# Examples

![](https://github.com/jimas95/Milestone/blob/main/gifs/task_default.gif)
![](https://github.com/jimas95/Milestone/blob/main/gifs/task_1.gif)
