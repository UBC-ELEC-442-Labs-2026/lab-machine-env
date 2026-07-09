# Lab Machine Env
This repo contains the code to be put onto the lab computers for controlling the QArms. Students will be able to run this code, however it will remain the read only section of the drive so that it will be publically visible and they cannot tamper with it. The following will briefly cover what is in each folder and what each program does. As the repo grows, add the documentation here.

The repo itself will be visible to students who wish to test any of the functionality here on their own devices through the virtual QArm. See the "Setting up the Virtual QArm" Section below.

## QArm-control
This folder contains programs made to ensure safe usage of the QArm. A work in progress at the moment, this will eventually contain the an abstraction layer above the Quanser libraries to enforce things such as a safe-zone.

This will also contain code that is used between multiple labs for convenience

**`QArm_keyboard_control.py`**:  
 Implements control of the QArm using the keyboard via pygame. Generates a window that displays the live joint positions of the arm. Also note that the pygame window must be focused for the controls to register.

 **`QArm_functions.py`**:  
This is a key program for QArm usage in this lab. It acts as a middleman between the lab scripts and the Quanser QArm libraries. It passes through nessesary functions to the user such as writing joint positions, reading joint positions, kinematic solvers, etc. and allows for enforcing workspace limits.

Once imported into your program, create a `QArm_Lab_Interface` object. This will initally give you access to the kinematic solvers. 

If you create a QArm object then pass it in using `QArm_Lab_Interface.attach_QArm(myArm)`, you will be able to able to use the QArm control functions such as `write_to_arm(phi)`, `read_from_arm()`, etc. Commands and read joint positions are placed in buffers, while the actual QArm control reads/writes to these buffers and take place in a seperate thread running at 25Hz.

## Lab-1
This contains programs used in lab 1 only.

**`integration_err.py`**:  
Contains a script to run a demo that draws circles, showing integration error over time. Also generates a 3D graph.

**`QArm_traj_controllers.py`**:
To be paired with `QArm_traj.py` which the students will have in their own repos.

# Setting up the Virtual QArm
For students looking to download the programs to run on their own machine to test on a virtual QArm, follow the instructions below:

[Add instructions], likely a slightly modified version of the code will be required - or tell them to toggle the hardware setting

# To-do:
Work on physical baseplate and props - update lab manuals as needed