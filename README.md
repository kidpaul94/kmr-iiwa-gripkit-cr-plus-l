<p align="center">
<img src=./images/logo.png width=40% height=40%>
</p>

# kmr-iiwa-gripkit-cr-plus-l
![ROS](https://img.shields.io/badge/ros-%230A0FF9.svg?style=for-the-badge&logo=ros&logoColor=white)
![CMake](https://img.shields.io/badge/CMake-%23008FBA.svg?style=for-the-badge&logo=cmake&logoColor=white)
![Python](https://img.shields.io/badge/python-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54)

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

URDF and MoveIt configuration (ROS1) files of [Gripkit-CR-Plus-L](https://weiss-robotics.com/gripkit/) and [KRM-iiwa 7](https://www.kuka.com/en-us/products/mobility/mobile-robot-systems/kmr-iiwa). Depending on user's inputs (e.g., end_effector, mobile_base, and controllers), the URDF and MoveIt ***reconfigure*** themselves. The reposity also provides a simple Gazebo environment and Python classes and functions to experiment ***object grasping***.  

<p align="center">
<img src=./images/demo.png width=60% height=60%> <img src=./images/demo.gif width=38% height=38%>
</p>

The project was done while [Hojun Lee](https://www.linkedin.com/in/hjunlee94/) was working for Barton Research Group ([BRG](https://brg.engin.umich.edu/)) at the University of Michigan.

## Table of Contents

- [Repository Structure](#repository-structure)
- [Download Process](#download-process)
- [Simulation](#simulation)
    - [Gazebo Only](#gazebo-only)
    - [MoveIt Only](#moveit-only)
    - [MoveIt and Gazebo](#moveit-and-gazebo)
- [Automated Grasping](#automated-grasping)
    - [How to Run](#how-to-run)
    - [Planning Support](#planning-support)
    - [Grasp Representation](#grasp-representation)
- [Potential Extensions](#potential-extensions)
- [Issues](#issues)
- [ToDo Lists](#todo-lists)

---

## Repository Structure

    ├── auto_grasp
    │   └── src                           # Python source codes
    ├── gripkit_cr_plus_l_ad_description
    ├── gripkit_cr_plus_l_ad_description
    │   ├── meshes                        # STL files
    |   └── urdf                          # URDF description
    ├── gripkit_cr_plus_l_bb_description
    ├── gripkit_cr_plus_l_cc_description
    ├── gripkit_cr_plus_l_dd_description
    ├── iiwa7_description
    ├── images              
    ├── kmp200_description
    ├── kmriiwa_description  
    ├── kmriiwa_moveit
    │   ├── config                        # config files
    │   └── launch                        # ROS-launch files
    ├── models
    │   ├── dict                          # grasp dictionaries
    │   ├── marker                        # waypoint marker
    │   └── obj_05                        # object folder
    └── worlds

## Download Process

> **Note**
This repository has been tested on [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) and [Ubuntu 20.04](https://releases.ubuntu.com/focal/).
It also depends on **numpy**, **scipy**, and **tqdm**:

    cd ~/catkin_ws/src
    git clone https://github.com/kidpaul94/kmr-iiwa-gripkit-cr-plus-l.git
    cd kmr-iiwa-gripkit-cr-plus-l/
    pip3 install -r requirements.txt
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash

## Simulation
  
### Gazebo Only:

> **Note**
This launch file is only for **model visualization** purpose. No ROS controllers will get initialized.

    roslaunch kmriiwa_description gazebo.launch robot_name:=(choose iiwa or kmriiwa) hardware_interface:=(choose Position, Velocity, or Effort) ee_type:=(choose ad, bb, cc, or dd)
    
### MoveIt Only:

    roslaunch kmriiwa_moveit demo.launch robot_name:=(choose iiwa or kmriiwa) hardware_interface:=(choose Position, Velocity, or Effort) ee_type:=(choose ad, bb, cc, or dd)

### MoveIt and Gazebo:

    roslaunch kmriiwa_moveit demo_gazebo.launch robot_name:=(choose iiwa or kmriiwa) hardware_interface:=(choose Position, Velocity, or Effort) ee_type:=(choose ad, bb, cc, or dd)

## Automated Grasping

### How to Run:

> **Note**
`simulation.py` receives several different arguments. Run the `--help` command to see everything it receives.

    cd auto_grasp/src
    python3 simulation.py --help

### Planning Support:

The current implementation of the reposity supports [top-down grasp](https://github.com/kidpaul94/kmr-iiwa-gripkit-cr-plus-l/blob/02d5848b2492457b04c335ec33cd980cd692e030/auto_grasp/src/robot_manager.py#L100), [cartesian_space](https://github.com/kidpaul94/kmr-iiwa-gripkit-cr-plus-l/blob/02d5848b2492457b04c335ec33cd980cd692e030/auto_grasp/src/robot_manager.py#L51), and [joint_space](https://github.com/kidpaul94/kmr-iiwa-gripkit-cr-plus-l/blob/02d5848b2492457b04c335ec33cd980cd692e030/auto_grasp/src/robot_manager.py#L21) planning. The cartesian_space and joint_space plannings receives list of ROS pose message and joint configuration that a robot model needs to follow. For **top-down grasp**, it only supports **grasping center & direction** format at the moment. More details about this format is described [below](#grasp-representation).

### Grasp Representation:

> **Note**
The representation is defined based on a coordinate system of each object 3D model.

<p align="center">
<img src=./images/representation.png width=25% height=25%>
</p>

The image above shows the **grasping center & direction** format that the **top-down grasp** function uses. In [`simulation.py`](https://github.com/kidpaul94/kmr-iiwa-gripkit-cr-plus-l/blob/main/auto_grasp/src/simulation.py), we provide a list of this representation as a grasp dictionary associated with a specific object model.

## Potential Extensions

To run your own world with other objects, simply put them in the models and worlds folders. You can also swap iiwa 7 with iiwa 14. Let me know if there are **additional extensions & features** you want to see in this project (e.g., adding more robot models). Contribution is always welcome!

## Issues

> **Warning**
[MoveIt](https://ros-planning.github.io/moveit_tutorials/) trajectory execution sometimes fails **without any error or warning**. At the moment, the program skips the trial by checking whether the gripper reaches to a final grasping configuration. More stable trajectory planning/execution can be achieved by swaping KDL Kinematics Solver (default) with [IKFast Kinematics Solver](https://ros-planning.github.io/moveit_tutorials/doc/ikfast/ikfast_tutorial.html) or [TRAC-IK Kinematics Solver](https://ros-planning.github.io/moveit_tutorials/doc/trac_ik/trac_ik_tutorial.html).

## ToDo Lists

| **Model & Controller parameters tuning** | ![Progress](https://progress-bar.dev/100) |
| --- | --- |
| **Automation of grasping** | ![Progress](https://progress-bar.dev/100) |
