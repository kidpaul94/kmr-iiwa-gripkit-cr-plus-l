# kmr-iiwa-gripkit-cr-plus-l
URDF and MoveIt configuration (ROS1) files of [Gripkit-CR-Plus-L](https://weiss-robotics.com/gripkit/) and [KRM-iiwa 7](https://www.kuka.com/en-us/products/mobility/mobile-robot-systems/kmr-iiwa). Depending on user's inputs (e.g., end_effector, mobile_base, and controllers), the URDF and MoveIt **reconfigure** themselves.

![Example 0](./images/demo.png)

## Table of Contents

- [Repository Structure](#repository-structure)
- [Requirements](#requirements)
- [Usages](#usages)
    - [Download Process](#download-process)
    - [Gazebo](#gazebo)
    - [MoveIt](#moveit)
    - [MoveIt and Gazebo](#moveit-and-gazebo)
    - [Automated Pick](#automated-pick)
- [Potential Extensions](#potential-extensions)
- [ToDo Lists](#todo-lists)

---

## Repository Structure
    ├── auto_pick
    │   └── src         # source codes
    ├── gripkit_cr_plus_l_ad_description
    ├── gripkit_cr_plus_l_ad_description
    │   ├── meshes      # STL files
    |   └── urdf        # URDF description
    ├── gripkit_cr_plus_l_bb_description
    ├── gripkit_cr_plus_l_cc_description
    ├── gripkit_cr_plus_l_dd_description
    ├── iiwa7_description
    ├── images              
    ├── kmp200_description
    ├── kmriiwa_description  
    ├── kmriiwa_moveit
    │   ├── config      # config files
    │   └── launch      # ROS-launch files
    ├── models
    │   └── rotary_arm  # object folder
    └── worlds

## Requirements

This repository has been tested on [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) and [Ubuntu 20.04](https://releases.ubuntu.com/focal/).
It also depends on **numpy**, and **scipy**:

    pip3 install -r requirements.txt

## Usages

### Download Process:

    cd ~/catkin_ws/src
    git clone https://github.com/kidpaul94/kmr-iiwa-gripkit-cr-plus-l.git
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
    
### Gazebo:

    roslaunch kmriiwa_description gazebo.launch robot_name:=(choose iiwa or kmriiwa) hardware_interface:=(choose Position, Velocity, or Effort) ee_type:=(choose ad, bb, cc, or dd)
    
### MoveIt:

    roslaunch kmriiwa_moveit demo.launch robot_name:=(choose iiwa or kmriiwa) hardware_interface:=(choose Position, Velocity, or Effort) ee_type:=(choose ad, bb, cc, or dd)

### MoveIt and Gazebo:

    roslaunch kmriiwa_moveit demo_gazebo.launch robot_name:=(choose iiwa or kmriiwa) hardware_interface:=(choose Position, Velocity, or Effort) ee_type:=(choose ad, bb, cc, or dd)

### Automated Pick:

<a href="https://git.io/typing-svg"><img src="https://readme-typing-svg.demolab.com?font=Anton&size=29&pause=1000&color=F70000&width=435&lines=TO+BE+CONTINUE" alt="Typing SVG" />
</a>
</br>
**Note:** You can also checkout more arguments in each launch file.

## Potential Extensions
To run your own world with other objects, simply put them in the **models** and **worlds** folders.

## ToDo Lists
- [ ] Tune physical parameters of ROS controller
- [ ] Automation of pick&place
