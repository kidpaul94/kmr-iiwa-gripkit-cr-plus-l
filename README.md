# KMR-iiwa-Gripkit-CR-Plus-L
URDF and MoveIt configuration (ROS1) files of [Gripkit-CR-Plus-L](https://weiss-robotics.com/gripkit/) and [KRM-iiwa 7](https://www.kuka.com/en-us/products/mobility/mobile-robot-systems/kmr-iiwa). Depending on user's inputs (e.g., end_effector, mobile_base, controllers), the URDF and MoveIt **reconfigure** themselves.

![Example 0](./images/demo.png)

## Repository Structure

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

## Usages
Download the repository:

    cd ~/catkin_ws/src
    git clone https://github.com/kidpaul94/KMR-iiwa-Gripkit-CR-Plus-L.git
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
    
To visual the URDF in gazebo:

    roslaunch kmriiwa_description gazebo.launch robot_name:=(choose iiwa or kmriiwa) hardware_interface:=(choose Position, Velocity, or Effort) ee_type:=(choose ad, bb, cc, or dd)
    
To run the MoveIt:

    roslaunch kmriiwa_moveit demo.launch robot_name:=(choose iiwa or kmriiwa) hardware_interface:=(choose Position, Velocity, or Effort) ee_type:=(choose ad, bb, cc, or dd)

To run the MoveIt with gazebo:

    roslaunch kmriiwa_moveit demo_gazebo.launch robot_name:=(choose iiwa or kmriiwa) hardware_interface:=(choose Position, Velocity, or Effort) ee_type:=(choose ad, bb, cc, or dd)

## Potential Extensions
To run your own world with other objects, simply put them in the **models** and **worlds** folders.

    
## ToDo Lists
- [ ] Tune physical parameters of ROS controller
