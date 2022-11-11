# KMR-iiwa-Gripkit-CR-Plus-L
URDF and MoveIt configuration (ROS1) files of [Gripkit-CR-Plus-L](https://weiss-robotics.com/gripkit/) and [KRM-iiwa 7](https://www.kuka.com/en-us/products/mobility/mobile-robot-systems/kmr-iiwa)

## Repository Structure

    ├── gripkit_cr_plus_l_ad_description
    │   ├── meshes   # STL files
    |   └── urdf     # URDF description folder
    ├── gripkit_cr_plus_l_bb_description
    ├── gripkit_cr_plus_l_cc_description
    ├── gripkit_cr_plus_l_dd_description
    ├── iiwa7_description              
    ├── kmp200_description
    ├── kmr_iiwa_gripkit_cr_plus_l_description  
    └── kmr_iiwa_gripkit_cr_plus_l_moveit

## Note
At the moment, we are currently preparing a reconfigurable moveit file.

## How to use
Download the repository:

    cd ~/catkin_ws/src
    git clone https://github.com/kidpaul94/KMR-iiwa-Gripkit-CR-Plus-L.git
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
    
To visual the URDF in gazebo:

    roslaunch kmr_iiwa_gripkit_cr_plus_l_description gazebo.launch ee_type:=(choose ad, bb, cc, or dd) mobile_base:=(choose kmp200 or None)
    
To run the MoveIt:

    roslaunch kmr_iiwa_gripkit_cr_plus_l_moveit_config demo.launch 

To run the MoveIt with gazebo:

    roslaunch kmr_iiwa_gripkit_cr_plus_l_moveit_config demo_gazebo.launch controller_type:=(choose Position, Velocity, Effort)
    
**ToDo Lists**
- [ ] prepare the reconfigurable moveit file
