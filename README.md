# KMR-iiwa-Gripkit-CR-Plus-L
URDF and MoveIt configuration (ROS1) files of [Gripkit-CR-Plus-L](https://weiss-robotics.com/gripkit/) and [KRM-iiwa 7](https://www.kuka.com/en-us/products/mobility/mobile-robot-systems/kmr-iiwa)

## Repository Structure

    ├── gripkit_cr_plus_l_ad.zip 
    │   ├── gripkit_cr_plus_l_ad_description   # URDF description folder
    |   └── gripkit_cr_plus_l_ad_moveit_config # MoveIt configuration folder
    ├── iiwa.zip              
    ├── kmp200.zip                            
    │   └── kmp200_description
    |   └── kmp200_moveit_config
    └── kmr_iiwa_gripkit_cr_plus_l_dd.zip

Note: For completness, all URDF models have corresponding MoveIt configurations. However, MoveIt for ROS1 has limited support on mobile base path planning. Hence, path execution in a real robot may be limited. ROS navigation and move_base package can be alternatives for mobile base path planning.

## How to use
**Each zip file is a compressed ros package**. Place the unzipped folder into *~/catkin_ws/src* and run the following commands:

    cd ~/catkin_ws/
    catkin_make
    source devel/setup.bash
    
To visual the URDF in rviz:  

    roslaunch (robot or gripper name)_description display.launch
    
To visual the URDF in gazebo:

    roslaunch (robot or gripper name)_description gazebo.launch 
    
To run the MoveIt:

    roslaunch (robot or gripper name)_moveit_config demo.launch 
