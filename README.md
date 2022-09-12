# KMR-iiwa-Gripkit-CR-Plus-L
URDF and MoveIt configuration (ROS1) files of [Gripkit-CR-Plus-L](https://weiss-robotics.com/gripkit/) and [KRM-iiwa](https://www.kuka.com/en-us/products/mobility/mobile-robot-systems/kmr-iiwa)

## Repository Structure

    ├── gripkit_cr_plus_l_ad.zip 
    │   ├── gripkit_cr_plus_l_ad_description   # URDF description folder
    |   └── gripkit_cr_plus_l_ad_moveit_config # MoveIt configuration folder
    ├── iiwa.zip              
    ├── kmp200.zip                            
    │   └── kmp200_description            
    └── kmr_iiwa_gripkit_cr_plus_l_dd.zip

Note: MoveIt for ROS1 does not officially support mobile base (e.g., PPR joint) control. Hence, all mobile bases are fixed. Custom implementation is necessary to support mobile base planning in MoveIt.    

## How to use
Place the unzipped folder into *~/catkin_ws/src* and run the following commands:

    cd ~/catkin_ws/
    catkin_make
    source devel/setup.bash
    
To visual the URDF in rviz:  

    roslaunch (folder_name)_description display.launch
    
To visual the URDF in gazebo:

    roslaunch (folder_name)_description gazebo.launch 
    
To run the MoveIt:

    roslaunch (folder_name)_moveit_config demo.launch 

Note: Currently, gazebo.launch does not run properly. This is because physical properties of some parts are not defined.
