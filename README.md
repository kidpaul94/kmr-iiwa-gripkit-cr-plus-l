# KMR-iiwa-Gripkit-CR-Plus
URDF files of [Gripkit-CR-Plus-L](https://weiss-robotics.com/gripkit/) and [KRM-iiwa](https://www.kuka.com/en-us/products/mobility/mobile-robot-systems/kmr-iiwa)

**How to use**

Place the unzipped folder into *~/catkin_ws/src* and run the following commands:

    cd ~/catkin_ws/
    catkin_make
    source devel/setup.bash
    
To visual the URDF in rviz:  

    roslaunch (folder_name)_description display.launch
    
To visual the URDF in gazebo:

    roslaunch (folder_name)_description gazebo.launch 
