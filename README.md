# Gripkit-CR-Plus-L
URDF files of [Gripkit-CR-Plus-L](https://weiss-robotics.com/gripkit/)

**How to use**

Place the unzipped folder into *~/catkin_ws/src* and run the following commands:

    cd ~/catkin_ws/
    catkin_make
    source devel/setup.bash
    
To visual the URDF in rviz:  

    roslaunch (gripper_name)_description display.launch
    
To visual the URDF in gazebo:

    roslaunch (gripper_name)_description gazebo.launch 
