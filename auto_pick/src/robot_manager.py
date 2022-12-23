import sys
import rospy
import numpy as np
import moveit_commander as mc
from geometry_msgs.msg import Pose

class Move_Robot():
    def __init__(self) -> None:
        mc.roscpp_initialize(sys.argv)
        self.arm = mc.MoveGroupCommander("iiwa")
        self.gripper = mc.MoveGroupCommander("gripper")
        self.arm_home = self.arm.get_current_joint_values()
        
    def joint_space(self, goal_config: list, degrees: bool = True) -> bool:
        """
        Command robot's movement in joint space. 

        Parameters
        ----------
        goal_config : 1xN : obj : `list`
            list of joint angles that the robot takes
        degrees: bool
            whether goal_cofig is in degree or radian

        Returns
        -------
        success : bool
            whether the execution is successful or not
        """
        joint_goal = self.arm.get_current_joint_values()
        print(f'Current joint states (radians): {joint_goal}')

        if degrees:
            goal_config = np.pi * goal_config / 180

        success = False
        try:
            success = self.arm.go(goal_config, wait=True)
            self.arm.stop()
        except Exception as e:
            print(e)

        return success
    
    def cartesian_space(self, waypoints: list, top_down: bool = True) -> bool:
        """
        Command robot's movement in cartesian space.

        Parameters
        ----------
        waypoints : 1xN : obj : `list`
            waypoints (Pose objects) that the robot follows
            
        Returns
        -------
        success : bool
            whether the execution is successful or not
        """
        if top_down:
            waypoints = self.top_down(waypoints[0])

        (plan, _) = self.arm.compute_cartesian_path(waypoints, 0.01, 0.0)  
        success = self.arm.execute(plan, wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()

        return success

    def top_down(self, goal_config: Pose) -> list:
        """
        Plan top-down grasping by creating waypoints.

        Parameters
        ----------
        goal_config : obj : `Pose`
            Goal pose that the robot will reach
            
        Returns
        -------
        waypoints : 1xN : obj : `list`
            waypoints that the robot follows
        """
        waypoints = [self.arm.get_current_pose().pose]
        wpose = goal_config
        wpose.position.z += 1.0

        waypoints.append(wpose)
        waypoints.append(goal_config)
        
        return waypoints
