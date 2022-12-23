import sys
import copy
import rospy
import moveit_commander as mc
from geometry_msgs.msg import Pose

class Move_Robot():
    def __init__(self):
        rospy.init_node("move_robot", anonymous=True)
        mc.roscpp_initialize(sys.argv)
        self.arm = mc.MoveGroupCommander("iiwa")
        
    def joint_space(self, joint_vals: list) -> None:
        """
        Command robot's movement in joint space 

        Parameters
        ----------
        joint_vals : 1xN : obj : `list`
            list of joint angles that the robot takes

        Returns
        -------
        success : bool
            whether the execution is successful or not
        """
        joint_goal = self.arm.get_current_joint_values()
        joint_goal[1] = joint_vals

        success = self.arm.go(joint_goal, wait=True)
        self.arm.stop()

        return success
    
    def cartesian_space(self, waypoints: list) -> None:
        """
        Command robot's movement in cartesian space 

        Parameters
        ----------
        waypoints : 1xN : obj : `list`
            list of waypoints that the robot follows
            
        Returns
        -------
        success : bool
            whether the execution is successful or not
        """
        self.arm.set_pose_target(waypoints)
        success = self.arm.go(wait=True)
        self.arm.stop()

        return success
