import sys
import copy
import numpy as np
import moveit_commander as mc
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R

from utils import rot_matrix

class Move_Robot():
    def __init__(self) -> None:
        mc.roscpp_initialize(sys.argv)
        self.arm = mc.MoveGroupCommander("iiwa")
        self.gripper = mc.MoveGroupCommander("gripper")
        self.arm_home = self.arm.get_current_joint_values()
        self.tdR = np.array([[-1.,0.,0.],[0.,1.,0.],[0.,0.,-1.]])
        
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
    
    def cartesian_space(self, waypoints: list, center: np.ndarray, 
                        direction: np.ndarray, top_down: bool = True) -> bool:
        """
        Command robot's movement in cartesian space.

        Parameters
        ----------
        waypoints : 1xN : obj : `list`
            waypoints (Pose objects) that the robot follows
        center : 3x1 : obj : `np.ndarray`
            array of potential gripper centers w.r.t the world frame
        direction : 3x1 : obj : `np.ndarray`
            array of potential gripper directions w.r.t the world frame
        top_down : bool
            whether execute top_down grasping or not
            
        Returns
        -------
        success : bool
            whether the execution is successful or not
        """
        if top_down:
            waypoints = self.top_down(waypoints[0], center, direction)

        (plan, _) = self.arm.compute_cartesian_path(waypoints, 0.01, 0.0)  
        success = self.arm.execute(plan, wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()

        return success

    def top_down(self, object_pose: Pose, center: np.ndarray, direction: np.ndarray) -> list:
        """
        Plan top-down grasping by creating waypoints.

        Parameters
        ----------
        object_pose : obj : `Pose`
            pose of the object center w.r.t the world frame
        center : 3x1 : obj : `np.ndarray`
            array of potential gripper centers w.r.t the world frame
        direction : 3x1 : obj : `np.ndarray`
            array of potential gripper directions w.r.t the world frame
            
        Returns
        -------
        waypoints : 1xN : obj : `list`
            waypoints that the robot follows
        """
        waypoints = []
        wpose = object_pose

        # Put the gripper on top of the object center
        wpose.orientation.x = 0.0
        wpose.orientation.y = 1.0
        wpose.orientation.z = 0.0
        wpose.orientation.w = 0.0
        wpose.position.z += 0.35
        waypoints.append(copy.deepcopy(wpose))

        # Aligh the gripper x-axis with the direction vector 
        # of a contact point pair 
        norm_dir = direction / np.linalg.norm(direction)
        rot = rot_matrix(norm_dir, np.array([-1.,0.,0.])) @ self.tdR
        quat = R.from_matrix(rot).as_quat()

        temp = np.eye(4)
        temp[:3,:3] = rot
        temp[:3,3] = center
        res = temp @ np.array([0., 0., -0.2, 1.]) 

        wpose.orientation.x = quat[0]
        wpose.orientation.y = quat[1]
        wpose.orientation.z = quat[2]
        wpose.orientation.w = quat[3]
        wpose.position.x = res[0]
        wpose.position.y = res[1]
        wpose.position.z = res[2]
        waypoints.append(copy.deepcopy(wpose))

        # Move the gripper towards the grasping center
        wpose.position.x = center[0]
        wpose.position.y = center[1]
        wpose.position.z = center[2]
        waypoints.append(wpose)
        
        return waypoints
