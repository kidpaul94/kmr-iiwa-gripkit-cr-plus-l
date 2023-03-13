import sys
import time
import copy
import rospy
import numpy as np
import moveit_commander as mc
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R
from gazebo_msgs.srv import SpawnModel, DeleteModel, GetLinkState, GetJointProperties

from utils import rot_matrix

class Move_Robot():
    def __init__(self) -> None:
        mc.roscpp_initialize(sys.argv)
        self.arm = mc.MoveGroupCommander("iiwa")
        self.arm.set_planner_id('RRTConnect')
        self.gripper = mc.MoveGroupCommander("gripper")
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

        try:
            success = self.arm.go(goal_config, wait=True)
            self.arm.stop()
        except Exception as e:
            print(e)

        return success
    
    def cartesian_space(self, waypoints: list, tp_heights: list = None, 
                        center: np.ndarray = None, direction: np.ndarray = None, 
                        top_down: bool = True, visualize: bool = True) -> bool:                
        """
        Command robot's movement in cartesian space.

        Parameters
        ----------
        waypoints : 1xN : obj : `list`
            waypoints (Pose objects) that the robot follows
        tp_heights : 1x2 : obj : `list` 
            heights of waypoints that the robot follow before grasping
        center : 3x1 : obj : `np.ndarray`
            array of potential gripper centers w.r.t the world frame
        direction : 3x1 : obj : `np.ndarray`
            array of potential gripper directions w.r.t the world frame
        top_down : bool
            whether execute top_down grasping or not
        visualize : bool
            whether visualize waypoints or not
            
        Returns
        -------
        success : bool
            whether the execution is successful or not
        waypoints : 1xN : obj : `list`
            waypoints (Pose objects) that the robot follows
        """
        if top_down:
            waypoints, grasp_pose = self.top_down(waypoints[0], tp_heights, center, direction)

        if visualize:
            self.spawn_markers(waypoints, grasp_pose)

        start = time.time()
        (plan, _) = self.arm.compute_cartesian_path(waypoints, 0.001, 0.0)  
        _ = self.arm.execute(plan, wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()
        duration = time.time() - start

        success = True if duration > 1 else False
        print(f'Planned Motion executed: {success}')

        if visualize:
            self.delete_markers(waypoints)

        return success, waypoints

    def top_down(self, object_pose: Pose = None, tp_heights: list = None, 
                 center: np.ndarray = None, direction: np.ndarray = None) -> list:
        """
        Plan top-down grasping by creating waypoints.

        Parameters
        ----------
        object_pose : obj : `Pose`
            pose of the object center w.r.t the world frame
        tp_heights : 1x2 : obj : `list` 
            heights of waypoints that the robot follow before grasping
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
        wpose = copy.deepcopy(object_pose)

        # Put the gripper on top of the object center
        wpose.orientation.x = 0.0
        wpose.orientation.y = 1.0
        wpose.orientation.z = 0.0
        wpose.orientation.w = 0.0
        wpose.position.z += tp_heights[0]

        # Aligh the gripper x-axis with the direction vector 
        # of a contact point pair 
        norm_dir = direction / np.linalg.norm(direction)
        rot = rot_matrix(norm_dir, np.array([-1.,0.,0.])) @ self.tdR
        quat = R.from_matrix(rot).as_quat()

        temp = np.eye(4)
        temp[:3,:3] = rot
        temp[:3,3] = center
        res = temp @ np.array([0., 0., -tp_heights[1], 1.])

        wpose.orientation.x = quat[0]
        wpose.orientation.y = quat[1]
        wpose.orientation.z = quat[2]
        wpose.orientation.w = quat[3]
        wpose.position.x = res[0]
        wpose.position.y = res[1]
        wpose.position.z = res[2]
        waypoints.append(copy.deepcopy(wpose))

        # Move the gripper towards the grasping center
        res = temp @ np.array([0., 0., -0.195, 1.])

        wpose.position.x = res[0]
        wpose.position.y = res[1]
        wpose.position.z = res[2]
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x = temp[0,3]
        wpose.position.y = temp[1,3]
        wpose.position.z = temp[2,3]
        
        return waypoints, wpose

    @staticmethod
    def is_grasped(joint_names: list = ['Slider01', 'Slider02'], close_pos: list = [0.0065, -0.0065],
                   threshold: float = 0.0005):
        """
        Return the robot to home configurations.

        Parameters
        ----------
        joint_names : 1xN : obj : `list`
            list of gripper joint names
        close_pos : 1xN : obj : `list`
            list of closed gripper joint positions
        threshold : float
            threshold value to determin whether the object is 
            within two fingers

        Returns
        -------
        bool : whether succeed in grasping    
        """
        rospy.wait_for_service('/gazebo/get_joint_properties')
        link_state_client = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
        for idx, name in enumerate(joint_names):
            state = link_state_client.call(joint_name=name)
            if abs(close_pos[idx] - state.position[0]) < threshold:   
                return False

        return True

    def gripper_control(self, command: bool = True) -> None:
        """
        Open or close gripper. 

        Parameters
        ----------
        command: bool
            close (True) or open (False) the robot gripper

        Returns
        -------
        None
        """
        if command:
            self.gripper.go([0.0065, -0.0065], wait=True)
        else:
            self.gripper.go([-0.021, 0.021], wait=True)
        self.gripper.stop()

    @staticmethod
    def spawn_markers(waypoints: list, grasp_pose: Pose) -> None:
        """
        Spawn markers in the gazebo world.

        Parameters
        ----------
        waypoints : 1x3 : obj : `list`
            waypoints that the robot follows
        grasp_pose : obj : `Pose`
            pose of the grasp w.r.t the world frame

        Returns
        -------
        None
        """
        spawn_marker_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        num_wp = len(waypoints)
        for i in range(num_wp):
            spawn_marker_client(model_name=f'marker_{i}',
            model_xml=open(f'../../models/marker/model.sdf', 'r').read(),
            robot_namespace='/foo', initial_pose=waypoints[i], reference_frame='world')
        spawn_marker_client(model_name=f'marker_{num_wp}',
        model_xml=open(f'../../models/marker/model.sdf', 'r').read(),
        robot_namespace='/foo', initial_pose=grasp_pose, reference_frame='world')

    @staticmethod
    def delete_markers(waypoints: list) -> None:
        """
        Delete markers in the gazebo world.

        Parameters
        ----------
        waypoints : 1x3 : obj : `list`
            waypoints that the robot follows

        Returns
        -------
        None
        """
        rospy.wait_for_service('/gazebo/delete_model')
        delete_marker_client = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        num_wp = len(waypoints)
        for i in range(num_wp + 1):
            delete_marker_client.call(model_name=f'marker_{i}')

    @staticmethod
    def get_link_pose(name: str) -> Pose:
        """
        Retrieve an object pose from the gazebo world.

        Parameters
        ----------
        name : string
            name of the object in the gazebo world

        Returns
        -------
        `Pose` : current pose of the object in the gazebo world
        """
        rospy.wait_for_service('/gazebo/get_link_state')
        link_state_client = rospy.ServiceProxy( '/gazebo/get_link_state', GetLinkState)
        state = link_state_client.call(link_name=name, reference_frame='map')

        return state.link_state.pose
