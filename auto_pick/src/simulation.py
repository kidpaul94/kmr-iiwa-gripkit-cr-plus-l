import time
import copy
import rospy
import argparse
import numpy as np

from utils import Conversion
from env_manager import EnvManager
from robot_manager import Move_Robot

def parse_args(argv=None) -> None:
    parser = argparse.ArgumentParser(description='auto_pick')
    parser.add_argument('--name', default='object_1', type=str,
                        help='name of the object in the gazebo world.')
    parser.add_argument('--sixd', default=[0., -0.65, 0.715, 0, 0, -180.], type=list,
                        help='list of xyz and three euler angles.')
    parser.add_argument('--sdf_names', default=['obj_05'], type=list,
                        help='sdf files of objects we sequentially spawn in the gazebo world.')
    parser.add_argument('--grasp_dicts', default='../../models/dict', type=str,
                        help='root_dir to a .txt file of cpps.')
    parser.add_argument('--tp_heights', default=[0.45, 0.3], type=list,
                        help='heights of waypoints that the robot follow before grasping.')

    global args
    args = parser.parse_args(argv)

class Autopick():
    def __init__ (self, args):
        self.EM, self.conv = EnvManager(), Conversion()
        self.mr = Move_Robot()
        self.pose = self.conv.list2pose(sixd=args.sixd)
        self.name, self.sdf_names = args.name, args.sdf_names
        self.grasp_dicts, self.tp_heights = args.grasp_dicts, args.tp_heights
    
    @staticmethod
    def avoid_collision(directions: np.ndarray) -> np.ndarray:
        """
        Remove grasp configurations that potentially causes collision with the table

        Parameters
        ----------
        directions : 3xN : obj : `np.ndarray`
            array of potential gripper directions w.r.t the world frame

        Returns
        -------
        selected_idx : 1xN : obj : `list`
            indicies of selected gripper centers and directions
        """
        selected_idx = []
        x, y = np.array([1., 0., 0.]), np.array([0., 1., 0.])

        for i in range(directions.shape[1]):
            angle_x = np.math.atan2(np.linalg.det([directions[:,i],x]),
                                    np.dot(directions[:,i],x))
            angle_y = np.math.atan2(np.linalg.det([directions[:,i],y]),
                                    np.dot(directions[:,i],y))
            if angle_x < 0.785 and angle_y < 0.785:
                selected_idx.append(i)

        return selected_idx

    def isgrasped(self, current_pose: np.ndarray, gripper_pose: np.ndarray) -> bool:
        """
        Check whether grasping is successful or not.

        Parameters
        ----------
        current_pose : 4x4 : obj : `np.ndarray`
            current object pose (transformation matrix)
        gripper_pose : 4x4 : obj : `np.ndarray`
            current gripper pose (transformation matrix)

        Returns
        -------
        success : bool
            result of the grasping
        """
        success = True
        dist = np.linalg.norm(current_pose[:3,3] - gripper_pose[:3,3])
        
        return success

    def execute(self, center: np.ndarray, direction: np.ndarray, sdf_name: str, 
                repeat: int = 100) -> float:
        temp = 0
        for _ in range(repeat):
            self.mr.cartesian_space(waypoints=[self.pose], tp_heights=self.tp_heights, 
                            center=center, direction=direction)
            self.mr.gripper_control()

            # Put the gripper on top of the object center
            pick_up = copy.deepcopy(self.pose)
            pick_up.orientation.x = 0.0
            pick_up.orientation.y = 1.0
            pick_up.orientation.z = 0.0
            pick_up.orientation.w = 0.0
            pick_up.position.z += self.tp_heights[0]

            self.mr.cartesian_space(waypoints=[pick_up], top_down=False)

            obj_pose = self.EM.get_gazebo_pose(name=self.name)
            obj_T = self.conv.pose2T(obj_pose)
            gripper_T = self.conv.pose2T(pick_up)
            temp = gripper_T @ np.array([0., 0., -0.25, 1.])
            gripper_T[:3,3] = temp[:3]

            if self.isgrasped(obj_T, gripper_T):
                temp = temp + 1

            self.mr.go_home()
            self.EM.delete_object(name=self.name)
            time.sleep(2)
            self.EM.spawn_object(name=self.name, pose=self.pose, sdf_name=sdf_name)
            time.sleep(2)
            self.EM.sync_with_gazebo()

        return temp / repeat

    def initialize(self) -> None:
        """
        Calculate grasp configurations w.r.t the world frame and execute them.

        Parameters
        ----------
        None

        Returns
        -------
        None
        """
        success_prob = []

        for obj in self.sdf_names:
            self.EM.spawn_object(name=self.name, pose=self.pose, sdf_name=obj)
            time.sleep(2)
            self.EM.sync_with_gazebo()

            path_dict = f'{self.grasp_dicts}/{obj}.txt'
            centers, directions = self.EM.added_objects[0].grasp_gen(path=path_dict)

            # Choose a grasp configuration that avoids collision with the environment (table)
            # refined = self.avoid_collision(directions)

            for idx in range(1):
                # Check the executing grasp configuration
                print(f'Grasp Center: {centers[:,0]}')
                print(f'Grasp Direction: {directions[:,0]}')

                res = self.execute(centers[:,0], directions[:,0], obj, 1)
                success_prob.append(res)

            self.EM.delete_object(name=self.name)
            time.sleep(2)

        print('Finished the simulation!!!')
    
if __name__ == "__main__":
    parse_args()
    rospy.init_node('auto_pick', anonymous=True)
    Autopick(args).initialize()
