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
    def __init__(self, args):
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

        for i in range(directions.shape[1]):
            roll = np.arctan2(directions[2,i], directions[1,i])
            pitch = np.arctan2(directions[2,i], directions[0,i])
            if np.abs(roll) < 0.785 and np.abs(pitch) < 0.785:
                selected_idx.append(i)

        return selected_idx

    def isgrasped(self, current_pose: np.ndarray, gripper_pose: np.ndarray, 
                  threshold: float = 0.05) -> bool:
        """
        Check whether grasping is successful or not.

        Parameters
        ----------
        current_pose : 4x4 : obj : `np.ndarray`
            current object pose (transformation matrix)
        gripper_pose : 4x4 : obj : `np.ndarray`
            current gripper pose (transformation matrix)
        threshold : float
            threshold value to determin whether the object is 
            within two fingers

        Returns
        -------
        success : bool
            result of the grasping
        """
        dist = np.linalg.norm(current_pose[:3,3] - gripper_pose[:3,3])
        success = True if dist < threshold else False
        
        return success

    def execute(self, center: np.ndarray, direction: np.ndarray, sdf_name: str, 
                repeat: int = 100) -> float:
        """
        Execute a given grasp configuration # times to obtain its probability of success

        Parameters
        ----------
        center : 3xN : obj : `np.ndarray`
            array of potential gripper centers w.r.t the world frame
        direction : 3xN : obj : `np.ndarray`
            array of potential gripper directions w.r.t the world frame
        sdf_name : string
            sdf file of objects we spawn in the gazebo world
        repreat : int
            number of iteration to execute the grasp configuration

        Returns
        -------
        float : probability of successing the grasp
        """
        attempt = 0
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

            self.mr.cartesian_space(waypoints=[pick_up], top_down=False, visualize=False)

            obj_pose = self.EM.get_gazebo_pose(name=self.name)
            obj_T = self.conv.pose2T(obj_pose)
            gripper_T = self.conv.pose2T(pick_up)
            temp = gripper_T @ np.array([0., 0., 0.198, 1.])
            gripper_T[:3,3] = temp[:3]

            if self.isgrasped(obj_T, gripper_T):
                attempt = attempt + 1

            self.EM.delete_object(name=self.name)
            time.sleep(2)

            self.mr.go_home()
            
            self.EM.spawn_object(name=self.name, pose=self.pose, sdf_name=sdf_name)
            time.sleep(2)
            self.EM.sync_with_gazebo()

        return attempt / repeat

    def initialize(self) -> None:
        """
        Initialize auto pick process by spawning a target object and calculate 
        grasp configurations w.r.t the world frame.

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
            refined = self.avoid_collision(directions)

            for idx in refined:
                # Check the executing grasp configuration
                print(f'Grasp Center: {centers[:,idx]}')
                print(f'Grasp Direction: {directions[:,idx]}')

                res = self.execute(centers[:,idx], directions[:,idx], obj, 1)
                success_prob.append(res)

            self.EM.delete_object(name=self.name)
            time.sleep(2)

        print(f'Probabilities: {success_prob}')
        print('Finished the simulation!!!')
    
if __name__ == "__main__":
    parse_args()
    rospy.init_node('auto_pick', anonymous=True)
    Autopick(args).initialize()
