import os
import time
import rospy
import argparse
import numpy as np
from tqdm import tqdm

from utils import Conversion
from env_manager import EnvManager
from robot_manager import Move_Robot

def parse_args(argv=None) -> None:
    parser = argparse.ArgumentParser(description='auto_pick')
    parser.add_argument('--name', default='object_1', type=str,
                        help='name of the object in the gazebo world.')
    parser.add_argument('--sixd', default=[0., -0.65, 0.715, 0, 0, -180.], type=list,
                        help='list of xyz and three euler angles.')
    parser.add_argument('--sdf_names', default=['obj_05'], type=str, nargs='+',
                        help='sdf files of objects we sequentially spawn in the gazebo world.')
    parser.add_argument('--grasp_dicts', default='../../models/dict', type=str,
                        help='root_dir to a .txt file of cpps.')
    parser.add_argument('--iter_sample', default=1, type=int,
                        help='number of iteration per grasp sample.')
    parser.add_argument('--tp_heights', default=[0.45, 0.3], type=list,
                        help='heights of waypoints that the robot follow before grasping.')
    parser.add_argument('--save_probs', default=True, type=bool,
                        help='whether to save the simulation results.')

    global args
    args = parser.parse_args(argv)

class Autopick():
    def __init__(self, args):
        self.EM, self.conv = EnvManager(), Conversion()
        self.mr = Move_Robot()
        self.pose = self.conv.list2pose(sixd=args.sixd)
        self.name, self.sdf_names = args.name, args.sdf_names
        self.grasp_dicts = args.grasp_dicts
        self.iter_sample, self.tp_heights = args.iter_sample, args.tp_heights
    
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
        total_list : 1xN : obj : `list`
            list indicating whether gripper centers and directions are selected
        num_selected : int
            total number of selected configurations
        """
        num_selected = 0
        refined_list = []
        for i in range(directions.shape[1]):
            # pitch = np.arctan(directions[2,i] / directions[0,i])
            roll = np.arctan(directions[2,i] / directions[1,i])

            if abs(roll) < 0.785:
                refined_list.append(i)
                num_selected = num_selected + 1
            else:
                refined_list.append(False)

        return refined_list, num_selected

    def final_check(self, current_pose: np.ndarray, gripper_pose: np.ndarray, 
                  threshold: float = 0.05) -> bool:
        """
        Check whether the gripper is in the correct final configuration.

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
            result of the check
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
        attempt, total = 0, repeat
        for _ in range(repeat):
            self.mr.gripper_control(command=False)
            time.sleep(1)
            executed, recorded = self.mr.cartesian_space(waypoints=[self.pose], tp_heights=self.tp_heights, 
                                                         center=center, direction=direction)
            time.sleep(1)
            if executed:
                link7_pose = self.mr.get_link_pose(name='link_7')
                ref_T= self.conv.pose2T(recorded[-1])
                gripper_T = self.conv.pose2T(link7_pose)
                # temp = gripper_T @ np.array([0., 0., 0.195, 1.])
                # gripper_T[:3,3] = temp[:3]

                if self.final_check(ref_T, gripper_T):
                    print('Object is inbetween the gripper fingers...')
                    self.mr.gripper_control()
                    time.sleep(1)

                    # Put the gripper on top of the object center
                    self.mr.cartesian_space(waypoints=[recorded[0]], top_down=False, visualize=False)
                    time.sleep(1)

                    if self.mr.is_grasped():
                        attempt = attempt + 1
                    print(f'{attempt} out of {total} succeeded!!!')
                else:
                    print('Failure in path executions!!!')
                    total = total - 1
            else:
                total = total - 1

            self.EM.delete_object(name=self.name)
            time.sleep(1)

            self.mr.joint_space(goal_config=[0.]*7, degrees=False)
            time.sleep(1)
            self.EM.spawn_object(name=self.name, pose=self.pose, sdf_name=sdf_name)
            time.sleep(1)
            self.EM.sync_with_gazebo()
            time.sleep(1)

        return attempt / total if total >= 1 else -1.0

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
        directory = f'result'
        if not os.path.exists(directory):
            print(f'Generate {directory} folder...')
            os.mkdir(directory)

        for obj in self.sdf_names:
            success_prob, entire_list = [], []
            self.EM.spawn_object(name=self.name, pose=self.pose, sdf_name=obj)
            time.sleep(1)
            self.EM.sync_with_gazebo()
            time.sleep(1)
            self.mr.gripper_control(command=False)

            path_dict = f'{self.grasp_dicts}/{obj}.txt'
            centers, directions = self.EM.added_objects[0].grasp_gen(path=path_dict)

            # Choose a grasp configuration that avoids collision with the environment (table)
            refined, num_selected = self.avoid_collision(directions)
            print(f'There are some infeasible grasp configurations in the provided set...')
            print(f'Number of refined configuration set: {num_selected} from {directions.shape[1]}')

            for idx in tqdm(refined):
                # Check the executing grasp configuration
                if idx:
                    res = self.execute(centers[:,idx], directions[:,idx], obj, self.iter_sample)
                    if res >= 0:
                        success_prob.append(res)
                        print(f'Object: {obj}')
                        print(f'Probabilities: {success_prob}')
                        print(f'Mean: {np.mean(success_prob)}, STD: {np.std(success_prob)}')
                    entire_list.append(res)
                else:
                    entire_list.append(idx)

            self.EM.delete_object(name=self.name)
            time.sleep(1)

            if args.save_probs:
                with open(f'{directory}/{obj}.txt', 'w') as output:
                    print(f'Generate {obj} grasp dictionaries...')
                    output.write(repr(entire_list))
                    output.close()

        print('Finished the simulation!!!')
    
if __name__ == "__main__":
    parse_args()
    rospy.init_node('auto_pick', anonymous=True)
    Autopick(args).initialize()
