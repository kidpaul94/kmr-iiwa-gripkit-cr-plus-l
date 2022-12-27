import time
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
    parser.add_argument('--sdf_name', default='obj_05', type=str,
                        help='sdf file of the object we spawn in the gazebo world.')
    parser.add_argument('--grasp_dicts', default='../../models/dict/obj_05.txt', type=str,
                        help='path to a .txt file of cpps.')
    parser.add_argument('--tp_heights', default=[0.45, 0.3], type=list,
                        help='heights of waypoints that the robot follow before grasping.')

    global args
    args = parser.parse_args(argv)

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

def auto_pick(args) -> None:
    """
    Calculate grasp configurations w.r.t the world frame and execute them.

    Parameters
    ----------
    None

    Returns
    -------
    None
    """
    EM, conv = EnvManager(), Conversion()
    pose = conv.list2pose(sixd=args.sixd)
    EM.spawn_object(name=args.name, pose=pose, sdf_name=args.sdf_name)
    time.sleep(2)
    EM.sync_with_gazebo()
    centers, directions = EM.added_objects[0].grasp_gen(path=args.grasp_dicts)
    
    # Check the executing grasp configuration
    print(f'Grasp Center: {centers[:,0]}')
    print(f'Grasp Direction: {directions[:,0]}')

    # Choose a grasp configuration that avoids collision with the environment (table)
    refined = avoid_collision()
    mr = Move_Robot()
    mr.cartesian_space(waypoints=[pose], tp_heights=args.tp_heights, center=centers[:,0], 
                       direction=directions[:,0])
    
if __name__ == "__main__":
    parse_args()
    rospy.init_node('auto_pick', anonymous=True)
    auto_pick(args)
    # rospy.spin()
