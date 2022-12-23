import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Point, Quaternion, Pose

def list2pose(vals: list, degrees: bool = True) -> Pose:
    """
    Convert 6D representation to ROS Pose message format.

    Parameters
    ----------
    vals : 1x6 : obj : `list`
        list of xyz and three euler angles
    degrees : bool
        whether euler angles are given in degree or radian

    Returns
    -------
    val : obj : `Pose`
        pose message composed with xyz and quaternion
    """
    position = Point(vals[0], vals[1], vals[2])
    temp = R.from_euler('xyz', vals[3:], degrees).as_quat()
    quat = Quaternion(temp[0], temp[1], temp[2], temp[3])

    return Pose(position=position, orientation=quat)

def noisy_pose(vals: list) -> list:
    """
    Add gaussian noise on pose values.

    Parameters
    ----------
    vals : 1x6 : obj : `list`
        list of xyz and three euler angles

    Returns
    -------
    vals : 1x6 : obj : `list`
        xyz and three euler angles that we add noise on
    """
    noise = np.random.normal(0,1,6)
    vals += noise

    return vals

def read_txt(path: str) -> list:
    
    return

def cpp2trans(cpps: list) -> np.ndarray:
    trans = []

    return trans
