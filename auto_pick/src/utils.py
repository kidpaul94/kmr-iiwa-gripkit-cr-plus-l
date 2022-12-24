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

def cpp2T(path: str = None) -> list:
    """
    Convert list of contact point pairs (cpps) to transformation matrices.

    Parameters
    ----------
    path : string
        path to a .txt file of cpps

    Returns
    -------
    trans : 1xN : obj : `np.ndarray`
        list of transformation matrices (np.ndarray).
    """
    T = []
    with open(path) as f:
        list = eval(f.read())

    for item in list:
        center = (item[:3] + item[3:]) / 2
        direction = item[3:] - item[:3]

    return np.asarray(T)
