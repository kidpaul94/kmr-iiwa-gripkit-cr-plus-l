from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Point, Quaternion, Pose

def list2Pose(vals: list, degrees: bool = True) -> Pose:
    """
    Convert 6D representation to ROS Pose message format

    Parameters
    ----------
    vals : 1x7 : obj : `list`
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

def noisy_pose(vals: Pose) -> Pose:
    """
    Add gaussian noise on pose values

    Parameters
    ----------
    vals : obj : `Pose`
        Pose values that we add noise on

    Returns
    -------
    """

    return
