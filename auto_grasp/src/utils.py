import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Point, Quaternion, Pose

class Conversion():
    
    @staticmethod
    def list2pose(sixd: list, degrees: bool = True) -> Pose:
        """
        Convert 6D representation to ROS Pose message format.

        Parameters
        ----------
        sixd : 1x6 : obj : `list`
            list of xyz and three euler angles
        degrees : bool
            whether euler angles are given in degree or radian

        Returns
        -------
        `Pose` : pose message composed with xyz and quaternion
        """
        position = Point(sixd[0], sixd[1], sixd[2])
        temp = R.from_euler('xyz', sixd[3:], degrees).as_quat()
        quat = Quaternion(temp[0], temp[1], temp[2], temp[3])

        return Pose(position=position, orientation=quat)

    @staticmethod
    def pose2T(pose: Pose) -> np.ndarray:
        """
        Convert ROS Pose message format to homegeneous transformation. 

        Parameters
        ----------
        pose : obj : `Pose`
            pose message composed with xyz and quaternion

        Returns
        -------
        res : 4x4 : obj : `np.ndarray`
            homegeneous transformation format
        """
        res = np.eye(4)
        res[:3,3] = [pose.position.x, pose.position.y, pose.position.z]
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, 
                pose.orientation.w]
        res[:3,:3] = R.from_quat(quat).as_matrix()

        return res

    @staticmethod
    def cpp2R(direction: np.ndarray, aprv: list) -> np.ndarray:
        """
        Convert contact point pair format to rotation matrix. 

        Parameters
        ----------
        direction : 3x1 : obj : `np.ndarray`
            gripper direction w.r.t the world frame
        aprv : 3x1 : obj : `list`
            approach vector of a gripper of a cpp

        Returns
        -------
        res : 4x4 : obj : `np.ndarray`
            homegeneous transformation format
        """
        last_ax = np.cross(aprv, direction)
        R = np.eye(3)
        R[:,0], R[:,1], R[:,2] = direction, last_ax, aprv
        
        return R

def rot_matrix(axis_1: np.ndarray, axis_2: np.ndarray) -> np.ndarray:
    """ 
    Calculate a rotation matrix that aligns the axis_2 with the axis_1.
    
    Parameters
    ----------
    axis_1 : 1x3 : obj : `np.ndarray`
        unit direction vector 
    axis_2 : 1x3 : obj : `np.ndarray`
        unit direction vector

    Returns
    -------
    R : 3x3 :obj:`numpy.ndarray`
        rotation matrix
    """
    R = np.eye(3, dtype=np.float64)

    try:
        v = np.cross(axis_2, axis_1)
        c = np.dot(axis_2, axis_1)
        Vmat = np.array([[0., -v[2], v[1]], [v[2], 0., -v[0]], [-v[1], v[0], 0.]])
        R = R + Vmat + Vmat @ Vmat / (1 + c)
    except:
        print('Vectors are in exactly opposite direction')

    return R

def noisy_pose(mu: list = [0.0, 0.0], sigma: list = [0.5, 2.0], bouds: list = [1.5, 5.0]) -> list:
    """
    Add gaussian noise on pose values.

    Parameters
    ----------
    mu : 1x2 : obj : `list`
        means of gaussian noise models
    sigma : 1x2 : obj : `list`
        standard deviations of gaussian noise models

    Returns
    -------
    noise_position : 3x1 : obj : `np.ndarray`
        array of position noise in xyz
    noise_orientation : 4x1 : obj : `np.ndarray`
        array of orientation noise in quaternion
    """
    np.random.seed()
    noise_position = np.random.normal(mu[0],sigma[0],3)
    noise_orientation = np.random.normal(mu[1],sigma[1],3)
    noise_position = np.clip(noise_position, -bouds[0], bouds[0]) * 0.01
    noise_orientation = np.clip(noise_orientation, -bouds[1], bouds[1]) * np.pi / 180
    noise_orientation = R.from_euler('xyz', noise_orientation).as_quat()

    return noise_position, noise_orientation 
