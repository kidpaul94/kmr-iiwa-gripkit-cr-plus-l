import rospy
import numpy as np
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import GetWorldProperties, SpawnModel, DeleteModel, GetModelState

from utils import Conversion, noisy_pose

class Model():
    def __init__(self, name: str = None, pose: Pose = None, sdf_name: str = None) -> None:  
        self.name, self.init_pose, self.sdf_name = name, pose, sdf_name
        self.base2obj = Conversion().pose2T(pose)

    def grasp_gen(self, path: str = None, is_6dof: bool = False, 
                  marker: bool = False) -> np.ndarray:
        """
        Generate grasp dictionary based on the current object pose and 
        contact point pairs (cpp).

        Parameters
        ----------
        path : string
            path to a .txt file of cpps

        Returns
        -------
        centers : 3xN : obj : `np.ndarray`
            array of potential gripper centers w.r.t the world frame
        directions : 3xN : obj : `np.ndarray`
            array of potential gripper directions w.r.t the world frame
        """
        with open(f"{path}_cpps.txt") as f:
            cpps = eval(f.read())
        cpps = np.asarray(cpps).T

        if is_6dof is True:
            with open(f"{path}_aprvs.txt") as f:
                aprvs = eval(f.read())
            for i in range(len(aprvs)):
                for j in range(len(aprvs[i])):
                    aprvs[i][j] = self.base2obj[:3,:3] @ aprvs[i][j]
        else:
            aprvs = None
        
        # array of potential gripper configurations w.r.t the object frame
        # 0.005 = 0.001(mm to m) * 0.5
        add_row = np.ones(cpps.shape[1])
        centers = 0.0005 * (cpps[:3,:] + cpps[3:,:])
        directions = cpps[:3,:] - cpps[3:,:]

        # array of potential gripper configurations w.r.t the world frame
        temp = self.base2obj @ np.vstack((centers, add_row))
        centers = temp[:3,:]
        directions = self.base2obj[:3,:3] @ directions
        if marker is True:
            with open(f"{path}_probs.txt") as f:
                probs = eval(f.read())
            idx = [ind for ind, ele in enumerate(probs) if ele > 0.5]
            aprvs = [aprvs[i] for i in idx]
            probs = [probs[i] for i in idx]
            centers, directions = centers[:,idx], directions[:,idx]
        else:
            probs = None

        return centers, directions, aprvs, probs

class EnvManager():
    def __init__(self) -> None:
        self.permanet_objects = self.get_gazebo_objects()
        self.added_objects = []

    def get_gazebo_objects(self) -> list:
        """
        Save a list of objects in the gazebo world.
        
        Parameters
        ----------
        None
        
        Returns
        -------
        obj_list : 1xN : obj : `list`
            object list composed with Models
        """
        rospy.wait_for_service('/gazebo/get_world_properties')
        world_state_client = rospy.ServiceProxy( '/gazebo/get_world_properties', GetWorldProperties)
        obj_names, obj_list = world_state_client.call().model_names, []
        for name in obj_names:
            pose = self.get_object_pose(name)
            obj_list.append(Model(name, pose))

        return obj_list

    def sync_with_gazebo(self) -> None:
        """
        Sync EnvManager objects with the list of objects in
        the gazebo world after deletion.

        Parameters
        ----------
        None
            
        Returns
        -------
        None
        """
        check_objects = self.get_gazebo_objects()
        added_org = set(temp.name for temp in self.added_objects)
        added_left, added_perm = [], []
        for obj in check_objects:
            if obj.name in added_org:
                added_left.append(obj)
            else:
                added_perm.append(obj)

        self.added_objects = added_left
        self.permanet_objects = added_perm
        
    def spawn_object(self, name: str, pose: Pose, sdf_name: str, is_noisy: bool = False) -> None:
        """
        Spawn an object in the gazebo world.

        Parameters
        ----------
        name : string
            name of the object in the gazebo world
        pose : obj : `Pose`
            pose of the object when spawning
        sdf_name : string
            sdf file of the object we spawn in the gazebo world
        is_noisy : bool
            whether add gaussian noise to the object pose

        Returns
        -------
        None
        """
        spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        spawn_model_client(model_name=name,
        model_xml=open(f'../../models/{sdf_name}/model.sdf', 'r').read(),
        robot_namespace='/foo', initial_pose=pose, reference_frame='world')
        if is_noisy:
            noise_position, noise_orientation = noisy_pose()
            pose.position.x = pose.position.x + noise_position[0]
            pose.position.y = pose.position.y + noise_position[1]
            pose.position.z = pose.position.z + noise_position[2]
            pose.orientation.x = pose.orientation.x + noise_orientation[0]
            pose.orientation.y = pose.orientation.y + noise_orientation[1]
            pose.orientation.z = pose.orientation.z + noise_orientation[2]
            pose.orientation.w = pose.orientation.w + noise_orientation[3]
        self.added_objects.append(Model(name, pose, sdf_name))

    @staticmethod
    def delete_object(name: str) -> None:
        """
        Delete an object in the gazebo world.

        Parameters
        ----------
        name : string
            name of the object in the gazebo world

        Returns
        -------
        None
        """
        rospy.wait_for_service('/gazebo/delete_model')
        delete_model_client = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        delete_model_client.call(model_name=name)

    @staticmethod
    def get_object_pose(name: str) -> Pose:
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
        rospy.wait_for_service('/gazebo/get_model_state')
        model_state_client = rospy.ServiceProxy( '/gazebo/get_model_state', GetModelState)
        state = model_state_client.call(model_name=name, relative_entity_name='map')

        return state.pose
